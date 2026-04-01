#!/usr/bin/env python3
"""
Visualization Bridge

Forwards selected ROS2 topics from the robot's local DDS (domain 42)
to a network-facing DDS (domain 43) at throttled rates for remote rviz.

Architecture: TWO SEPARATE PROCESSES sharing a multiprocessing.Queue.
  Subscriber process: CYCLONEDDS_URI=cyclonedds_local.xml, domain 42
                      Subscribes to robot topics, pushes serialized msgs to queue.
  Publisher  process: CYCLONEDDS_URI=cyclonedds_viz_bridge.xml, domain 43
                      Pulls from queue, publishes over WiFi for laptop rviz.

Both processes watch viz_bridge_config.yaml via inotify and hot-reload
subscriptions/publishers on change — no container restart needed.

Why two processes: CycloneDDS reads CYCLONEDDS_URI once at lib-load time per
process. Swapping the env var between rclpy.init() calls in the same process
does nothing — both contexts use the first config. Separate processes get
separate envs set before any ROS import happens.

Why spawn (not fork): fork copies the parent's CycloneDDS state into both
children. When the second child tries to create a domain, CycloneDDS sees a
conflict on shared-memory/socket resources left by the first child's init and
crashes with "failed to create domain". spawn starts a fresh interpreter with
no inherited CycloneDDS state, so each child initializes cleanly.

Usage (on robot):
  CYCLONE_INTERFACE_IP=192.168.50.1 python3 viz_bridge.py
On laptop:
  ROS_DOMAIN_ID=43 CYCLONE_INTERFACE_IP=192.168.50.97 rviz2
"""

import argparse
import ctypes
import importlib
import multiprocessing
import os
import select
import struct
import sys
import time
import threading
from pathlib import Path

import yaml

# ---------------------------------------------------------------------------
# CRITICAL: set start method to 'spawn' before any Process() calls.
# Must be at module level inside the if __name__ == "__main__" guard (done in
# main()) but we also call it here defensively for direct imports.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def resolve_msg_type(type_string):
    parts = type_string.rsplit(".", 1)
    if len(parts) != 2:
        raise ValueError(f"Invalid message type: {type_string}")
    module = importlib.import_module(parts[0])
    return getattr(module, parts[1])


def load_config(config_path):
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def make_qos(topic, rate):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    if rate == 0 or "static" in topic or topic == "/map":
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
    elif topic == "/tf":
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
    else:
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )


# ---------------------------------------------------------------------------
# inotify watcher — no extra deps, uses Linux kernel API via ctypes
# ---------------------------------------------------------------------------

IN_CLOSE_WRITE = 0x00000008  # file closed after write (most editors)
IN_MOVED_TO = 0x00000080  # file moved into dir (vim/emacs atomic swap-write)
IN_CREATE = 0x00000100  # file created in dir


def watch_config(config_path, callback):
    """
    Watches the parent directory of config_path for writes to that file.
    Watches the directory (not the file) so it catches atomic editor saves.
    Calls callback() with a small debounce. Runs forever — call from a daemon thread.
    """
    libc = ctypes.CDLL("libc.so.6", use_errno=True)
    fd = libc.inotify_init()
    if fd < 0:
        print("[inotify] init failed — hot-reload disabled", flush=True)
        return

    watch_dir = str(Path(config_path).parent)
    watch_name = Path(config_path).name
    mask = IN_CLOSE_WRITE | IN_MOVED_TO | IN_CREATE

    wd = libc.inotify_add_watch(fd, watch_dir.encode(), ctypes.c_uint32(mask))
    if wd < 0:
        print(
            f"[inotify] watch failed on {watch_dir} — hot-reload disabled", flush=True
        )
        os.close(fd)
        return

    print(f"[inotify] watching {config_path}", flush=True)
    HDR = struct.calcsize("iIII")

    while True:
        r, _, _ = select.select([fd], [], [], 1.0)
        if not r:
            continue
        raw = os.read(fd, 4096)
        offset = 0
        while offset + HDR <= len(raw):
            wd_ev, mask_ev, cookie, name_len = struct.unpack_from("iIII", raw, offset)
            offset += HDR
            name = b""
            if name_len > 0:
                name = raw[offset : offset + name_len].rstrip(b"\x00")
                offset += name_len
            if name.decode(errors="replace") == watch_name:
                time.sleep(0.2)  # debounce — editors sometimes fire two events
                print("[inotify] config changed, reloading...", flush=True)
                try:
                    callback()
                except Exception as e:
                    print(f"[inotify] reload error: {e}", flush=True)


# ---------------------------------------------------------------------------
# Subscriber process (domain 42, local)
# ---------------------------------------------------------------------------


def subscriber_process(config_path, queue, stop_event):
    """
    CYCLONEDDS_URI must be set to cyclonedds_local.xml before this function runs.
    Subscribes to configured topics, pushes (topic, type_str, serialized) to queue.
    Hot-reloads subscriptions on config change without restarting.
    """
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import serialize_message

    config = load_config(config_path)
    bridge_cfg = config["bridge"]
    domain = bridge_cfg["robot_domain_id"]

    print(
        f"[SUB] domain={domain} CYCLONEDDS_URI={os.environ.get('CYCLONEDDS_URI', 'unset')}",
        flush=True,
    )

    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = Node("viz_bridge_sub", context=ctx)

    lock = threading.Lock()
    subscriptions = {}  # topic -> (sub_handle, interval_ref)
    last_pub = {}  # topic -> last forward timestamp
    msg_counts = {}  # topic -> count

    def make_callback(topic, interval_ref):
        # interval_ref is a list[float] so reload can update rate without
        # recreating the subscription
        def cb(msg):
            now = time.monotonic()
            if interval_ref[0] > 0 and (now - last_pub.get(topic, 0)) < interval_ref[0]:
                return
            try:
                serialized = serialize_message(msg)
                queue.put_nowait(
                    (
                        topic,
                        type(msg).__module__ + "." + type(msg).__qualname__,
                        serialized,
                    )
                )
                with lock:
                    last_pub[topic] = now
                    msg_counts[topic] = msg_counts.get(topic, 0) + 1
            except Exception as e:
                print(f"[SUB] queue error on {topic}: {e}", flush=True)

        return cb

    def apply_config(topics_cfg):
        new_topics = {e["topic"] for e in topics_cfg}

        # Remove dropped topics
        for topic in list(subscriptions.keys() - new_topics):
            sub, _ = subscriptions.pop(topic)
            node.destroy_subscription(sub)
            print(f"[SUB] removed {topic}", flush=True)

        # Add new / update rates
        for entry in topics_cfg:
            topic = entry["topic"]
            type_str = entry["type"]
            rate = float(entry.get("rate", 1.0))
            min_interval = (1.0 / rate) if rate > 0 else 0.0
            rate_str = f"{rate:.1f}Hz" if rate > 0 else "relay"

            if topic in subscriptions:
                subscriptions[topic][1][0] = min_interval  # update rate in-place
                print(f"[SUB] rate updated {topic} @ {rate_str}", flush=True)
            else:
                try:
                    msg_type = resolve_msg_type(type_str)
                except Exception as e:
                    print(f"[SUB] SKIP {topic}: {e}", flush=True)
                    continue
                ref = [min_interval]
                sub = node.create_subscription(
                    msg_type, topic, make_callback(topic, ref), make_qos(topic, rate)
                )
                subscriptions[topic] = (sub, ref)
                print(f"[SUB] added {topic} @ {rate_str}", flush=True)

    with lock:
        apply_config(config["topics"])
    print(f"[SUB] ready: {len(subscriptions)} subscriptions", flush=True)

    def on_change():
        try:
            cfg = load_config(config_path)
            with lock:
                apply_config(cfg["topics"])
            print(f"[SUB] reload done: {len(subscriptions)} subscriptions", flush=True)
        except Exception as e:
            print(f"[SUB] reload failed: {e}", flush=True)

    threading.Thread(
        target=watch_config, args=(config_path, on_change), daemon=True
    ).start()

    def stats():
        while not stop_event.is_set():
            time.sleep(30)
            with lock:
                total = sum(msg_counts.values())
                active = sum(1 for v in msg_counts.values() if v > 0)
            print(
                f"[SUB] {total} msgs, {active}/{len(subscriptions)} active, queue={queue.qsize()}",
                flush=True,
            )

    threading.Thread(target=stats, daemon=True).start()

    executor = MultiThreadedExecutor(context=ctx)
    executor.add_node(node)
    try:
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        print("[SUB] shutdown complete", flush=True)


# ---------------------------------------------------------------------------
# Publisher process (domain 43, network/WiFi)
# ---------------------------------------------------------------------------


def publisher_process(config_path, queue, stop_event):
    """
    CYCLONEDDS_URI must be set to cyclonedds_viz_bridge.xml before this runs.
    Pulls (topic, type_str, serialized) from queue and publishes on domain 43.
    Hot-reloads publishers on config change.
    """
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message

    config = load_config(config_path)
    bridge_cfg = config["bridge"]
    domain = bridge_cfg["viz_domain_id"]

    print(
        f"[PUB] domain={domain} CYCLONEDDS_URI={os.environ.get('CYCLONEDDS_URI', 'unset')}",
        flush=True,
    )

    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = Node("viz_bridge_pub", context=ctx)

    lock = threading.Lock()
    publishers = {}  # topic -> (publisher, msg_type)
    msg_counts = {}  # topic -> count

    def apply_config(topics_cfg):
        new_topics = {e["topic"] for e in topics_cfg}

        # Remove dropped
        for topic in list(publishers.keys() - new_topics):
            pub, _ = publishers.pop(topic)
            node.destroy_publisher(pub)
            print(f"[PUB] removed {topic}", flush=True)

        # Add new
        for entry in topics_cfg:
            topic = entry["topic"]
            type_str = entry["type"]
            rate = float(entry.get("rate", 1.0))
            if topic in publishers:
                continue
            try:
                msg_type = resolve_msg_type(type_str)
            except Exception as e:
                print(f"[PUB] SKIP {topic}: {e}", flush=True)
                continue
            pub = node.create_publisher(msg_type, topic, make_qos(topic, rate))
            publishers[topic] = (pub, msg_type)
            print(f"[PUB] advertised {topic}", flush=True)

    with lock:
        apply_config(config["topics"])
    print(f"[PUB] ready: {len(publishers)} publishers on domain {domain}", flush=True)

    def on_change():
        try:
            cfg = load_config(config_path)
            with lock:
                apply_config(cfg["topics"])
            print(f"[PUB] reload done: {len(publishers)} publishers", flush=True)
        except Exception as e:
            print(f"[PUB] reload failed: {e}", flush=True)

    threading.Thread(
        target=watch_config, args=(config_path, on_change), daemon=True
    ).start()

    try:
        while not stop_event.is_set():
            try:
                item = queue.get(timeout=0.1)
            except Exception:
                continue

            topic, type_str, serialized = item

            with lock:
                entry = publishers.get(topic)
            if entry is None:
                continue  # topic removed from config

            pub, msg_type = entry
            try:
                msg = deserialize_message(serialized, msg_type)
                pub.publish(msg)
                msg_counts[topic] = msg_counts.get(topic, 0) + 1
            except Exception as e:
                print(f"[PUB] publish error on {topic}: {e}", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        print("[PUB] shutdown complete", flush=True)


# ---------------------------------------------------------------------------
# Entrypoints for spawned processes — must be module-level for pickling
# ---------------------------------------------------------------------------


def _run_sub(config_path, local_xml, domain_id, queue, stop_event):
    """Entrypoint for the subscriber child (spawned, fresh interpreter)."""
    os.environ["CYCLONEDDS_URI"] = f"file://{local_xml}"
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    subscriber_process(config_path, queue, stop_event)


def _run_pub(config_path, net_xml, domain_id, queue, stop_event):
    """Entrypoint for the publisher child (spawned, fresh interpreter)."""
    os.environ["CYCLONEDDS_URI"] = f"file://{net_xml}"
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    publisher_process(config_path, queue, stop_event)


# ---------------------------------------------------------------------------
# Main — spawns two processes, each with correct env set before any ROS import
# ---------------------------------------------------------------------------


def main():
    # Must be called before any Process() instantiation.
    # 'spawn' starts a fresh Python interpreter per child — no inherited
    # CycloneDDS state — so each domain initializes cleanly without conflict.
    multiprocessing.set_start_method("spawn")

    parser = argparse.ArgumentParser(description="ROS2 Viz Bridge")
    parser.add_argument("--config", default="/workspace/viz_bridge_config.yaml")
    args = parser.parse_args()

    if not Path(args.config).exists():
        print(f"[VizBridge] config not found: {args.config}")
        sys.exit(1)

    if not os.environ.get("CYCLONE_INTERFACE_IP"):
        print("[VizBridge] ERROR: CYCLONE_INTERFACE_IP not set")
        print("Usage: CYCLONE_INTERFACE_IP=192.168.50.1 python3 viz_bridge.py")
        sys.exit(1)

    config = load_config(args.config)
    bridge_cfg = config["bridge"]
    local_xml = "/workspace/cyclonedds_local.xml"
    net_xml = bridge_cfg.get(
        "cyclonedds_config", "/workspace/cyclonedds_viz_bridge.xml"
    )

    print(
        f"[VizBridge] start_method=spawn (fresh interpreter per child, no CycloneDDS state leakage)"
    )
    print(f"[VizBridge] CYCLONE_INTERFACE_IP={os.environ['CYCLONE_INTERFACE_IP']}")
    print(f"[VizBridge] SUB: domain {bridge_cfg['robot_domain_id']} via {local_xml}")
    print(f"[VizBridge] PUB: domain {bridge_cfg['viz_domain_id']} via {net_xml}")
    print(f"[VizBridge] hot-reload: enabled via inotify")

    queue = multiprocessing.Queue(maxsize=1000)
    stop_event = multiprocessing.Event()

    sub_proc = multiprocessing.Process(
        target=_run_sub,
        args=(args.config, local_xml, bridge_cfg["robot_domain_id"], queue, stop_event),
        name="viz-sub",
        daemon=True,
    )
    pub_proc = multiprocessing.Process(
        target=_run_pub,
        args=(args.config, net_xml, bridge_cfg["viz_domain_id"], queue, stop_event),
        name="viz-pub",
        daemon=True,
    )

    sub_proc.start()
    pub_proc.start()
    print(f"[VizBridge] SUB pid={sub_proc.pid}  PUB pid={pub_proc.pid}", flush=True)

    # Monitor both children — daemon=True means they die with the parent, but
    # we still want to log crashes loudly rather than hanging forever on join().
    try:
        while True:
            time.sleep(2)
            if not sub_proc.is_alive():
                print(
                    f"[VizBridge] ERROR: SUB process died (exitcode={sub_proc.exitcode}), shutting down",
                    flush=True,
                )
                stop_event.set()
                pub_proc.terminate()
                break
            if not pub_proc.is_alive():
                print(
                    f"[VizBridge] ERROR: PUB process died (exitcode={pub_proc.exitcode}), shutting down",
                    flush=True,
                )
                stop_event.set()
                sub_proc.terminate()
                break
    except KeyboardInterrupt:
        print("\n[VizBridge] interrupted — shutting down...", flush=True)
        stop_event.set()

    sub_proc.join(timeout=3)
    pub_proc.join(timeout=3)
    if sub_proc.is_alive():
        sub_proc.terminate()
    if pub_proc.is_alive():
        pub_proc.terminate()
    print("[VizBridge] done", flush=True)


if __name__ == "__main__":
    main()
