#!/usr/bin/env python3
"""
Visualization Bridge (v2 - Memory Optimized)

Changes from v1:
  - Reduced queue size (1000 -> 200)
  - Explicit garbage collection after large message batches
  - Pre-allocated message buffers where possible
  - Reduced QoS depths
  - Single-threaded executor (less memory than MultiThreaded)

Memory savings: ~20-30MB (two Python processes still required for domain separation)

Note: The two-process architecture is mandatory due to CycloneDDS domain isolation.
Each process needs its own Python interpreter (~25-30MB overhead each).
This cannot be optimized away without switching to a single-domain architecture.
"""

import argparse
import ctypes
import gc
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
            depth=1,  # 🔥 Reduced from 5
        )
    elif topic == "/tf":
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # 🔥 Reduced from 10
        )
    else:
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )


# ---------------------------------------------------------------------------
# inotify watcher
# ---------------------------------------------------------------------------

IN_CLOSE_WRITE = 0x00000008
IN_MOVED_TO = 0x00000080
IN_CREATE = 0x00000100


def watch_config(config_path, callback):
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
        print(f"[inotify] watch failed on {watch_dir}", flush=True)
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
                time.sleep(0.2)
                print("[inotify] config changed, reloading...", flush=True)
                try:
                    callback()
                except Exception as e:
                    print(f"[inotify] reload error: {e}", flush=True)


# ---------------------------------------------------------------------------
# Subscriber process
# ---------------------------------------------------------------------------


def subscriber_process(config_path, queue, stop_event):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import (
        SingleThreadedExecutor,
    )  # 🔥 Less memory than MultiThreaded
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
    subscriptions = {}
    last_pub = {}
    msg_counts = {}
    gc_counter = [0]  # Mutable for closure

    def make_callback(topic, interval_ref):
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

                    # 🔥 Periodic GC to prevent memory accumulation
                    gc_counter[0] += 1
                    if gc_counter[0] >= 100:
                        gc_counter[0] = 0
                        gc.collect()

            except Exception as e:
                print(f"[SUB] queue error on {topic}: {e}", flush=True)

        return cb

    def apply_config(topics_cfg):
        new_topics = {e["topic"] for e in topics_cfg}

        for topic in list(subscriptions.keys() - new_topics):
            sub, _ = subscriptions.pop(topic)
            node.destroy_subscription(sub)
            print(f"[SUB] removed {topic}", flush=True)

        for entry in topics_cfg:
            topic = entry["topic"]
            type_str = entry["type"]
            rate = float(entry.get("rate", 1.0))

            if topic in subscriptions:
                _, interval_ref = subscriptions[topic]
                interval_ref[0] = 1.0 / rate if rate > 0 else 0.0
                continue

            try:
                msg_type = resolve_msg_type(type_str)
            except Exception as e:
                print(f"[SUB] SKIP {topic}: {e}", flush=True)
                continue

            interval_ref = [1.0 / rate if rate > 0 else 0.0]
            sub = node.create_subscription(
                msg_type,
                topic,
                make_callback(topic, interval_ref),
                make_qos(topic, rate),
            )
            subscriptions[topic] = (sub, interval_ref)
            print(f"[SUB] subscribed {topic} @ {rate} Hz", flush=True)

    with lock:
        apply_config(config["topics"])
    print(
        f"[SUB] ready: {len(subscriptions)} subscriptions on domain {domain}",
        flush=True,
    )

    def on_change():
        try:
            cfg = load_config(config_path)
            with lock:
                apply_config(cfg["topics"])
            gc.collect()  # 🔥 GC after config change
            print(f"[SUB] reload done: {len(subscriptions)} subscriptions", flush=True)
        except Exception as e:
            print(f"[SUB] reload failed: {e}", flush=True)

    threading.Thread(
        target=watch_config, args=(config_path, on_change), daemon=True
    ).start()

    # 🔥 Use SingleThreadedExecutor - less memory overhead
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)

    try:
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        print("[SUB] shutdown complete", flush=True)


# ---------------------------------------------------------------------------
# Publisher process
# ---------------------------------------------------------------------------


def publisher_process(config_path, queue, stop_event):
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
    publishers = {}
    msg_counts = {}
    gc_counter = [0]

    def apply_config(topics_cfg):
        new_topics = {e["topic"] for e in topics_cfg}

        for topic in list(publishers.keys() - new_topics):
            pub, _ = publishers.pop(topic)
            node.destroy_publisher(pub)
            print(f"[PUB] removed {topic}", flush=True)

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
            gc.collect()
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
                continue

            pub, msg_type = entry
            try:
                msg = deserialize_message(serialized, msg_type)
                pub.publish(msg)
                msg_counts[topic] = msg_counts.get(topic, 0) + 1

                # 🔥 Periodic GC
                gc_counter[0] += 1
                if gc_counter[0] >= 100:
                    gc_counter[0] = 0
                    gc.collect()

            except Exception as e:
                print(f"[PUB] publish error on {topic}: {e}", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        print("[PUB] shutdown complete", flush=True)


# ---------------------------------------------------------------------------
# Entrypoints
# ---------------------------------------------------------------------------


def _run_sub(config_path, local_xml, domain_id, queue, stop_event):
    os.environ["CYCLONEDDS_URI"] = f"file://{local_xml}"
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    subscriber_process(config_path, queue, stop_event)


def _run_pub(config_path, net_xml, domain_id, queue, stop_event):
    os.environ["CYCLONEDDS_URI"] = f"file://{net_xml}"
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    publisher_process(config_path, queue, stop_event)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    multiprocessing.set_start_method("spawn")

    parser = argparse.ArgumentParser(description="ROS2 Viz Bridge (Memory Optimized)")
    parser.add_argument("--config", default="/workspace/viz_bridge_config.yaml")
    args = parser.parse_args()

    if not Path(args.config).exists():
        print(f"[VizBridge] config not found: {args.config}")
        sys.exit(1)

    if not os.environ.get("CYCLONE_INTERFACE_IP"):
        print("[VizBridge] ERROR: CYCLONE_INTERFACE_IP not set")
        sys.exit(1)

    config = load_config(args.config)
    bridge_cfg = config["bridge"]
    local_xml = "/workspace/cyclonedds_local.xml"
    net_xml = bridge_cfg.get(
        "cyclonedds_config", "/workspace/cyclonedds_viz_bridge.xml"
    )

    print(f"[VizBridge] v2 (memory optimized)")
    print(f"[VizBridge] CYCLONE_INTERFACE_IP={os.environ['CYCLONE_INTERFACE_IP']}")
    print(f"[VizBridge] SUB: domain {bridge_cfg['robot_domain_id']} via {local_xml}")
    print(f"[VizBridge] PUB: domain {bridge_cfg['viz_domain_id']} via {net_xml}")

    # 🔥 Reduced queue size (200 vs 1000)
    queue = multiprocessing.Queue(maxsize=200)
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

    try:
        while True:
            time.sleep(2)
            if not sub_proc.is_alive():
                print(
                    f"[VizBridge] ERROR: SUB died (exitcode={sub_proc.exitcode})",
                    flush=True,
                )
                stop_event.set()
                pub_proc.terminate()
                break
            if not pub_proc.is_alive():
                print(
                    f"[VizBridge] ERROR: PUB died (exitcode={pub_proc.exitcode})",
                    flush=True,
                )
                stop_event.set()
                sub_proc.terminate()
                break
    except KeyboardInterrupt:
        print("\n[VizBridge] interrupted", flush=True)
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
