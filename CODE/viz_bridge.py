#!/usr/bin/env python3
"""
Visualization Bridge (v3 - Ultra Lightweight)

Optimizations for 4GB Jetson:
  - BEST_EFFORT QoS everywhere (drop frames, don't buffer)
  - Queue depth = 1 (only latest message)
  - Minimal queue size (10 messages max)
  - No message caching
  - Aggressive garbage collection
  - Direct publish without intermediate storage

This version prioritizes stability over frame delivery.
Some frames WILL be dropped - that's intentional.
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


def resolve_msg_type(type_string):
    parts = type_string.rsplit(".", 1)
    if len(parts) != 2:
        raise ValueError(f"Invalid message type: {type_string}")
    module = importlib.import_module(parts[0])
    return getattr(module, parts[1])


def load_config(config_path):
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def make_qos_best_effort():
    """Ultra-lightweight QoS - drop frames rather than buffer."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,  # Drop if slow
        durability=DurabilityPolicy.VOLATILE,  # No persistence
        history=HistoryPolicy.KEEP_LAST,
        depth=1,  # Only latest
    )


def make_qos_for_topic(topic):
    """Special handling for specific topics."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    # Static topics need TRANSIENT_LOCAL to receive latched data
    if "static" in topic or topic == "/map":
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

    # Everything else: BEST_EFFORT, depth=1
    return make_qos_best_effort()


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
        return

    watch_dir = str(Path(config_path).parent)
    watch_name = Path(config_path).name
    mask = IN_CLOSE_WRITE | IN_MOVED_TO | IN_CREATE

    wd = libc.inotify_add_watch(fd, watch_dir.encode(), ctypes.c_uint32(mask))
    if wd < 0:
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
            wd_ev, mask_ev, cookie, name_len = struct.unpack_from(
                "iIII", raw, offset)
            offset += HDR
            name = b""
            if name_len > 0:
                name = raw[offset:offset + name_len].rstrip(b"\x00")
                offset += name_len
            if name.decode(errors="replace") == watch_name:
                time.sleep(0.2)
                print("[inotify] config changed, reloading...", flush=True)
                try:
                    callback()
                except Exception as e:
                    print(f"[inotify] reload error: {e}", flush=True)


# ---------------------------------------------------------------------------
# Subscriber process - LIGHTWEIGHT
# ---------------------------------------------------------------------------


def subscriber_process(config_path, queue, stop_event):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.serialization import serialize_message

    config = load_config(config_path)
    bridge_cfg = config["bridge"]
    domain = bridge_cfg["robot_domain_id"]

    print(f"[SUB] domain={domain} (BEST_EFFORT mode)", flush=True)

    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = Node("viz_bridge_sub", context=ctx)

    lock = threading.Lock()
    subscriptions = {}
    last_pub = {}
    dropped_count = {}

    def make_callback(topic, interval_ref):

        def cb(msg):
            now = time.monotonic()

            # Rate limiting
            if interval_ref[0] > 0 and (
                    now - last_pub.get(topic, 0)) < interval_ref[0]:
                return

            # Try to queue - if full, DROP (don't block)
            try:
                serialized = serialize_message(msg)

                # Non-blocking put - if queue full, drop this frame
                try:
                    queue.put_nowait((
                        topic,
                        type(msg).__module__ + "." + type(msg).__qualname__,
                        serialized,
                    ))
                    last_pub[topic] = now
                except:
                    # Queue full - drop frame (this is intentional)
                    dropped_count[topic] = dropped_count.get(topic, 0) + 1
                    if dropped_count[topic] % 50 == 0:
                        print(
                            f"[SUB] dropped {dropped_count[topic]} frames on {topic}",
                            flush=True,
                        )

                # Aggressive cleanup
                del serialized

            except Exception as e:
                print(f"[SUB] error on {topic}: {e}", flush=True)

            # Periodic GC
            gc.collect(generation=0)  # Only young generation - fast

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

            # Use BEST_EFFORT QoS
            qos = make_qos_for_topic(topic)

            sub = node.create_subscription(msg_type, topic,
                                           make_callback(topic, interval_ref),
                                           qos)
            subscriptions[topic] = (sub, interval_ref)
            print(f"[SUB] subscribed {topic} @ {rate} Hz (BEST_EFFORT)",
                  flush=True)

    with lock:
        apply_config(config["topics"])
    print(f"[SUB] ready: {len(subscriptions)} subscriptions", flush=True)

    def on_change():
        try:
            cfg = load_config(config_path)
            with lock:
                apply_config(cfg["topics"])
            gc.collect()
        except Exception as e:
            print(f"[SUB] reload failed: {e}", flush=True)

    threading.Thread(target=watch_config,
                     args=(config_path, on_change),
                     daemon=True).start()

    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)

    try:
        while not stop_event.is_set():
            executor.spin_once(
                timeout_sec=0.05)  # Faster spin for responsiveness
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        print("[SUB] shutdown", flush=True)


# ---------------------------------------------------------------------------
# Publisher process - LIGHTWEIGHT
# ---------------------------------------------------------------------------


def publisher_process(config_path, queue, stop_event):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message

    config = load_config(config_path)
    bridge_cfg = config["bridge"]
    domain = bridge_cfg["viz_domain_id"]

    print(f"[PUB] domain={domain} (BEST_EFFORT mode)", flush=True)

    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = Node("viz_bridge_pub", context=ctx)

    lock = threading.Lock()
    publishers = {}
    pub_count = {}

    def apply_config(topics_cfg):
        new_topics = {e["topic"] for e in topics_cfg}

        for topic in list(publishers.keys() - new_topics):
            pub, _ = publishers.pop(topic)
            node.destroy_publisher(pub)
            print(f"[PUB] removed {topic}", flush=True)

        for entry in topics_cfg:
            topic = entry["topic"]
            type_str = entry["type"]
            if topic in publishers:
                continue
            try:
                msg_type = resolve_msg_type(type_str)
            except Exception as e:
                print(f"[PUB] SKIP {topic}: {e}", flush=True)
                continue

            # Use BEST_EFFORT QoS
            qos = make_qos_for_topic(topic)

            pub = node.create_publisher(msg_type, topic, qos)
            publishers[topic] = (pub, msg_type)
            print(f"[PUB] advertised {topic} (BEST_EFFORT)", flush=True)

    with lock:
        apply_config(config["topics"])
    print(f"[PUB] ready: {len(publishers)} publishers", flush=True)

    def on_change():
        try:
            cfg = load_config(config_path)
            with lock:
                apply_config(cfg["topics"])
            gc.collect()
        except Exception as e:
            print(f"[PUB] reload failed: {e}", flush=True)

    threading.Thread(target=watch_config,
                     args=(config_path, on_change),
                     daemon=True).start()

    gc_counter = 0

    try:
        while not stop_event.is_set():
            try:
                # Short timeout - don't block long
                item = queue.get(timeout=0.05)
            except:
                continue

            topic, type_str, serialized = item

            with lock:
                entry = publishers.get(topic)
            if entry is None:
                del serialized
                continue

            pub, msg_type = entry
            try:
                msg = deserialize_message(serialized, msg_type)
                pub.publish(msg)

                pub_count[topic] = pub_count.get(topic, 0) + 1
                if pub_count[topic] % 30 == 0:
                    print(f"[PUB] {topic}: {pub_count[topic]} msgs sent",
                          flush=True)

                # Immediate cleanup
                del msg
                del serialized

            except Exception as e:
                print(f"[PUB] error on {topic}: {e}", flush=True)

            # Frequent GC
            gc_counter += 1
            if gc_counter >= 10:
                gc_counter = 0
                gc.collect(generation=0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        print("[PUB] shutdown", flush=True)


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

    parser = argparse.ArgumentParser(
        description="ROS2 Viz Bridge (Ultra Lightweight)")
    parser.add_argument("--config",
                        default="/workspace/viz_bridge_config.yaml")
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
    net_xml = bridge_cfg.get("cyclonedds_config",
                             "/workspace/cyclonedds_viz_bridge.xml")

    print(f"[VizBridge] v3 ULTRA LIGHTWEIGHT")
    print(f"[VizBridge] - BEST_EFFORT QoS (frames will drop)")
    print(f"[VizBridge] - Queue size: 10")
    print(f"[VizBridge] - Aggressive GC")

    # 🔥 TINY queue - only 10 messages max
    queue = multiprocessing.Queue(maxsize=10)
    stop_event = multiprocessing.Event()

    sub_proc = multiprocessing.Process(
        target=_run_sub,
        args=(args.config, local_xml, bridge_cfg["robot_domain_id"], queue,
              stop_event),
        name="viz-sub",
        daemon=True,
    )
    pub_proc = multiprocessing.Process(
        target=_run_pub,
        args=(args.config, net_xml, bridge_cfg["viz_domain_id"], queue,
              stop_event),
        name="viz-pub",
        daemon=True,
    )

    sub_proc.start()
    pub_proc.start()
    print(f"[VizBridge] SUB pid={sub_proc.pid}  PUB pid={pub_proc.pid}",
          flush=True)

    try:
        while True:
            time.sleep(2)
            if not sub_proc.is_alive():
                print(f"[VizBridge] SUB died (exitcode={sub_proc.exitcode})",
                      flush=True)
                stop_event.set()
                pub_proc.terminate()
                break
            if not pub_proc.is_alive():
                print(f"[VizBridge] PUB died (exitcode={pub_proc.exitcode})",
                      flush=True)
                stop_event.set()
                sub_proc.terminate()
                break
    except KeyboardInterrupt:
        print("\n[VizBridge] stopping...", flush=True)
        stop_event.set()

    sub_proc.join(timeout=2)
    pub_proc.join(timeout=2)
    if sub_proc.is_alive():
        sub_proc.terminate()
    if pub_proc.is_alive():
        pub_proc.terminate()
    print("[VizBridge] done", flush=True)


if __name__ == "__main__":
    main()
