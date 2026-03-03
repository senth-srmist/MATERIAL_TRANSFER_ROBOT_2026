#!/usr/bin/env python3
"""
Visualization Bridge

Forwards selected ROS2 topics from the robot's local-only DDS (domain 42)
to a network-facing DDS (domain 43) at throttled rates for remote rviz.

Two separate rclpy contexts in one process:
  Context 1 (local):   domain 42, localhost CycloneDDS - subscribes to robot
  Context 2 (network): domain 43, WiFi CycloneDDS - publishes for laptop

Topics and rates defined in viz_bridge_config.yaml.

Usage:
  On robot:  CYCLONE_INTERFACE_IP=192.168.50.1 python3 viz_bridge.py
  On laptop: ROS_DOMAIN_ID=43 CYCLONE_INTERFACE_IP=192.168.50.97 rviz2
"""

import argparse
import importlib
import os
import sys
import time
import threading
from pathlib import Path

import yaml
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor


def resolve_msg_type(type_string):
    parts = type_string.rsplit(".", 1)
    if len(parts) != 2:
        raise ValueError(f"Invalid message type: {type_string}")
    module = importlib.import_module(parts[0])
    return getattr(module, parts[1])


class TopicBridge:
    def __init__(self, topic, msg_type, rate, sub_node, pub_node):
        self.topic = topic
        self.rate = rate
        self.min_interval = (1.0 / rate) if rate > 0 else 0.0
        self.last_pub_time = 0.0
        self.msg_count = 0

        if rate == 0 or "static" in topic or topic == "/map":
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST, depth=5)
        elif topic == "/tf":
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=10)
        else:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.publisher = pub_node.create_publisher(msg_type, topic, qos)
        sub_node.create_subscription(msg_type, topic, self._callback, qos)

    def _callback(self, msg):
        now = time.monotonic()
        if self.min_interval > 0:
            if (now - self.last_pub_time) < self.min_interval:
                return
        self.publisher.publish(msg)
        self.last_pub_time = now
        self.msg_count += 1


class VizBridge:
    def __init__(self, config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        bridge_cfg = config["bridge"]
        topics_cfg = config["topics"]
        robot_domain = bridge_cfg["robot_domain_id"]
        viz_domain = bridge_cfg["viz_domain_id"]
        cyclone_cfg = bridge_cfg["cyclonedds_config"]

        print(f"[VizBridge] Robot domain: {robot_domain} (local)")
        print(f"[VizBridge] Viz domain:   {viz_domain} (network)")
        print(f"[VizBridge] Topics:       {len(topics_cfg)}")

        # Context 1: LOCAL (subscribes to robot topics via localhost)
        self.local_ctx = Context()
        rclpy.init(context=self.local_ctx,
                    args=["--ros-args", "-r", "__node:=viz_bridge_local"],
                    domain_id=robot_domain)
        self.local_node = Node("viz_bridge_local", context=self.local_ctx)

        # Context 2: NETWORK (publishes for laptop rviz)
        original_cyclone = os.environ.get("CYCLONEDDS_URI", "")
        os.environ["CYCLONEDDS_URI"] = f"file://{cyclone_cfg}"

        self.net_ctx = Context()
        rclpy.init(context=self.net_ctx,
                    args=["--ros-args", "-r", "__node:=viz_bridge_net"],
                    domain_id=viz_domain)
        self.net_node = Node("viz_bridge_net", context=self.net_ctx)

        if original_cyclone:
            os.environ["CYCLONEDDS_URI"] = original_cyclone
        else:
            os.environ.pop("CYCLONEDDS_URI", None)

        # Create topic bridges from config
        self.bridges = []
        for entry in topics_cfg:
            topic = entry["topic"]
            type_str = entry["type"]
            rate = float(entry.get("rate", 1.0))
            try:
                msg_type = resolve_msg_type(type_str)
            except Exception as e:
                print(f"[VizBridge] SKIP {topic}: {e}")
                continue

            bridge = TopicBridge(topic=topic, msg_type=msg_type, rate=rate,
                                 sub_node=self.local_node, pub_node=self.net_node)
            rate_str = f"{rate:.1f}Hz" if rate > 0 else "relay"
            print(f"[VizBridge]   {topic} @ {rate_str}")
            self.bridges.append(bridge)

        print(f"[VizBridge] Ready: {len(self.bridges)} topics bridged")

        self.local_executor = SingleThreadedExecutor(context=self.local_ctx)
        self.local_executor.add_node(self.local_node)
        self.net_executor = SingleThreadedExecutor(context=self.net_ctx)
        self.net_executor.add_node(self.net_node)

    def spin(self):
        local_thread = threading.Thread(target=self._spin_safe,
                                         args=(self.local_executor, "local"),
                                         daemon=True)
        local_thread.start()

        stats_thread = threading.Thread(target=self._print_stats, daemon=True)
        stats_thread.start()

        try:
            self._spin_safe(self.net_executor, "network")
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def _spin_safe(self, executor, name):
        try:
            executor.spin()
        except Exception as e:
            print(f"[VizBridge] {name} executor error: {e}")

    def _print_stats(self):
        while True:
            time.sleep(30)
            total = sum(b.msg_count for b in self.bridges)
            active = sum(1 for b in self.bridges if b.msg_count > 0)
            print(f"[VizBridge] Stats: {total} msgs forwarded, "
                  f"{active}/{len(self.bridges)} topics active")

    def shutdown(self):
        print("[VizBridge] Shutting down...")
        try:
            self.local_executor.shutdown()
            self.local_node.destroy_node()
            rclpy.shutdown(context=self.local_ctx)
        except Exception:
            pass
        try:
            self.net_executor.shutdown()
            self.net_node.destroy_node()
            rclpy.shutdown(context=self.net_ctx)
        except Exception:
            pass
        print("[VizBridge] Done")


def main():
    parser = argparse.ArgumentParser(description="ROS2 Viz Bridge")
    parser.add_argument("--config", default="/workspace/viz_bridge_config.yaml",
                        help="Path to bridge config YAML")
    args = parser.parse_args()

    if not Path(args.config).exists():
        print(f"[VizBridge] Config not found: {args.config}")
        sys.exit(1)

    if not os.environ.get("CYCLONE_INTERFACE_IP"):
        print("[VizBridge] ERROR: CYCLONE_INTERFACE_IP not set")
        print("Usage: CYCLONE_INTERFACE_IP=192.168.50.1 python3 viz_bridge.py")
        sys.exit(1)

    bridge = VizBridge(args.config)
    bridge.spin()


if __name__ == "__main__":
    main()
