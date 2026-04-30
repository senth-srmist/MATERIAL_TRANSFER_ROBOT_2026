#!/usr/bin/env python3
"""
Nav2 Navigation Benchmark Script

Measures system metrics during navigation:
- CPU usage (system + per-process)
- Memory usage
- Costmap size
- Tile switches (if tile-based)

Usage:
    # Whole map benchmark
    python3 nav_benchmark.py --mode whole --output whole_map_results.json --duration 120

    # Tile-based benchmark
    python3 nav_benchmark.py --mode tiles --output tile_results.json --duration 120
"""

import argparse
import json
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid

try:
    import psutil

    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False
    print("Warning: psutil not installed. Install with: pip install psutil")


class NavBenchmark(Node):
    def __init__(self, mode="tiles", duration=120):
        super().__init__("nav_benchmark")

        self.mode = mode
        self.duration = duration
        self.start_time = None
        self.samples = []
        self.tile_switches = []
        self.current_map_origin = None
        self.current_costmap_cells = 0
        self.running = False
        self._last_log_bucket = -1  # tracks 10s log intervals

        # Track Nav2 processes
        self.nav2_processes = [
            "bt_navigator",
            "controller_server",
            "planner_server",
            "map_server",
            "lifecycle_manager",
        ]

        # QoS for map topics
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Subscribe to map (to detect tile switches)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )

        # Subscribe to costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.costmap_callback, 10
        )

        # Sampling timer (1 Hz)
        self.sample_timer = self.create_timer(1.0, self.sample_callback)

        self.get_logger().info(
            f"NavBenchmark initialized - Mode: {mode}, Duration: {duration}s"
        )

    def map_callback(self, msg):
        """Detect tile switches by monitoring map origin changes."""
        if not self.running:
            self.current_map_origin = (
                round(msg.info.origin.position.x, 2),
                round(msg.info.origin.position.y, 2),
            )
            return

        new_origin = (
            round(msg.info.origin.position.x, 2),
            round(msg.info.origin.position.y, 2),
        )

        if (
            self.current_map_origin is not None
            and new_origin != self.current_map_origin
        ):
            elapsed = time.time() - self.start_time
            self.tile_switches.append(
                {
                    "timestamp": elapsed,
                    "from_origin": self.current_map_origin,
                    "to_origin": new_origin,
                    "map_width": msg.info.width,
                    "map_height": msg.info.height,
                }
            )
            self.get_logger().info(
                f"Tile switch detected: {self.current_map_origin} -> {new_origin}"
            )

        self.current_map_origin = new_origin

    def costmap_callback(self, msg):
        """Track costmap size."""
        self.current_costmap_cells = msg.info.width * msg.info.height

    def get_process_metrics(self):
        """Get CPU/memory for Nav2 processes."""
        metrics = {}

        if not HAS_PSUTIL:
            return metrics

        for proc in psutil.process_iter(["pid", "name", "cpu_percent", "memory_info"]):
            try:
                name = proc.info["name"]
                for nav_proc in self.nav2_processes:
                    if nav_proc in name.lower():
                        metrics[nav_proc] = {
                            "cpu_percent": proc.info["cpu_percent"],
                            "memory_mb": proc.info["memory_info"].rss / (1024 * 1024)
                            if proc.info["memory_info"]
                            else 0,
                        }
                        break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        return metrics

    def sample_callback(self):
        """Collect metrics sample."""
        if not self.running:
            return

        elapsed = time.time() - self.start_time

        if elapsed >= self.duration:
            self.running = False
            self.get_logger().info("Benchmark duration reached")
            return

        sample = {
            "timestamp": round(elapsed, 2),
            "costmap_cells": self.current_costmap_cells,
            "map_origin": self.current_map_origin,
        }

        # System metrics
        if HAS_PSUTIL:
            sample["system"] = {
                "cpu_percent": psutil.cpu_percent(),
                "memory_mb": psutil.virtual_memory().used / (1024 * 1024),
                "memory_percent": psutil.virtual_memory().percent,
            }
            sample["processes"] = self.get_process_metrics()

        self.samples.append(sample)

        # Log progress once every 10 seconds
        current_bucket = int(elapsed) // 10
        if current_bucket != self._last_log_bucket:
            self._last_log_bucket = current_bucket
            self.get_logger().info(
                f"[{elapsed:.0f}s] CPU: {sample.get('system', {}).get('cpu_percent', 'N/A')}%, "
                f"Costmap: {self.current_costmap_cells:,} cells, "
                f"Tile switches: {len(self.tile_switches)}"
            )

    def start(self):
        """Start benchmark."""
        self.start_time = time.time()
        self.running = True
        self.samples = []
        self.tile_switches = []
        self.get_logger().info(
            f"Benchmark started - {self.mode} mode for {self.duration}s"
        )

    def get_results(self):
        """Compile and return results."""
        if not self.samples:
            return {}

        # Calculate statistics
        cpu_values = [s["system"]["cpu_percent"] for s in self.samples if "system" in s]
        mem_values = [s["system"]["memory_mb"] for s in self.samples if "system" in s]
        costmap_values = [
            s["costmap_cells"] for s in self.samples if s["costmap_cells"] > 0
        ]

        results = {
            "metadata": {
                "mode": self.mode,
                "duration_seconds": self.duration,
                "start_time": datetime.now().isoformat(),
                "sample_count": len(self.samples),
            },
            "summary": {
                "cpu": {
                    "mean": sum(cpu_values) / len(cpu_values) if cpu_values else 0,
                    "max": max(cpu_values) if cpu_values else 0,
                    "min": min(cpu_values) if cpu_values else 0,
                },
                "memory_mb": {
                    "mean": sum(mem_values) / len(mem_values) if mem_values else 0,
                    "max": max(mem_values) if mem_values else 0,
                    "min": min(mem_values) if mem_values else 0,
                },
                "costmap_cells": {
                    "mean": sum(costmap_values) / len(costmap_values)
                    if costmap_values
                    else 0,
                    "max": max(costmap_values) if costmap_values else 0,
                    "min": min(costmap_values) if costmap_values else 0,
                },
                "tile_switches": len(self.tile_switches),
            },
            "samples": self.samples,
            "tile_switches": self.tile_switches,
        }

        return results


def main():
    parser = argparse.ArgumentParser(description="Nav2 Navigation Benchmark")
    parser.add_argument(
        "--mode",
        choices=["whole", "tiles"],
        default="tiles",
        help="Benchmark mode: whole (single map) or tiles (tile switching)",
    )
    parser.add_argument(
        "--output", "-o", default="benchmark_results.json", help="Output JSON file"
    )
    parser.add_argument(
        "--duration", "-d", type=int, default=120, help="Benchmark duration in seconds"
    )
    args, remaining_args = parser.parse_known_args()

    rclpy.init(args=remaining_args)
    node = NavBenchmark(mode=args.mode, duration=args.duration)

    # Start benchmark
    node.start()

    print(f"\n{'=' * 60}")
    print(f"NAV2 BENCHMARK - {args.mode.upper()} MODE")
    print(f"{'=' * 60}")
    print(f"Duration: {args.duration} seconds")
    print(f"Output: {args.output}")
    print(f"\nNavigate the robot around to collect metrics...")
    print(f"Press Ctrl+C to stop early\n")

    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nBenchmark interrupted")

    # Get results
    results = node.get_results()

    # Save results
    with open(args.output, "w") as f:
        json.dump(results, f, indent=2)

    # Print summary
    print(f"\n{'=' * 60}")
    print("BENCHMARK RESULTS")
    print(f"{'=' * 60}")
    summary = results.get("summary", {})
    print(f"Mode:           {args.mode}")
    print(f"Samples:        {results.get('metadata', {}).get('sample_count', 0)}")
    print(f"CPU Mean:       {summary.get('cpu', {}).get('mean', 0):.1f}%")
    print(f"CPU Max:        {summary.get('cpu', {}).get('max', 0):.1f}%")
    print(f"Memory Mean:    {summary.get('memory_mb', {}).get('mean', 0):.1f} MB")
    print(
        f"Costmap Mean:   {summary.get('costmap_cells', {}).get('mean', 0):,.0f} cells"
    )
    print(f"Tile Switches:  {summary.get('tile_switches', 0)}")
    print(f"\nResults saved to: {args.output}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
