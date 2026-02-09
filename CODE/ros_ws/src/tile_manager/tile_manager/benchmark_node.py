#!/usr/bin/env python3

"""
Benchmark Node

Collects performance metrics for research comparison:
    - Memory usage (RSS, VMS)
    - CPU usage
    - Map switch times (via subscription)
    - Position tracking

Outputs:
    - CSV file with time-series data
    - Summary statistics on shutdown

Usage:
    ros2 run tile_manager benchmark_node
    ros2 run tile_manager benchmark_node --ros-args -p output_dir:=/path/to/results
"""

import os
import time
import csv
from datetime import datetime

import psutil
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32, String


class BenchmarkNode(Node):

    def __init__(self):
        super().__init__("benchmark_node")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('output_dir', '/tmp/benchmark_results')
        self.declare_parameter('sample_rate', 10.0)  # Hz
        self.declare_parameter('experiment_name', 'tiled')  # 'tiled' or 'whole'

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        sample_rate = self.get_parameter('sample_rate').get_parameter_value().double_value
        self.experiment_name = self.get_parameter('experiment_name').get_parameter_value().string_value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # ---------------- STATE ----------------
        self.start_time = time.time()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_tile = 1
        self.tile_switches = []  # List of (timestamp, from_tile, to_tile, switch_time_ms)

        # Process handles for monitoring
        self.own_process = psutil.Process()
        self.monitored_processes = {}  # pid -> (name, process)

        # ---------------- DATA STORAGE ----------------
        self.metrics_data = []  # List of dicts for CSV

        # ---------------- CSV SETUP ----------------
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(
            self.output_dir, 
            f"benchmark_{self.experiment_name}_{timestamp_str}.csv"
        )
        self.switch_csv_filename = os.path.join(
            self.output_dir,
            f"tile_switches_{self.experiment_name}_{timestamp_str}.csv"
        )

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(PoseStamped, '/robot_pose', self._on_pose, 10)
        self.create_subscription(Int32, '/benchmark/current_tile', self._on_tile_change, 10)
        self.create_subscription(Float32, '/benchmark/switch_time_ms', self._on_switch_time, 10)

        # ---------------- TIMER ----------------
        self.create_timer(1.0 / sample_rate, self._collect_metrics)

        # ---------------- FIND PROCESSES TO MONITOR ----------------
        self._discover_processes()

        self.get_logger().info("=" * 50)
        self.get_logger().info("Benchmark Node Started")
        self.get_logger().info(f"  Experiment: {self.experiment_name}")
        self.get_logger().info(f"  Output: {self.csv_filename}")
        self.get_logger().info(f"  Sample rate: {sample_rate} Hz")
        self.get_logger().info(f"  Monitoring {len(self.monitored_processes)} processes")
        self.get_logger().info("=" * 50)

    # ================== PROCESS DISCOVERY ==================
    def _discover_processes(self):
        """Find ROS2 processes to monitor"""
        target_names = [
            'map_server',
            'pose_monitor',
            'tile_switcher',
            'controller_server',
            'planner_server',
            'bt_navigator',
        ]

        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                for target in target_names:
                    if target in cmdline:
                        self.monitored_processes[proc.pid] = (target, proc)
                        self.get_logger().info(f"  Monitoring: {target} (PID {proc.pid})")
                        break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

    # ================== CALLBACKS ==================
    def _on_pose(self, msg: PoseStamped):
        """Update current position"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def _on_tile_change(self, msg: Int32):
        """Record tile change"""
        self.current_tile = msg.data

    def _on_switch_time(self, msg: Float32):
        """Record tile switch time"""
        self.tile_switches.append({
            'timestamp': time.time() - self.start_time,
            'switch_time_ms': msg.data,
            'tile': self.current_tile
        })
        self.get_logger().info(f"Tile switch recorded: {msg.data:.2f} ms")

    # ================== METRICS COLLECTION ==================
    def _collect_metrics(self):
        """Collect metrics at regular intervals"""
        elapsed = time.time() - self.start_time

        # Collect memory and CPU for all monitored processes
        total_memory_mb = 0.0
        total_cpu = 0.0
        process_metrics = {}

        for pid, (name, proc) in list(self.monitored_processes.items()):
            try:
                mem_info = proc.memory_info()
                cpu = proc.cpu_percent(interval=None)
                mem_mb = mem_info.rss / (1024 * 1024)
                
                total_memory_mb += mem_mb
                total_cpu += cpu
                process_metrics[f'{name}_mem_mb'] = mem_mb
                process_metrics[f'{name}_cpu'] = cpu
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                # Process died, remove from monitoring
                del self.monitored_processes[pid]

        # System-wide metrics
        system_mem = psutil.virtual_memory()
        system_cpu = psutil.cpu_percent(interval=None)

        # Build metrics record
        record = {
            'timestamp': elapsed,
            'x': self.current_x,
            'y': self.current_y,
            'tile': self.current_tile,
            'total_mem_mb': total_memory_mb,
            'total_cpu': total_cpu,
            'system_mem_percent': system_mem.percent,
            'system_cpu': system_cpu,
            **process_metrics
        }

        self.metrics_data.append(record)

        # Log periodically (every 5 seconds)
        if int(elapsed) % 5 == 0 and elapsed > 0:
            self.get_logger().info(
                f"[{elapsed:.1f}s] Mem: {total_memory_mb:.1f} MB | "
                f"CPU: {total_cpu:.1f}% | Tile: {self.current_tile}"
            )

    # ================== SAVE DATA ==================
    def save_results(self):
        """Save collected data to CSV files"""
        if not self.metrics_data:
            self.get_logger().warn("No metrics data collected!")
            return

        # Save time-series metrics
        fieldnames = list(self.metrics_data[0].keys())
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.metrics_data)
        
        self.get_logger().info(f"Saved {len(self.metrics_data)} records to {self.csv_filename}")

        # Save tile switch data
        if self.tile_switches:
            with open(self.switch_csv_filename, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['timestamp', 'switch_time_ms', 'tile'])
                writer.writeheader()
                writer.writerows(self.tile_switches)
            
            self.get_logger().info(f"Saved {len(self.tile_switches)} tile switches to {self.switch_csv_filename}")

        # Print summary
        self._print_summary()

    def _print_summary(self):
        """Print summary statistics"""
        if not self.metrics_data:
            return

        mem_values = [r['total_mem_mb'] for r in self.metrics_data]
        cpu_values = [r['total_cpu'] for r in self.metrics_data]

        self.get_logger().info("=" * 50)
        self.get_logger().info("BENCHMARK SUMMARY")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Experiment: {self.experiment_name}")
        self.get_logger().info(f"Duration: {self.metrics_data[-1]['timestamp']:.1f} seconds")
        self.get_logger().info(f"Samples: {len(self.metrics_data)}")
        self.get_logger().info("-" * 50)
        self.get_logger().info("MEMORY (MB):")
        self.get_logger().info(f"  Min:  {min(mem_values):.2f}")
        self.get_logger().info(f"  Max:  {max(mem_values):.2f}")
        self.get_logger().info(f"  Avg:  {sum(mem_values)/len(mem_values):.2f}")
        self.get_logger().info("-" * 50)
        self.get_logger().info("CPU (%):")
        self.get_logger().info(f"  Min:  {min(cpu_values):.2f}")
        self.get_logger().info(f"  Max:  {max(cpu_values):.2f}")
        self.get_logger().info(f"  Avg:  {sum(cpu_values)/len(cpu_values):.2f}")
        
        if self.tile_switches:
            switch_times = [s['switch_time_ms'] for s in self.tile_switches]
            self.get_logger().info("-" * 50)
            self.get_logger().info("TILE SWITCHES:")
            self.get_logger().info(f"  Count: {len(switch_times)}")
            self.get_logger().info(f"  Min:   {min(switch_times):.2f} ms")
            self.get_logger().info(f"  Max:   {max(switch_times):.2f} ms")
            self.get_logger().info(f"  Avg:   {sum(switch_times)/len(switch_times):.2f} ms")
        
        self.get_logger().info("=" * 50)


def main():
    rclpy.init()
    node = BenchmarkNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
