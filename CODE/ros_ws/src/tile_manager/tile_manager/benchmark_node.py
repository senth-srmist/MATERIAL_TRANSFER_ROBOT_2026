#!/usr/bin/env python3
"""
Benchmark Node (Fixed)

Collects performance metrics for tile switching research:
    - Memory usage (map_server, tile_switcher)
    - CPU usage (map_server, tile_switcher)
    - Map switch times
    - Costmap update rate
    - Disk I/O

FIX: Properly identifies only the actual ROS2 node processes,
     not ros2 run wrappers or spawn processes.

Usage:
    ros2 run tile_manager benchmark_node
    ros2 run tile_manager benchmark_node --ros-args -p experiment_name:=tiled
"""

import os
import time
import csv
from datetime import datetime

import psutil
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


class BenchmarkNode(Node):

    def __init__(self):
        super().__init__("benchmark_node")

        # ---------------- PARAMETERS ----------------
        default_output = os.path.join(os.path.expanduser('~'), 'benchmark_results')
        self.declare_parameter('output_dir', default_output)
        self.declare_parameter('sample_rate', 10.0)
        self.declare_parameter('experiment_name', 'tiled')

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        sample_rate = self.get_parameter('sample_rate').get_parameter_value().double_value
        self.experiment_name = self.get_parameter('experiment_name').get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)

        # ---------------- STATE ----------------
        self.start_time = time.time()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_tile = 1
        self.tile_switches = []

        # Costmap tracking
        self.costmap_update_count = 0
        self.costmap_full_count = 0
        self.last_costmap_time = time.time()
        self.costmap_update_rate = 0.0

        # Disk I/O baseline
        self.disk_io_baseline = psutil.disk_io_counters()
        self.last_disk_io = self.disk_io_baseline

        # Process handles - now stores single process per target
        self.monitored_processes = {}  # name -> (pid, process)

        # ---------------- DATA STORAGE ----------------
        self.metrics_data = []

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

        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._on_costmap_full,
            10
        )
        self.create_subscription(
            OccupancyGridUpdate,
            '/global_costmap/costmap_updates',
            self._on_costmap_update,
            10
        )

        # ---------------- TIMER ----------------
        self.create_timer(1.0 / sample_rate, self._collect_metrics)
        
        # Periodically rediscover processes (in case they restart)
        self.create_timer(5.0, self._discover_processes)

        # ---------------- FIND PROCESSES ----------------
        self._discover_processes()

        self.get_logger().info("=" * 50)
        self.get_logger().info("Benchmark Node Started (Fixed)")
        self.get_logger().info(f"  Experiment: {self.experiment_name}")
        self.get_logger().info(f"  Output: {self.csv_filename}")
        self.get_logger().info(f"  Sample rate: {sample_rate} Hz")
        self.get_logger().info("=" * 50)

    # ================== PROCESS DISCOVERY (FIXED) ==================
    def _discover_processes(self):
        """
        Find the ACTUAL map_server and tile_switcher processes.
        
        Excludes:
        - ros2 run wrappers
        - spawn processes  
        - Any process where the target is just an argument, not the script
        """
        # Target patterns: (name, must_contain, must_not_contain)
        targets = [
            ('map_server', 'map_server', ['ros2 run', 'ros2 launch', 'spawn']),
            ('tile_switcher', 'tile_switcher', ['ros2 run', 'ros2 launch', 'spawn']),
        ]
        
        for target_name, must_contain, exclude_patterns in targets:
            # Skip if already found and still running
            if target_name in self.monitored_processes:
                pid, proc = self.monitored_processes[target_name]
                try:
                    if proc.is_running():
                        continue
                except psutil.NoSuchProcess:
                    pass
                # Process died, remove it
                del self.monitored_processes[target_name]
            
            # Search for the process
            best_match = None
            best_match_score = -1
            
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = proc.info['cmdline']
                    if not cmdline:
                        continue
                    
                    cmdline_str = ' '.join(cmdline)
                    
                    # Must contain the target
                    if must_contain not in cmdline_str:
                        continue
                    
                    # Must not contain exclusion patterns
                    excluded = False
                    for pattern in exclude_patterns:
                        if pattern in cmdline_str:
                            excluded = True
                            break
                    if excluded:
                        continue
                    
                    # Score: prefer processes where target is in the executable/script name
                    # (last argument that looks like a path or module)
                    score = 0
                    
                    # Check if it's a Python script with the target name
                    for arg in cmdline:
                        if must_contain in arg and (arg.endswith('.py') or '/' in arg):
                            score = 10
                            break
                    
                    # Check if process name contains target
                    if must_contain in proc.info['name']:
                        score += 5
                    
                    # Prefer processes with fewer arguments (more likely to be the actual node)
                    score += max(0, 10 - len(cmdline))
                    
                    if score > best_match_score:
                        best_match = proc
                        best_match_score = score
                        
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue
            
            if best_match:
                self.monitored_processes[target_name] = (best_match.pid, best_match)
                try:
                    cmdline_short = ' '.join(best_match.cmdline())[:80]
                except:
                    cmdline_short = "N/A"
                self.get_logger().info(
                    f"  Found {target_name}: PID {best_match.pid} ({cmdline_short}...)"
                )

    # ================== CALLBACKS ==================
    def _on_pose(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def _on_tile_change(self, msg: Int32):
        self.current_tile = msg.data

    def _on_switch_time(self, msg: Float32):
        self.tile_switches.append({
            'timestamp': time.time() - self.start_time,
            'switch_time_ms': msg.data,
            'tile': self.current_tile
        })
        self.get_logger().info(f"Tile switch recorded: {msg.data:.2f} ms")

    def _on_costmap_full(self, msg: OccupancyGrid):
        self.costmap_full_count += 1

    def _on_costmap_update(self, msg: OccupancyGridUpdate):
        self.costmap_update_count += 1
        now = time.time()
        dt = now - self.last_costmap_time
        if dt > 0:
            alpha = 0.3
            instant_rate = 1.0 / dt
            self.costmap_update_rate = alpha * instant_rate + (1 - alpha) * self.costmap_update_rate
        self.last_costmap_time = now

    # ================== METRICS COLLECTION ==================
    def _collect_metrics(self):
        elapsed = time.time() - self.start_time

        # Process metrics
        map_server_mem = 0.0
        map_server_cpu = 0.0
        tile_switcher_mem = 0.0
        tile_switcher_cpu = 0.0

        for name, (pid, proc) in list(self.monitored_processes.items()):
            try:
                mem_mb = proc.memory_info().rss / (1024 * 1024)
                cpu = proc.cpu_percent(interval=None)
                
                if name == 'map_server':
                    map_server_mem = mem_mb
                    map_server_cpu = cpu
                elif name == 'tile_switcher':
                    tile_switcher_mem = mem_mb
                    tile_switcher_cpu = cpu
                    
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                del self.monitored_processes[name]

        # Total is now ONLY the two processes we care about
        total_mem = map_server_mem + tile_switcher_mem
        total_cpu = map_server_cpu + tile_switcher_cpu

        # Disk I/O
        current_disk_io = psutil.disk_io_counters()
        disk_read_mb = (current_disk_io.read_bytes - self.last_disk_io.read_bytes) / (1024 * 1024)
        disk_write_mb = (current_disk_io.write_bytes - self.last_disk_io.write_bytes) / (1024 * 1024)
        self.last_disk_io = current_disk_io

        total_disk_read_mb = (current_disk_io.read_bytes - self.disk_io_baseline.read_bytes) / (1024 * 1024)
        total_disk_write_mb = (current_disk_io.write_bytes - self.disk_io_baseline.write_bytes) / (1024 * 1024)

        # Build record
        record = {
            'timestamp': elapsed,
            'x': self.current_x,
            'y': self.current_y,
            'tile': self.current_tile,
            'map_server_mem_mb': map_server_mem,
            'map_server_cpu': map_server_cpu,
            'tile_switcher_mem_mb': tile_switcher_mem,
            'tile_switcher_cpu': tile_switcher_cpu,
            'total_mem_mb': total_mem,
            'total_cpu': total_cpu,
            'costmap_update_rate_hz': self.costmap_update_rate,
            'costmap_update_count': self.costmap_update_count,
            'costmap_full_count': self.costmap_full_count,
            'disk_read_mb': disk_read_mb,
            'disk_write_mb': disk_write_mb,
            'total_disk_read_mb': total_disk_read_mb,
            'total_disk_write_mb': total_disk_write_mb,
        }

        self.metrics_data.append(record)

        # Log every 5 seconds
        if int(elapsed) % 5 == 0 and elapsed > 0:
            self.get_logger().info(
                f"[{elapsed:.1f}s] map_server: {map_server_mem:.1f}MB | "
                f"tile_switcher: {tile_switcher_mem:.1f}MB | "
                f"Total: {total_mem:.1f}MB | Tile: {self.current_tile}"
            )

    # ================== SAVE DATA ==================
    def save_results(self):
        if not self.metrics_data:
            self.get_logger().warn("No metrics data collected!")
            return

        # Fixed column order
        fieldnames = [
            'timestamp', 'x', 'y', 'tile',
            'map_server_mem_mb', 'map_server_cpu',
            'tile_switcher_mem_mb', 'tile_switcher_cpu',
            'total_mem_mb', 'total_cpu',
            'costmap_update_rate_hz', 'costmap_update_count', 'costmap_full_count',
            'disk_read_mb', 'disk_write_mb', 'total_disk_read_mb', 'total_disk_write_mb',
        ]
        
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.metrics_data)

        self.get_logger().info(f"Saved {len(self.metrics_data)} records to {self.csv_filename}")

        if self.tile_switches:
            with open(self.switch_csv_filename, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['timestamp', 'switch_time_ms', 'tile'])
                writer.writeheader()
                writer.writerows(self.tile_switches)
            self.get_logger().info(f"Saved {len(self.tile_switches)} switches to {self.switch_csv_filename}")

        self._print_summary()

    def _print_summary(self):
        if not self.metrics_data:
            return

        self.get_logger().info("=" * 60)
        self.get_logger().info("BENCHMARK SUMMARY")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Experiment: {self.experiment_name}")
        self.get_logger().info(f"Duration: {self.metrics_data[-1]['timestamp']:.1f} seconds")
        self.get_logger().info(f"Samples: {len(self.metrics_data)}")

        # Map server
        ms_mem = [r['map_server_mem_mb'] for r in self.metrics_data if r['map_server_mem_mb'] > 0]
        if ms_mem:
            self.get_logger().info("-" * 60)
            self.get_logger().info("MAP SERVER:")
            self.get_logger().info(f"  Memory: {min(ms_mem):.2f} / {max(ms_mem):.2f} / {sum(ms_mem)/len(ms_mem):.2f} MB (min/max/avg)")

        # Tile switcher
        ts_mem = [r['tile_switcher_mem_mb'] for r in self.metrics_data if r['tile_switcher_mem_mb'] > 0]
        if ts_mem:
            self.get_logger().info("-" * 60)
            self.get_logger().info("TILE SWITCHER:")
            self.get_logger().info(f"  Memory: {min(ts_mem):.2f} / {max(ts_mem):.2f} / {sum(ts_mem)/len(ts_mem):.2f} MB (min/max/avg)")

        # Total
        total_mem = [r['total_mem_mb'] for r in self.metrics_data if r['total_mem_mb'] > 0]
        if total_mem:
            self.get_logger().info("-" * 60)
            self.get_logger().info("TOTAL (map_server + tile_switcher only):")
            self.get_logger().info(f"  Memory: {min(total_mem):.2f} / {max(total_mem):.2f} / {sum(total_mem)/len(total_mem):.2f} MB (min/max/avg)")

        # Tile switches
        if self.tile_switches:
            switch_times = [s['switch_time_ms'] for s in self.tile_switches]
            self.get_logger().info("-" * 60)
            self.get_logger().info("TILE SWITCHES:")
            self.get_logger().info(f"  Count: {len(switch_times)}")
            self.get_logger().info(f"  Time: {min(switch_times):.2f} / {max(switch_times):.2f} / {sum(switch_times)/len(switch_times):.2f} ms (min/max/avg)")

        self.get_logger().info("=" * 60)


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
