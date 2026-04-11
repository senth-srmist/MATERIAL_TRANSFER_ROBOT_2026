#!/usr/bin/env python3
"""
System Monitor Node

Publishes system resource metrics for tuning sessions.
Detects memory leaks via rolling linear regression on per-node RSS.

Publishes:
  /metrics/system (Float32MultiArray, 1Hz):
    [0] total_cpu_percent
    [1] total_ram_used_mb
    [2] total_ram_available_mb
    [3] total_ram_percent
    [4] swap_used_mb

  /metrics/node_memory (Float32MultiArray, 1Hz):
    Per monitored node: [rss_mb, cpu_percent, rss_trend_mb_per_min]
    Layout: [node0_rss, node0_cpu, node0_trend, node1_rss, ...]
    Node order matches --monitor-nodes parameter.

  /metrics/memory_alerts (String):
    Published when a node's RSS trend exceeds threshold.
    Format: "LEAK_SUSPECT:node_name:rss_mb:trend_mb_per_min"
"""

import os
import re
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32MultiArray, String


def _read_file(path: str) -> str:
    try:
        with open(path, "r") as f:
            return f.read()
    except (OSError, PermissionError):
        return ""


class ProcessMonitor:
    """Track a single process by name pattern."""

    def __init__(self, name_pattern: str, window_seconds: float = 120.0):
        self.name_pattern = name_pattern
        self.pid = None
        self._pattern = re.compile(name_pattern)

        # Rolling window for leak detection
        self._window_seconds = window_seconds
        self._rss_history = deque()  # (timestamp, rss_mb)

        # Cached /proc reads
        self._prev_utime = 0
        self._prev_stime = 0
        self._prev_wall = 0.0
        self._hz = os.sysconf("SC_CLK_TCK")

    def find_pid(self):
        """Scan /proc for matching process."""
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            cmdline = _read_file(f"/proc/{entry}/cmdline").replace("\x00", " ")
            if self._pattern.search(cmdline):
                self.pid = int(entry)
                return True
        self.pid = None
        return False

    def get_rss_mb(self) -> float:
        if self.pid is None:
            return 0.0
        status = _read_file(f"/proc/{self.pid}/status")
        for line in status.splitlines():
            if line.startswith("VmRSS:"):
                parts = line.split()
                if len(parts) >= 2:
                    return float(parts[1]) / 1024.0  # kB to MB
        # Process may have died
        self.pid = None
        return 0.0

    def get_cpu_percent(self) -> float:
        if self.pid is None:
            return 0.0
        stat = _read_file(f"/proc/{self.pid}/stat")
        if not stat:
            self.pid = None
            return 0.0
        parts = stat.split()
        if len(parts) < 15:
            return 0.0

        try:
            utime = int(parts[13])
            stime = int(parts[14])
        except (ValueError, IndexError):
            return 0.0

        now = time.monotonic()
        dt = now - self._prev_wall
        if dt < 0.1 or self._prev_wall == 0.0:
            self._prev_utime = utime
            self._prev_stime = stime
            self._prev_wall = now
            return 0.0

        ticks = (utime - self._prev_utime) + (stime - self._prev_stime)
        cpu = (ticks / self._hz) / dt * 100.0

        self._prev_utime = utime
        self._prev_stime = stime
        self._prev_wall = now

        return cpu

    def update_trend(self, rss_mb: float) -> float:
        """Add RSS sample and return trend in MB/minute via linear regression."""
        now = time.monotonic()
        self._rss_history.append((now, rss_mb))

        # Prune old samples
        cutoff = now - self._window_seconds
        while self._rss_history and self._rss_history[0][0] < cutoff:
            self._rss_history.popleft()

        # Need at least 10 samples for meaningful regression
        if len(self._rss_history) < 10:
            return 0.0

        # Simple linear regression: slope of rss vs time
        n = len(self._rss_history)
        sum_t = 0.0
        sum_r = 0.0
        sum_tr = 0.0
        sum_tt = 0.0
        t0 = self._rss_history[0][0]

        for t, r in self._rss_history:
            t_offset = t - t0
            sum_t += t_offset
            sum_r += r
            sum_tr += t_offset * r
            sum_tt += t_offset * t_offset

        denom = n * sum_tt - sum_t * sum_t
        if abs(denom) < 1e-12:
            return 0.0

        slope = (n * sum_tr - sum_t * sum_r) / denom  # MB per second
        return slope * 60.0  # MB per minute


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__("system_monitor")

        # Parameters
        self.declare_parameter(
            "monitor_nodes",
            [
                "controller_server",
                "planner_server",
                "pid_controller",
                "bt_navigator",
                "local_costmap",
                "global_costmap",
                "tuning_metrics",
            ],
        )
        self.declare_parameter("leak_threshold_mb_per_min", 0.5)
        self.declare_parameter("publish_rate", 1.0)

        node_names = self.get_parameter("monitor_nodes").value
        self._leak_threshold = self.get_parameter("leak_threshold_mb_per_min").value
        rate = self.get_parameter("publish_rate").value

        # Create process monitors
        self._monitors = {}
        for name in node_names:
            self._monitors[name] = ProcessMonitor(name)

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._system_pub = self.create_publisher(
            Float32MultiArray, "/metrics/system", best_effort_qos
        )
        self._node_mem_pub = self.create_publisher(
            Float32MultiArray, "/metrics/node_memory", best_effort_qos
        )
        self._alert_pub = self.create_publisher(
            String, "/metrics/memory_alerts", reliable_qos
        )

        # Pre-allocate
        self._sys_msg = Float32MultiArray()
        self._sys_msg.data = [0.0] * 5
        self._node_msg = Float32MultiArray()
        self._node_msg.data = [0.0] * (len(node_names) * 3)
        self._alert_msg = String()

        # PID scan timer (find PIDs every 5 seconds)
        self.create_timer(5.0, self._scan_pids)
        # Main publish timer
        self.create_timer(1.0 / rate, self._publish)

        # Initial scan
        self._scan_pids()

        self.get_logger().info(
            f"System monitor started — tracking {len(node_names)} nodes, "
            f"leak threshold: {self._leak_threshold} MB/min"
        )

    def _scan_pids(self):
        for name, monitor in self._monitors.items():
            if monitor.pid is None:
                if monitor.find_pid():
                    self.get_logger().debug(f"Found {name} at PID {monitor.pid}")

    def _get_system_stats(self):
        """Read system-wide CPU and memory from /proc."""
        # Memory from /proc/meminfo
        meminfo = _read_file("/proc/meminfo")
        mem = {}
        for line in meminfo.splitlines():
            parts = line.split()
            if len(parts) >= 2:
                key = parts[0].rstrip(":")
                mem[key] = int(parts[1])  # in kB

        total = mem.get("MemTotal", 1) / 1024.0
        available = mem.get("MemAvailable", 0) / 1024.0
        used = total - available
        swap_total = mem.get("SwapTotal", 0) / 1024.0
        swap_free = mem.get("SwapFree", 0) / 1024.0
        swap_used = swap_total - swap_free

        # CPU from /proc/stat — simplified total
        stat = _read_file("/proc/stat")
        # We'll use a simple approach: just report used%
        if not hasattr(self, "_prev_cpu"):
            self._prev_cpu = None

        first_line = stat.splitlines()[0] if stat else ""
        parts = first_line.split()
        if len(parts) >= 8:
            vals = [int(x) for x in parts[1:8]]
            total_ticks = sum(vals)
            idle_ticks = vals[3]

            if self._prev_cpu is not None:
                dt_total = total_ticks - self._prev_cpu[0]
                dt_idle = idle_ticks - self._prev_cpu[1]
                if dt_total > 0:
                    cpu_pct = (1.0 - dt_idle / dt_total) * 100.0
                else:
                    cpu_pct = 0.0
            else:
                cpu_pct = 0.0

            self._prev_cpu = (total_ticks, idle_ticks)
        else:
            cpu_pct = 0.0

        return cpu_pct, used, available, (used / total * 100.0 if total > 0 else 0.0), swap_used

    def _publish(self):
        # System stats
        cpu, used, avail, ram_pct, swap = self._get_system_stats()
        d = self._sys_msg.data
        d[0] = cpu
        d[1] = used
        d[2] = avail
        d[3] = ram_pct
        d[4] = swap
        self._system_pub.publish(self._sys_msg)

        # Per-node stats
        d = self._node_msg.data
        for i, (name, monitor) in enumerate(self._monitors.items()):
            base = i * 3
            rss = monitor.get_rss_mb()
            cpu = monitor.get_cpu_percent()
            trend = monitor.update_trend(rss)

            d[base + 0] = rss
            d[base + 1] = cpu
            d[base + 2] = trend

            # Leak alert
            if abs(trend) > self._leak_threshold and rss > 10.0:
                self._alert_msg.data = (
                    f"LEAK_SUSPECT:{name}:rss={rss:.1f}MB:trend={trend:.2f}MB/min"
                )
                self._alert_pub.publish(self._alert_msg)
                self.get_logger().warn(self._alert_msg.data)

        self._node_mem_pub.publish(self._node_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
