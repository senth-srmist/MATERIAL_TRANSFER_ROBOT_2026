#!/usr/bin/env python3
"""
System Supervisor Node

Monitors all robot subsystems and performs categorized recovery:
  Category 1 (auto-restart): controller_node, map_server, teleop
  Category 2 (abort tasks + restart): nav2, mission_controller
  Category 3 (e-stop + cascade shutdown): zed, odom_base_publisher

Monitors:
  - Process liveness (PID checks via /proc)
  - Topic freshness (subscription-based heartbeats)
  - Data quality (odom jump detection, quaternion sanity)
  - System resources (CPU, memory per-process and total)

Publishes:
  /robot_health (RobotHealth) - full system status at 1Hz
  /cmd_vel_out (Twist) - emergency zero velocity on cat3

Subscribes:
  /odom/base_link - for data quality checks
  /active_tile - mission state heartbeat
  /motor_controller/diagnostics - controller heartbeat
"""

import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32MultiArray

from system_supervisor.msg import NodeHealth, RobotHealth

import math
import numpy as np


# ============================================================================
# Constants
# ============================================================================

class SystemState(IntEnum):
    NOMINAL = 0
    DEGRADED = 1
    LOCALIZATION_LOST = 2
    EMERGENCY_STOP = 3


class NodeStatus(IntEnum):
    RUNNING = 0
    STALE = 1
    DEAD = 2
    RESTARTING = 3
    SHUTDOWN = 4


class Category(IntEnum):
    """Recovery category for each monitored node."""
    CAT1_AUTO_RESTART = 1
    CAT2_ABORT_RESTART = 2
    CAT3_CASCADE_SHUTDOWN = 3


# ============================================================================
# Monitored node configuration
# ============================================================================

@dataclass
class MonitoredNode:
    """Configuration and runtime state for a monitored node."""
    name: str
    category: Category

    # How to detect liveness
    heartbeat_topic: str = ""          # Topic to subscribe for heartbeat
    process_name: str = ""             # Process name to check in /proc

    # Restart command (for cat1 and cat2)
    restart_cmd: List[str] = field(default_factory=list)

    # Thresholds
    heartbeat_timeout: float = 3.0     # Seconds without heartbeat = STALE
    max_restarts: int = 3              # Stop retrying after this many
    restart_cooldown: float = 10.0     # Minimum seconds between restarts

    # Runtime state (not config)
    status: int = NodeStatus.RUNNING
    last_heartbeat: float = 0.0        # monotonic time
    restart_count: int = 0
    error_count: int = 0
    last_error: str = ""
    last_restart_time: float = 0.0
    pid: Optional[int] = None
    cpu_percent: float = 0.0
    memory_mb: float = 0.0
    _prev_cpu_ticks: int = 0
    _prev_cpu_time: float = 0.0


# ============================================================================
# Data quality tracker
# ============================================================================

@dataclass
class OdomQuality:
    """Tracks odom data quality for anomaly detection."""
    last_x: float = 0.0
    last_y: float = 0.0
    last_time: float = 0.0
    initialized: bool = False
    consecutive_anomalies: int = 0

    # Thresholds
    max_position_jump: float = 2.0     # Meters between consecutive msgs
    max_velocity: float = 2.0          # m/s — physical limit of robot
    anomaly_threshold: int = 3         # Consecutive anomalies before cat3


# ============================================================================
# System Supervisor
# ============================================================================

class SystemSupervisor(Node):

    # Resource warning thresholds
    CPU_WARN_THRESHOLD = 85.0          # Percent
    MEMORY_WARN_THRESHOLD_MB = 100.0   # MB remaining free

    def __init__(self):
        super().__init__("system_supervisor")

        self._system_state = SystemState.NOMINAL
        self._autonomous_enabled = True
        self._shutdown_in_progress = False

        # ====== Monitored nodes ======
        self._nodes: Dict[str, MonitoredNode] = {}
        self._configure_monitored_nodes()

        # ====== Data quality ======
        self._odom_quality = OdomQuality()

        # ====== System resource tracking ======
        self._cpu_count = os.cpu_count() or 1
        self._prev_total_cpu = 0
        self._prev_idle_cpu = 0
        self._system_cpu_percent = 0.0         # 0-100% (averaged across cores)
        self._system_memory_used_mb = 0.0
        self._system_memory_total_mb = 0.0

        self.get_logger().info(f"Detected {self._cpu_count} CPU cores")

        # ====== E-stop publisher ======
        estop_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._estop_pub = self.create_publisher(
            Twist, "/cmd_vel_out", estop_qos
        )

        # ====== Health publisher ======
        self._health_pub = self.create_publisher(
            RobotHealth, "/robot_health", 10
        )

        # ====== Subscriptions for heartbeats ======
        self._setup_heartbeat_subscriptions()

        # ====== Odom subscription for data quality ======
        self.create_subscription(
            Odometry, "/odom/base_link", self._odom_quality_cb,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # ====== Timers ======
        # Health check at 2Hz
        self.create_timer(0.5, self._health_check)
        # Resource monitoring at 1Hz
        self.create_timer(1.0, self._resource_check)
        # Publish health at 1Hz
        self.create_timer(1.0, self._publish_health)

        # Initialize heartbeat timestamps
        now = time.monotonic()
        for node in self._nodes.values():
            node.last_heartbeat = now

        self.get_logger().info(
            f"System Supervisor started: monitoring {len(self._nodes)} nodes"
        )

    # ======================================================================
    # Configuration
    # ======================================================================

    def _configure_monitored_nodes(self):
        """Define all monitored nodes with their categories and restart commands."""

        # --- Category 3: Cascade shutdown (no restart) ---
        self._nodes["zed"] = MonitoredNode(
            name="zed",
            category=Category.CAT3_CASCADE_SHUTDOWN,
            heartbeat_topic="/zed/zed_node/odom",
            process_name="zed_wrapper",
            heartbeat_timeout=3.0,
        )
        self._nodes["odom_base_publisher"] = MonitoredNode(
            name="odom_base_publisher",
            category=Category.CAT3_CASCADE_SHUTDOWN,
            heartbeat_topic="/odom/base_link",
            process_name="odom_base_publisher",
            heartbeat_timeout=3.0,
        )

        # --- Category 2: Abort tasks + restart ---
        self._nodes["nav2"] = MonitoredNode(
            name="nav2",
            category=Category.CAT2_ABORT_RESTART,
            heartbeat_topic="/local_costmap/costmap",
            process_name="controller_server",
            heartbeat_timeout=5.0,
            max_restarts=2,
            restart_cooldown=15.0,
            restart_cmd=[
                "ros2", "launch", "robot_navigation", "navigation.launch.py",
            ],
        )
        self._nodes["mission_controller"] = MonitoredNode(
            name="mission_controller",
            category=Category.CAT2_ABORT_RESTART,
            heartbeat_topic="/active_tile",
            process_name="mission_service",
            heartbeat_timeout=5.0,
            max_restarts=3,
            restart_cooldown=10.0,
            restart_cmd=[
                "ros2", "run", "mission_controller", "mission_service.py",
            ],
        )

        # --- Category 1: Auto-restart ---
        self._nodes["controller_node"] = MonitoredNode(
            name="controller_node",
            category=Category.CAT1_AUTO_RESTART,
            heartbeat_topic="/motor_controller/diagnostics",
            process_name="controller_node",
            heartbeat_timeout=2.0,
            max_restarts=5,
            restart_cooldown=5.0,
            restart_cmd=[
                "ros2", "run", "sabertooth_diff_drive", "controller_node",
            ],
        )
        self._nodes["map_server"] = MonitoredNode(
            name="map_server",
            category=Category.CAT1_AUTO_RESTART,
            heartbeat_topic="/map",
            process_name="map_server",
            heartbeat_timeout=5.0,
            max_restarts=3,
            restart_cooldown=10.0,
            restart_cmd=[
                "ros2", "launch", "tile_manager", "map_server.launch.py",
            ],
        )

    # ======================================================================
    # Heartbeat subscriptions
    # ======================================================================

    def _setup_heartbeat_subscriptions(self):
        """Create lightweight subscriptions for each monitored topic."""

        # We use generic AnyMsg-style approach:
        # Subscribe to known message types on the heartbeat topics.
        # The callback just updates the timestamp — we don't process the data.

        heartbeat_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ZED odom
        self.create_subscription(
            Odometry, "/zed/zed_node/odom",
            lambda msg: self._heartbeat("zed"),
            heartbeat_qos,
        )

        # Odom base publisher
        self.create_subscription(
            Odometry, "/odom/base_link",
            lambda msg: self._heartbeat("odom_base_publisher"),
            heartbeat_qos,
        )

        # Nav2 (local costmap publishes frequently when active)
        from nav_msgs.msg import OccupancyGrid
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap",
            lambda msg: self._heartbeat("nav2"),
            heartbeat_qos,
        )

        # Mission controller (active tile — transient local)
        self.create_subscription(
            Int32, "/active_tile",
            lambda msg: self._heartbeat("mission_controller"),
            reliable_qos,
        )

        # Controller node diagnostics
        self.create_subscription(
            Float32MultiArray, "/motor_controller/diagnostics",
            lambda msg: self._heartbeat("controller_node"),
            heartbeat_qos,
        )

        # Map server (map — transient local, infrequent but alive)
        from nav_msgs.msg import OccupancyGrid
        self.create_subscription(
            OccupancyGrid, "/map",
            lambda msg: self._heartbeat("map_server"),
            reliable_qos,
        )

    def _heartbeat(self, node_name: str):
        """Update heartbeat timestamp for a node."""
        if node_name in self._nodes:
            self._nodes[node_name].last_heartbeat = time.monotonic()
            # If node was STALE or RESTARTING, mark it RUNNING again
            node = self._nodes[node_name]
            if node.status in (NodeStatus.STALE, NodeStatus.RESTARTING):
                node.status = NodeStatus.RUNNING
                node.last_error = ""
                self.get_logger().info(f"[{node_name}] recovered — RUNNING")

    # ======================================================================
    # Data quality monitoring
    # ======================================================================

    def _odom_quality_cb(self, msg: Odometry):
        """Check odom data for anomalies (jumps, impossible velocities)."""
        q = self._odom_quality
        now = time.monotonic()

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation

        # Check quaternion validity
        quat_mag = math.sqrt(o.x**2 + o.y**2 + o.z**2 + o.w**2)
        if not math.isfinite(quat_mag) or quat_mag < 0.5 or quat_mag > 1.5:
            self._record_anomaly("odom_base_publisher", "Invalid quaternion")
            return

        # Check for NaN
        if not (math.isfinite(x) and math.isfinite(y)):
            self._record_anomaly("odom_base_publisher", "NaN in position")
            return

        if not q.initialized:
            q.last_x = x
            q.last_y = y
            q.last_time = now
            q.initialized = True
            return

        # Position jump detection
        dt = now - q.last_time
        if dt > 0.01:
            dx = x - q.last_x
            dy = y - q.last_y
            dist = math.sqrt(dx * dx + dy * dy)

            # Jump detection (teleportation)
            if dist > q.max_position_jump:
                self._record_anomaly(
                    "odom_base_publisher",
                    f"Position jump: {dist:.2f}m in {dt:.2f}s"
                )
            # Impossible velocity
            elif dt > 0 and (dist / dt) > q.max_velocity:
                self._record_anomaly(
                    "odom_base_publisher",
                    f"Impossible velocity: {dist/dt:.2f} m/s"
                )
            else:
                # Good data — reset anomaly counter
                q.consecutive_anomalies = 0

        q.last_x = x
        q.last_y = y
        q.last_time = now

    def _record_anomaly(self, node_name: str, message: str):
        """Record a data quality anomaly. Escalate if persistent."""
        q = self._odom_quality
        q.consecutive_anomalies += 1

        node = self._nodes.get(node_name)
        if node:
            node.error_count += 1
            node.last_error = message

        self.get_logger().warn(
            f"[{node_name}] Data anomaly ({q.consecutive_anomalies}/"
            f"{q.anomaly_threshold}): {message}"
        )

        if q.consecutive_anomalies >= q.anomaly_threshold:
            self.get_logger().error(
                f"[{node_name}] Persistent data anomaly — "
                "treating as Category 3 failure"
            )
            self._handle_failure(node_name)

    # ======================================================================
    # Health check (2Hz)
    # ======================================================================

    def _health_check(self):
        """Check all monitored nodes for liveness."""
        if self._shutdown_in_progress:
            return

        now = time.monotonic()

        for name, node in self._nodes.items():
            if node.status == NodeStatus.SHUTDOWN:
                continue  # Already shut down by cascade

            if node.status == NodeStatus.RESTARTING:
                # Check if restart cooldown is over
                if now - node.last_restart_time > node.restart_cooldown:
                    # Still no heartbeat after restart + cooldown
                    node.error_count += 1
                    node.last_error = "No heartbeat after restart"
                    self.get_logger().warn(
                        f"[{name}] No heartbeat after restart"
                    )
                continue

            # Check heartbeat freshness
            age = now - node.last_heartbeat
            if age > node.heartbeat_timeout:
                if node.status == NodeStatus.RUNNING:
                    node.status = NodeStatus.STALE
                    node.last_error = f"Heartbeat timeout ({age:.1f}s)"
                    self.get_logger().warn(
                        f"[{name}] STALE — no heartbeat for {age:.1f}s"
                    )

                # If stale for 2x timeout, treat as dead
                if age > node.heartbeat_timeout * 2:
                    node.status = NodeStatus.DEAD
                    node.last_error = f"Dead ({age:.1f}s without heartbeat)"
                    self.get_logger().error(f"[{name}] DEAD")
                    self._handle_failure(name)

        # Update system state
        self._update_system_state()

    # ======================================================================
    # Failure handling (categorized)
    # ======================================================================

    def _handle_failure(self, node_name: str):
        """Handle a node failure according to its category."""
        node = self._nodes.get(node_name)
        if node is None:
            return

        if node.category == Category.CAT3_CASCADE_SHUTDOWN:
            self._handle_cat3(node_name)
        elif node.category == Category.CAT2_ABORT_RESTART:
            self._handle_cat2(node_name)
        elif node.category == Category.CAT1_AUTO_RESTART:
            self._handle_cat1(node_name)

    def _handle_cat1(self, node_name: str):
        """Category 1: Simple auto-restart."""
        node = self._nodes[node_name]

        if node.restart_count >= node.max_restarts:
            self.get_logger().error(
                f"[{node_name}] Max restarts ({node.max_restarts}) reached. "
                "Giving up — system DEGRADED"
            )
            node.status = NodeStatus.DEAD
            return

        now = time.monotonic()
        if now - node.last_restart_time < node.restart_cooldown:
            return  # Too soon to restart

        self.get_logger().info(
            f"[{node_name}] Category 1 restart "
            f"({node.restart_count + 1}/{node.max_restarts})"
        )
        self._restart_node(node)

    def _handle_cat2(self, node_name: str):
        """Category 2: Abort active tasks, then restart."""
        node = self._nodes[node_name]

        if node.restart_count >= node.max_restarts:
            self.get_logger().error(
                f"[{node_name}] Max restarts ({node.max_restarts}) reached. "
                "System DEGRADED — autonomous navigation unavailable"
            )
            node.status = NodeStatus.DEAD
            self._autonomous_enabled = False
            return

        now = time.monotonic()
        if now - node.last_restart_time < node.restart_cooldown:
            return

        self.get_logger().warn(
            f"[{node_name}] Category 2 failure — aborting tasks and restarting "
            f"({node.restart_count + 1}/{node.max_restarts})"
        )

        # Send stop to prevent uncontrolled movement during restart
        self._send_estop()

        self._restart_node(node)

    def _handle_cat3(self, node_name: str):
        """
        Category 3: Cascade shutdown of autonomous systems.

        1. E-stop (zero cmd_vel)
        2. Shutdown Nav2 + mission_controller
        3. Keep teleop, controller_node, map_server, supervisor alive
        4. Publish LOCALIZATION_LOST
        """
        if self._shutdown_in_progress:
            return

        self._shutdown_in_progress = True
        self._autonomous_enabled = False

        self.get_logger().fatal(
            f"[{node_name}] CATEGORY 3 FAILURE — "
            "LOCALIZATION LOST — SHUTTING DOWN AUTONOMOUS SYSTEMS"
        )

        # Step 1: Immediate e-stop
        self._send_estop()
        self._send_estop()  # Send twice for reliability

        # Step 2: Mark localization as lost
        self._system_state = SystemState.LOCALIZATION_LOST

        # Step 3: Shutdown autonomous nodes (cat2 nodes)
        for name, node in self._nodes.items():
            if node.category == Category.CAT2_ABORT_RESTART:
                self.get_logger().warn(
                    f"[{name}] Shutting down (cascade from {node_name})"
                )
                self._kill_node(node)
                node.status = NodeStatus.SHUTDOWN

        # Step 4: Mark cat3 nodes as shutdown
        for name, node in self._nodes.items():
            if node.category == Category.CAT3_CASCADE_SHUTDOWN:
                node.status = NodeStatus.SHUTDOWN

        # Step 5: Publish health immediately
        self._publish_health()

        self.get_logger().fatal(
            "=== AUTONOMOUS SYSTEMS SHUTDOWN COMPLETE ===\n"
            "  Localization is lost. Robot is stopped.\n"
            "  Teleop is still available for manual recovery.\n"
            "  Restart the full stack to resume autonomous operation."
        )

    # ======================================================================
    # Node restart / kill
    # ======================================================================

    def _restart_node(self, node: MonitoredNode):
        """Restart a node using its configured command."""
        if not node.restart_cmd:
            self.get_logger().warn(
                f"[{node.name}] No restart command configured"
            )
            return

        # Kill existing process first
        self._kill_node(node)

        # Start new process
        try:
            env = os.environ.copy()
            proc = subprocess.Popen(
                node.restart_cmd,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,  # New process group for clean kill
            )
            node.pid = proc.pid
            node.restart_count += 1
            node.last_restart_time = time.monotonic()
            node.status = NodeStatus.RESTARTING
            node.last_heartbeat = time.monotonic()  # Grace period

            self.get_logger().info(
                f"[{node.name}] Restarted (PID {proc.pid})"
            )
        except Exception as e:
            self.get_logger().error(
                f"[{node.name}] Restart failed: {e}"
            )
            node.error_count += 1
            node.last_error = f"Restart failed: {e}"

    def _kill_node(self, node: MonitoredNode):
        """Kill a node's process if running."""
        # Try by PID first
        if node.pid is not None:
            try:
                os.killpg(os.getpgid(node.pid), signal.SIGTERM)
                time.sleep(0.5)
                # Force kill if still alive
                try:
                    os.killpg(os.getpgid(node.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except ProcessLookupError:
                pass
            except Exception as e:
                self.get_logger().warn(
                    f"[{node.name}] Kill by PID failed: {e}"
                )
            node.pid = None

        # Also try by process name (catches processes started by launch files)
        if node.process_name:
            try:
                subprocess.run(
                    ["pkill", "-f", node.process_name],
                    timeout=2.0,
                    capture_output=True,
                )
            except Exception:
                pass

    # ======================================================================
    # E-stop
    # ======================================================================

    def _send_estop(self):
        """Publish zero velocity — immediate stop."""
        msg = Twist()  # All zeros by default
        self._estop_pub.publish(msg)

    # ======================================================================
    # System state
    # ======================================================================

    def _update_system_state(self):
        """Update overall system state based on node statuses."""
        if self._system_state == SystemState.LOCALIZATION_LOST:
            return  # Sticky until full restart

        any_dead = False
        any_stale = False

        for node in self._nodes.values():
            if node.status == NodeStatus.DEAD:
                any_dead = True
            elif node.status == NodeStatus.STALE:
                any_stale = True

        if any_dead:
            self._system_state = SystemState.DEGRADED
        elif any_stale:
            self._system_state = SystemState.DEGRADED
        else:
            self._system_state = SystemState.NOMINAL

    # ======================================================================
    # Resource monitoring (1Hz)
    # ======================================================================

    def _resource_check(self):
        """Read system-wide and per-process CPU/memory from /proc."""
        self._read_system_resources()
        self._read_process_resources()

        # Warn on high resource usage
        free_mb = self._system_memory_total_mb - self._system_memory_used_mb
        if free_mb < self.MEMORY_WARN_THRESHOLD_MB:
            self.get_logger().warn(
                f"Low memory: {free_mb:.0f}MB free "
                f"({self._system_memory_used_mb:.0f}/"
                f"{self._system_memory_total_mb:.0f}MB)"
            )

        if self._system_cpu_percent > self.CPU_WARN_THRESHOLD:
            self.get_logger().warn(
                f"High CPU: {self._system_cpu_percent:.1f}%"
            )

    def _read_system_resources(self):
        """Read system-wide CPU and memory from /proc."""
        # --- CPU from /proc/stat ---
        try:
            with open("/proc/stat", "r") as f:
                line = f.readline()
            parts = line.split()
            # user nice system idle iowait irq softirq steal
            total = sum(int(p) for p in parts[1:9])
            idle = int(parts[4]) + int(parts[5])  # idle + iowait

            d_total = total - self._prev_total_cpu
            d_idle = idle - self._prev_idle_cpu

            if d_total > 0:
                self._system_cpu_percent = (
                    (d_total - d_idle) / d_total
                ) * 100.0
            self._prev_total_cpu = total
            self._prev_idle_cpu = idle
        except Exception:
            pass

        # --- Memory from /proc/meminfo ---
        try:
            meminfo = {}
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        key = parts[0].rstrip(":")
                        meminfo[key] = int(parts[1])  # kB

            total_kb = meminfo.get("MemTotal", 0)
            available_kb = meminfo.get("MemAvailable", 0)

            self._system_memory_total_mb = total_kb / 1024.0
            self._system_memory_used_mb = (total_kb - available_kb) / 1024.0
        except Exception:
            pass

    def _read_process_resources(self):
        """
        Read per-process CPU and memory for monitored nodes.

        CPU is reported as 0 to N*100% where N = cpu_count.
        e.g., on a 4-core Jetson: a process using 2 full cores = 200%.
        This matches htop/top behavior and what you see on the Jetson.

        We also aggregate child processes (threads spawned by launch files)
        by reading /proc/[pid]/task/* for the main PID.
        """
        clk_hz = os.sysconf("SC_CLK_TCK")

        for node in self._nodes.values():
            if node.status == NodeStatus.SHUTDOWN:
                node.cpu_percent = 0.0
                node.memory_mb = 0.0
                continue

            pid = self._find_pid(node)
            if pid is None:
                node.cpu_percent = 0.0
                node.memory_mb = 0.0
                continue

            node.pid = pid

            # --- CPU from /proc/[pid]/stat ---
            # utime(13) + stime(14) + cutime(15) + cstime(16)
            # cutime/cstime include waited-for children
            try:
                with open(f"/proc/{pid}/stat", "r") as f:
                    stat = f.read().split()
                ticks = (
                    int(stat[13]) + int(stat[14])    # own user + system
                    + int(stat[15]) + int(stat[16])  # children user + system
                )
                now = time.monotonic()

                if node._prev_cpu_time > 0:
                    dt = now - node._prev_cpu_time
                    d_ticks = ticks - node._prev_cpu_ticks
                    if dt > 0:
                        # Raw percent: 0 to N*100% (matches htop)
                        node.cpu_percent = (d_ticks / clk_hz / dt) * 100.0

                node._prev_cpu_ticks = ticks
                node._prev_cpu_time = now
            except Exception:
                pass

            # --- Memory from /proc/[pid]/status ---
            try:
                with open(f"/proc/{pid}/status", "r") as f:
                    for line in f:
                        if line.startswith("VmRSS:"):
                            rss_kb = int(line.split()[1])
                            node.memory_mb = rss_kb / 1024.0
                            break
            except Exception:
                pass

    def _find_pid(self, node: MonitoredNode) -> Optional[int]:
        """Find PID for a node by process name in /proc."""
        if node.pid is not None:
            # Verify existing PID is alive
            if os.path.exists(f"/proc/{node.pid}"):
                return node.pid
            node.pid = None

        if not node.process_name:
            return None

        try:
            result = subprocess.run(
                ["pgrep", "-f", node.process_name],
                capture_output=True, text=True, timeout=1.0,
            )
            if result.returncode == 0:
                pids = result.stdout.strip().split("\n")
                if pids and pids[0]:
                    return int(pids[0])
        except Exception:
            pass

        return None

    # ======================================================================
    # Health publisher (1Hz)
    # ======================================================================

    def _publish_health(self):
        """Publish full system health."""
        msg = RobotHealth()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.cpu_percent = float(self._system_cpu_percent)
        msg.cpu_count = self._cpu_count
        msg.memory_used_mb = float(self._system_memory_used_mb)
        msg.memory_total_mb = float(self._system_memory_total_mb)

        msg.system_state = int(self._system_state)
        msg.autonomous_enabled = self._autonomous_enabled

        now = time.monotonic()

        for node in self._nodes.values():
            nh = NodeHealth()
            nh.name = node.name
            nh.status = int(node.status)
            nh.cpu_percent = float(node.cpu_percent)
            nh.memory_mb = float(node.memory_mb)
            nh.last_heartbeat_age = float(now - node.last_heartbeat)
            nh.restart_count = int(node.restart_count)
            nh.error_count = int(node.error_count)
            nh.last_error = node.last_error
            nh.message = self._status_label(node.status)
            msg.nodes.append(nh)

        # System message
        if self._system_state == SystemState.NOMINAL:
            msg.system_message = "All systems nominal"
        elif self._system_state == SystemState.DEGRADED:
            dead = [n.name for n in self._nodes.values()
                    if n.status == NodeStatus.DEAD]
            msg.system_message = f"Degraded — failed: {', '.join(dead)}"
        elif self._system_state == SystemState.LOCALIZATION_LOST:
            msg.system_message = (
                "LOCALIZATION LOST — autonomous systems shutdown. "
                "Teleop available for manual recovery."
            )
        elif self._system_state == SystemState.EMERGENCY_STOP:
            msg.system_message = "EMERGENCY STOP"

        self._health_pub.publish(msg)

    @staticmethod
    def _status_label(status: int) -> str:
        labels = {
            NodeStatus.RUNNING: "OK",
            NodeStatus.STALE: "Stale",
            NodeStatus.DEAD: "Dead",
            NodeStatus.RESTARTING: "Restarting",
            NodeStatus.SHUTDOWN: "Shutdown (cascade)",
        }
        return labels.get(status, "Unknown")


# ==========================================================================
# Main
# ==========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SystemSupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Supervisor shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
