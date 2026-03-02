#!/usr/bin/env python3
"""
System Supervisor Node (v2)

Two-phase health monitoring with categorized recovery.

Phase 1 (BOOT): Each node starts as WAITING. No timeouts, no actions.
  The supervisor just watches for the first heartbeat on each node's
  topic. Only after a generous per-node boot deadline (60s+) does it
  warn that a node failed to start.

Phase 2 (RUNNING): Once a node has published at least once, it moves
  to RUNNING. Now tight heartbeat timeouts (2-5s) apply. If the node
  goes silent, the supervisor acts according to its category.

Categories:
  1 (auto-restart): controller_node, map_server
  2 (abort + restart): nav2, mission_controller
  3 (cascade shutdown): zed, odom_base_publisher

Publishes:
  /robot_health (RobotHealth) — full system status at 1Hz
"""

import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from enum import IntEnum
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
from std_msgs.msg import Float32MultiArray

from system_supervisor.msg import NodeHealth, RobotHealth

# ============================================================================
# Enums
# ============================================================================


class SystemState(IntEnum):
    NOMINAL = 0
    DEGRADED = 1
    LOCALIZATION_LOST = 2
    EMERGENCY_STOP = 3


class NodeStatus(IntEnum):
    WAITING = 0  # Boot phase — never published yet
    RUNNING = 1  # Has published, heartbeat fresh
    STALE = 2  # Was running, heartbeat timed out
    DEAD = 3  # Stale for too long, recovery attempted
    RESTARTING = 4  # Recovery in progress
    SHUTDOWN = 5  # Cascade shutdown (cat3)


class Category(IntEnum):
    CAT1_AUTO_RESTART = 1
    CAT2_ABORT_RESTART = 2
    CAT3_CASCADE_SHUTDOWN = 3


# ============================================================================
# Monitored node config + state
# ============================================================================


@dataclass
class MonitoredNode:
    name: str
    category: Category

    # Detection
    heartbeat_topic: str = ""
    process_name: str = ""

    # Restart
    restart_cmd: List[str] = field(default_factory=list)

    # Monitoring mode
    pid_only: bool = False  # If True, use PID check not topic heartbeat

    # Thresholds
    boot_timeout: float = 60.0  # Max time to wait for first heartbeat
    heartbeat_timeout: float = 3.0  # Timeout AFTER first heartbeat received
    max_restarts: int = 3
    restart_cooldown: float = 10.0

    # Runtime state
    status: int = NodeStatus.WAITING
    ever_seen: bool = False  # True after first heartbeat
    last_heartbeat: float = 0.0
    boot_start: float = 0.0  # When supervisor started tracking
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
# System Supervisor
# ============================================================================


class SystemSupervisor(Node):

    def __init__(self):
        super().__init__("system_supervisor")

        self._system_state = SystemState.NOMINAL
        self._autonomous_enabled = True
        self._shutdown_in_progress = False

        # CPU info
        self._cpu_count = os.cpu_count() or 1
        self._prev_total_cpu = 0
        self._prev_idle_cpu = 0
        self._system_cpu_percent = 0.0
        self._system_memory_used_mb = 0.0
        self._system_memory_total_mb = 0.0

        # Configure nodes
        self._nodes: Dict[str, MonitoredNode] = {}
        self._configure_nodes()

        # Mark boot start for all nodes
        now = time.monotonic()
        for node in self._nodes.values():
            node.boot_start = now

        # E-stop publisher
        self._estop_pub = self.create_publisher(
            Twist,
            "/cmd_vel_out",
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # Health publisher
        self._health_pub = self.create_publisher(RobotHealth, "/robot_health",
                                                 10)

        # Heartbeat subscriptions
        self._setup_subscriptions()

        # Timers
        self.create_timer(1.0, self._health_check)  # Check at 1Hz
        self.create_timer(2.0, self._resource_check)  # Resources at 0.5Hz
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f"Supervisor started: {len(self._nodes)} nodes, {self._cpu_count} CPU cores"
        )

    # ==================================================================
    # Node configuration
    # ==================================================================

    def _configure_nodes(self):
        # --- Cat 3: cascade shutdown ---
        self._nodes["zed"] = MonitoredNode(
            name="zed",
            category=Category.CAT3_CASCADE_SHUTDOWN,
            heartbeat_topic="/zed/zed_node/odom",
            process_name="zed_wrapper",
            boot_timeout=60.0,
            heartbeat_timeout=5.0,
        )
        self._nodes["odom_bridge"] = MonitoredNode(
            name="odom_bridge",
            category=Category.CAT3_CASCADE_SHUTDOWN,
            heartbeat_topic="/odom/base_link",
            process_name="odom_base_publisher",
            boot_timeout=90.0,
            heartbeat_timeout=5.0,
        )

        # --- Cat 2: abort + restart ---
        self._nodes["nav2"] = MonitoredNode(
            name="nav2",
            category=Category.CAT2_ABORT_RESTART,
            process_name="controller_server",
            pid_only=True,  # No continuously publishing topic
            boot_timeout=120.0,
            heartbeat_timeout=10.0,
            max_restarts=2,
            restart_cooldown=15.0,
            restart_cmd=[
                "ros2",
                "launch",
                "robot_navigation",
                "navigation.launch.py",
            ],
        )
        self._nodes["mission"] = MonitoredNode(
            name="mission",
            category=Category.CAT2_ABORT_RESTART,
            process_name="mission_service",
            pid_only=True,  # /active_tile only publishes on change
            boot_timeout=60.0,
            heartbeat_timeout=10.0,
            max_restarts=3,
            restart_cooldown=10.0,
            restart_cmd=[
                "ros2",
                "run",
                "mission_controller",
                "mission_service.py",
            ],
        )

        # --- Cat 1: auto-restart ---
        self._nodes["motors"] = MonitoredNode(
            name="motors",
            category=Category.CAT1_AUTO_RESTART,
            heartbeat_topic="/motor_controller/diagnostics",
            process_name="controller_node",
            boot_timeout=30.0,
            heartbeat_timeout=3.0,
            max_restarts=5,
            restart_cooldown=5.0,
            restart_cmd=[
                "ros2",
                "run",
                "sabertooth_diff_drive",
                "controller_node",
            ],
        )
        self._nodes["map_server"] = MonitoredNode(
            name="map_server",
            category=Category.CAT1_AUTO_RESTART,
            process_name="map_server",
            pid_only=True,  # /map is transient local, publishes once
            boot_timeout=60.0,
            heartbeat_timeout=10.0,
            max_restarts=3,
            restart_cooldown=10.0,
            restart_cmd=[
                "ros2",
                "launch",
                "tile_manager",
                "map_server.launch.py",
            ],
        )

    # ==================================================================
    # Subscriptions — just update timestamps
    # ==================================================================

    def _setup_subscriptions(self):
        """Subscribe only to topics that publish continuously."""
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ZED odom — publishes at camera rate (15-30Hz), always
        self.create_subscription(
            Odometry,
            "/zed/zed_node/odom",
            lambda _: self._heartbeat("zed"),
            best_effort,
        )

        # Odom bridge — publishes whenever ZED publishes
        self.create_subscription(
            Odometry,
            "/odom/base_link",
            lambda _: self._heartbeat("odom_bridge"),
            best_effort,
        )

        # Motor diagnostics — publishes at control rate (10Hz), always
        self.create_subscription(
            Float32MultiArray,
            "/motor_controller/diagnostics",
            lambda _: self._heartbeat("motors"),
            best_effort,
        )

        # nav2, mission, map_server are pid_only — no subscriptions needed

    def _heartbeat(self, name: str):
        node = self._nodes.get(name)
        if node is None:
            return

        now = time.monotonic()
        node.last_heartbeat = now

        if not node.ever_seen:
            node.ever_seen = True
            node.status = NodeStatus.RUNNING
            elapsed = now - node.boot_start
            self.get_logger().info(
                f"[{name}] First heartbeat after {elapsed:.1f}s — RUNNING")
        elif node.status in (NodeStatus.STALE, NodeStatus.RESTARTING):
            node.status = NodeStatus.RUNNING
            node.last_error = ""
            self.get_logger().info(f"[{name}] Recovered — RUNNING")

    # ==================================================================
    # Health check (1Hz)
    # ==================================================================

    def _health_check(self):
        if self._shutdown_in_progress:
            return

        now = time.monotonic()

        for name, node in self._nodes.items():
            if node.status == NodeStatus.SHUTDOWN:
                continue

            if node.pid_only:
                self._check_pid_only(name, node, now)
            else:
                self._check_topic_based(name, node, now)

        self._update_system_state()

    def _check_pid_only(self, name: str, node: MonitoredNode, now: float):
        """
        For nodes without a continuous topic: alive = PID exists.
        Phase 1: wait for PID to appear (boot).
        Phase 2: if PID disappears, it's dead.
        """
        pid = self._find_pid(node)
        pid_alive = pid is not None

        if not node.ever_seen:
            # Phase 1: waiting for process to start
            if pid_alive:
                node.ever_seen = True
                node.status = NodeStatus.RUNNING
                node.last_heartbeat = now
                elapsed = now - node.boot_start
                self.get_logger().info(
                    f"[{name}] Process found (PID {pid}) after {elapsed:.1f}s — RUNNING"
                )
            else:
                boot_elapsed = now - node.boot_start
                if boot_elapsed > node.boot_timeout:
                    node.status = NodeStatus.DEAD
                    node.last_error = f"Process never started ({boot_elapsed:.0f}s)"
                    self.get_logger().error(
                        f"[{name}] Process not found within {node.boot_timeout:.0f}s"
                    )
                    self._handle_failure(name)
        else:
            # Phase 2: process was running, check if still alive
            if pid_alive:
                node.last_heartbeat = now
                if node.status in (NodeStatus.STALE, NodeStatus.RESTARTING):
                    node.status = NodeStatus.RUNNING
                    node.last_error = ""
                    self.get_logger().info(f"[{name}] Recovered — RUNNING")
            else:
                if node.status == NodeStatus.RESTARTING:
                    if now - node.last_restart_time < node.restart_cooldown:
                        return  # Still in cooldown
                if node.status != NodeStatus.DEAD:
                    node.status = NodeStatus.DEAD
                    node.last_error = "Process died"
                    self.get_logger().error(f"[{name}] DEAD — process gone")
                    self._handle_failure(name)

    def _check_topic_based(self, name: str, node: MonitoredNode, now: float):
        """
        For nodes with a continuous topic: alive = recent heartbeat.
        Phase 1: wait for first message.
        Phase 2: timeout if messages stop.
        """
        if not node.ever_seen:
            # Phase 1: waiting for first heartbeat
            boot_elapsed = now - node.boot_start
            if boot_elapsed > node.boot_timeout:
                node.status = NodeStatus.DEAD
                node.last_error = (f"Never started ({boot_elapsed:.0f}s > "
                                   f"{node.boot_timeout:.0f}s boot timeout)")
                self.get_logger().error(
                    f"[{name}] Failed to start within {node.boot_timeout:.0f}s"
                )
                self._handle_failure(name)
            return

        # Phase 2: check heartbeat freshness
        if node.status == NodeStatus.RESTARTING:
            if now - node.last_restart_time > node.restart_cooldown:
                node.error_count += 1
                node.last_error = "No heartbeat after restart"
                self.get_logger().warn(f"[{name}] No heartbeat after restart")
                node.status = NodeStatus.STALE
            else:
                return

        age = now - node.last_heartbeat

        if age > node.heartbeat_timeout:
            if node.status == NodeStatus.RUNNING:
                node.status = NodeStatus.STALE
                node.last_error = f"Heartbeat timeout ({age:.1f}s)"
                self.get_logger().warn(
                    f"[{name}] STALE — no heartbeat for {age:.1f}s")

            if age > node.heartbeat_timeout * 2:
                if node.status != NodeStatus.DEAD:
                    node.status = NodeStatus.DEAD
                    node.last_error = f"Dead ({age:.1f}s)"
                    self.get_logger().error(f"[{name}] DEAD")
                    self._handle_failure(name)

        self._update_system_state()

    # ==================================================================
    # Failure handling
    # ==================================================================

    def _handle_failure(self, name: str):
        node = self._nodes.get(name)
        if node is None:
            return

        if node.category == Category.CAT3_CASCADE_SHUTDOWN:
            self._cascade_shutdown(name)
        elif node.category == Category.CAT2_ABORT_RESTART:
            self._abort_and_restart(name)
        elif node.category == Category.CAT1_AUTO_RESTART:
            self._auto_restart(name)

    def _auto_restart(self, name: str):
        node = self._nodes[name]
        if node.restart_count >= node.max_restarts:
            self.get_logger().error(
                f"[{name}] Max restarts reached — DEGRADED")
            return

        now = time.monotonic()
        if now - node.last_restart_time < node.restart_cooldown:
            return

        self.get_logger().info(
            f"[{name}] Cat1 restart ({node.restart_count + 1}/{node.max_restarts})"
        )
        self._restart_node(node)

    def _abort_and_restart(self, name: str):
        node = self._nodes[name]
        if node.restart_count >= node.max_restarts:
            self.get_logger().error(
                f"[{name}] Max restarts reached — autonomous DEGRADED")
            self._autonomous_enabled = False
            return

        now = time.monotonic()
        if now - node.last_restart_time < node.restart_cooldown:
            return

        self.get_logger().warn(
            f"[{name}] Cat2 — stopping robot and restarting")
        self._send_estop()
        self._restart_node(node)

    def _cascade_shutdown(self, trigger_name: str):
        """Cat3: E-stop, shutdown autonomous systems, keep teleop."""
        if self._shutdown_in_progress:
            return

        self._shutdown_in_progress = True
        self._autonomous_enabled = False
        self._system_state = SystemState.LOCALIZATION_LOST

        self.get_logger().fatal(f"=== CATEGORY 3: {trigger_name} FAILED ===")

        # 1. E-stop
        self._send_estop()
        self._send_estop()

        # 2. Kill autonomous nodes (cat2)
        for name, node in self._nodes.items():
            if node.category == Category.CAT2_ABORT_RESTART:
                self.get_logger().warn(f"[{name}] Shutting down (cascade)")
                self._kill_node(node)
                node.status = NodeStatus.SHUTDOWN

        # 3. Mark cat3 nodes
        for name, node in self._nodes.items():
            if node.category == Category.CAT3_CASCADE_SHUTDOWN:
                node.status = NodeStatus.SHUTDOWN

        self.get_logger().fatal(
            "Autonomous systems shutdown. Teleop still available.\n"
            "Restart the full stack to resume.")

        self._publish_health()

    # ==================================================================
    # Node restart / kill
    # ==================================================================

    def _restart_node(self, node: MonitoredNode):
        if not node.restart_cmd:
            self.get_logger().warn(f"[{node.name}] No restart command")
            return

        self._kill_node(node)

        try:
            proc = subprocess.Popen(
                node.restart_cmd,
                env=os.environ.copy(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            node.pid = proc.pid
            node.restart_count += 1
            node.last_restart_time = time.monotonic()
            node.status = NodeStatus.RESTARTING
            node.last_heartbeat = time.monotonic()
            self.get_logger().info(f"[{node.name}] Restarted (PID {proc.pid})")
        except Exception as e:
            self.get_logger().error(f"[{node.name}] Restart failed: {e}")
            node.error_count += 1
            node.last_error = f"Restart failed: {e}"

    def _kill_node(self, node: MonitoredNode):
        if node.pid is not None:
            try:
                os.killpg(os.getpgid(node.pid), signal.SIGTERM)
                time.sleep(0.5)
                try:
                    os.killpg(os.getpgid(node.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except (ProcessLookupError, PermissionError):
                pass
            node.pid = None

        if node.process_name:
            try:
                subprocess.run(
                    ["pkill", "-f", node.process_name],
                    timeout=2.0,
                    capture_output=True,
                )
            except Exception:
                pass

    def _send_estop(self):
        self._estop_pub.publish(Twist())

    # ==================================================================
    # System state
    # ==================================================================

    def _update_system_state(self):
        if self._system_state == SystemState.LOCALIZATION_LOST:
            return

        any_dead = any(n.status == NodeStatus.DEAD
                       for n in self._nodes.values())
        if any_dead:
            self._system_state = SystemState.DEGRADED
        else:
            self._system_state = SystemState.NOMINAL

    # ==================================================================
    # Resource monitoring (0.5Hz)
    # ==================================================================

    def _resource_check(self):
        self._read_system_cpu()
        self._read_system_memory()
        self._read_process_resources()

    def _read_system_cpu(self):
        try:
            with open("/proc/stat", "r") as f:
                parts = f.readline().split()
            total = sum(int(p) for p in parts[1:9])
            idle = int(parts[4]) + int(parts[5])
            dt = total - self._prev_total_cpu
            di = idle - self._prev_idle_cpu
            if dt > 0:
                self._system_cpu_percent = ((dt - di) / dt) * 100.0
            self._prev_total_cpu = total
            self._prev_idle_cpu = idle
        except Exception:
            pass

    def _read_system_memory(self):
        try:
            mem = {}
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    p = line.split()
                    if len(p) >= 2:
                        mem[p[0].rstrip(":")] = int(p[1])
            self._system_memory_total_mb = mem.get("MemTotal", 0) / 1024.0
            avail = mem.get("MemAvailable", 0) / 1024.0
            self._system_memory_used_mb = self._system_memory_total_mb - avail
        except Exception:
            pass

    def _read_process_resources(self):
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

            try:
                with open(f"/proc/{pid}/stat", "r") as f:
                    stat = f.read().split()
                ticks = sum(int(stat[i]) for i in (13, 14, 15, 16))
                now = time.monotonic()
                if node._prev_cpu_time > 0:
                    dt = now - node._prev_cpu_time
                    if dt > 0:
                        node.cpu_percent = ((ticks - node._prev_cpu_ticks) /
                                            clk_hz / dt) * 100.0
                node._prev_cpu_ticks = ticks
                node._prev_cpu_time = now
            except Exception:
                pass

            try:
                with open(f"/proc/{pid}/status", "r") as f:
                    for line in f:
                        if line.startswith("VmRSS:"):
                            node.memory_mb = int(line.split()[1]) / 1024.0
                            break
            except Exception:
                pass

    def _find_pid(self, node: MonitoredNode) -> Optional[int]:
        if node.pid and os.path.exists(f"/proc/{node.pid}"):
            return node.pid
        node.pid = None
        if not node.process_name:
            return None
        try:
            r = subprocess.run(
                ["pgrep", "-f", node.process_name],
                capture_output=True,
                text=True,
                timeout=1.0,
            )
            if r.returncode == 0:
                pids = r.stdout.strip().split("\n")
                if pids and pids[0]:
                    return int(pids[0])
        except Exception:
            pass
        return None

    # ==================================================================
    # Health publisher (1Hz)
    # ==================================================================

    def _publish_health(self):
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
            nh.restart_count = int(node.restart_count)
            nh.error_count = int(node.error_count)
            nh.last_error = node.last_error

            if node.ever_seen:
                nh.last_heartbeat_age = float(now - node.last_heartbeat)
                nh.message = self._label(node.status)
            else:
                nh.last_heartbeat_age = float(now - node.boot_start)
                nh.message = (
                    f"Waiting for first heartbeat ({now - node.boot_start:.0f}s)"
                )

            msg.nodes.append(nh)

        if self._system_state == SystemState.NOMINAL:
            msg.system_message = "All systems nominal"
        elif self._system_state == SystemState.DEGRADED:
            dead = [
                n.name for n in self._nodes.values()
                if n.status == NodeStatus.DEAD
            ]
            msg.system_message = f"Degraded: {', '.join(dead)}"
        elif self._system_state == SystemState.LOCALIZATION_LOST:
            msg.system_message = (
                "LOCALIZATION LOST — autonomous shutdown, teleop available")

        self._health_pub.publish(msg)

    @staticmethod
    def _label(status: int) -> str:
        return {
            NodeStatus.WAITING: "Starting",
            NodeStatus.RUNNING: "OK",
            NodeStatus.STALE: "Stale",
            NodeStatus.DEAD: "Dead",
            NodeStatus.RESTARTING: "Restarting",
            NodeStatus.SHUTDOWN: "Shutdown",
        }.get(status, "Unknown")


# ==========================================================================


def main(args=None):
    rclpy.init(args=args)
    node = SystemSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
