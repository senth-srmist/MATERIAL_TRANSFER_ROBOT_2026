#!/usr/bin/env python3
"""
System Supervisor Node (v3)

State machine with on-demand node lifecycle management.

States:
  IDLE        — Only always-on nodes running (motors). Watching active_jobs.
  ACTIVATING  — Bringing up nav stack sequentially with health gates.
  ACTIVE      — Full monitoring, categorized recovery.
  DEACTIVATING — Shutting down nav stack sequentially.

Always-on nodes (monitored in all states):
  - motors (controller_node) — topic heartbeat

On-demand nodes (started/stopped by supervisor):
  - zed (zed_wrapper)
  - odom_bridge (odom_base_publisher)
  - map_server
  - nav2 (controller_server)
  - mission (mission_service)

Startup sequence (health-gated):
  ZED -> odom_bridge -> map_server -> Nav2 -> mission_controller

Shutdown sequence:
  mission -> Nav2 -> map_server -> odom_bridge -> ZED

Publishes:
  /robot_health (RobotHealth)     — full system status at 1Hz
  /system/nav_ready (Empty)       — nav stack is up
  /system/nav_shutdown (Empty)    — nav stack going down

Subscribes:
  /system/active_jobs (Int32)     — from job_manager
"""

import importlib
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool

from system_supervisor.msg import NodeHealth, RobotHealth

# ============================================================================
# Enums
# ============================================================================


class SupervisorState(IntEnum):
    IDLE = 0
    ACTIVATING = 1
    ACTIVE = 2
    DEACTIVATING = 3


class SystemState(IntEnum):
    NOMINAL = 0
    DEGRADED = 1
    LOCALIZATION_LOST = 2
    EMERGENCY_STOP = 3


class NodeStatus(IntEnum):
    WAITING = 0
    RUNNING = 1
    STALE = 2
    DEAD = 3
    RESTARTING = 4
    SHUTDOWN = 5
    OFF = 6  # Not started yet (on-demand nodes in IDLE)


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

    # Restart / start
    start_cmd: List[str] = field(default_factory=list)

    # Monitoring mode
    pid_only: bool = False
    always_on: bool = False  # True = monitored in all states

    # Thresholds
    boot_timeout: float = 60.0
    heartbeat_timeout: float = 3.0
    max_restarts: int = 3
    restart_cooldown: float = 10.0

    # Runtime state
    status: int = NodeStatus.OFF
    ever_seen: bool = False
    last_heartbeat: float = 0.0
    boot_start: float = 0.0
    restart_count: int = 0
    error_count: int = 0
    last_error: str = ""
    last_restart_time: float = 0.0
    pid: Optional[int] = None
    cpu_percent: float = 0.0
    memory_mb: float = 0.0
    _prev_cpu_ticks: int = 0
    _prev_cpu_time: float = 0.0

    def reset(self):
        """Reset runtime state for next activation cycle."""
        self.status = NodeStatus.OFF
        self.ever_seen = False
        self.last_heartbeat = 0.0
        self.boot_start = 0.0
        self.restart_count = 0
        self.error_count = 0
        self.last_error = ""
        self.last_restart_time = 0.0
        self.pid = None
        self.cpu_percent = 0.0
        self.memory_mb = 0.0
        self._prev_cpu_ticks = 0
        self._prev_cpu_time = 0.0


# ============================================================================
# System Supervisor
# ============================================================================


class SystemSupervisor(Node):
    # Built from config at init
    STARTUP_SEQUENCE = []  # Populated by _load_config
    SHUTDOWN_SEQUENCE = []  # Populated by _load_config

    def __init__(self):
        super().__init__("system_supervisor")

        self._supervisor_state = SupervisorState.IDLE
        self._system_state = SystemState.NOMINAL
        self._autonomous_enabled = False
        self._shutdown_in_progress = False
        self._nav_needed = False

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

        # Mark boot start for always-on nodes
        now = time.monotonic()
        for node in self._nodes.values():
            if node.always_on:
                node.boot_start = now
                node.status = NodeStatus.WAITING

        # Publishers
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
        self._health_pub = self.create_publisher(RobotHealth, "/robot_health", 10)
        self._nav_ready_pub = self.create_publisher(Empty, "/system/nav_ready", 10)
        self._nav_shutdown_pub = self.create_publisher(
            Empty, "/system/nav_shutdown", 10
        )

        # Subscriptions
        self._setup_subscriptions()

        # Nav needed subscription (from job_manager)
        self.create_subscription(
            Bool,
            "/system/nav_needed",
            self._nav_needed_cb,
            10,
        )

        # Timers
        self.create_timer(1.0, self._tick)  # Main loop at 1Hz
        self.create_timer(2.0, self._resource_check)  # Resources at 0.5Hz
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f"Supervisor started in IDLE mode, {self._cpu_count} CPU cores"
        )

    # ==================================================================
    # Config loading
    # ==================================================================

    def _configure_nodes(self):
        """Load node definitions from YAML config file."""
        self.declare_parameter(
            "config_file",
            "/workspace/ros_ws/install/system_supervisor/share/"
            "system_supervisor/config/supervisor_config.yaml",
        )
        config_path = self.get_parameter("config_file").value

        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().fatal(f"Cannot load config: {config_path} ({e})")
            raise

        self.get_logger().info(f"Config loaded: {config_path}")

        category_map = {
            1: Category.CAT1_AUTO_RESTART,
            2: Category.CAT2_ABORT_RESTART,
            3: Category.CAT3_CASCADE_SHUTDOWN,
        }

        # Always-on nodes
        for name, cfg in config.get("always_on", {}).items():
            self._nodes[name] = MonitoredNode(
                name=name,
                category=category_map[cfg["category"]],
                heartbeat_topic=cfg.get("heartbeat_topic", ""),
                process_name=cfg.get("process_name", ""),
                start_cmd=cfg.get("start_cmd", []),
                pid_only=cfg.get("pid_only", False),
                always_on=True,
                boot_timeout=cfg.get("boot_timeout", 60.0),
                heartbeat_timeout=cfg.get("heartbeat_timeout", 3.0),
                max_restarts=cfg.get("max_restarts", 3),
                restart_cooldown=cfg.get("restart_cooldown", 10.0),
            )

        # On-demand nodes
        startup_entries = []
        for name, cfg in config.get("on_demand", {}).items():
            self._nodes[name] = MonitoredNode(
                name=name,
                category=category_map[cfg["category"]],
                heartbeat_topic=cfg.get("heartbeat_topic", ""),
                process_name=cfg.get("process_name", ""),
                start_cmd=cfg.get("start_cmd", []),
                pid_only=cfg.get("pid_only", False),
                always_on=False,
                boot_timeout=cfg.get("boot_timeout", 60.0),
                heartbeat_timeout=cfg.get("heartbeat_timeout", 3.0),
                max_restarts=cfg.get("max_restarts", 3),
                restart_cooldown=cfg.get("restart_cooldown", 10.0),
            )
            startup_entries.append(
                (
                    cfg.get("startup_order", 99),
                    name,
                    cfg.get("wait_type", "pid"),
                    cfg.get("heartbeat_topic", ""),
                    cfg.get("wait_timeout", 60.0),
                )
            )

        # Sort by startup_order, build sequences
        startup_entries.sort(key=lambda x: x[0])
        self.STARTUP_SEQUENCE = [
            (name, wait_type, topic, timeout)
            for _, name, wait_type, topic, timeout in startup_entries
        ]
        self.SHUTDOWN_SEQUENCE = [
            name for _, name, _, _, _ in reversed(startup_entries)
        ]

        # Store msg types for dynamic subscriptions
        self._heartbeat_msg_types = {}
        for section in ("always_on", "on_demand"):
            for name, cfg in config.get(section, {}).items():
                if cfg.get("heartbeat_topic") and cfg.get("heartbeat_msg_type"):
                    self._heartbeat_msg_types[name] = (
                        cfg["heartbeat_topic"],
                        cfg["heartbeat_msg_type"],
                    )

        always_names = [n for n, nd in self._nodes.items() if nd.always_on]
        demand_names = [n for n, nd in self._nodes.items() if not nd.always_on]
        self.get_logger().info(f"Always-on: {always_names}, On-demand: {demand_names}")
        self.get_logger().info(
            f"Startup order: {[s[0] for s in self.STARTUP_SEQUENCE]}"
        )

    # ==================================================================
    # Subscriptions — built dynamically from config
    # ==================================================================

    def _setup_subscriptions(self):
        """Create heartbeat subscriptions for nodes that have topics."""
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for name, (topic, msg_type_str) in self._heartbeat_msg_types.items():
            try:
                msg_type = self._resolve_msg_type(msg_type_str)
            except Exception as e:
                self.get_logger().warn(
                    f"[{name}] Cannot resolve {msg_type_str}: {e}, "
                    f"skipping subscription"
                )
                continue

            # Capture name in closure
            cb = lambda msg, n=name: self._heartbeat(n)
            self.create_subscription(msg_type, topic, cb, best_effort)
            self.get_logger().info(f"[{name}] Subscribed to {topic} ({msg_type_str})")

    @staticmethod
    def _resolve_msg_type(type_string):
        """Resolve 'nav_msgs.msg.Odometry' to the actual Python class."""
        parts = type_string.rsplit(".", 1)
        if len(parts) != 2:
            raise ValueError(f"Invalid type: {type_string}")
        module = importlib.import_module(parts[0])
        return getattr(module, parts[1])

    def _heartbeat(self, name: str):
        node = self._nodes.get(name)
        if node is None or node.status == NodeStatus.OFF:
            return

        now = time.monotonic()
        node.last_heartbeat = now

        if not node.ever_seen:
            node.ever_seen = True
            node.status = NodeStatus.RUNNING
            elapsed = now - node.boot_start
            self.get_logger().info(
                f"[{name}] First heartbeat after {elapsed:.1f}s — RUNNING"
            )
        elif node.status in (NodeStatus.STALE, NodeStatus.RESTARTING):
            node.status = NodeStatus.RUNNING
            node.last_error = ""
            self.get_logger().info(f"[{name}] Recovered — RUNNING")

    def _nav_needed_cb(self, msg: Bool):
        self._nav_needed = msg.data

    # ==================================================================
    # Main tick (1Hz)
    # ==================================================================

    def _tick(self):
        state = self._supervisor_state

        if state == SupervisorState.IDLE:
            self._tick_idle()
        elif state == SupervisorState.ACTIVATING:
            pass  # Activation runs in blocking method, won't reach here
        elif state == SupervisorState.ACTIVE:
            self._tick_active()
        elif state == SupervisorState.DEACTIVATING:
            pass  # Deactivation runs in blocking method

        # Always monitor always-on nodes
        self._check_always_on()

    def _tick_idle(self):
        if self._nav_needed:
            self.get_logger().info("Nav needed, activating nav stack")
            self._supervisor_state = SupervisorState.ACTIVATING
            self._activate_nav_stack()

    def _tick_active(self):
        if self._shutdown_in_progress:
            return

        # Check if nav no longer needed
        if not self._nav_needed:
            self.get_logger().info("Nav no longer needed, deactivating")
            self._supervisor_state = SupervisorState.DEACTIVATING
            self._deactivate_nav_stack()
            return

        # Normal health monitoring for on-demand nodes
        now = time.monotonic()
        for name, node in self._nodes.items():
            if node.always_on or node.status == NodeStatus.OFF:
                continue
            if node.status == NodeStatus.SHUTDOWN:
                continue

            if node.pid_only:
                self._check_pid_only(name, node, now)
            else:
                self._check_topic_based(name, node, now)

        self._update_system_state()

    def _check_always_on(self):
        """Monitor always-on nodes regardless of supervisor state."""
        now = time.monotonic()
        for name, node in self._nodes.items():
            if not node.always_on:
                continue
            if node.status == NodeStatus.OFF:
                continue
            self._check_topic_based(name, node, now)

    # ==================================================================
    # Activation sequence
    # ==================================================================

    def _activate_nav_stack(self):
        """Start nav stack nodes sequentially with health gates."""
        self.get_logger().info("=== ACTIVATING NAV STACK ===")
        self._system_state = SystemState.NOMINAL
        self._shutdown_in_progress = False

        for name, wait_type, wait_topic, timeout in self.STARTUP_SEQUENCE:
            node = self._nodes[name]
            node.reset()

            self.get_logger().info(f"[{name}] Starting...")

            # Start the process
            if not self._start_node(node):
                self.get_logger().error(
                    f"[{name}] Failed to start, aborting activation"
                )
                self._abort_activation(name)
                return

            node.status = NodeStatus.WAITING
            node.boot_start = time.monotonic()

            # Wait for health gate
            if wait_type == "topic":
                if not self._wait_for_heartbeat(name, node, timeout):
                    self.get_logger().error(
                        f"[{name}] No heartbeat within {timeout}s, aborting activation"
                    )
                    self._abort_activation(name)
                    return
            elif wait_type == "pid":
                if not self._wait_for_pid(name, node, timeout):
                    self.get_logger().error(
                        f"[{name}] PID not found within {timeout}s, aborting activation"
                    )
                    self._abort_activation(name)
                    return

            self.get_logger().info(f"[{name}] Ready")

        # All nodes up
        self._autonomous_enabled = True
        self._supervisor_state = SupervisorState.ACTIVE
        self._nav_ready_pub.publish(Empty())

        self.get_logger().info("=== NAV STACK ACTIVE ===")

    def _wait_for_heartbeat(self, name, node, timeout):
        """Block until node receives first heartbeat or timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if node.ever_seen and node.status == NodeStatus.RUNNING:
                return True
            time.sleep(0.5)
        return False

    def _wait_for_pid(self, name, node, timeout):
        """Block until node PID found or timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pid = self._find_pid(node)
            if pid is not None:
                node.ever_seen = True
                node.status = NodeStatus.RUNNING
                node.last_heartbeat = time.monotonic()
                self.get_logger().info(f"[{name}] Process found (PID {pid})")
                return True
            time.sleep(0.5)
        return False

    def _abort_activation(self, failed_name):
        """Activation failed — shut down what was started, return to IDLE."""
        self.get_logger().error(f"=== ACTIVATION ABORTED (failed: {failed_name}) ===")
        self._send_estop()

        # Shut down in reverse order up to and including the failed node
        for name in reversed(self.SHUTDOWN_SEQUENCE):
            node = self._nodes[name]
            if node.status != NodeStatus.OFF:
                self.get_logger().info(f"[{name}] Stopping (abort cleanup)")
                self._kill_node(node)
                node.reset()

        self._autonomous_enabled = False
        self._supervisor_state = SupervisorState.IDLE
        self._system_state = SystemState.DEGRADED

        self.get_logger().warn("Returned to IDLE after failed activation")

    # ==================================================================
    # Deactivation sequence
    # ==================================================================

    def _deactivate_nav_stack(self):
        """Shut down nav stack nodes sequentially."""
        self.get_logger().info("=== DEACTIVATING NAV STACK ===")

        # Signal job_manager that nav is going away
        self._nav_shutdown_pub.publish(Empty())
        self._autonomous_enabled = False

        for name in self.SHUTDOWN_SEQUENCE:
            node = self._nodes[name]
            if node.status == NodeStatus.OFF:
                continue

            self.get_logger().info(f"[{name}] Stopping...")
            self._kill_node(node)
            node.reset()
            self.get_logger().info(f"[{name}] Stopped")

        self._supervisor_state = SupervisorState.IDLE
        self._system_state = SystemState.NOMINAL
        self._shutdown_in_progress = False

        self.get_logger().info("=== NAV STACK IDLE ===")

    # ==================================================================
    # Health checks (reused from v2)
    # ==================================================================

    def _check_pid_only(self, name, node, now):
        pid = self._find_pid(node)
        pid_alive = pid is not None

        if not node.ever_seen:
            if pid_alive:
                node.ever_seen = True
                node.status = NodeStatus.RUNNING
                node.last_heartbeat = now
            else:
                boot_elapsed = now - node.boot_start
                if boot_elapsed > node.boot_timeout:
                    node.status = NodeStatus.DEAD
                    node.last_error = f"Process never started ({boot_elapsed:.0f}s)"
                    self.get_logger().error(f"[{name}] {node.last_error}")
                    self._handle_failure(name)
        else:
            if pid_alive:
                node.last_heartbeat = now
                if node.status in (NodeStatus.STALE, NodeStatus.RESTARTING):
                    node.status = NodeStatus.RUNNING
                    node.last_error = ""
                    self.get_logger().info(f"[{name}] Recovered — RUNNING")
            else:
                if node.status == NodeStatus.RESTARTING:
                    if now - node.last_restart_time < node.restart_cooldown:
                        return
                if node.status != NodeStatus.DEAD:
                    node.status = NodeStatus.DEAD
                    node.last_error = "Process died"
                    self.get_logger().error(f"[{name}] DEAD — process gone")
                    self._handle_failure(name)

    def _check_topic_based(self, name, node, now):
        if not node.ever_seen:
            boot_elapsed = now - node.boot_start
            if boot_elapsed > node.boot_timeout:
                node.status = NodeStatus.DEAD
                node.last_error = (
                    f"Never started ({boot_elapsed:.0f}s > "
                    f"{node.boot_timeout:.0f}s boot timeout)"
                )
                self.get_logger().error(f"[{name}] {node.last_error}")
                self._handle_failure(name)
            return

        if node.status == NodeStatus.RESTARTING:
            if now - node.last_restart_time > node.restart_cooldown:
                node.error_count += 1
                node.last_error = "No heartbeat after restart"
                self.get_logger().warn(f"[{name}] {node.last_error}")
                node.status = NodeStatus.STALE
            else:
                return

        age = now - node.last_heartbeat

        if age > node.heartbeat_timeout:
            if node.status == NodeStatus.RUNNING:
                node.status = NodeStatus.STALE
                node.last_error = f"Heartbeat timeout ({age:.1f}s)"
                self.get_logger().warn(f"[{name}] STALE — no heartbeat for {age:.1f}s")
            if age > node.heartbeat_timeout * 2:
                if node.status != NodeStatus.DEAD:
                    node.status = NodeStatus.DEAD
                    node.last_error = f"Dead ({age:.1f}s)"
                    self.get_logger().error(f"[{name}] DEAD")
                    self._handle_failure(name)

    # ==================================================================
    # Failure handling
    # ==================================================================

    def _handle_failure(self, name):
        node = self._nodes.get(name)
        if node is None:
            return

        # Always-on nodes: auto-restart in any state
        if node.always_on:
            self._auto_restart(name)
            return

        # On-demand nodes: only handle in ACTIVE state
        if self._supervisor_state != SupervisorState.ACTIVE:
            return

        if node.category == Category.CAT3_CASCADE_SHUTDOWN:
            self._cascade_shutdown(name)
        elif node.category == Category.CAT2_ABORT_RESTART:
            self._abort_and_restart(name)
        elif node.category == Category.CAT1_AUTO_RESTART:
            self._auto_restart(name)

    def _auto_restart(self, name):
        node = self._nodes[name]
        if node.restart_count >= node.max_restarts:
            self.get_logger().error(f"[{name}] Max restarts reached")
            return

        now = time.monotonic()
        if now - node.last_restart_time < node.restart_cooldown:
            return

        self.get_logger().info(
            f"[{name}] Cat1 restart ({node.restart_count + 1}/{node.max_restarts})"
        )
        self._restart_node(node)

    def _abort_and_restart(self, name):
        node = self._nodes[name]
        if node.restart_count >= node.max_restarts:
            self.get_logger().error(f"[{name}] Max restarts reached — DEGRADED")
            self._autonomous_enabled = False
            return

        now = time.monotonic()
        if now - node.last_restart_time < node.restart_cooldown:
            return

        self.get_logger().warn(f"[{name}] Cat2 — e-stop and restarting")
        self._send_estop()
        self._restart_node(node)

    def _cascade_shutdown(self, trigger_name):
        """Cat3: E-stop, shutdown autonomous systems, keep teleop + motors."""
        if self._shutdown_in_progress:
            return

        self._shutdown_in_progress = True
        self._autonomous_enabled = False
        self._system_state = SystemState.LOCALIZATION_LOST

        self.get_logger().fatal(f"=== CATEGORY 3: {trigger_name} FAILED ===")

        self._send_estop()
        self._send_estop()

        # Signal job_manager
        self._nav_shutdown_pub.publish(Empty())

        # Kill all on-demand nodes
        for name in self.SHUTDOWN_SEQUENCE:
            node = self._nodes[name]
            if node.status not in (NodeStatus.OFF, NodeStatus.SHUTDOWN):
                self.get_logger().warn(f"[{name}] Shutting down (cascade)")
                self._kill_node(node)
                node.status = NodeStatus.SHUTDOWN

        self.get_logger().fatal(
            "Autonomous systems shutdown. Teleop still available.\n"
            "Waiting for jobs to stop, then return to IDLE."
        )

        self._publish_health()

        # Return to IDLE — job_manager will see nav_shutdown and
        # eventually active_jobs will hit 0
        self._supervisor_state = SupervisorState.IDLE

    # ==================================================================
    # Node start / restart / kill
    # ==================================================================

    def _start_node(self, node):
        """Start a node process. Returns True if process started."""
        if not node.start_cmd:
            self.get_logger().warn(f"[{node.name}] No start command")
            return False

        try:
            proc = subprocess.Popen(
                node.start_cmd,
                env=os.environ.copy(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            node.pid = proc.pid
            self.get_logger().info(f"[{node.name}] Started (PID {proc.pid})")
            return True
        except Exception as e:
            self.get_logger().error(f"[{node.name}] Start failed: {e}")
            node.last_error = f"Start failed: {e}"
            return False

    def _restart_node(self, node):
        if not node.start_cmd:
            self.get_logger().warn(f"[{node.name}] No start command")
            return

        self._kill_node(node)

        try:
            proc = subprocess.Popen(
                node.start_cmd,
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

    def _kill_node(self, node):
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
        any_dead = any(
            n.status == NodeStatus.DEAD
            for n in self._nodes.values()
            if n.status != NodeStatus.OFF
        )
        self._system_state = SystemState.DEGRADED if any_dead else SystemState.NOMINAL

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
            if node.status in (NodeStatus.SHUTDOWN, NodeStatus.OFF):
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
                        node.cpu_percent = (
                            (ticks - node._prev_cpu_ticks) / clk_hz / dt
                        ) * 100.0
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

    def _find_pid(self, node):
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
        msg.supervisor_state = int(self._supervisor_state)
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

            if node.status == NodeStatus.OFF:
                nh.last_heartbeat_age = 0.0
                nh.message = "Off"
            elif node.ever_seen:
                nh.last_heartbeat_age = float(now - node.last_heartbeat)
                nh.message = self._label(node.status)
            else:
                nh.last_heartbeat_age = float(now - node.boot_start)
                nh.message = f"Waiting ({now - node.boot_start:.0f}s)"

            msg.nodes.append(nh)

        state_labels = {
            SupervisorState.IDLE: "IDLE",
            SupervisorState.ACTIVATING: "ACTIVATING",
            SupervisorState.ACTIVE: "ACTIVE",
            SupervisorState.DEACTIVATING: "DEACTIVATING",
        }
        sv_label = state_labels.get(self._supervisor_state, "UNKNOWN")

        if self._system_state == SystemState.NOMINAL:
            msg.system_message = f"{sv_label} — All systems nominal"
        elif self._system_state == SystemState.DEGRADED:
            dead = [n.name for n in self._nodes.values() if n.status == NodeStatus.DEAD]
            msg.system_message = f"{sv_label} — Degraded: {', '.join(dead)}"
        elif self._system_state == SystemState.LOCALIZATION_LOST:
            msg.system_message = f"{sv_label} — LOCALIZATION LOST, teleop available"

        self._health_pub.publish(msg)

    @staticmethod
    def _label(status):
        return {
            NodeStatus.WAITING: "Starting",
            NodeStatus.RUNNING: "OK",
            NodeStatus.STALE: "Stale",
            NodeStatus.DEAD: "Dead",
            NodeStatus.RESTARTING: "Restarting",
            NodeStatus.SHUTDOWN: "Shutdown",
            NodeStatus.OFF: "Off",
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
