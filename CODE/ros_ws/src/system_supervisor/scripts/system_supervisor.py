#!/usr/bin/env python3
"""
System Supervisor Node (v4.1)

State machine with on-demand node lifecycle management.

States:
  BOOTING     — Waiting for all always-on nodes to come up.
                Uses progress-based detection (stall timeout).
                Publishes /system/ready when all always-on nodes are RUNNING.
  IDLE        — All always-on nodes running. Watching nav_needed.
  ACTIVATING  — Bringing up nav stack sequentially with health gates.
  ACTIVE      — Full monitoring, categorized recovery.
  DEACTIVATING — Shutting down nav stack sequentially.

Boot detection:
  Instead of fixed boot timeouts, tracks progress milestones:
    - PID found = progress (reset stall timer)
    - Heartbeat received = boot complete
  If no progress for stall_timeout seconds = node stuck.
  Absolute max_boot_timeout as safety net.

Dynamic args:
  Nodes can specify dynamic_args in config to read values from files
  at start/restart time. Format: "file:<filepath>:<format_string>"
  Example: "file:/tmp/current_tile.txt:map:=tile{}.yaml"

Publishes:
  /robot_health (RobotHealth)     — full system status at 1Hz
  /system/ready (Empty)           — all always-on nodes are up
  /system/nav_ready (Empty)       — nav stack is up
  /system/nav_shutdown (Empty)    — nav stack going down

Subscribes:
  /system/nav_needed (Bool)       — from job_manager
"""

import importlib
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
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
    BOOTING = 0
    IDLE = 1
    ACTIVATING = 2
    ACTIVE = 3
    DEACTIVATING = 4


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
    OFF = 6


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
    dynamic_args: List[str] = field(default_factory=list)

    # Monitoring mode
    pid_only: bool = False
    always_on: bool = False

    # Thresholds
    stall_timeout: float = 15.0  # No progress for this long = stuck
    max_boot_timeout: float = 120.0  # Absolute safety net
    heartbeat_timeout: float = 3.0
    max_restarts: int = 3
    restart_cooldown: float = 10.0

    # Runtime state
    status: int = NodeStatus.OFF
    ever_seen: bool = False
    pid_found: bool = False  # Progress milestone: PID appeared
    last_heartbeat: float = 0.0
    last_progress: float = 0.0  # Last time any progress was detected
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
        self.status = NodeStatus.OFF
        self.ever_seen = False
        self.pid_found = False
        self.last_heartbeat = 0.0
        self.last_progress = 0.0
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

    def mark_progress(self):
        self.last_progress = time.monotonic()


# ============================================================================
# System Supervisor
# ============================================================================


class SystemSupervisor(Node):
    STARTUP_SEQUENCE = []
    SHUTDOWN_SEQUENCE = []

    def __init__(self):
        super().__init__("system_supervisor")

        self._supervisor_state = SupervisorState.BOOTING
        self._system_state = SystemState.NOMINAL
        self._autonomous_enabled = False
        self._shutdown_in_progress = False
        self._nav_needed = False
        self._system_ready_published = False

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
                node.last_progress = now  # Initial progress = supervisor started
                node.status = NodeStatus.WAITING

        # Publishers
        self._estop_pub = self.create_publisher(
            Twist,
            "/cmd_vel_estop",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._health_pub = self.create_publisher(RobotHealth, "/robot_health", qos)
        self._ready_pub = self.create_publisher(Empty, "/system/ready", qos)
        self._nav_ready_pub = self.create_publisher(Empty, "/system/nav_ready", qos)
        self._nav_shutdown_pub = self.create_publisher(
            Empty, "/system/nav_shutdown", qos
        )

        # Subscriptions
        self._setup_subscriptions()

        self.create_subscription(
            Bool,
            "/system/nav_needed",
            self._nav_needed_cb,
            qos,
        )

        self.declare_parameter("boot_delay", 60.0)
        self._boot_delay = self.get_parameter("boot_delay").value
        self._boot_delay_start = time.monotonic()

        self.get_logger().info(
            f"Supervisor waiting {self._boot_delay}s for nodes to spawn..."
        )

        # Timers
        self.create_timer(1.0, self._tick)
        self.create_timer(2.0, self._resource_check)
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f"Supervisor started in BOOTING mode, {self._cpu_count} CPU cores"
        )

    # ==================================================================
    # Config loading
    # ==================================================================

    def _configure_nodes(self):
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
                dynamic_args=cfg.get("dynamic_args", []),
                pid_only=cfg.get("pid_only", False),
                always_on=True,
                stall_timeout=cfg.get("stall_timeout", 15.0),
                max_boot_timeout=cfg.get("max_boot_timeout", 120.0),
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
                dynamic_args=cfg.get("dynamic_args", []),
                pid_only=cfg.get("pid_only", False),
                always_on=False,
                stall_timeout=cfg.get("stall_timeout", 15.0),
                max_boot_timeout=cfg.get("max_boot_timeout", 120.0),
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
                    cfg.get("stall_timeout", 15.0),
                    cfg.get("max_boot_timeout", 120.0),
                )
            )

        startup_entries.sort(key=lambda x: x[0])
        self.STARTUP_SEQUENCE = [
            (name, wait_type, topic, stall_t, max_t)
            for _, name, wait_type, topic, stall_t, max_t in startup_entries
        ]
        self.SHUTDOWN_SEQUENCE = [
            name for _, name, _, _, _, _ in reversed(startup_entries)
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
    # Subscriptions
    # ==================================================================

    def _setup_subscriptions(self):
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
                    f"[{name}] Cannot resolve {msg_type_str}: {e}, skipping"
                )
                continue

            cb = lambda msg, n=name: self._heartbeat(n)
            self.create_subscription(msg_type, topic, cb, best_effort)
            self.get_logger().info(f"[{name}] Subscribed to {topic} ({msg_type_str})")

    @staticmethod
    def _resolve_msg_type(type_string):
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
        node.mark_progress()

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
        # Wait for boot delay before doing anything
        if time.monotonic() - self._boot_delay_start < self._boot_delay:
            return

        state = self._supervisor_state

        if state == SupervisorState.BOOTING:
            self._tick_booting()
        elif state == SupervisorState.IDLE:
            self._tick_idle()
        elif state == SupervisorState.ACTIVE:
            self._tick_active()

        # Always monitor always-on nodes (except during BOOTING — handled separately)
        if state != SupervisorState.BOOTING:
            self._check_always_on()

    def _tick_booting(self):
        """Check if all always-on nodes are up using progress-based detection."""
        now = time.monotonic()
        all_running = True

        for name, node in self._nodes.items():
            if not node.always_on:
                continue

            if node.status == NodeStatus.RUNNING:
                continue

            all_running = False

            # Check for progress: PID appearing is a milestone
            if not node.pid_found:
                pid = self._find_pid(node)
                if pid is not None:
                    node.pid_found = True
                    node.mark_progress()
                    self.get_logger().info(f"[{name}] PID found ({pid}) — progress")

            # For pid_only nodes, PID found = RUNNING
            if node.pid_only and node.pid_found and not node.ever_seen:
                node.ever_seen = True
                node.status = NodeStatus.RUNNING
                node.last_heartbeat = now
                elapsed = now - node.boot_start
                self.get_logger().info(
                    f"[{name}] PID-only node ready after {elapsed:.1f}s — RUNNING"
                )
                continue

            # Check stall: no progress for stall_timeout
            stall_elapsed = now - node.last_progress
            if stall_elapsed > node.stall_timeout:
                node.status = NodeStatus.DEAD
                node.last_error = f"Stalled — no progress for {stall_elapsed:.0f}s"
                self.get_logger().error(f"[{name}] {node.last_error}")
                self._handle_failure(name)

            # Check absolute timeout
            boot_elapsed = now - node.boot_start
            if boot_elapsed > node.max_boot_timeout:
                node.status = NodeStatus.DEAD
                node.last_error = (
                    f"Max boot timeout ({boot_elapsed:.0f}s > "
                    f"{node.max_boot_timeout:.0f}s)"
                )
                self.get_logger().error(f"[{name}] {node.last_error}")
                self._handle_failure(name)

        if all_running:
            self._supervisor_state = SupervisorState.IDLE
            self._system_state = SystemState.NOMINAL

            if not self._system_ready_published:
                self._ready_pub.publish(Empty())
                self._system_ready_published = True

            always_names = [
                n
                for n, nd in self._nodes.items()
                if nd.always_on and nd.status == NodeStatus.RUNNING
            ]
            self.get_logger().info(f"=== ALL ALWAYS-ON NODES READY: {always_names} ===")
            self.get_logger().info("Supervisor transitioning to IDLE")

    def _tick_idle(self):
        if self._nav_needed:
            self.get_logger().info("Nav needed, activating nav stack")
            self._supervisor_state = SupervisorState.ACTIVATING
            self._activate_nav_stack()

    def _tick_active(self):
        if self._shutdown_in_progress:
            return

        if not self._nav_needed:
            self.get_logger().info("Nav no longer needed, deactivating")
            self._supervisor_state = SupervisorState.DEACTIVATING
            self._deactivate_nav_stack()
            return

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
        now = time.monotonic()
        for name, node in self._nodes.items():
            if not node.always_on:
                continue
            if node.status == NodeStatus.OFF:
                continue

            if node.pid_only:
                self._check_pid_only(name, node, now)
            else:
                self._check_topic_based(name, node, now)

    # ==================================================================
    # Activation sequence (progress-based)
    # ==================================================================

    def _activate_nav_stack(self):
        self.get_logger().info("=== ACTIVATING NAV STACK ===")
        self._system_state = SystemState.NOMINAL
        self._shutdown_in_progress = False

        for name, wait_type, wait_topic, stall_t, max_t in self.STARTUP_SEQUENCE:
            node = self._nodes[name]
            node.reset()

            self.get_logger().info(f"[{name}] Starting...")

            if not self._start_node(node):
                self.get_logger().error(
                    f"[{name}] Failed to start, aborting activation"
                )
                self._abort_activation(name)
                return

            node.status = NodeStatus.WAITING
            node.boot_start = time.monotonic()
            node.last_progress = time.monotonic()

            if wait_type == "topic":
                if not self._wait_for_heartbeat_progress(name, node, stall_t, max_t):
                    self.get_logger().error(
                        f"[{name}] Boot failed, aborting activation"
                    )
                    self._abort_activation(name)
                    return
            elif wait_type == "pid":
                if not self._wait_for_pid_progress(name, node, stall_t, max_t):
                    self.get_logger().error(
                        f"[{name}] Boot failed, aborting activation"
                    )
                    self._abort_activation(name)
                    return

            self.get_logger().info(f"[{name}] Ready")

        self._autonomous_enabled = True
        self._supervisor_state = SupervisorState.ACTIVE
        self._nav_ready_pub.publish(Empty())

        self.get_logger().info("=== NAV STACK ACTIVE ===")

    def _wait_for_heartbeat_progress(self, name, node, stall_timeout, max_timeout):
        """Wait for heartbeat using progress-based detection."""
        deadline = time.monotonic() + max_timeout

        while time.monotonic() < deadline:
            # Check if boot complete
            if node.ever_seen and node.status == NodeStatus.RUNNING:
                return True

            # Check for PID progress
            if not node.pid_found:
                pid = self._find_pid(node)
                if pid is not None:
                    node.pid_found = True
                    node.mark_progress()
                    self.get_logger().info(f"[{name}] PID found — progress")

            # Check stall
            stall = time.monotonic() - node.last_progress
            if stall > stall_timeout:
                node.last_error = f"Stalled — no progress for {stall:.0f}s"
                self.get_logger().error(f"[{name}] {node.last_error}")
                return False

            time.sleep(0.5)

        node.last_error = f"Max boot timeout ({max_timeout:.0f}s)"
        return False

    def _wait_for_pid_progress(self, name, node, stall_timeout, max_timeout):
        """Wait for PID using progress-based detection."""
        deadline = time.monotonic() + max_timeout

        while time.monotonic() < deadline:
            pid = self._find_pid(node)
            if pid is not None:
                node.ever_seen = True
                node.pid_found = True
                node.status = NodeStatus.RUNNING
                node.last_heartbeat = time.monotonic()
                node.mark_progress()
                self.get_logger().info(f"[{name}] Process found (PID {pid})")
                return True

            # Check stall
            stall = time.monotonic() - node.last_progress
            if stall > stall_timeout:
                node.last_error = f"Stalled — PID not found for {stall:.0f}s"
                self.get_logger().error(f"[{name}] {node.last_error}")
                return False

            time.sleep(0.5)

        node.last_error = f"Max boot timeout ({max_timeout:.0f}s)"
        return False

    def _abort_activation(self, failed_name):
        self.get_logger().error(f"=== ACTIVATION ABORTED (failed: {failed_name}) ===")
        self._send_estop()

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
        self.get_logger().info("=== DEACTIVATING NAV STACK ===")

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
    # Health checks
    # ==================================================================

    def _check_pid_only(self, name, node, now):
        pid = self._find_pid(node)
        pid_alive = pid is not None

        if not node.ever_seen:
            if pid_alive:
                node.ever_seen = True
                node.pid_found = True
                node.status = NodeStatus.RUNNING
                node.last_heartbeat = now
                node.mark_progress()
            else:
                # Progress-based: check stall
                stall = now - node.last_progress
                if stall > node.stall_timeout:
                    node.status = NodeStatus.DEAD
                    node.last_error = f"Stalled ({stall:.0f}s no progress)"
                    self.get_logger().error(f"[{name}] {node.last_error}")
                    self._handle_failure(name)
                elif now - node.boot_start > node.max_boot_timeout:
                    node.status = NodeStatus.DEAD
                    node.last_error = f"Max boot timeout exceeded"
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
            # Progress-based: check stall
            stall = now - node.last_progress
            if stall > node.stall_timeout:
                # Check if PID exists as partial progress
                if not node.pid_found:
                    pid = self._find_pid(node)
                    if pid is not None:
                        node.pid_found = True
                        node.mark_progress()
                        return  # Got progress, reset stall

                node.status = NodeStatus.DEAD
                node.last_error = f"Stalled ({stall:.0f}s no progress)"
                self.get_logger().error(f"[{name}] {node.last_error}")
                self._handle_failure(name)
            elif now - node.boot_start > node.max_boot_timeout:
                node.status = NodeStatus.DEAD
                node.last_error = f"Max boot timeout exceeded"
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

        if node.always_on:
            self._auto_restart(name)
            return

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
        if self._shutdown_in_progress:
            return

        self._shutdown_in_progress = True
        self._autonomous_enabled = False
        self._system_state = SystemState.LOCALIZATION_LOST

        self.get_logger().fatal(f"=== CATEGORY 3: {trigger_name} FAILED ===")

        self._send_estop()
        self._send_estop()

        self._nav_shutdown_pub.publish(Empty())

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
        self._supervisor_state = SupervisorState.IDLE

    # ==================================================================
    # Node start / restart / kill
    # ==================================================================

    def _build_command(self, node) -> List[str]:
        """
        Build the full command with dynamic args resolved.

        Dynamic args format: "file:<filepath>:<format_string>"
        - Reads content from <filepath>
        - Strips whitespace
        - Substitutes into <format_string> where {} appears
        - Appends result to command

        Example:
            dynamic_args: ["file:/tmp/current_tile.txt:map:=tile{}.yaml"]
            If file contains "3", appends "map:=tile3.yaml" to command
        """
        cmd = list(node.start_cmd)

        for arg in node.dynamic_args:
            if arg.startswith("file:"):
                parts = arg.split(":", 2)
                if len(parts) != 3:
                    self.get_logger().warn(
                        f"[{node.name}] Invalid dynamic_arg format: {arg}"
                    )
                    continue

                _, filepath, format_str = parts

                try:
                    value = Path(filepath).read_text().strip()
                    resolved = format_str.format(value)
                    cmd.append(resolved)
                    self.get_logger().debug(
                        f"[{node.name}] Dynamic arg: {filepath} -> {resolved}"
                    )
                except FileNotFoundError:
                    self.get_logger().warn(
                        f"[{node.name}] Dynamic arg file not found: {filepath}, skipping"
                    )
                except Exception as e:
                    self.get_logger().warn(f"[{node.name}] Dynamic arg error: {e}")
            else:
                # Unknown format, just append as-is
                cmd.append(arg)

        return cmd

    def _start_node(self, node):
        if not node.start_cmd:
            self.get_logger().warn(f"[{node.name}] No start command")
            return False

        cmd = self._build_command(node)

        try:
            proc = subprocess.Popen(
                cmd,
                env=os.environ.copy(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            node.pid = proc.pid
            self.get_logger().info(f"[{node.name}] Started (PID {proc.pid}): {' '.join(cmd)}")
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

        cmd = self._build_command(node)

        try:
            proc = subprocess.Popen(
                cmd,
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
            node.last_progress = time.monotonic()
            self.get_logger().info(f"[{node.name}] Restarted (PID {proc.pid}): {' '.join(cmd)}")
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
            SupervisorState.BOOTING: "BOOTING",
            SupervisorState.IDLE: "IDLE",
            SupervisorState.ACTIVATING: "ACTIVATING",
            SupervisorState.ACTIVE: "ACTIVE",
            SupervisorState.DEACTIVATING: "DEACTIVATING",
        }
        sv_label = state_labels.get(self._supervisor_state, "UNKNOWN")

        if self._supervisor_state == SupervisorState.BOOTING:
            waiting = [
                n.name
                for n in self._nodes.values()
                if n.always_on and n.status != NodeStatus.RUNNING
            ]
            msg.system_message = f"{sv_label} — Waiting for: {', '.join(waiting)}"
        elif self._system_state == SystemState.NOMINAL:
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
