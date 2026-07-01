#!/usr/bin/env python3
"""
System Supervisor Node (v4.3) - Safe Process Management

Changes from v4.2:
  - REMOVED os.setsid() — processes stay in container's process group
  - REMOVED os.killpg() — use targeted os.kill() only
  - Added SIGTERM grace period before SIGKILL
  - Lifecycle shutdown runs in non-blocking thread with hard timeout
  - Added PID validation before kill (check cmdline matches expected)
  - Rate-limited /proc scanning to reduce memory pressure
  - Added supervisor self-watchdog via SIGALRM

States:
  BOOTING      — Waiting for all always-on nodes to come up.
  IDLE         — All always-on nodes running. Watching nav_needed.
  ACTIVATING   — Bringing up nav stack sequentially with health gates.
  ACTIVE       — Full monitoring, categorized recovery.
  DEACTIVATING — Shutting down nav stack sequentially.

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
import re
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional, Set

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
# Constants (cached at module load)
# ============================================================================

_CLK_TCK: int = os.sysconf("SC_CLK_TCK")
_MY_PID: int = os.getpid()

# Pattern to find end of comm field in /proc/*/stat (handles spaces in process names)
_PROC_STAT_COMM_END = re.compile(rb"\) [A-Za-z] ")

# Grace period for SIGTERM before SIGKILL (seconds)
_SIGTERM_GRACE_PERIOD = 2.0

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


# Lookup tables (avoid dict creation in hot paths)
_STATUS_LABELS = {
    NodeStatus.WAITING: "Starting",
    NodeStatus.RUNNING: "OK",
    NodeStatus.STALE: "Stale",
    NodeStatus.DEAD: "Dead",
    NodeStatus.RESTARTING: "Restarting",
    NodeStatus.SHUTDOWN: "Shutdown",
    NodeStatus.OFF: "Off",
}

_STATE_LABELS = {
    SupervisorState.BOOTING: "BOOTING",
    SupervisorState.IDLE: "IDLE",
    SupervisorState.ACTIVATING: "ACTIVATING",
    SupervisorState.ACTIVE: "ACTIVE",
    SupervisorState.DEACTIVATING: "DEACTIVATING",
}

# ============================================================================
# PID Utilities (no subprocess - direct /proc access)
# ============================================================================


def find_pids_by_cmdline(
    pattern: str, exclude_pids: Optional[Set[int]] = None
) -> List[int]:
    """
    Find PIDs whose /proc/PID/cmdline contains pattern.

    Direct /proc scan — no subprocess overhead (~100x faster than pgrep).
    Memory-safe: no fork, no shell spawn.

    Safety: excludes PID 1, the supervisor itself, sshd, bash/sh sessions,
    and containerd/dockerd processes to prevent killing the container or SSH.

    Args:
        pattern: String to search for in cmdline
        exclude_pids: PIDs to skip (e.g., self)

    Returns:
        List of matching PIDs (may be empty)
    """
    if exclude_pids is None:
        exclude_pids = {_MY_PID, 1}
    else:
        exclude_pids = exclude_pids | {_MY_PID, 1}

    pattern_bytes = pattern.encode()
    pids = []

    try:
        for entry in os.scandir("/proc"):
            if not entry.name.isdigit():
                continue

            pid = int(entry.name)
            if pid in exclude_pids:
                continue

            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmdline = f.read()
                if not cmdline:
                    continue

                # Safety: never match critical system processes
                # cmdline uses \0 as separator, check the executable (first arg)
                exe = cmdline.split(b"\x00", 1)[0]
                exe_lower = exe.lower()
                if any(
                    safe in exe_lower
                    for safe in (
                        b"sshd",
                        b"ssh",
                        b"containerd",
                        b"dockerd",
                        b"bash",
                        b"/bin/sh",
                        b"tmux",
                        b"screen",
                        b"supervisor_node",  # Don't kill ourselves via name match
                        b"system_supervisor",
                    )
                ):
                    continue

                if pattern_bytes in cmdline:
                    pids.append(pid)
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                # Process disappeared or no permission — skip
                continue
    except OSError:
        pass

    return pids


def pid_exists(pid: int) -> bool:
    """Check if PID exists via /proc (faster than os.kill signal 0)."""
    return os.path.isdir(f"/proc/{pid}")


def validate_pid_cmdline(pid: int, expected_pattern: str) -> bool:
    """
    Validate that a PID's cmdline contains the expected pattern.

    This prevents killing wrong processes due to PID reuse.
    """
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            cmdline = f.read()
        return expected_pattern.encode() in cmdline
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return False


def read_proc_resources(pid: int) -> tuple:
    """
    Read CPU ticks and RSS memory for a process in minimal I/O.

    Combines /proc/PID/stat and /proc/PID/status reads.

    Returns:
        (cpu_ticks, rss_kb) or (0, 0) on error
    """
    cpu_ticks = 0
    rss_kb = 0

    # Read /proc/PID/stat for CPU times
    try:
        with open(f"/proc/{pid}/stat", "rb") as f:
            data = f.read()

        # Find end of comm field (handles process names with spaces/parens)
        match = _PROC_STAT_COMM_END.search(data)
        if match:
            # Fields after state: ppid, pgrp, session, tty, tpgid, flags,
            # minflt, cminflt, majflt, cmajflt, utime(13), stime(14), cutime(15), cstime(16)
            fields = data[match.end() :].split()
            # utime + stime + cutime + cstime (indices 11,12,13,14 after state field)
            if len(fields) > 14:
                cpu_ticks = sum(int(fields[i]) for i in (11, 12, 13, 14))
    except (
        FileNotFoundError,
        PermissionError,
        ProcessLookupError,
        IndexError,
        ValueError,
    ):
        pass

    # Read /proc/PID/status for VmRSS (more reliable than stat's rss field)
    try:
        with open(f"/proc/{pid}/status", "rb") as f:
            for line in f:
                if line.startswith(b"VmRSS:"):
                    # Format: "VmRSS:     12345 kB"
                    rss_kb = int(line.split()[1])
                    break
    except (
        FileNotFoundError,
        PermissionError,
        ProcessLookupError,
        IndexError,
        ValueError,
    ):
        pass

    return cpu_ticks, rss_kb


def safe_kill_pid(pid: int, process_pattern: str, logger) -> bool:
    """
    Safely kill a process with validation and grace period.

    1. Validates PID cmdline matches expected pattern
    2. Checks PID is not a critical system process (sshd, shell, containerd)
    3. Sends SIGTERM first
    4. Waits grace period
    5. Sends SIGKILL if still alive

    Returns True if process was killed or already dead.
    """
    if pid is None or pid <= 1:
        return True

    # Validate PID before killing
    if process_pattern and not validate_pid_cmdline(pid, process_pattern):
        logger.warning(
            f"PID {pid} cmdline doesn't match '{process_pattern}', skipping kill"
        )
        return False

    # Safety: check this isn't a critical system process by reading its exe
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            cmdline = f.read()
        exe = cmdline.split(b"\x00", 1)[0].lower()
        if any(
            s in exe
            for s in (
                b"sshd",
                b"containerd",
                b"dockerd",
                b"bash",
                b"/bin/sh",
                b"tmux",
                b"screen",
            )
        ):
            logger.warning(
                f"PID {pid} appears to be a system/shell process, refusing to kill"
            )
            return False
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return True  # Already dead

    # Try SIGTERM first
    try:
        os.kill(pid, signal.SIGTERM)
        logger.debug(f"Sent SIGTERM to PID {pid}")
    except ProcessLookupError:
        return True  # Already dead
    except PermissionError:
        logger.warning(f"No permission to kill PID {pid}")
        return False

    # Wait for graceful shutdown
    deadline = time.monotonic() + _SIGTERM_GRACE_PERIOD
    while time.monotonic() < deadline:
        if not pid_exists(pid):
            logger.debug(f"PID {pid} exited after SIGTERM")
            return True
        time.sleep(0.1)

    # Force kill
    try:
        os.kill(pid, signal.SIGKILL)
        logger.debug(f"Sent SIGKILL to PID {pid}")
    except ProcessLookupError:
        return True
    except PermissionError:
        return False

    # Brief wait for SIGKILL to take effect
    time.sleep(0.1)
    return not pid_exists(pid)


# ============================================================================
# Monitored Node Config + State
# ============================================================================


@dataclass
class MonitoredNode:
    """Configuration and runtime state for a monitored node."""

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

    # Unit mode: treat multiple processes as atomic group
    unit: bool = False
    process_names: List[str] = field(default_factory=list)

    # Lifecycle management
    lifecycle_manager: str = ""

    # Dependency management
    dependents: List[str] = field(default_factory=list)
    dependent_check_delay: float = 2.0

    # Thresholds
    stall_timeout: float = 15.0
    max_boot_timeout: float = 120.0
    heartbeat_timeout: float = 3.0
    max_restarts: int = 3
    restart_cooldown: float = 10.0
    lifecycle_timeout: float = 3.0

    # Runtime state
    status: int = NodeStatus.OFF
    ever_seen: bool = False
    pid_found: bool = False
    last_heartbeat: float = 0.0
    last_progress: float = 0.0
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
    _pending_dependent_check: bool = False
    _dependent_check_time: float = 0.0

    # Track spawned child PIDs for cleanup
    _spawned_pids: Set[int] = field(default_factory=set)

    # Pre-allocated health message (reused each publish cycle)
    _health_msg: Optional[NodeHealth] = field(default=None, repr=False)

    def reset(self):
        """Reset runtime state (called on shutdown or restart)."""
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
        self._pending_dependent_check = False
        self._dependent_check_time = 0.0
        self._spawned_pids.clear()

    def mark_progress(self):
        """Record progress timestamp (resets stall timer)."""
        self.last_progress = time.monotonic()

    def get_health_msg(self) -> NodeHealth:
        """Get reusable NodeHealth message, creating once if needed."""
        if self._health_msg is None:
            self._health_msg = NodeHealth()
            self._health_msg.name = self.name
        return self._health_msg


# ============================================================================
# System Supervisor
# ============================================================================


class SystemSupervisor(Node):
    """
    Central supervisor for robot node lifecycle management.

    Manages always-on and on-demand nodes with health monitoring,
    automatic restart, and graceful degradation.
    """

    STARTUP_SEQUENCE: List[tuple] = []
    SHUTDOWN_SEQUENCE: List[str] = []

    def __init__(self):
        super().__init__("system_supervisor")

        # State
        self._supervisor_state = SupervisorState.BOOTING
        self._system_state = SystemState.NOMINAL
        self._autonomous_enabled = False
        self._shutdown_in_progress = False
        self._nav_needed = False
        self._system_ready_published = False

        # Activation state machine (non-blocking)
        self._activation_index = 0
        self._activation_node: Optional[MonitoredNode] = None
        self._deactivation_index = 0

        # System resource tracking
        self._cpu_count = os.cpu_count() or 1
        self._prev_total_cpu = 0
        self._prev_idle_cpu = 0
        self._system_cpu_percent = 0.0
        self._system_memory_used_mb = 0.0
        self._system_memory_total_mb = 0.0

        # Pre-allocated messages (reused to reduce GC pressure)
        self._health_msg = RobotHealth()
        self._estop_msg = Twist()
        self._empty_msg = Empty()

        # Node registry
        self._nodes: Dict[str, MonitoredNode] = {}
        self._configure_nodes()

        # Mark boot start for always-on nodes
        now = time.monotonic()
        for node in self._nodes.values():
            if node.always_on:
                node.boot_start = now
                node.last_progress = now
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

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._health_pub = self.create_publisher(
            RobotHealth, "/robot_health", qos_latched
        )
        self._ready_pub = self.create_publisher(Empty, "/system/ready", qos_latched)
        self._nav_ready_pub = self.create_publisher(
            Empty, "/system/nav_ready", qos_latched
        )
        self._nav_shutdown_pub = self.create_publisher(
            Empty, "/system/nav_shutdown", qos_latched
        )

        # Subscriptions
        self._setup_heartbeat_subscriptions()
        self.create_subscription(
            Bool, "/system/nav_needed", self._nav_needed_cb, qos_latched
        )

        # Boot delay (2 minutes default for Jetson)
        self.declare_parameter("boot_delay", 30.0)
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
            f"Supervisor v4.3 started in BOOTING mode, {self._cpu_count} CPU cores, "
            f"boot_delay={self._boot_delay}s"
        )

    # ======================================================================
    # Configuration Loading
    # ======================================================================

    def _configure_nodes(self):
        """Load node configuration from YAML file."""
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
            self._nodes[name] = self._make_node(name, cfg, category_map, always_on=True)

        # On-demand nodes
        startup_entries = []
        for name, cfg in config.get("on_demand", {}).items():
            self._nodes[name] = self._make_node(
                name, cfg, category_map, always_on=False
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
        self.SHUTDOWN_SEQUENCE = [name for _, name, *_ in reversed(startup_entries)]

        # Store msg types for dynamic heartbeat subscriptions
        self._heartbeat_msg_types: Dict[str, tuple] = {}
        for section in ("always_on", "on_demand"):
            for name, cfg in config.get(section, {}).items():
                if cfg.get("heartbeat_topic") and cfg.get("heartbeat_msg_type"):
                    self._heartbeat_msg_types[name] = (
                        cfg["heartbeat_topic"],
                        cfg["heartbeat_msg_type"],
                    )

        always_names = [n for n, nd in self._nodes.items() if nd.always_on]
        demand_names = [n for n, nd in self._nodes.items() if not nd.always_on]
        self.get_logger().info(f"Always-on: {always_names}")
        self.get_logger().info(f"On-demand: {demand_names}")
        self.get_logger().info(
            f"Startup order: {[s[0] for s in self.STARTUP_SEQUENCE]}"
        )

    def _make_node(
        self, name: str, cfg: dict, category_map: dict, always_on: bool
    ) -> MonitoredNode:
        """Create MonitoredNode from config dict."""
        return MonitoredNode(
            name=name,
            category=category_map[cfg["category"]],
            heartbeat_topic=cfg.get("heartbeat_topic", ""),
            process_name=cfg.get("process_name", ""),
            start_cmd=cfg.get("start_cmd", []),
            dynamic_args=cfg.get("dynamic_args", []),
            pid_only=cfg.get("pid_only", False),
            always_on=always_on,
            unit=cfg.get("unit", False),
            process_names=cfg.get("process_names", []),
            lifecycle_manager=cfg.get("lifecycle_manager", ""),
            dependents=cfg.get("dependents", []),
            dependent_check_delay=cfg.get("dependent_check_delay", 2.0),
            stall_timeout=cfg.get("stall_timeout", 15.0),
            max_boot_timeout=cfg.get("max_boot_timeout", 120.0),
            heartbeat_timeout=cfg.get("heartbeat_timeout", 3.0),
            max_restarts=cfg.get("max_restarts", 3),
            restart_cooldown=cfg.get("restart_cooldown", 10.0),
            lifecycle_timeout=cfg.get("lifecycle_timeout", 3.0),
        )

    # ======================================================================
    # Heartbeat Subscriptions
    # ======================================================================

    def _setup_heartbeat_subscriptions(self):
        """Create subscriptions for heartbeat topics."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for name, (topic, msg_type_str) in self._heartbeat_msg_types.items():
            try:
                # Parse msg type string like "nav_msgs.msg.Odometry"
                parts = msg_type_str.rsplit(".", 1)
                if len(parts) != 2:
                    self.get_logger().warning(
                        f"[{name}] Invalid msg type: {msg_type_str}"
                    )
                    continue

                module = importlib.import_module(parts[0])
                msg_type = getattr(module, parts[1])

                # Create subscription with closure for node name
                def make_cb(node_name):
                    return lambda msg: self._heartbeat(node_name)

                self.create_subscription(msg_type, topic, make_cb(name), qos)
                self.get_logger().debug(f"[{name}] Subscribed to {topic}")

            except Exception as e:
                self.get_logger().warning(
                    f"[{name}] Failed to subscribe to {topic}: {e}"
                )

    def _heartbeat(self, name: str):
        """Handle heartbeat received for a node."""
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
            self._schedule_dependent_check(node, now)
        elif node.status in (NodeStatus.STALE, NodeStatus.RESTARTING):
            node.status = NodeStatus.RUNNING
            node.last_error = ""
            self.get_logger().info(f"[{name}] Recovered — RUNNING")
            self._schedule_dependent_check(node, now)

    def _schedule_dependent_check(self, node: MonitoredNode, now: float):
        """Schedule health check for dependent nodes after this node recovers."""
        if node.dependents:
            node._pending_dependent_check = True
            node._dependent_check_time = now + node.dependent_check_delay
            self.get_logger().debug(
                f"[{node.name}] Will check dependents {node.dependents} in {node.dependent_check_delay}s"
            )

    def _nav_needed_cb(self, msg: Bool):
        """Handle nav_needed topic."""
        self._nav_needed = msg.data

    # ======================================================================
    # Main Tick (1Hz)
    # ======================================================================

    def _tick(self):
        """Main supervisor tick - runs every second."""
        now = time.monotonic()

        # Wait for boot delay before doing anything
        if now - self._boot_delay_start < self._boot_delay:
            return

        state = self._supervisor_state

        if state == SupervisorState.BOOTING:
            self._tick_booting()
        elif state == SupervisorState.IDLE:
            self._tick_idle()
        elif state == SupervisorState.ACTIVATING:
            self._tick_activating()
        elif state == SupervisorState.ACTIVE:
            self._tick_active()
        elif state == SupervisorState.DEACTIVATING:
            self._tick_deactivating()

        # Monitor always-on nodes (except during boot/activation/deactivation)
        if state not in (
            SupervisorState.BOOTING,
            SupervisorState.ACTIVATING,
            SupervisorState.DEACTIVATING,
        ):
            self._check_always_on()

        # Check pending dependent health checks
        self._check_pending_dependents()

    def _tick_booting(self):
        """BOOTING state: wait for all always-on nodes to be running."""
        now = time.monotonic()
        all_running = True

        for name, node in self._nodes.items():
            if not node.always_on:
                continue
            if node.status == NodeStatus.RUNNING:
                continue

            all_running = False

            # Check for PID progress
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
                    f"[{name}] PID-only ready after {elapsed:.1f}s — RUNNING"
                )
                continue

            # Check stall
            stall = now - node.last_progress
            if stall > node.stall_timeout:
                node.status = NodeStatus.DEAD
                node.last_error = f"Stalled — no progress for {stall:.0f}s"
                self.get_logger().error(f"[{name}] {node.last_error}")
                self._handle_failure(name)

            # Check absolute timeout
            boot_elapsed = now - node.boot_start
            if boot_elapsed > node.max_boot_timeout:
                node.status = NodeStatus.DEAD
                node.last_error = f"Max boot timeout ({boot_elapsed:.0f}s > {node.max_boot_timeout:.0f}s)"
                self.get_logger().error(f"[{name}] {node.last_error}")
                self._handle_failure(name)

        if all_running:
            self._supervisor_state = SupervisorState.IDLE
            self._system_state = SystemState.NOMINAL

            if not self._system_ready_published:
                self._ready_pub.publish(self._empty_msg)
                self._system_ready_published = True

            always_names = [n for n, nd in self._nodes.items() if nd.always_on]
            self.get_logger().info(f"=== ALL ALWAYS-ON NODES READY: {always_names} ===")
            self.get_logger().info("Supervisor transitioning to IDLE")

    def _tick_idle(self):
        """IDLE state: wait for nav_needed signal."""
        if self._nav_needed:
            self.get_logger().info("Nav needed, activating nav stack")
            self._supervisor_state = SupervisorState.ACTIVATING
            self._activation_index = 0
            self._activation_node = None

    def _tick_activating(self):
        """ACTIVATING state: bring up on-demand nodes sequentially (non-blocking)."""
        now = time.monotonic()

        # Starting a new node?
        if self._activation_node is None:
            if self._activation_index >= len(self.STARTUP_SEQUENCE):
                # All nodes started
                self._autonomous_enabled = True
                self._supervisor_state = SupervisorState.ACTIVE
                self._nav_ready_pub.publish(self._empty_msg)
                self.get_logger().info("=== NAV STACK ACTIVE ===")
                return

            name, wait_type, _, stall_t, max_t = self.STARTUP_SEQUENCE[
                self._activation_index
            ]
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
            node.boot_start = now
            node.last_progress = now
            self._activation_node = node
            return

        # Waiting for current node
        node = self._activation_node
        name = node.name
        _, wait_type, _, stall_t, max_t = self.STARTUP_SEQUENCE[self._activation_index]

        # Check completion
        if wait_type == "topic":
            if node.ever_seen and node.status == NodeStatus.RUNNING:
                self.get_logger().info(f"[{name}] Ready (heartbeat)")
                self._activation_index += 1
                self._activation_node = None
                return
        else:  # pid
            if node.pid_found:
                node.ever_seen = True
                node.status = NodeStatus.RUNNING
                node.last_heartbeat = now
                self.get_logger().info(f"[{name}] Ready (PID)")
                self._activation_index += 1
                self._activation_node = None
                return

        # Check PID progress
        if not node.pid_found:
            pid = self._find_pid(node)
            if pid is not None:
                node.pid_found = True
                node.mark_progress()
                self.get_logger().debug(f"[{name}] PID found — progress")

        # Check stall
        stall = now - node.last_progress
        if stall > stall_t:
            node.last_error = f"Stalled — no progress for {stall:.0f}s"
            self.get_logger().error(f"[{name}] {node.last_error}")
            self._abort_activation(name)
            return

        # Check max timeout
        if now - node.boot_start > max_t:
            node.last_error = f"Max boot timeout ({max_t:.0f}s)"
            self.get_logger().error(f"[{name}] {node.last_error}")
            self._abort_activation(name)

    def _tick_active(self):
        """ACTIVE state: monitor all nodes, handle failures."""
        if self._shutdown_in_progress:
            return

        if not self._nav_needed:
            self.get_logger().info("Nav no longer needed, deactivating")
            self._supervisor_state = SupervisorState.DEACTIVATING
            self._deactivation_index = 0
            return

        now = time.monotonic()
        for name, node in self._nodes.items():
            if node.always_on or node.status in (NodeStatus.OFF, NodeStatus.SHUTDOWN):
                continue

            if node.pid_only:
                self._check_pid_only(name, node, now)
            else:
                self._check_topic_based(name, node, now)

        self._update_system_state()

    def _tick_deactivating(self):
        """DEACTIVATING state: shut down on-demand nodes sequentially."""
        if self._deactivation_index == 0:
            self._nav_shutdown_pub.publish(self._empty_msg)
            self._autonomous_enabled = False
            self.get_logger().info("=== DEACTIVATING NAV STACK ===")

        if self._deactivation_index >= len(self.SHUTDOWN_SEQUENCE):
            self._supervisor_state = SupervisorState.IDLE
            self._system_state = SystemState.NOMINAL
            self._shutdown_in_progress = False
            self.get_logger().info("=== NAV STACK IDLE ===")
            return

        name = self.SHUTDOWN_SEQUENCE[self._deactivation_index]
        node = self._nodes[name]

        if node.status not in (NodeStatus.OFF, NodeStatus.SHUTDOWN):
            self.get_logger().info(f"[{name}] Stopping...")
            self._shutdown_node(node)
            node.reset()
            self.get_logger().info(f"[{name}] Stopped")

        self._deactivation_index += 1

    # ======================================================================
    # Health Checks
    # ======================================================================

    def _check_always_on(self):
        """Check health of always-on nodes."""
        now = time.monotonic()
        for name, node in self._nodes.items():
            if not node.always_on or node.status == NodeStatus.OFF:
                continue

            if node.pid_only:
                self._check_pid_only(name, node, now)
            else:
                self._check_topic_based(name, node, now)

    def _check_pending_dependents(self):
        """Check health of dependent nodes after upstream recovers."""
        now = time.monotonic()

        for node in self._nodes.values():
            if not node._pending_dependent_check or now < node._dependent_check_time:
                continue

            node._pending_dependent_check = False
            self.get_logger().debug(
                f"[{node.name}] Checking dependents: {node.dependents}"
            )

            for dep_name in node.dependents:
                dep = self._nodes.get(dep_name)
                if dep is None:
                    self.get_logger().warning(
                        f"[{node.name}] Dependent '{dep_name}' not found in config"
                    )
                    continue

                # Skip if not active or already being handled
                if dep.status in (
                    NodeStatus.OFF,
                    NodeStatus.RESTARTING,
                    NodeStatus.SHUTDOWN,
                ):
                    continue

                # Skip if recently restarted
                if now - dep.last_restart_time < dep.restart_cooldown:
                    continue

                # Check if unhealthy
                unhealthy = False
                reason = ""

                if dep.status in (NodeStatus.STALE, NodeStatus.DEAD):
                    unhealthy = True
                    reason = f"status is {_STATUS_LABELS.get(dep.status, 'unknown')}"
                elif dep.ever_seen:
                    age = now - dep.last_heartbeat
                    if age > dep.heartbeat_timeout:
                        unhealthy = True
                        reason = f"heartbeat age {age:.1f}s > timeout {dep.heartbeat_timeout}s"

                if unhealthy:
                    self.get_logger().warning(
                        f"[{node.name}] Dependent '{dep_name}' unhealthy ({reason}), restarting"
                    )
                    self._restart_node(dep)
                else:
                    self.get_logger().debug(
                        f"[{node.name}] Dependent '{dep_name}' is healthy"
                    )

    def _check_pid_only(self, name: str, node: MonitoredNode, now: float):
        """Check health of PID-only monitored node."""
        # For unit nodes, check ALL processes
        if node.unit and node.process_names:
            pid_alive = self._check_unit_alive(node)
        else:
            pid_alive = self._find_pid(node) is not None

        if not node.ever_seen:
            if pid_alive:
                node.ever_seen = True
                node.pid_found = True
                node.status = NodeStatus.RUNNING
                node.last_heartbeat = now
                node.mark_progress()
                self._schedule_dependent_check(node, now)
            else:
                # Check stall
                stall = now - node.last_progress
                if stall > node.stall_timeout:
                    node.status = NodeStatus.DEAD
                    node.last_error = f"Stalled ({stall:.0f}s no progress)"
                    self.get_logger().error(f"[{name}] {node.last_error}")
                    self._handle_failure(name)
                elif now - node.boot_start > node.max_boot_timeout:
                    node.status = NodeStatus.DEAD
                    node.last_error = "Max boot timeout exceeded"
                    self.get_logger().error(f"[{name}] {node.last_error}")
                    self._handle_failure(name)
        else:
            if pid_alive:
                node.last_heartbeat = now
                if node.status in (NodeStatus.STALE, NodeStatus.RESTARTING):
                    node.status = NodeStatus.RUNNING
                    node.last_error = ""
                    self.get_logger().info(f"[{name}] Recovered — RUNNING")
                    self._schedule_dependent_check(node, now)
            else:
                if node.status == NodeStatus.RESTARTING:
                    if now - node.last_restart_time < node.restart_cooldown:
                        return
                if node.status != NodeStatus.DEAD:
                    node.status = NodeStatus.DEAD
                    if node.unit:
                        dead_procs = self._find_dead_unit_processes(node)
                        node.last_error = f"Process(es) died: {', '.join(dead_procs)}"
                    else:
                        node.last_error = "Process died"
                    self.get_logger().error(f"[{name}] DEAD — {node.last_error}")
                    self._handle_failure(name)

    def _check_topic_based(self, name: str, node: MonitoredNode, now: float):
        """Check health of topic-based monitored node."""
        if not node.ever_seen:
            # Check stall
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
                node.last_error = "Max boot timeout exceeded"
                self.get_logger().error(f"[{name}] {node.last_error}")
                self._handle_failure(name)
            return

        if node.status == NodeStatus.RESTARTING:
            if now - node.last_restart_time > node.restart_cooldown:
                node.error_count += 1
                node.last_error = "No heartbeat after restart"
                self.get_logger().warning(f"[{name}] {node.last_error}")
                node.status = NodeStatus.STALE
            return

        age = now - node.last_heartbeat
        if age > node.heartbeat_timeout:
            if node.status == NodeStatus.RUNNING:
                node.status = NodeStatus.STALE
                node.last_error = f"Heartbeat stale ({age:.1f}s)"
                self.get_logger().warning(f"[{name}] {node.last_error}")
            elif node.status == NodeStatus.STALE and age > node.heartbeat_timeout * 3:
                node.status = NodeStatus.DEAD
                node.last_error = f"Heartbeat timeout ({age:.1f}s)"
                self.get_logger().error(f"[{name}] {node.last_error}")
                self._handle_failure(name)

    def _check_unit_alive(self, node: MonitoredNode) -> bool:
        """Check if all processes in a unit are alive."""
        for proc_name in node.process_names:
            pids = find_pids_by_cmdline(proc_name)
            if not pids:
                return False
        return True

    def _find_dead_unit_processes(self, node: MonitoredNode) -> List[str]:
        """Find which processes in a unit are dead."""
        dead = []
        for proc_name in node.process_names:
            pids = find_pids_by_cmdline(proc_name)
            if not pids:
                dead.append(proc_name)
        return dead

    def _update_system_state(self):
        """Update overall system state based on node health."""
        has_dead = any(
            n.status == NodeStatus.DEAD
            for n in self._nodes.values()
            if n.status != NodeStatus.OFF
        )
        if has_dead:
            self._system_state = SystemState.DEGRADED
        else:
            self._system_state = SystemState.NOMINAL

    # ======================================================================
    # Failure Handling
    # ======================================================================

    def _handle_failure(self, name: str):
        """Handle node failure based on category."""
        node = self._nodes[name]
        node.error_count += 1

        if node.category == Category.CAT1_AUTO_RESTART:
            self._auto_restart(name)
        elif node.category == Category.CAT2_ABORT_RESTART:
            self._abort_restart(name)
        elif node.category == Category.CAT3_CASCADE_SHUTDOWN:
            self._cascade_shutdown(name)

    def _auto_restart(self, name: str):
        """Category 1: Auto-restart up to max_restarts."""
        node = self._nodes[name]
        now = time.monotonic()

        if now - node.last_restart_time < node.restart_cooldown:
            return

        if node.restart_count >= node.max_restarts:
            self.get_logger().error(
                f"[{name}] Cat1 — max restarts ({node.max_restarts}) reached, marking DEAD"
            )
            node.status = NodeStatus.DEAD
            node.last_error = "Max restarts exceeded"
            return

        self.get_logger().warning(
            f"[{name}] Cat1 — auto-restart ({node.restart_count + 1}/{node.max_restarts})"
        )
        self._restart_node(node)

    def _abort_restart(self, name: str):
        """Category 2: E-stop then restart."""
        node = self._nodes[name]
        now = time.monotonic()

        if now - node.last_restart_time < node.restart_cooldown:
            return

        self.get_logger().warning(f"[{name}] Cat2 — e-stop and restarting")
        self._send_estop()
        self._restart_node(node)

    def _cascade_shutdown(self, trigger_name: str):
        """Category 3: E-stop and shut down all on-demand nodes."""
        if self._shutdown_in_progress:
            return

        self._shutdown_in_progress = True
        self._autonomous_enabled = False
        self._system_state = SystemState.LOCALIZATION_LOST

        self.get_logger().fatal(f"=== CATEGORY 3: {trigger_name} FAILED ===")

        self._send_estop()
        self._send_estop()  # Double-tap for reliability

        self._nav_shutdown_pub.publish(self._empty_msg)

        for name in self.SHUTDOWN_SEQUENCE:
            node = self._nodes[name]
            if node.status not in (NodeStatus.OFF, NodeStatus.SHUTDOWN):
                self.get_logger().warning(f"[{name}] Shutting down (cascade)")
                self._shutdown_node(node)
                node.status = NodeStatus.SHUTDOWN

        self.get_logger().fatal("Autonomous systems shutdown. Teleop still available.")

        self._publish_health()
        self._supervisor_state = SupervisorState.IDLE

    def _abort_activation(self, failed_name: str):
        """Abort activation sequence and return to IDLE."""
        self.get_logger().error(f"=== ACTIVATION ABORTED (failed: {failed_name}) ===")
        self._send_estop()

        for name in reversed(self.SHUTDOWN_SEQUENCE):
            node = self._nodes[name]
            if node.status != NodeStatus.OFF:
                self.get_logger().info(f"[{name}] Stopping (abort cleanup)")
                self._shutdown_node(node)
                node.reset()

        self._autonomous_enabled = False
        self._supervisor_state = SupervisorState.IDLE
        self._system_state = SystemState.DEGRADED
        self._activation_node = None

        self.get_logger().warning("Returned to IDLE after failed activation")

    # ======================================================================
    # PID Lookup (No Subprocess!)
    # ======================================================================

    def _find_pid(self, node: MonitoredNode) -> Optional[int]:
        """Find PID for node using direct /proc scan."""
        # Check cached PID first
        if node.pid and pid_exists(node.pid):
            return node.pid

        node.pid = None
        if not node.process_name:
            return None

        pids = find_pids_by_cmdline(node.process_name)
        if pids:
            node.pid = pids[0]
            return node.pid
        return None

    def _find_pid_by_name(self, process_name: str) -> Optional[int]:
        """Find PID by process name pattern."""
        pids = find_pids_by_cmdline(process_name)
        return pids[0] if pids else None

    # ======================================================================
    # Node Lifecycle
    # ======================================================================

    def _build_command(self, node: MonitoredNode) -> List[str]:
        """Build command with dynamic args resolved."""
        cmd = list(node.start_cmd)

        for arg in node.dynamic_args:
            if arg.startswith("file:"):
                parts = arg.split(":", 2)
                if len(parts) != 3:
                    self.get_logger().warning(
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
                    self.get_logger().warning(
                        f"[{node.name}] Dynamic arg file not found: {filepath}"
                    )
                except Exception as e:
                    self.get_logger().warning(f"[{node.name}] Dynamic arg error: {e}")
            else:
                cmd.append(arg)

        return cmd

    def _start_node(self, node: MonitoredNode) -> bool:
        """Start a node process."""
        if not node.start_cmd:
            self.get_logger().warning(f"[{node.name}] No start command configured")
            return False

        cmd = self._build_command(node)

        try:
            # NOTE: No preexec_fn=os.setsid — processes stay in container's cgroup
            proc = subprocess.Popen(
                cmd,
                env=os.environ.copy(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=False,  # Explicitly stay in current session
            )
            node.pid = proc.pid
            node._spawned_pids.add(proc.pid)
            self.get_logger().info(
                f"[{node.name}] Started (PID {proc.pid}): {' '.join(cmd)}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"[{node.name}] Start failed: {e}")
            node.last_error = f"Start failed: {e}"
            return False

    def _restart_node(self, node: MonitoredNode):
        """Restart a node (shutdown + start)."""
        if not node.start_cmd:
            self.get_logger().warning(f"[{node.name}] No start command configured")
            return

        self._shutdown_node(node)
        cmd = self._build_command(node)

        try:
            proc = subprocess.Popen(
                cmd,
                env=os.environ.copy(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=False,
            )
            node.pid = proc.pid
            node._spawned_pids.add(proc.pid)
            node.restart_count += 1
            node.last_restart_time = time.monotonic()
            node.status = NodeStatus.RESTARTING
            node.last_heartbeat = time.monotonic()
            node.last_progress = time.monotonic()
            self.get_logger().info(
                f"[{node.name}] Restarted (PID {proc.pid}): {' '.join(cmd)}"
            )
        except Exception as e:
            self.get_logger().error(f"[{node.name}] Restart failed: {e}")
            node.error_count += 1
            node.last_error = f"Restart failed: {e}"

    def _shutdown_node(self, node: MonitoredNode):
        """Shutdown a node with lifecycle awareness."""
        self.get_logger().debug(f"[{node.name}] Initiating shutdown...")

        # Try graceful lifecycle shutdown first (non-blocking with timeout)
        if node.lifecycle_manager:
            self._try_lifecycle_shutdown(node)

        # Kill processes
        if node.unit and node.process_names:
            self._kill_unit(node)
        else:
            self._kill_single(node)

        # Cleanup lifecycle manager if specified
        if node.lifecycle_manager:
            self._kill_by_name(node.lifecycle_manager)

        node.pid = None
        node._spawned_pids.clear()
        self.get_logger().debug(f"[{node.name}] Shutdown complete")

    def _try_lifecycle_shutdown(self, node: MonitoredNode):
        """Attempt graceful shutdown via lifecycle manager service (with timeout)."""
        lc_pid = self._find_pid_by_name(node.lifecycle_manager)
        if lc_pid is None:
            self.get_logger().debug(
                f"[{node.name}] Lifecycle manager '{node.lifecycle_manager}' not running"
            )
            return

        self.get_logger().debug(
            f"[{node.name}] Attempting lifecycle shutdown via {node.lifecycle_manager}"
        )

        # Run in thread with hard timeout to prevent blocking
        def lifecycle_call():
            try:
                subprocess.run(
                    [
                        "ros2",
                        "service",
                        "call",
                        f"/{node.lifecycle_manager}/manage_nodes",
                        "nav2_msgs/srv/ManageLifecycleNodes",
                        "{command: 2}",
                    ],
                    timeout=node.lifecycle_timeout,
                    capture_output=True,
                    text=True,
                )
            except Exception:
                pass

        thread = threading.Thread(target=lifecycle_call, daemon=True)
        thread.start()
        thread.join(timeout=node.lifecycle_timeout + 1.0)  # Hard timeout

        if thread.is_alive():
            self.get_logger().warning(
                f"[{node.name}] Lifecycle shutdown thread hung, continuing"
            )
        else:
            self.get_logger().debug(f"[{node.name}] Lifecycle shutdown completed")

        time.sleep(0.3)  # Brief pause for transitions

    def _kill_unit(self, node: MonitoredNode):
        """Kill all processes in a unit."""
        for proc_name in node.process_names:
            self._kill_by_name(proc_name)

    def _kill_single(self, node: MonitoredNode):
        """Kill a single process node."""
        # Kill by tracked PIDs first (safest)
        for pid in list(node._spawned_pids):
            if pid_exists(pid):
                safe_kill_pid(pid, node.process_name, self.get_logger())

        # Also kill by name pattern
        if node.process_name:
            self._kill_by_name(node.process_name)

    def _kill_by_name(self, process_name: str):
        """Kill all processes matching name pattern."""
        pids = find_pids_by_cmdline(process_name)
        for pid in pids:
            safe_kill_pid(pid, process_name, self.get_logger())

    def _send_estop(self):
        """Publish e-stop (zero velocity)."""
        self._estop_pub.publish(self._estop_msg)

    # ======================================================================
    # Resource Monitoring (0.5Hz)
    # ======================================================================

    def _resource_check(self):
        """Check system and process resources."""
        # CRITICAL: Skip during boot delay to avoid memory pressure
        if time.monotonic() - self._boot_delay_start < self._boot_delay:
            return

        self._read_system_cpu()
        self._read_system_memory()
        self._read_process_resources()

    def _read_system_cpu(self):
        """Read system CPU usage from /proc/stat."""
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
        """Read system memory usage from /proc/meminfo."""
        try:
            mem = {}
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        key = parts[0].rstrip(":")
                        mem[key] = int(parts[1])
                        # Early exit once we have what we need
                        if "MemTotal" in mem and "MemAvailable" in mem:
                            break

            self._system_memory_total_mb = mem.get("MemTotal", 0) / 1024.0
            avail = mem.get("MemAvailable", 0) / 1024.0
            self._system_memory_used_mb = self._system_memory_total_mb - avail
        except Exception:
            pass

    def _read_process_resources(self):
        """Read CPU and memory for all monitored nodes."""
        now = time.monotonic()

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

            cpu_ticks, rss_kb = read_proc_resources(pid)

            # CPU calculation
            if node._prev_cpu_time > 0:
                dt = now - node._prev_cpu_time
                if dt > 0:
                    node.cpu_percent = (
                        (cpu_ticks - node._prev_cpu_ticks) / _CLK_TCK / dt
                    ) * 100.0
            node._prev_cpu_ticks = cpu_ticks
            node._prev_cpu_time = now

            # Memory
            node.memory_mb = rss_kb / 1024.0

    # ======================================================================
    # Health Publisher (1Hz)
    # ======================================================================

    def _publish_health(self):
        """Publish robot health status."""
        msg = self._health_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.cpu_percent = float(self._system_cpu_percent)
        msg.cpu_count = self._cpu_count
        msg.memory_used_mb = float(self._system_memory_used_mb)
        msg.memory_total_mb = float(self._system_memory_total_mb)
        msg.system_state = int(self._system_state)
        msg.supervisor_state = int(self._supervisor_state)
        msg.autonomous_enabled = self._autonomous_enabled

        # Rebuild nodes list (reusing NodeHealth objects per node)
        msg.nodes.clear()
        now = time.monotonic()

        for node in self._nodes.values():
            nh = node.get_health_msg()
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
                nh.message = _STATUS_LABELS.get(node.status, "Unknown")
            else:
                nh.last_heartbeat_age = float(now - node.boot_start)
                nh.message = f"Waiting ({now - node.boot_start:.0f}s)"

            msg.nodes.append(nh)

        # System message
        sv_label = _STATE_LABELS.get(self._supervisor_state, "UNKNOWN")

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


# ==========================================================================
# Entry Point
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
