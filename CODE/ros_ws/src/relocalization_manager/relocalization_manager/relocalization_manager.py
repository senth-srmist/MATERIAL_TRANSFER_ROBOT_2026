#!/usr/bin/env python3
"""
Relocalization Manager Node (v2 — with obstacle safety + search pattern)

Orchestrates ArUco-based relocalization for:
  1. Boot-time localization (robot starts at unknown position)
  2. Category 3 recovery (ZED crashed, pose lost)

State machine:
  IDLE → SPINNING → WAITING → VALIDATING → LOCALIZED
              ↓ (no marker found after 360°)
           REPOSITIONING → SPINNING (try from new position)
              ↓ (max retries)
           FAILED

Obstacle safety runs in ALL states:
  - Subscribes to ZED depth image
  - Samples center region for nearest obstacle distance
  - If obstacle < safety_distance: stop all motion, enter AVOIDING
  - AVOIDING: back up slightly, turn away, resume previous state

Publishes:
  /relocalize/status      (String)  — state name
  /relocalize/result      (String)  — JSON: {"tile": 2, "x": 1.5, ...}
  /cmd_vel_relocalize     (Twist)   — motor commands through twist_mux

Subscribes:
  /relocalize/request     (Empty)   — trigger relocalization
  /zed/zed_node/depth/depth_registered (Image) — depth for obstacle safety
  /tf                               — monitors map→base_link for ArUco

Services:
  /relocalize/trigger     (Trigger) — blocking service for supervisor
"""

import math
import json
import time
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

import tf2_ros
import yaml


class RelocState:
    IDLE = "IDLE"
    SPINNING = "SPINNING"
    REPOSITIONING = "REPOSITIONING"
    AVOIDING = "AVOIDING"
    WAITING = "WAITING_FOR_CORRECTION"
    VALIDATING = "VALIDATING"
    LOCALIZED = "LOCALIZED"
    FAILED = "FAILED"


class RelocalizationManager(Node):

    def __init__(self):
        super().__init__("relocalization_manager")

        # ── Parameters ──────────────────────────────────────────────
        # Spin
        self.declare_parameter("spin_velocity", 0.3)  # rad/s
        self.declare_parameter("spin_timeout",
                               50.0)  # seconds for full 360°+ sweep
        # Repositioning
        self.declare_parameter("reposition_distance",
                               1.0)  # meters to drive between spins
        self.declare_parameter("reposition_velocity", 0.2)  # m/s forward
        self.declare_parameter("max_reposition_attempts",
                               3)  # spin-move cycles before fail
        # Correction detection
        self.declare_parameter("correction_timeout",
                               5.0)  # wait for TF stabilization
        self.declare_parameter("pose_jump_threshold",
                               0.5)  # meters for jump detection
        # Validation
        self.declare_parameter("validation_samples", 5)
        self.declare_parameter("validation_tolerance", 0.1)  # meters spread
        # Obstacle safety
        self.declare_parameter("safety_distance",
                               0.4)  # meters — stop if obstacle closer
        self.declare_parameter("critical_distance",
                               0.25)  # meters — back up immediately
        self.declare_parameter("depth_sample_width",
                               60)  # pixels — center column width to check
        self.declare_parameter("avoidance_backup_dist",
                               0.2)  # meters to back up
        self.declare_parameter("avoidance_turn_angle",
                               1.0)  # radians to turn away
        # General
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("tiles_config", "")
        self.declare_parameter("cmd_rate", 10.0)

        self._spin_vel = self.get_parameter("spin_velocity").value
        self._spin_timeout = self.get_parameter("spin_timeout").value
        self._repos_dist = self.get_parameter("reposition_distance").value
        self._repos_vel = self.get_parameter("reposition_velocity").value
        self._max_repos = self.get_parameter("max_reposition_attempts").value
        self._correction_timeout = self.get_parameter(
            "correction_timeout").value
        self._jump_threshold = self.get_parameter("pose_jump_threshold").value
        self._val_samples = self.get_parameter("validation_samples").value
        self._val_tolerance = self.get_parameter("validation_tolerance").value
        self._safety_dist = self.get_parameter("safety_distance").value
        self._critical_dist = self.get_parameter("critical_distance").value
        self._depth_sample_w = self.get_parameter("depth_sample_width").value
        self._avoid_backup = self.get_parameter("avoidance_backup_dist").value
        self._avoid_turn = self.get_parameter("avoidance_turn_angle").value
        self._max_retries = self.get_parameter("max_retries").value
        self._cmd_rate = self.get_parameter("cmd_rate").value

        # ── Load tile bounds ────────────────────────────────────────
        tiles_path = self.get_parameter("tiles_config").value
        self._tiles = {}
        if tiles_path:
            try:
                with open(tiles_path) as f:
                    config = yaml.safe_load(f)
                for tid, tdata in config.get("tiles", {}).items():
                    self._tiles[int(tid)] = {
                        "bounds": tdata["bounds"],
                        "file": tdata["file"],
                    }
                self.get_logger().info(
                    f"Loaded {len(self._tiles)} tiles from {tiles_path}")
            except Exception as e:
                self.get_logger().warn(f"Could not load tiles config: {e}")

        # ── State ───────────────────────────────────────────────────
        self._state = RelocState.IDLE
        self._pre_avoid_state = None  # state to resume after avoidance
        self._retry_count = 0
        self._repos_count = 0
        self._spin_start_time = 0.0
        self._last_pose = None
        self._correction_detected = False
        self._correction_time = 0.0
        self._validation_poses = []
        self._result_tile = None
        self._result_pose = None
        # Repositioning state
        self._repos_start_pose = None
        self._repos_driven = 0.0
        # Avoidance state
        self._avoid_phase = 0  # 0=backup, 1=turn
        self._avoid_start_time = 0.0
        self._avoid_start_yaw = 0.0
        # Depth data
        self._nearest_obstacle = float("inf")
        self._depth_valid = False
        # Accumulated yaw during spin (to detect full rotation)
        self._spin_start_yaw = None
        self._spin_accumulated = 0.0
        self._spin_prev_yaw = None

        # ── TF ──────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ─────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ──────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_relocalize",
                                              reliable_qos)
        self._status_pub = self.create_publisher(String, "/relocalize/status",
                                                 latched_qos)
        self._result_pub = self.create_publisher(String, "/relocalize/result",
                                                 latched_qos)

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(Empty, "/relocalize/request",
                                 self._request_cb, reliable_qos)
        # Depth image for obstacle safety
        self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self._depth_cb,
            best_effort_qos,
        )

        # ── Service ─────────────────────────────────────────────────
        self.create_service(
            Trigger,
            "/relocalize/trigger",
            self._trigger_service_cb,
        )

        # ── Timer ───────────────────────────────────────────────────
        period = 1.0 / self._cmd_rate
        self.create_timer(period, self._tick)

        # Pre-allocate
        self._twist_msg = Twist()
        self._status_msg = String()
        self._result_msg = String()
        self._zero_twist = Twist()

        self._publish_status(RelocState.IDLE)
        self.get_logger().info(
            f"Relocalization manager v2 ready — safety_dist={self._safety_dist}m, "
            f"repos_dist={self._repos_dist}m")

    # ══════════════════════════════════════════════════════════════
    # Depth callback — obstacle safety
    # ══════════════════════════════════════════════════════════════

    def _depth_cb(self, msg: Image):
        """
        Process depth image to find nearest obstacle in front of robot.

        ZED depth is 32FC1 (float meters). We sample a narrow vertical strip
        in the center of the image — this is what's directly ahead.
        """
        try:
            if msg.encoding != "32FC1":
                return

            h = msg.height
            w = msg.width
            half_sample = self._depth_sample_w // 2

            # Center column bounds
            col_start = max(0, w // 2 - half_sample)
            col_end = min(w, w // 2 + half_sample)

            # Only check the middle third vertically (skip floor and ceiling)
            row_start = h // 3
            row_end = 2 * h // 3

            # Parse the float32 depth data
            # Each pixel is 4 bytes (float32)
            step = msg.step  # bytes per row
            min_dist = float("inf")

            # Sample every 4th row and every 4th column for speed
            for r in range(row_start, row_end, 4):
                row_offset = r * step
                for c in range(col_start, col_end, 4):
                    byte_offset = row_offset + c * 4
                    if byte_offset + 4 > len(msg.data):
                        continue
                    val = struct.unpack_from("f", msg.data, byte_offset)[0]
                    if math.isfinite(val) and 0.1 < val < min_dist:
                        min_dist = val

            self._nearest_obstacle = min_dist
            self._depth_valid = True

        except Exception as e:
            self.get_logger().debug(f"Depth processing error: {e}")
            self._depth_valid = False

    # ══════════════════════════════════════════════════════════════
    # Request handlers
    # ══════════════════════════════════════════════════════════════

    def _request_cb(self, msg: Empty):
        self._start_relocalization()

    def _trigger_service_cb(self, request, response):
        if self._state == RelocState.LOCALIZED:
            response.success = True
            response.message = f"Already localized at tile {self._result_tile}"
            return response

        if self._state not in (RelocState.IDLE, RelocState.FAILED):
            response.success = False
            response.message = f"Already in progress: {self._state}"
            return response

        self._start_relocalization()

        timeout = (self._spin_timeout * (self._max_repos + 1) *
                   (self._max_retries + 1) + 60.0)
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            if self._state == RelocState.LOCALIZED:
                response.success = True
                response.message = json.dumps({
                    "tile":
                    self._result_tile,
                    "x":
                    round(self._result_pose[0], 3),
                    "y":
                    round(self._result_pose[1], 3),
                    "yaw":
                    round(self._result_pose[2], 3),
                })
                return response
            elif self._state == RelocState.FAILED:
                response.success = False
                response.message = "Relocalization failed"
                return response
            time.sleep(0.1)

        response.success = False
        response.message = "Timeout"
        return response

    def _start_relocalization(self):
        if self._state in (
                RelocState.SPINNING,
                RelocState.REPOSITIONING,
                RelocState.WAITING,
                RelocState.VALIDATING,
                RelocState.AVOIDING,
        ):
            self.get_logger().warn("Relocalization already in progress")
            return

        self.get_logger().info("=== RELOCALIZATION REQUESTED ===")
        self._retry_count = 0
        self._repos_count = 0
        self._correction_detected = False
        self._validation_poses = []
        self._result_tile = None
        self._result_pose = None
        self._begin_spin()

    # ══════════════════════════════════════════════════════════════
    # State transitions
    # ══════════════════════════════════════════════════════════════

    def _begin_spin(self):
        self._state = RelocState.SPINNING
        self._spin_start_time = time.monotonic()
        self._correction_detected = False
        self._last_pose = self._get_current_pose()

        # Track yaw for full rotation detection
        if self._last_pose is not None:
            self._spin_start_yaw = self._last_pose[2]
            self._spin_prev_yaw = self._last_pose[2]
        else:
            self._spin_start_yaw = None
            self._spin_prev_yaw = None
        self._spin_accumulated = 0.0

        self._publish_status(RelocState.SPINNING)
        self.get_logger().info(
            f"Spinning at {self._spin_vel:.1f} rad/s "
            f"(repos {self._repos_count}/{self._max_repos}, "
            f"retry {self._retry_count}/{self._max_retries})")

    def _begin_reposition(self):
        """Drive forward to a new position, then spin again."""
        self._repos_count += 1

        if self._repos_count > self._max_repos:
            self.get_logger().warn(
                "Max reposition attempts — retrying full cycle")
            self._retry_or_fail()
            return

        self._state = RelocState.REPOSITIONING
        self._repos_start_pose = self._get_current_pose()
        self._repos_driven = 0.0
        self._publish_status(RelocState.REPOSITIONING)
        self.get_logger().info(
            f"Repositioning: driving {self._repos_dist}m forward "
            f"(attempt {self._repos_count}/{self._max_repos})")

    def _enter_avoidance(self, reason: str):
        """Interrupt current state, handle obstacle."""
        if self._state == RelocState.AVOIDING:
            return  # already avoiding

        self._pre_avoid_state = self._state
        self._state = RelocState.AVOIDING
        self._avoid_phase = 0  # start with backup
        self._avoid_start_time = time.monotonic()

        pose = self._get_current_pose()
        self._avoid_start_yaw = pose[2] if pose else 0.0

        self._stop_motors()
        self._publish_status(RelocState.AVOIDING)
        self.get_logger().warn(f"OBSTACLE: {reason} — entering avoidance")

    def _retry_or_fail(self):
        self._retry_count += 1
        self._repos_count = 0  # reset repos counter for new retry

        if self._retry_count >= self._max_retries:
            self.get_logger().error(
                f"Relocalization FAILED after {self._max_retries} retries")
            self._state = RelocState.FAILED
            self._publish_status(RelocState.FAILED)
            self._stop_motors()
            return

        self.get_logger().info(
            f"Retrying relocalization ({self._retry_count}/{self._max_retries})"
        )
        self._spin_vel = -self._spin_vel  # reverse direction
        self._begin_spin()

    # ══════════════════════════════════════════════════════════════
    # Main tick
    # ══════════════════════════════════════════════════════════════

    def _tick(self):
        # ── Obstacle safety check (runs in ALL active states) ──
        if self._state in (RelocState.SPINNING, RelocState.REPOSITIONING):
            if self._depth_valid and self._nearest_obstacle < self._critical_dist:
                self._enter_avoidance(
                    f"critical obstacle at {self._nearest_obstacle:.2f}m")
                return

            if self._state == RelocState.REPOSITIONING:
                # During forward motion, also check safety distance (not just critical)
                if self._depth_valid and self._nearest_obstacle < self._safety_dist:
                    self._enter_avoidance(
                        f"obstacle at {self._nearest_obstacle:.2f}m during reposition"
                    )
                    return

        # ── State machine ──
        if self._state == RelocState.SPINNING:
            self._tick_spinning()
        elif self._state == RelocState.REPOSITIONING:
            self._tick_repositioning()
        elif self._state == RelocState.AVOIDING:
            self._tick_avoiding()
        elif self._state == RelocState.WAITING:
            self._tick_waiting()
        elif self._state == RelocState.VALIDATING:
            self._tick_validating()

    def _tick_spinning(self):
        now = time.monotonic()

        # Track accumulated rotation for full-360 detection
        current = self._get_current_pose()
        if current is not None and self._spin_prev_yaw is not None:
            delta_yaw = current[2] - self._spin_prev_yaw
            # Normalize to [-pi, pi]
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            self._spin_accumulated += abs(delta_yaw)
            self._spin_prev_yaw = current[2]

        # Check if we've completed a full 360° without finding a marker
        if self._spin_accumulated > 2 * math.pi + 0.3:  # ~370° for margin
            self.get_logger().info(
                "Full rotation complete — no marker found, repositioning")
            self._stop_motors()
            self._begin_reposition()
            return

        # Safety timeout (in case yaw tracking fails)
        if now - self._spin_start_time > self._spin_timeout:
            self.get_logger().warn("Spin timeout — repositioning")
            self._stop_motors()
            self._begin_reposition()
            return

        # Send spin command
        self._twist_msg.angular.z = self._spin_vel
        self._twist_msg.linear.x = 0.0
        self._cmd_pub.publish(self._twist_msg)

        # Check for pose jump (ArUco correction)
        if current is not None and self._last_pose is not None:
            dx = current[0] - self._last_pose[0]
            dy = current[1] - self._last_pose[1]
            jump = math.sqrt(dx * dx + dy * dy)

            if jump > self._jump_threshold:
                self.get_logger().info(
                    f"Pose jump detected: {jump:.2f}m — ArUco correction received"
                )
                self._stop_motors()
                self._correction_detected = True
                self._correction_time = now
                self._state = RelocState.WAITING
                self._publish_status(RelocState.WAITING)
                return

        # Update baseline (slow blend to detect jumps)
        if current is not None:
            if self._last_pose is None:
                self._last_pose = current
            else:
                alpha = 0.05
                self._last_pose = (
                    self._last_pose[0] * (1 - alpha) + current[0] * alpha,
                    self._last_pose[1] * (1 - alpha) + current[1] * alpha,
                    current[2],
                    current[3],
                )

    def _tick_repositioning(self):
        """Drive forward reposition_distance meters, then spin again."""
        # Check if we've driven far enough
        current = self._get_current_pose()
        if current is not None and self._repos_start_pose is not None:
            dx = current[0] - self._repos_start_pose[0]
            dy = current[1] - self._repos_start_pose[1]
            self._repos_driven = math.sqrt(dx * dx + dy * dy)

        if self._repos_driven >= self._repos_dist:
            self.get_logger().info(
                f"Repositioned {self._repos_driven:.2f}m — spinning again")
            self._stop_motors()
            self._begin_spin()
            return

        # Also check for ArUco correction during movement
        # (marker might be visible while driving)
        if current is not None and self._last_pose is not None:
            dx = current[0] - self._last_pose[0]
            dy = current[1] - self._last_pose[1]
            # Subtract expected motion (forward velocity × dt)
            # Simple heuristic: if jump is much larger than expected, it's ArUco
            expected_motion = (self._repos_vel * (1.0 / self._cmd_rate) * 3
                               )  # generous margin
            jump = math.sqrt(dx * dx + dy * dy)
            if jump > max(self._jump_threshold, expected_motion * 5):
                self.get_logger().info(
                    f"Pose jump during reposition: {jump:.2f}m — ArUco correction!"
                )
                self._stop_motors()
                self._correction_detected = True
                self._correction_time = time.monotonic()
                self._state = RelocState.WAITING
                self._publish_status(RelocState.WAITING)
                return

        # Update baseline during repositioning (faster blend since we're moving)
        if current is not None:
            self._last_pose = current

        # Drive forward
        self._twist_msg.linear.x = self._repos_vel
        self._twist_msg.angular.z = 0.0
        self._cmd_pub.publish(self._twist_msg)

    def _tick_avoiding(self):
        """
        Two-phase avoidance:
          Phase 0: back up avoidance_backup_dist
          Phase 1: turn avoidance_turn_angle
        Then resume previous state.
        """
        now = time.monotonic()
        current = self._get_current_pose()

        if self._avoid_phase == 0:
            # Back up
            elapsed = now - self._avoid_start_time
            backed_up = abs(self._repos_vel) * elapsed  # approximate

            if backed_up >= self._avoid_backup or elapsed > 3.0:
                # Switch to turn phase
                self._avoid_phase = 1
                self._avoid_start_time = now
                if current is not None:
                    self._avoid_start_yaw = current[2]
                self.get_logger().info("Avoidance: backed up, now turning")
            else:
                self._twist_msg.linear.x = -abs(self._repos_vel)  # reverse
                self._twist_msg.angular.z = 0.0
                self._cmd_pub.publish(self._twist_msg)

        elif self._avoid_phase == 1:
            # Turn away
            if current is not None:
                delta_yaw = abs(current[2] - self._avoid_start_yaw)
                # Normalize
                if delta_yaw > math.pi:
                    delta_yaw = 2 * math.pi - delta_yaw

                if delta_yaw >= self._avoid_turn:
                    # Done avoiding
                    self._stop_motors()
                    self.get_logger().info(
                        f"Avoidance complete — resuming {self._pre_avoid_state}"
                    )

                    # Resume previous state
                    if self._pre_avoid_state == RelocState.SPINNING:
                        self._begin_spin()
                    elif self._pre_avoid_state == RelocState.REPOSITIONING:
                        self._begin_reposition()
                    else:
                        self._begin_spin()
                    return

            elapsed = now - self._avoid_start_time
            if elapsed > 5.0:
                # Timeout — just resume
                self._stop_motors()
                self.get_logger().warn("Avoidance turn timeout — resuming")
                self._begin_spin()
                return

            self._twist_msg.linear.x = 0.0
            self._twist_msg.angular.z = abs(self._spin_vel)  # turn away
            self._cmd_pub.publish(self._twist_msg)

    def _tick_waiting(self):
        """Wait for TF to stabilize after correction."""
        now = time.monotonic()
        if now - self._correction_time > self._correction_timeout:
            self.get_logger().info("Correction settled, validating pose...")
            self._state = RelocState.VALIDATING
            self._validation_poses = []
            self._publish_status(RelocState.VALIDATING)

    def _tick_validating(self):
        """Collect pose samples and check stability."""
        current = self._get_current_pose()
        if current is None:
            return

        self._validation_poses.append(current[:3])

        if len(self._validation_poses) >= self._val_samples:
            xs = [p[0] for p in self._validation_poses]
            ys = [p[1] for p in self._validation_poses]
            spread = max(max(xs) - min(xs), max(ys) - min(ys))

            if spread > self._val_tolerance:
                self.get_logger().warn(
                    f"Pose unstable (spread: {spread:.3f}m), retrying")
                self._retry_or_fail()
                return

            avg_x = sum(xs) / len(xs)
            avg_y = sum(ys) / len(ys)
            avg_yaw = self._validation_poses[-1][2]

            tile_id = self._find_tile(avg_x, avg_y)
            if tile_id is None:
                self.get_logger().warn(
                    f"Position ({avg_x:.2f}, {avg_y:.2f}) not in any tile")
                self._retry_or_fail()
                return

            # SUCCESS
            self._result_tile = tile_id
            self._result_pose = (avg_x, avg_y, avg_yaw)
            self._state = RelocState.LOCALIZED

            self._write_tile_file(tile_id)
            self._publish_status(RelocState.LOCALIZED)

            result = {
                "tile": tile_id,
                "x": round(avg_x, 3),
                "y": round(avg_y, 3),
                "yaw": round(avg_yaw, 3),
            }
            self._result_msg.data = json.dumps(result)
            self._result_pub.publish(self._result_msg)

            self.get_logger().info(
                f"=== LOCALIZED: tile {tile_id} at "
                f"({avg_x:.2f}, {avg_y:.2f}) yaw={avg_yaw:.2f} ===")

    # ══════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════

    def _get_current_pose(self):
        """Returns (x, y, yaw, time) or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            return (x, y, yaw, time.monotonic())
        except Exception:
            return None

    def _find_tile(self, x: float, y: float) -> int:
        """Find tile containing (x, y). Returns tile ID or None."""
        for tid, tdata in self._tiles.items():
            bounds = tdata["bounds"]
            margin = 1.0
            if (bounds[0] - margin <= x <= bounds[1] + margin
                    and bounds[2] - margin <= y <= bounds[3] + margin):
                return tid
        return None

    def _write_tile_file(self, tile_id: int):
        try:
            with open("/tmp/current_tile.txt", "w") as f:
                f.write(str(tile_id))
            self.get_logger().info(
                f"Wrote tile {tile_id} to /tmp/current_tile.txt")
        except Exception as e:
            self.get_logger().error(f"Failed to write tile file: {e}")

    def _stop_motors(self):
        self._cmd_pub.publish(self._zero_twist)

    def _publish_status(self, status: str):
        self._status_msg.data = status
        self._status_pub.publish(self._status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node._cmd_pub.publish(twist)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
