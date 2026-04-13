#!/usr/bin/env python3
"""
odom_base_publisher node (v3 - 2D Flattening + Path-compatible)

Bridges ZED camera odometry to Nav2's expected TF tree:
  1. Gets URDF transform (zed_camera_link → base_link)
  2. Transforms ZED camera odom to base_link odom
  3. On FIRST message: stores initial pose as offset
  4. Flattens result to 2D — zeroes Z, strips roll/pitch, keeps only yaw
     (prevents camera tilt from making robot appear underground/floating)
  5. Publishes odom → base_link TF (starts at identity)
  6. Publishes map → odom TF (drift correction from ZED SLAM)
  7. Re-publishes Odometry on /odom/base_link — flat 2D pose, compatible
     with RViz Path display and Nav2

Changes from v2:
  - _flatten_to_2d(): strips Z translation and roll/pitch from T_odom_base
  - _publish_base_odom() now writes the flattened 2D pose so RViz path
    stays on the ground plane
  - Added path_pub publishing nav_msgs/Path on /odom/base_link/path
    for direct use in RViz without needing ZED's odom topic

Original optimizations retained:
  - Pre-allocated output messages (reused each callback)
  - Preallocated scratch matrices
  - Analytical rigid-body inverse (no LU decomposition)
  - Rotation matrix re-orthogonalization
  - Safe quaternion extraction with NaN protection
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import numpy as np

# ============================================================================
# Matrix utilities (module-level, no allocations in hot path)
# ============================================================================


def _quat_normalize(q):
    """Normalize quaternion [x, y, z, w]. Returns unit quaternion."""
    arr = np.array(q, dtype=np.float64)
    norm = np.linalg.norm(arr)
    if norm < 1e-10:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return arr / norm


def _to_mat(t, q, out=None):
    """
    Convert translation + quaternion to 4×4 homogeneous matrix.
    Quaternion is normalized before conversion.
    """
    if out is None:
        out = np.eye(4, dtype=np.float64)

    qn = _quat_normalize(q)
    x, y, z, w = qn

    out[0, 0] = 1 - 2 * (y * y + z * z)
    out[0, 1] = 2 * (x * y - z * w)
    out[0, 2] = 2 * (x * z + y * w)
    out[0, 3] = t[0]
    out[1, 0] = 2 * (x * y + z * w)
    out[1, 1] = 1 - 2 * (x * x + z * z)
    out[1, 2] = 2 * (y * z - x * w)
    out[1, 3] = t[1]
    out[2, 0] = 2 * (x * z - y * w)
    out[2, 1] = 2 * (y * z + x * w)
    out[2, 2] = 1 - 2 * (x * x + y * y)
    out[2, 3] = t[2]
    out[3, 0] = 0.0
    out[3, 1] = 0.0
    out[3, 2] = 0.0
    out[3, 3] = 1.0

    return out


def _rigid_inv(M, out=None):
    """
    Analytical inverse of a rigid body transform (SE(3)).

    For M = [R | t], inverse = [R^T | -R^T * t]
             [0 | 1]            [0   |    1    ]

    ~10x faster than np.linalg.inv and numerically more stable.
    """
    if out is None:
        out = np.eye(4, dtype=np.float64)

    R = M[:3, :3]
    t = M[:3, 3]

    out[:3, :3] = R.T
    out[0, 3] = -(R[0, 0] * t[0] + R[1, 0] * t[1] + R[2, 0] * t[2])
    out[1, 3] = -(R[0, 1] * t[0] + R[1, 1] * t[1] + R[2, 1] * t[2])
    out[2, 3] = -(R[0, 2] * t[0] + R[1, 2] * t[1] + R[2, 2] * t[2])
    out[3, 0] = 0.0
    out[3, 1] = 0.0
    out[3, 2] = 0.0
    out[3, 3] = 1.0

    return out


def _reorthogonalize(M):
    """
    Re-orthogonalize the rotation submatrix of a 4×4 transform
    using SVD projection onto SO(3). Prevents drift from accumulated
    floating-point errors.
    """
    R = M[:3, :3]
    U, _, Vt = np.linalg.svd(R)
    det = np.linalg.det(U @ Vt)
    if det < 0:
        U[:, 2] *= -1
    M[:3, :3] = U @ Vt


def _mat_to_q(R):
    """
    Extract quaternion [x, y, z, w] from 3×3 rotation matrix.
    Uses Shepperd's method with NaN-safe sqrt.
    """
    tr = R[0, 0] + R[1, 1] + R[2, 2]

    if tr > 0:
        val = max(0.0, tr + 1.0)
        s = 2.0 * np.sqrt(val)
        if s < 1e-10:
            return [0.0, 0.0, 0.0, 1.0]
        return [
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
            0.25 * s,
        ]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        val = max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        s = 2.0 * np.sqrt(val)
        if s < 1e-10:
            return [0.0, 0.0, 0.0, 1.0]
        return [
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[2, 1] - R[1, 2]) / s,
        ]
    elif R[1, 1] > R[2, 2]:
        val = max(0.0, 1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        s = 2.0 * np.sqrt(val)
        if s < 1e-10:
            return [0.0, 0.0, 0.0, 1.0]
        return [
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
            (R[0, 2] - R[2, 0]) / s,
        ]
    else:
        val = max(0.0, 1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        s = 2.0 * np.sqrt(val)
        if s < 1e-10:
            return [0.0, 0.0, 0.0, 1.0]
        return [
            (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            0.25 * s,
            (R[1, 0] - R[0, 1]) / s,
        ]


def _fill_tf(msg, M):
    """Fill TransformStamped from 4×4 matrix."""
    msg.transform.translation.x = float(M[0, 3])
    msg.transform.translation.y = float(M[1, 3])
    msg.transform.translation.z = float(M[2, 3])
    q = _mat_to_q(M[:3, :3])
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])


def _is_valid_pose(p, o):
    """Check if position and orientation contain finite values."""
    return (
        np.isfinite(p.x)
        and np.isfinite(p.y)
        and np.isfinite(p.z)
        and np.isfinite(o.x)
        and np.isfinite(o.y)
        and np.isfinite(o.z)
        and np.isfinite(o.w)
        and (o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w) > 0.01
    )


def _flatten_to_2d(M):
    """
    Flatten a 4×4 SE(3) transform to a pure 2D ground-plane transform.

    - Zeroes Z translation (robot stays on ground)
    - Extracts only yaw from the rotation matrix (strips roll and pitch)
    - Rebuilds a pure yaw rotation matrix

    This prevents camera tilt (30° nose-down) and any ZED vertical drift
    from propagating into the base_link pose, keeping the robot model
    flat on the ground in RViz and Nav2.
    """
    yaw = math.atan2(M[1, 0], M[0, 0])
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    # Zero Z, keep X/Y translation
    M[2, 3] = 0.0

    # Rebuild pure yaw rotation (zeros out roll and pitch)
    M[0, 0] = cos_y
    M[0, 1] = -sin_y
    M[0, 2] = 0.0
    M[1, 0] = sin_y
    M[1, 1] = cos_y
    M[1, 2] = 0.0
    M[2, 0] = 0.0
    M[2, 1] = 0.0
    M[2, 2] = 1.0


# ============================================================================
# Node
# ============================================================================


class OdomBasePublisherNode(Node):
    REORTHOG_INTERVAL = 100
    ZED_TIMEOUT = 2.0
    HEALTH_LOG_INTERVAL = 30.0
    MAX_PATH_POSES = 100  # cap path history to avoid unbounded memory

    def __init__(self):
        super().__init__("odom_base_publisher")

        # Parameters
        self.declare_parameter("camera_frame", "zed_camera_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_topic", "/zed/zed_node/odom")
        self.declare_parameter("pose_topic", "/zed/zed_node/pose")

        self.camera_frame = self.get_parameter("camera_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Transforms (preallocated)
        self.T_camera_to_base = None
        self.T_initial_inv = None
        self.T_odom_base = np.eye(4, dtype=np.float64)
        self.T_odom_base_valid = False
        self._odom_msg_count = 0

        # Timestamp tracking
        self._last_odom_stamp = None

        # ZED health monitoring
        self._last_zed_time = self.get_clock().now()
        self._zed_healthy = True

        # Scratch matrices (avoid allocation in hot path)
        self._scratch_a = np.eye(4, dtype=np.float64)
        self._scratch_b = np.eye(4, dtype=np.float64)
        self._scratch_c = np.eye(4, dtype=np.float64)

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Pre-allocated TF messages (reused each callback)
        self._tf_odom_base = TransformStamped()
        self._tf_odom_base.header.frame_id = self.odom_frame
        self._tf_odom_base.child_frame_id = self.base_frame

        self._tf_map_odom = TransformStamped()
        self._tf_map_odom.header.frame_id = self.map_frame
        self._tf_map_odom.child_frame_id = self.odom_frame

        # Pre-allocated Odometry output (reused each callback)
        self._odom_out = Odometry()
        self._odom_out.header.frame_id = self.odom_frame
        self._odom_out.child_frame_id = self.base_frame

        # Path message — grows as robot moves, published on each odom tick
        # Use odom_frame so it lines up with the odom → base_link TF
        self._path_msg = Path()
        self._path_msg.header.frame_id = self.odom_frame

        # Subscribers
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._odom_cb, qos
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("pose_topic").value, self._pose_cb, qos
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom/base_link", qos)
        self.path_pub = self.create_publisher(Path, "/odom/base_link/path", qos)

        # Timers
        self._lookup_timer = self.create_timer(0.5, self._lookup_transform)
        self._health_timer = self.create_timer(1.0, self._check_zed_health)
        self._health_log_timer = self.create_timer(
            self.HEALTH_LOG_INTERVAL, self._log_health
        )

        self.get_logger().info("Waiting for URDF transform...")

    # ======================================================================
    # URDF transform lookup
    # ======================================================================

    def _lookup_transform(self):
        """Get zed_camera_link → base_link from URDF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
            tr = t.transform.translation
            ro = t.transform.rotation
            self.T_camera_to_base = _to_mat(
                [tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w]
            )
            self.get_logger().info(
                f"URDF transform acquired: {self.camera_frame} <- {self.base_frame}"
            )
            self._lookup_timer.cancel()

        except tf2_ros.LookupException:
            pass
        except tf2_ros.ExtrapolationException:
            pass
        except Exception as e:
            self.get_logger().warning(f"URDF lookup error: {e}")

    # ======================================================================
    # ZED health monitoring
    # ======================================================================

    def _check_zed_health(self):
        """Monitor ZED data freshness."""
        if self.T_camera_to_base is None:
            return
        elapsed = (self.get_clock().now() - self._last_zed_time).nanoseconds * 1e-9
        if elapsed > self.ZED_TIMEOUT:
            if self._zed_healthy:
                self._zed_healthy = False
                self.get_logger().error(
                    f"ZED data timeout ({elapsed:.1f}s) — TF will go stale"
                )
        else:
            if not self._zed_healthy:
                self._zed_healthy = True
                self.get_logger().info("ZED data recovered")

    def _log_health(self):
        """Periodic health status."""
        if not self.T_odom_base_valid:
            self.get_logger().info("Waiting for first odom message...")
            return
        status = "healthy" if self._zed_healthy else "TIMEOUT"
        self.get_logger().debug(
            f"Odom bridge: ZED={status} msgs={self._odom_msg_count}"
        )

    # ======================================================================
    # Odom callback (odom → base_link TF)
    # ======================================================================

    def _odom_cb(self, msg: Odometry):
        if self.T_camera_to_base is None:
            return

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        if not _is_valid_pose(p, o):
            self.get_logger().warning("Invalid odom pose from ZED, skipping")
            return

        self._last_zed_time = self.get_clock().now()

        # Camera pose from ZED odom
        T_odom_cam = _to_mat([p.x, p.y, p.z], [o.x, o.y, o.z, o.w], out=self._scratch_a)

        # Base pose in odom frame (camera frame → base_link frame)
        T_odom_base_raw = T_odom_cam @ self.T_camera_to_base

        # First message: capture initial pose to zero out starting position
        if self.T_initial_inv is None:
            self.T_initial_inv = _rigid_inv(T_odom_base_raw.copy())
            self.get_logger().info("Odom initialized at base_link")

        # Subtract initial pose so odom → base_link starts at identity
        np.dot(self.T_initial_inv, T_odom_base_raw, out=self.T_odom_base)

        # ── 2D FLATTEN ────────────────────────────────────────────────────
        # Strip camera tilt (roll/pitch) and vertical drift (Z) so the
        # robot stays flat on the ground plane in RViz and Nav2.
        _flatten_to_2d(self.T_odom_base)
        # ─────────────────────────────────────────────────────────────────

        # Periodic re-orthogonalization
        self._odom_msg_count += 1
        if self._odom_msg_count % self.REORTHOG_INTERVAL == 0:
            _reorthogonalize(self.T_odom_base)

        self.T_odom_base_valid = True
        self._last_odom_stamp = msg.header.stamp

        # Publish odom → base_link TF
        self._tf_odom_base.header.stamp = msg.header.stamp
        _fill_tf(self._tf_odom_base, self.T_odom_base)
        self.tf_broadcaster.sendTransform(self._tf_odom_base)

        # Publish /odom/base_link Odometry + /odom/base_link/path
        self._publish_base_odom(msg)

    # ======================================================================
    # Pose callback (map → odom TF)
    # ======================================================================

    def _pose_cb(self, msg: PoseStamped):
        if self.T_camera_to_base is None or not self.T_odom_base_valid:
            return

        p = msg.pose.position
        o = msg.pose.orientation

        if not _is_valid_pose(p, o):
            self.get_logger().warning("Invalid SLAM pose from ZED, skipping")
            return

        # Camera pose in map
        T_map_cam = _to_mat([p.x, p.y, p.z], [o.x, o.y, o.z, o.w], out=self._scratch_b)

        # Base pose in map
        T_map_base = T_map_cam @ self.T_camera_to_base

        # map → odom = map→base × inv(odom→base)
        T_odom_base_inv = _rigid_inv(self.T_odom_base, out=self._scratch_c)
        T_map_odom = T_map_base @ T_odom_base_inv

        _reorthogonalize(T_map_odom)

        # Publish map → odom TF
        self._tf_map_odom.header.stamp = self.get_clock().now().to_msg()
        _fill_tf(self._tf_map_odom, T_map_odom)
        self.tf_broadcaster.sendTransform(self._tf_map_odom)

    # ======================================================================
    # Re-publish odometry + path in base_link frame
    # ======================================================================

    def _publish_base_odom(self, zed_msg: Odometry):
        """
        Re-publish ZED odometry with flattened 2D pose in base_link frame
        on /odom/base_link, and append to the running path on
        /odom/base_link/path for RViz visualization.
        """
        stamp = zed_msg.header.stamp
        out = self._odom_out
        out.header.stamp = stamp

        # Pose from our flattened 2D transform
        q = _mat_to_q(self.T_odom_base[:3, :3])
        out.pose.pose.position.x = float(self.T_odom_base[0, 3])
        out.pose.pose.position.y = float(self.T_odom_base[1, 3])
        out.pose.pose.position.z = 0.0  # always on ground
        out.pose.pose.orientation.x = float(q[0])
        out.pose.pose.orientation.y = float(q[1])
        out.pose.pose.orientation.z = float(q[2])
        out.pose.pose.orientation.w = float(q[3])

        # Copy twist (ZED reports in body frame; linear.z will be ~0 after flatten)
        out.twist.twist.linear.x = zed_msg.twist.twist.linear.x
        out.twist.twist.linear.y = zed_msg.twist.twist.linear.y
        out.twist.twist.linear.z = 0.0  # zeroed — robot is on ground
        out.twist.twist.angular.x = 0.0  # zeroed — no roll rate
        out.twist.twist.angular.y = 0.0  # zeroed — no pitch rate
        out.twist.twist.angular.z = zed_msg.twist.twist.angular.z

        # Copy covariance
        out.pose.covariance = zed_msg.pose.covariance
        out.twist.covariance = zed_msg.twist.covariance

        self.odom_pub.publish(out)

        # ── PATH ─────────────────────────────────────────────────────────
        # Build a PoseStamped from the current flat pose and append to path.
        # Cap history to MAX_PATH_POSES to avoid unbounded memory growth.
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = self.odom_frame
        pose_stamped.pose = out.pose.pose  # already flat 2D

        self._path_msg.header.stamp = stamp
        self._path_msg.poses.append(pose_stamped)
        if len(self._path_msg.poses) > self.MAX_PATH_POSES:
            self._path_msg.poses.pop(0)

        self.path_pub.publish(self._path_msg)
        # ─────────────────────────────────────────────────────────────────


# ============================================================================
# Main
# ============================================================================


def main(args=None):
    rclpy.init(args=args)
    node = OdomBasePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
