#!/usr/bin/env python3
"""
odom_base_publisher node

Bridges ZED's camera-centric tracking to Nav2's base_link-centric navigation.

ZED v5 wrapper has no base_frame parameter and always tracks from
zed_camera_link. With publish_tf disabled, ZED still publishes TOPICS:
  /zed/zed_node/odom  → VIO camera pose in odom frame
  /zed/zed_node/pose  → SLAM camera pose in map frame (loop closure)

This node:
  1. Reads base_link → zed_camera_link from URDF (robot_state_publisher)
  2. odom topic  → transforms to base pose → publishes odom → base_link
  3. pose topic  → derives drift correction  → publishes map → odom

TF result:
  map → odom → base_link → zed_camera_link
  (node) (node)   (URDF)

ZED config required:
  publish_tf: false
  publish_map_tf: false
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import numpy as np


class OdomBasePublisherNode(Node):
    def __init__(self):
        super().__init__('odom_base_publisher')

        # Declare parameters
        self.declare_parameter('camera_frame', 'zed_camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('pose_topic', '/zed/zed_node/pose')

        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Static transform from URDF (computed once)
        self.T_camera_to_base = None

        # Latest odom→base for map→odom computation
        self.T_odom_base = None

        # QoS matching ZED defaults
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)
        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, qos)

        # Retry until URDF transform is available
        self._lookup_timer = self.create_timer(1.0, self._lookup_transform)

        self.get_logger().info(
            f'Started. Waiting for URDF TF: {self.base_frame} → {self.camera_frame}')

    # ------------------------------------------------------------------ #
    # Startup: read static transform from URDF                           #
    # ------------------------------------------------------------------ #
    def _lookup_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            tr = t.transform.translation
            ro = t.transform.rotation
            self.T_camera_to_base = _to_mat(
                [tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w])
            self.get_logger().info(
                f'URDF TF acquired: {self.camera_frame} → {self.base_frame} '
                f'xyz({tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f})')
            self._lookup_timer.cancel()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    # ------------------------------------------------------------------ #
    # Odom callback → publish odom → base_link                           #
    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg: Odometry):
        if self.T_camera_to_base is None:
            return
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        T_odom_cam = _to_mat([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

        # T_odom_base = T_odom_camera × T_camera_to_base
        self.T_odom_base = T_odom_cam @ self.T_camera_to_base

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        _fill_tf(tf_msg, self.T_odom_base)
        self.tf_broadcaster.sendTransform(tf_msg)

    # ------------------------------------------------------------------ #
    # Pose callback → publish map → odom (drift correction)              #
    # ------------------------------------------------------------------ #
    def _pose_cb(self, msg: PoseStamped):
        if self.T_camera_to_base is None or self.T_odom_base is None:
            return
        p = msg.pose.position
        o = msg.pose.orientation
        T_map_cam = _to_mat([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

        # T_map_base = T_map_camera × T_camera_to_base
        T_map_base = T_map_cam @ self.T_camera_to_base
        # T_map_odom = T_map_base × inv(T_odom_base)
        T_map_odom = T_map_base @ np.linalg.inv(self.T_odom_base)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        _fill_tf(tf_msg, T_map_odom)
        self.tf_broadcaster.sendTransform(tf_msg)


# ====================================================================== #
# Pure numpy helpers (no scipy)                                           #
# ====================================================================== #

def _to_mat(t, q):
    """[x,y,z] + [qx,qy,qz,qw] → 4×4 homogeneous matrix."""
    x, y, z, w = q
    M = np.eye(4)
    M[0, 0] = 1 - 2 * (y * y + z * z)
    M[0, 1] = 2 * (x * y - z * w)
    M[0, 2] = 2 * (x * z + y * w)
    M[1, 0] = 2 * (x * y + z * w)
    M[1, 1] = 1 - 2 * (x * x + z * z)
    M[1, 2] = 2 * (y * z - x * w)
    M[2, 0] = 2 * (x * z - y * w)
    M[2, 1] = 2 * (y * z + x * w)
    M[2, 2] = 1 - 2 * (x * x + y * y)
    M[0, 3], M[1, 3], M[2, 3] = t
    return M


def _mat_to_q(R):
    """3×3 rotation → [qx, qy, qz, qw]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        return [(R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s,
                0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        return [0.25 * s,
                (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        return [(R[0, 1] + R[1, 0]) / s,
                0.25 * s,
                (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        return [(R[0, 2] + R[2, 0]) / s,
                (R[1, 2] + R[2, 1]) / s,
                0.25 * s,
                (R[1, 0] - R[0, 1]) / s]


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


if __name__ == '__main__':
    main()
