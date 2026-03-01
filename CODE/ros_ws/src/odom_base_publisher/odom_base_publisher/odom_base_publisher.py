#!/usr/bin/env python3
"""
odom_base_publisher node

Simple approach:
  1. Get URDF transform (zed_camera_link → base_link)
  2. Transform ZED's camera odom to base_link odom
  3. On FIRST message only: store initial pose as offset
  4. Publish odom → base_link (starts at identity, then moves with robot)
  5. Publish map → odom (drift correction from ZED SLAM)

At startup: odom frame = base_link frame (identity transform)
Then robot moves away from there normally.
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

        # Transforms
        self.T_camera_to_base = None  # From URDF (static)
        self.T_initial = None  # Initial base pose (computed once)
        self.T_odom_base = None  # Current odom→base (for map→odom calc)

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._odom_cb, qos
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("pose_topic").value, self._pose_cb, qos
        )

        # Timer to get URDF transform
        self._lookup_timer = self.create_timer(0.5, self._lookup_transform)
        self.get_logger().info("Waiting for URDF transform...")

    def _lookup_transform(self):
        """Get zed_camera_link → base_link from URDF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )

            tr = t.transform.translation
            ro = t.transform.rotation
            self.T_camera_to_base = _to_mat(
                [tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w]
            )

            self.get_logger().info("URDF transform acquired")
            self._lookup_timer.cancel()
        except:
            pass

    def _odom_cb(self, msg: Odometry):
        if self.T_camera_to_base is None:
            return

        # Camera pose from ZED
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        T_odom_cam = _to_mat([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

        # Base pose = camera pose × camera_to_base
        T_odom_base_raw = T_odom_cam @ self.T_camera_to_base

        # First message: store initial pose (odom = base_link at startup)
        if self.T_initial is None:
            self.T_initial = T_odom_base_raw.copy()
            self.get_logger().info("Odom initialized at base_link")

        # Subtract initial: odom→base starts at identity
        self.T_odom_base = np.linalg.inv(self.T_initial) @ T_odom_base_raw

        # Publish odom → base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        _fill_tf(tf_msg, self.T_odom_base)
        self.tf_broadcaster.sendTransform(tf_msg)

    def _pose_cb(self, msg: PoseStamped):
        if self.T_camera_to_base is None or self.T_odom_base is None:
            return

        # Camera pose in map (from ZED SLAM)
        p = msg.pose.position
        o = msg.pose.orientation
        T_map_cam = _to_mat([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

        # Base pose in map
        T_map_base = T_map_cam @ self.T_camera_to_base

        # map→odom = map→base × inv(odom→base)
        T_map_odom = T_map_base @ np.linalg.inv(self.T_odom_base)

        # Publish map → odom with CURRENT time (not message time)
        # This is drift correction - should always be "now"
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        _fill_tf(tf_msg, T_map_odom)
        self.tf_broadcaster.sendTransform(tf_msg)


def _to_mat(t, q):
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
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        return [
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
            0.25 * s,
        ]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        return [
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[2, 1] - R[1, 2]) / s,
        ]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        return [
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
            (R[0, 2] - R[2, 0]) / s,
        ]
    else:
        s = 2.0 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        return [
            (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            0.25 * s,
            (R[1, 0] - R[0, 1]) / s,
        ]


def _fill_tf(msg, M):
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


if __name__ == "__main__":
    main()
