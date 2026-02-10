#!/usr/bin/env python3

"""
Pose Monitor Node

Responsibilities:
    - Subscribe to TF (map -> zed_camera_link)
    - Publish full PoseStamped to /robot_pose (unmodified from TF)
    - Compute and publish movement direction separately

This node:
    - Knows nothing about tiles, maps, or Nav2
    - Does NOT constrain or assume environment (flat floor, etc.)
    - Passes through full 6DOF pose from TF
    - Provides movement heading as supplementary data
"""

import math
import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.duration

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class PoseMonitor(Node):

    def __init__(self):
        super().__init__("pose_monitor")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('source_frame', 'zed_camera_link')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('movement_threshold', 0.02)

        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.movement_threshold = self.get_parameter('movement_threshold').get_parameter_value().double_value

        # ---------------- TF2 LISTENER ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- STATE ----------------
        self.last_x = None
        self.last_y = None
        self.last_movement_yaw = 0.0  # Movement direction (separate from pose orientation)

        # ---------------- PUBLISHERS ----------------
        # Full pose from TF (unmodified)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        
        # Movement direction (computed from position delta)
        self.movement_yaw_pub = self.create_publisher(Float32, '/robot_movement_yaw', 10)

        # ---------------- VISUALIZATION ----------------
        self.path_pub = self.create_publisher(Marker, '/path_taken', 10)
        self.path_marker = self._init_path_marker()

        # ---------------- TIMER ----------------
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(f"✅ Pose Monitor started")
        self.get_logger().info(f"   TF: {self.target_frame} → {self.source_frame}")
        self.get_logger().info(f"   Publishes: /robot_pose (full 6DOF)")
        self.get_logger().info(f"   Publishes: /robot_movement_yaw (movement direction)")

    # ================== INIT ==================
    def _init_path_marker(self):
        """Initialize path visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "path_taken"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 5
        return marker

    # ================== TIMER CALLBACK ==================
    def _on_timer(self):
        """Timer callback: lookup TF, publish full pose"""
        transform = self._lookup_tf()
        if transform is None:
            return

        # Publish full pose (unmodified from TF)
        self._publish_pose(transform)
        
        # Compute and publish movement direction
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        self._publish_movement_yaw(x, y)
        
        # Visualization
        self._update_visualization(x, y)
        
        # Log
        z = transform.transform.translation.z
        q = transform.transform.rotation
        self.get_logger().info(
            f"[POSE] pos=({x:.2f}, {y:.2f}, {z:.2f}) "
            f"quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}) "
            f"move_yaw={math.degrees(self.last_movement_yaw):.1f}°"
        )

    # ================== TF ==================
    def _lookup_tf(self):
        """Lookup transform from target_frame to source_frame"""
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"No TF {self.target_frame}→{self.source_frame}: {e}",
                throttle_duration_sec=2.0
            )
            return None

    # ================== PUBLISHING ==================
    def _publish_pose(self, transform):
        """Publish full PoseStamped from TF (no modifications)"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame

        # Pass through position exactly as received
        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z

        # Pass through orientation exactly as received (full quaternion)
        msg.pose.orientation.x = transform.transform.rotation.x
        msg.pose.orientation.y = transform.transform.rotation.y
        msg.pose.orientation.z = transform.transform.rotation.z
        msg.pose.orientation.w = transform.transform.rotation.w

        self.pose_pub.publish(msg)

    def _publish_movement_yaw(self, x, y):
        """Compute and publish movement direction from position delta"""
        if self.last_x is None or self.last_y is None:
            self.last_x = x
            self.last_y = y
            return

        dx = x - self.last_x
        dy = y - self.last_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance > self.movement_threshold:
            self.last_movement_yaw = math.atan2(dy, dx)

        self.last_x = x
        self.last_y = y

        # Publish movement yaw
        msg = Float32()
        msg.data = self.last_movement_yaw
        self.movement_yaw_pub.publish(msg)

    # ================== VISUALIZATION ==================
    def _update_visualization(self, x, y):
        """Update and publish path marker"""
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0

        self.path_marker.points.append(p)
        if len(self.path_marker.points) > 300:
            self.path_marker.points.pop(0)

        self.path_marker.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_marker)


def main():
    rclpy.init()
    node = PoseMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
