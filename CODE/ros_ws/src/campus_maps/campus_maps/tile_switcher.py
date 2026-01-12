#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import os

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from ament_index_python.packages import get_package_share_directory


class TileSwitcher(Node):

    def __init__(self):
        super().__init__('tile_switcher')

        # -------- MAP PATHS --------
        pkg_share = get_package_share_directory('campus_maps')
        self.tile1 = os.path.join(pkg_share, 'maps', 'tile01.yaml')
        self.tile2 = os.path.join(pkg_share, 'maps', 'tile02.yaml')

        # -------- INITIAL POSE (CORRIDOR START) --------
        self.start_x = 16.0   # meters (computed from pixel)
        self.start_y = 1.0    # meters
        self.start_yaw = 0.0  # radians (X axis along corridor)

        # -------- SWITCHING LIMITS --------
        self.switch_forward_x = 5.0   # Tile1 → Tile2
        self.switch_back_x = 4.0      # Tile2 → Tile1 (hysteresis)

        self.current_tile = 1
        self.last_switch_time = time.time()
        self.initial_pose_sent = False

        # -------- SUBSCRIBER --------
        self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10
        )

        # -------- SERVICES --------
        self.map_loader = self.create_client(LoadMap, '/map_server/load_map')
        self.costmap_clear = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entire_costmap'
        )

        # -------- PUBLISHER --------
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.get_logger().info("✅ Tile switcher initialized")

    # ------------------------------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Publish initial pose ONCE when odom starts
        if not self.initial_pose_sent:
            self.publish_initial_pose()
            self.initial_pose_sent = True
            self.get_logger().info("📍 Initial pose set inside corridor")

        self.get_logger().info(
            f"[ODOM] x={x:.2f}  y={y:.2f}  tile={self.current_tile}"
        )

        now = time.time()

        # ---- Tile 1 → Tile 2 ----
        if self.current_tile == 1 and x > self.switch_forward_x:
            if now - self.last_switch_time > 3.0:
                self.switch_map(self.tile2, x, y, new_tile=2)

        # ---- Tile 2 → Tile 1 ----
        elif self.current_tile == 2 and x < self.switch_back_x:
            if now - self.last_switch_time > 3.0:
                self.switch_map(self.tile1, x, y, new_tile=1)

    # ------------------------------------------------
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.start_x
        msg.pose.pose.position.y = self.start_y

        # Quaternion for yaw = 0.0 (facing +X)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1

        self.initpose_pub.publish(msg)

    # ------------------------------------------------
    def switch_map(self, map_yaml, x, y, new_tile):
        self.get_logger().warn(f"🔁 Switching to TILE {new_tile}")

        if not self.map_loader.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ map_server/load_map not available")
            return

        req = LoadMap.Request()
        req.map_url = map_yaml
        self.map_loader.call_async(req)

        self.publish_relocalized_pose(x, y)

        if self.costmap_clear.wait_for_service(timeout_sec=2.0):
            self.costmap_clear.call_async(ClearEntireCostmap.Request())

        self.current_tile = new_tile
        self.last_switch_time = time.time()

    # ------------------------------------------------
    def publish_relocalized_pose(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0

        self.initpose_pub.publish(msg)
        self.get_logger().info("📍 AMCL reinitialized after map switch")


def main():
    rclpy.init()
    node = TileSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
