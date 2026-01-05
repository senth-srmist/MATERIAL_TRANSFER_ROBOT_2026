#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from nav2_msgs.srv import ClearEntireCostmap

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import os
import time


class TileSwitcher(Node):

    def __init__(self):
        super().__init__('tile_switcher')

        # -------- MAP PATHS --------
        pkg_share = get_package_share_directory('campus_maps')
        self.tile1 = os.path.join(pkg_share, 'maps', 'tile01.yaml')
        self.tile2 = os.path.join(pkg_share, 'maps', 'tile02.yaml')

        # -------- SWITCHING LIMITS --------
        self.switch_forward_x = 5.0   # Tile1 → Tile2
        self.switch_back_x = 4.0      # Tile2 → Tile1 (hysteresis)

        self.current_tile = 1
        self.last_switch_time = time.time()

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

        self.get_logger().info("✅ Tile switcher READY")

    # ------------------------------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.get_logger().info(
            f"[ODOM] x={x:.2f} y={y:.2f} tile={self.current_tile}"
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
    def switch_map(self, map_yaml, x, y, new_tile):
        self.get_logger().warn(f"🔁 Switching to TILE {new_tile}")

        if not self.map_loader.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ map_server/load_map not available")
            return

        req = LoadMap.Request()
        req.map_url = map_yaml
        self.map_loader.call_async(req)

        self.reset_localization(x, y)

        if self.costmap_clear.wait_for_service(timeout_sec=2.0):
            self.costmap_clear.call_async(ClearEntireCostmap.Request())

        self.current_tile = new_tile
        self.last_switch_time = time.time()

    # ------------------------------------------------
    def reset_localization(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1

        self.initpose_pub.publish(msg)
        self.get_logger().info("📍 AMCL reset after map switch")


def main():
    rclpy.init()
    node = TileSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
