#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory
import yaml
import os


class TileSwitcher(Node):

    def __init__(self):
        super().__init__('tile_switcher')

        # ---- PACKAGE PATH ----
        pkg_share = get_package_share_directory('campus_maps')

        self.tile_1_yaml = os.path.join(pkg_share, 'maps', 'tile01.yaml')
        self.tile_2_yaml = os.path.join(pkg_share, 'maps', 'tile02.yaml')

        self.switch_x = 12.0   # meters
        self.current_tile = 1

        # ---- QoS (CRITICAL FIX) ----
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ---- SUBSCRIBERS ----
        self.create_subscription(
            Odometry,
            "/zed/odom",
            self.odom_callback,
            10
        )

        # ---- PUBLISHERS ----
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/map",
            map_qos     # ✅ FIXED QoS
        )

        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10
        )

        # ---- NAV2 SERVICE ----
        self.clear_costmap = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entire_costmap"
        )

        self.get_logger().info("Tile switcher started")
        self.get_logger().info(f"Tile 1 YAML: {self.tile_1_yaml}")
        self.get_logger().info(f"Tile 2 YAML: {self.tile_2_yaml}")

    # ---------------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if x > self.switch_x and self.current_tile == 1:
            self.get_logger().info("Switching to Tile 2")
            self.load_and_publish_map(self.tile_2_yaml)
            self.reset_localization(x, y)
            self.current_tile = 2

    # ---------------------------------
    def load_and_publish_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)

        self.get_logger().info(f"Loaded map YAML: {yaml_path}")

        # NOTE:
        # Actual OccupancyGrid publishing is done by map_server
        # This node only TRIGGERS switching + localization reset

        # Clear Nav2 costmaps
        if self.clear_costmap.wait_for_service(timeout_sec=2.0):
            self.clear_costmap.call_async(ClearEntireCostmap.Request())

    # ---------------------------------
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


def main():
    rclpy.init()
    node = TileSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
