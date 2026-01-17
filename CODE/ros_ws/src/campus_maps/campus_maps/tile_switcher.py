#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import os
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from lifecycle_msgs.srv import GetState
from ament_index_python.packages import get_package_share_directory


class TileSwitcher(Node):

    def __init__(self):
        super().__init__('tile_switcher')

        # ---------------- MAP PATHS ----------------
        pkg_share = get_package_share_directory('campus_maps')
        self.tile1 = os.path.join(pkg_share, 'maps', 'tile01.yaml')
        self.tile2 = os.path.join(pkg_share, 'maps', 'tile02.yaml')

        # ---------------- SWITCH LOGIC ----------------
        self.switch_forward_x = 15.0
        self.switch_back_x = 14.0
        self.current_tile = 1
        self.last_switch_time = time.time()

        # ---------------- INITIAL POSE (CORRIDOR) ----------------
        self.init_x = 16.0     # meters
        self.init_y = 1.0      # meters
        self.init_yaw = 0.0    # radians (along corridor)

        self.initialized = False

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10
        )

        # ---------------- SERVICES ----------------
        self.map_loader = self.create_client(LoadMap, '/map_server/load_map')
        self.costmap_clear = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entire_costmap'
        )

        # 🔧 AMCL lifecycle state client
        self.amcl_state_client = self.create_client(
            GetState,
            '/amcl/get_state'
        )

        # ---------------- PUBLISHERS ----------------
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 🔧 Timer to check AMCL lifecycle
        self.create_timer(1.0, self.check_amcl_and_initialize)

        self.get_logger().info("✅ Tile Switcher started")
        self.get_logger().info("⏳ Waiting for AMCL to become ACTIVE...")

    # ------------------------------------------------
    def check_amcl_and_initialize(self):
        if self.initialized:
            return

        if not self.amcl_state_client.wait_for_service(timeout_sec=0.2):
            return

        req = GetState.Request()
        future = self.amcl_state_client.call_async(req)
        future.add_done_callback(self.handle_amcl_state)

    # ------------------------------------------------
    def handle_amcl_state(self, future):
        if self.initialized:
            return

        try:
            state = future.result().current_state.label
            if state == "active":
                self.get_logger().info("🧭 AMCL ACTIVE → publishing initial pose")
                self.publish_initial_pose()
                self.initialized = True
        except Exception as e:
            self.get_logger().error(f"Failed to get AMCL state: {e}")

    # ------------------------------------------------
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.init_x
        msg.pose.pose.position.y = self.init_y
        msg.pose.pose.position.z = 0.0

        # yaw → quaternion
        msg.pose.pose.orientation.z = math.sin(self.init_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.init_yaw / 2.0)

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1

        self.initpose_pub.publish(msg)

        self.get_logger().info(
            f"🚀 INITIAL POSE APPLIED → x={self.init_x:.2f}, y={self.init_y:.2f}"
        )

    # ------------------------------------------------
    def odom_callback(self, msg):
        if not self.initialized:
            return  # ❗ prevent switching before localization

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.get_logger().info(
            f"[ODOM] x={x:.2f} y={y:.2f} tile={self.current_tile}"
        )

        now = time.time()

        if self.current_tile == 1 and x > self.switch_forward_x:
            if now - self.last_switch_time > 3.0:
                self.switch_map(self.tile2, x, y, 2)

        elif self.current_tile == 2 and x < self.switch_back_x:
            if now - self.last_switch_time > 3.0:
                self.switch_map(self.tile1, x, y, 1)

    # ------------------------------------------------
    def switch_map(self, yaml_file, x, y, new_tile):
        self.get_logger().warn(f"🔁 Switching to TILE {new_tile}")

        if not self.map_loader.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ map_server/load_map not available")
            return

        req = LoadMap.Request()
        req.map_url = yaml_file
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
