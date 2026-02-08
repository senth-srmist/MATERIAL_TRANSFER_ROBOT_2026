#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.duration
import time
import os
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from tf2_ros import TransformException
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from lifecycle_msgs.srv import GetState
from ament_index_python.packages import get_package_share_directory


class TileSwitcher(Node):

    def __init__(self):
        super().__init__("tile_switcher")

        # ---------------- MAP PATHS ----------------
        pkg_share = get_package_share_directory("campus_maps")
        self.tiles = {
            1: os.path.join(pkg_share, "maps", "tile01.yaml"),
            2: os.path.join(pkg_share, "maps", "tile02.yaml"),
            3: os.path.join(pkg_share, "maps", "tile03.yaml"),
            4: os.path.join(pkg_share, "maps", "tile04.yaml"),
            5: os.path.join(pkg_share, "maps", "tile05.yaml"),
        }

        # ---------------- TRIGGER ZONES (2D bounding boxes) ----------------
        # Each zone: (x_min, x_max, y_min, y_max, from_tile, to_tile)
        self.trigger_zones = [
            # tile01 <-> tile02
            (1.00, 2.66, 9.40, 12.60, 1, 2),  # forward
            (1.00, 2.66, 9.40, 12.60, 2, 1),  # backward
            # tile02 <-> tile03
            (15.47, 17.44, 9.40, 12.60, 2, 3),
            (15.47, 17.44, 9.40, 12.60, 3, 2),
            # tile03 <-> tile04
            (20.00, 21.47, 12.35, 15.03, 3, 4),
            (20.00, 21.47, 12.35, 15.03, 4, 3),
            # tile04 <-> tile05
            (38.62, 40.60, 12.35, 15.03, 4, 5),
            (38.62, 40.60, 12.35, 15.03, 5, 4),
        ]

        self.current_tile = 1
        self.last_switch_time = time.time()
        self.switch_cooldown = 3.0  # seconds

        # ---------------- INITIAL POSE ----------------
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 1.57
        self.initialized = False

        # ---------------- SUBSCRIBER ----------------
        self.create_subscription(Odometry, "/zed/zed_node/odom",
                                 self.pose_callback, 10)

        # ---------------- SERVICES ----------------
        self.map_loader = self.create_client(LoadMap, "/map_server/load_map")
        self.costmap_clear = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entire_costmap")
        self.amcl_state_client = self.create_client(GetState,
                                                    "/amcl/get_state")

        # ---------------- TF2 LISTENER ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- PUBLISHER ----------------
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                  "/initialpose", 10)

        # ---------------- TIMER ----------------
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
                self.get_logger().info(
                    "🧭 AMCL ACTIVE → publishing initial pose")
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

        msg.pose.pose.orientation.z = math.sin(self.init_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.init_yaw / 2.0)

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1

        self.initpose_pub.publish(msg)

        self.get_logger().info(
            f"🚀 INITIAL POSE SET → x={self.init_x:.2f}, y={self.init_y:.2f}")

    # ------------------------------------------------
    def pose_callback(self, msg):
        if not self.initialized:
            return

        # Try to get robot position in map frame via TF
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
        except TransformException as e:
            # Fallback to odom if TF not available
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().debug(f"TF lookup failed, using odom: {e}")

        self.get_logger().info(
            f"[MAP] x={x:.2f} y={y:.2f} tile={self.current_tile}")

        now = time.time()
        if now - self.last_switch_time < self.switch_cooldown:
            return

        # -------- CHECK TRIGGER ZONES --------
        for zone in self.trigger_zones:
            x_min, x_max, y_min, y_max, from_tile, to_tile = zone

            if from_tile != self.current_tile:
                continue

            if x_min <= x <= x_max and y_min <= y <= y_max:
                self.get_logger().info(
                    f"📍 Entered trigger zone at ({x:.2f}, {y:.2f})")
                self.switch_to_tile(to_tile, x, y)
                return

    # ------------------------------------------------
    def switch_to_tile(self, new_tile, x, y):
        if new_tile not in self.tiles:
            return

        self.get_logger().warn(f"🔁 Switching to TILE {new_tile}")

        if not self.map_loader.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ map_server/load_map not available")
            return

        req = LoadMap.Request()
        req.map_url = self.tiles[new_tile]
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


if __name__ == "__main__":
    main()
