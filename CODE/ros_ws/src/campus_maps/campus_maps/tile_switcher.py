#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.duration
import time
import os

import tf2_ros
from tf2_ros import TransformException
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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

        # ---------------- SERVICES ----------------
        self.map_loader = self.create_client(LoadMap, "/map_server/load_map")
        self.costmap_clear = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entire_costmap")

        # ---------------- TF2 LISTENER ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- TIMER ----------------
        self.create_timer(0.05, self.check_pose)
        
        # ---------------- VISUALIZATION ----------------
        self.path_pub = self.create_publisher(Marker, "/path_taken", 10)

        self.path_marker = Marker()
        self.path_marker.header.frame_id = "map"
        self.path_marker.ns = "path_taken"
        self.path_marker.id = 0
        self.path_marker.type = Marker.LINE_STRIP
        self.path_marker.action = Marker.ADD

        self.path_marker.scale.x = 0.05  # line thickness (meters)

        # Color: bright green
        self.path_marker.color.r = 1.0
        self.path_marker.color.g = 0.0
        self.path_marker.color.b = 0.0
        self.path_marker.color.a = 1.0

        # Lifetime: how long it stays (0 = forever)
        self.path_marker.lifetime.sec = 5   # change to taste

    # ------------------------------------------------
    def check_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "zed_camera_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y

        except TransformException:
            self.get_logger().warn("No TF map→zed_camera_center yet")
            return

        self.get_logger().info(
            f"[MAP] x={x:.2f} y={y:.2f} tile={self.current_tile}"
        )

        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0

        self.path_marker.points.append(p)

        # Optional: limit trail length (prevents infinite growth)
        MAX_POINTS = 300
        if len(self.path_marker.points) > MAX_POINTS:
            self.path_marker.points.pop(0)

        self.path_marker.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_marker)

        now = time.time()
        if now - self.last_switch_time < self.switch_cooldown:
            return

        for zone in self.trigger_zones:
            x_min, x_max, y_min, y_max, from_tile, to_tile = zone

            if from_tile != self.current_tile:
                continue

            if x_min <= x <= x_max and y_min <= y <= y_max:
                self.get_logger().info(
                    f"📍 Entered trigger zone at ({x:.2f}, {y:.2f})"
                )
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

        #self.reset_localization(x, y)

        if self.costmap_clear.wait_for_service(timeout_sec=2.0):
            self.costmap_clear.call_async(ClearEntireCostmap.Request())

        self.current_tile = new_tile
        self.last_switch_time = time.time()

def main():
    rclpy.init()
    node = TileSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
