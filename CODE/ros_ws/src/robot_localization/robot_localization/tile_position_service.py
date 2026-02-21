#!/usr/bin/env python3
"""
Tile Position Service - Returns which tile the robot is currently in.
Service: /get_current_tile
"""

import rclpy
from rclpy.node import Node
import yaml
import math
from pathlib import Path

from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

from tile_manager.srv import GetCurrentTile


class TilePositionService(Node):
    def __init__(self):
        super().__init__("tile_position_service")

        self.declare_parameter("config_file", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")

        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )
        self.robot_frame = (
            self.get_parameter("robot_frame").get_parameter_value().string_value
        )

        if not config_file:
            try:
                pkg_share = get_package_share_directory("tile_manager")
                config_file = str(Path(pkg_share) / "config" / "tiles_config.yaml")
            except Exception as e:
                self.get_logger().error(f"tile_manager package not found: {e}")
                config_file = ""

        self.tiles = {}
        if config_file:
            self.load_config(config_file)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.srv = self.create_service(
            GetCurrentTile, "get_current_tile", self.get_current_tile_callback
        )

        self.get_logger().info(f"TilePositionService ready: {len(self.tiles)} tiles")

    def load_config(self, config_path: str):
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            for tile_id, tile_data in config.get("tiles", {}).items():
                bounds = tile_data.get("bounds", [])
                if len(bounds) == 4:
                    self.tiles[int(tile_id)] = {
                        "x_min": bounds[0],
                        "x_max": bounds[1],
                        "y_min": bounds[2],
                        "y_max": bounds[3],
                    }

            self.get_logger().info(f"Loaded {len(self.tiles)} tiles from {config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return x, y, yaw, True
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return 0.0, 0.0, 0.0, False

    def find_tile(self, x: float, y: float) -> int:
        for tile_id, bounds in self.tiles.items():
            if (
                bounds["x_min"] <= x <= bounds["x_max"]
                and bounds["y_min"] <= y <= bounds["y_max"]
            ):
                return tile_id
        return -1

    def get_current_tile_callback(self, request, response):
        x, y, yaw, valid = self.get_robot_pose()

        response.x = x
        response.y = y
        response.yaw = yaw
        response.valid = valid

        if not valid:
            response.tile_id = -1
            response.message = "TF lookup failed"
            return response

        tile_id = self.find_tile(x, y)
        response.tile_id = tile_id
        response.message = (
            "OK" if tile_id != -1 else f"Outside all tiles at ({x:.2f}, {y:.2f})"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TilePositionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
