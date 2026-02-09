#!/usr/bin/env python3

"""
Tile Swithcer Node

Responsibilities:
    - Subscribe to /robot_pose (full 6DOF PoseStamped)
    - Subscribe to /robot_movement_yaw (movement direction)
    - Check trigger regions using position + movement direction
    - Switch maps via Nav2 map_server
    - Clear costmaps

This node:
    - Does NOT care how pose was obtained
    - Uses movement direction for heading-based triggers
    - Handles all tile/map switching logic
"""

import math
import time
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from ament_index_python.packages import get_package_share_directory


class TileSwitcher(Node):

    def __init__(self):
        super().__init__("tile_switcher")

        # ---------------- MAP PATHS ----------------
        pkg_share = get_package_share_directory("tile_manager")
        self.tiles = {
            1: os.path.join(pkg_share, "maps", "tile01.yaml"),
            2: os.path.join(pkg_share, "maps", "tile02.yaml"),
            3: os.path.join(pkg_share, "maps", "tile03.yaml"),
            4: os.path.join(pkg_share, "maps", "tile04.yaml"),
            5: os.path.join(pkg_share, "maps", "tile05.yaml"),
        }

        # ---------------- TRIGGER ZONES ----------------
        # (x_min, x_max, y_min, y_max, from_tile, to_tile, heading)
        self.trigger_zones = [
            (1.00, 2.66, 9.40, 12.60, 1, 2, "+x"),
            (1.00, 2.66, 9.40, 12.60, 2, 1, "-x"),
            (15.47, 17.44, 9.40, 12.60, 2, 3, "+x"),
            (15.47, 17.44, 9.40, 12.60, 3, 2, "-x"),
            (20.00, 21.47, 12.35, 15.03, 3, 4, "+x"),
            (20.00, 21.47, 12.35, 15.03, 4, 3, "-x"),
            (38.62, 40.60, 12.35, 15.03, 4, 5, "+x"),
            (38.62, 40.60, 12.35, 15.03, 5, 4, "-x"),
        ]

        # ---------------- STATE ----------------
        self.current_tile = 1
        self.last_switch_time = time.time()
        self.switch_cooldown = 0.5
        
        # Latest data
        self.current_x = None
        self.current_y = None
        self.current_movement_yaw = None

        # ---------------- SERVICES ----------------
        self.map_loader = self.create_client(LoadMap, "/map_server/load_map")
        self.costmap_clear = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entire_costmap"
        )

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(PoseStamped, "/robot_pose", self._on_pose, 10)
        self.create_subscription(Float32, "/robot_movement_yaw", self._on_movement_yaw, 10)

        self.get_logger().info(f"Tile Manager started")
        self.get_logger().info(f"   Current tile: {self.current_tile}")

    # ================== CALLBACKS ==================
    def _on_pose(self, msg: PoseStamped):
        """Receive full pose, extract position"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
        self._try_check_transition()

    def _on_movement_yaw(self, msg: Float32):
        """Receive movement direction"""
        self.current_movement_yaw = msg.data
        
        self._try_check_transition()

    # ================== TRANSITION LOGIC ==================
    def _try_check_transition(self):
        """Check transition if we have all required data"""
        if self.current_x is None or self.current_y is None:
            return
        if self.current_movement_yaw is None:
            return

        heading = self._yaw_to_heading(self.current_movement_yaw)
        
        self.get_logger().info(
            f"[TILE] x={self.current_x:.2f} y={self.current_y:.2f} "
            f"heading={heading} tile={self.current_tile}"
        )

        self._check_transition(self.current_x, self.current_y, heading)

    def _check_transition(self, x, y, heading):
        """Check if robot should transition to new tile"""
        if time.time() - self.last_switch_time < self.switch_cooldown:
            return

        for zone in self.trigger_zones:
            x_min, x_max, y_min, y_max, from_tile, to_tile, required_heading = zone

            if from_tile != self.current_tile:
                continue

            if self._in_zone(x, y, x_min, x_max, y_min, y_max):
                if heading == required_heading:
                    self.get_logger().info(
                        f"Trigger at ({x:.2f}, {y:.2f}) heading {heading}"
                    )
                    self._switch_tile(to_tile)
                    return

    def _in_zone(self, x, y, x_min, x_max, y_min, y_max):
        """Check if position is inside zone"""
        return x_min <= x <= x_max and y_min <= y <= y_max

    # ================== HEADING INTERPRETATION ==================
    def _yaw_to_heading(self, yaw):
        """Convert yaw (radians) to heading string (+x, -x, +y, -y)"""
        # Normalize to [-pi, pi]
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi

        if -math.pi / 4 <= yaw < math.pi / 4:
            return "+x"
        elif math.pi / 4 <= yaw < 3 * math.pi / 4:
            return "+y"
        elif -3 * math.pi / 4 <= yaw < -math.pi / 4:
            return "-y"
        else:
            return "-x"

    # ================== MAP SWITCHING ==================
    def _switch_tile(self, new_tile):
        """Switch to new tile map"""
        if new_tile not in self.tiles:
            self.get_logger().error(f"Unknown tile: {new_tile}")
            return

        self.get_logger().warn(f"Switching to TILE {new_tile}")

        if not self.map_loader.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("map_server/load_map unavailable")
            return

        req = LoadMap.Request()
        req.map_url = self.tiles[new_tile]
        self.map_loader.call_async(req)

        if self.costmap_clear.wait_for_service(timeout_sec=2.0):
            self.costmap_clear.call_async(ClearEntireCostmap.Request())

        self.current_tile = new_tile
        self.last_switch_time = time.time()
        self.get_logger().info(f"Now on TILE {new_tile}")


def main():
    rclpy.init()
    node = TileSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
