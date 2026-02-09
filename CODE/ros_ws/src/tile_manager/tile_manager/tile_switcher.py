#!/usr/bin/env python3

"""
Tile Switcher Node

Responsibilities:
    - Load tile configuration from YAML
    - Subscribe to /robot_pose (full 6DOF PoseStamped)
    - Subscribe to /robot_movement_yaw (movement direction)
    - Check trigger regions using position + movement direction
    - Switch maps via Nav2 map_server
    - Clear costmaps
    - Publish benchmark metrics

This node:
    - Does NOT care how pose was obtained
    - Uses movement direction for heading-based triggers
    - Loads all map data from external config file
"""

import math
import time
import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from ament_index_python.packages import get_package_share_directory


class TileSwitcher(Node):

    def __init__(self):
        super().__init__("tile_switcher")

        # ---------------- LOAD CONFIG ----------------
        self.pkg_share = get_package_share_directory("tile_manager")
        config_path = os.path.join(self.pkg_share, "config", "tiles_config.yaml")
        
        self.config = self._load_config(config_path)
        if self.config is None:
            self.get_logger().error(f"Failed to load config from {config_path}")
            return

        # ---------------- PARSE CONFIG ----------------
        self.tiles = self._parse_tiles(self.config.get("tiles", {}))
        self.trigger_zones = self._parse_trigger_zones(self.config.get("trigger_zones", []))
        
        settings = self.config.get("settings", {})
        self.current_tile = settings.get("initial_tile", 1)
        self.switch_cooldown = settings.get("switch_cooldown", 0.5)

        # ---------------- STATE ----------------
        self.last_switch_time = time.time()
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

        # ---------------- BENCHMARK PUBLISHERS ----------------
        self.switch_time_pub = self.create_publisher(Float32, "/benchmark/switch_time_ms", 10)
        self.current_tile_pub = self.create_publisher(Int32, "/benchmark/current_tile", 10)

        # Publish initial tile
        self._publish_current_tile()

        # ---------------- LOG STARTUP ----------------
        self.get_logger().info("Tile Switcher started")
        self.get_logger().info(f"  Config: {config_path}")
        self.get_logger().info(f"  Tiles loaded: {len(self.tiles)}")
        self.get_logger().info(f"  Trigger zones: {len(self.trigger_zones)}")
        self.get_logger().info(f"  Initial tile: {self.current_tile}")
        self.get_logger().info(f"  Cooldown: {self.switch_cooldown}s")
        self.get_logger().info(f"  Benchmark topics: /benchmark/switch_time_ms, /benchmark/current_tile")

    # ================== CONFIG LOADING ==================
    def _load_config(self, config_path):
        """Load YAML configuration file"""
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")
            return None

    def _parse_tiles(self, tiles_config):
        """Parse tiles config into {id: full_path} dict"""
        tiles = {}
        maps_dir = os.path.join(self.pkg_share, "maps")
        
        for tile_id, tile_info in tiles_config.items():
            tile_file = tile_info.get("file", f"tile{tile_id:02d}.yaml")
            tiles[int(tile_id)] = os.path.join(maps_dir, tile_file)
            
        return tiles

    def _parse_trigger_zones(self, zones_config):
        """Parse trigger zones config into list of tuples"""
        zones = []
        
        for zone in zones_config:
            bounds = zone.get("bounds", [0, 0, 0, 0])
            zones.append((
                bounds[0],              # x_min
                bounds[1],              # x_max
                bounds[2],              # y_min
                bounds[3],              # y_max
                zone.get("from_tile"),  # from_tile
                zone.get("to_tile"),    # to_tile
                zone.get("heading"),    # heading
            ))
            
        return zones

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

    # ================== BENCHMARK PUBLISHERS ==================
    def _publish_current_tile(self):
        """Publish current tile ID for benchmark"""
        msg = Int32()
        msg.data = self.current_tile
        self.current_tile_pub.publish(msg)

    def _publish_switch_time(self, switch_time_ms):
        """Publish tile switch time for benchmark"""
        msg = Float32()
        msg.data = switch_time_ms
        self.switch_time_pub.publish(msg)

    # ================== MAP SWITCHING ==================
    def _switch_tile(self, new_tile):
        """Switch to new tile map"""
        if new_tile not in self.tiles:
            self.get_logger().error(f"Unknown tile: {new_tile}")
            return

        self.get_logger().warn(f"Switching to TILE {new_tile}")

        # Start timing
        switch_start = time.perf_counter()

        if not self.map_loader.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("map_server/load_map unavailable")
            return

        req = LoadMap.Request()
        req.map_url = self.tiles[new_tile]
        
        # Synchronous call to measure actual load time
        future = self.map_loader.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        # End timing for map load
        switch_end = time.perf_counter()
        switch_time_ms = (switch_end - switch_start) * 1000

        # Clear costmaps (not included in switch time measurement)
        if self.costmap_clear.wait_for_service(timeout_sec=2.0):
            self.costmap_clear.call_async(ClearEntireCostmap.Request())

        # Update state
        self.current_tile = new_tile
        self.last_switch_time = time.time()

        # Publish benchmark metrics
        self._publish_switch_time(switch_time_ms)
        self._publish_current_tile()

        self.get_logger().info(f"Now on TILE {new_tile} (switch took {switch_time_ms:.2f} ms)")


def main():
    rclpy.init()
    node = TileSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
