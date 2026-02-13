#!/usr/bin/env python3
"""
Tile Switcher Node (Memory Optimized)

Fixes:
    - Explicit future cleanup to prevent memory growth
    - Reusable request objects
    - Garbage collection hints after switches
"""

import math
import time
import os
import gc
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

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("check_rate", 5.0)
        self.declare_parameter("verbose", False)

        check_rate = self.get_parameter(
            "check_rate").get_parameter_value().double_value
        self.verbose = self.get_parameter(
            "verbose").get_parameter_value().bool_value

        # ---------------- LOAD CONFIG ----------------
        self.pkg_share = get_package_share_directory("tile_manager")
        config_path = os.path.join(self.pkg_share, "config",
                                   "tiles_config.yaml")

        config = self._load_config(config_path)
        if config is None:
            self.get_logger().error(
                f"Failed to load config from {config_path}")
            return

        # ---------------- PARSE CONFIG ----------------
        self.tiles = self._parse_tiles(config.get("tiles", {}))
        self.trigger_zones = self._parse_trigger_zones(
            config.get("trigger_zones", []))

        settings = config.get("settings", {})
        self.current_tile = settings.get("initial_tile", 1)
        self.switch_cooldown = settings.get("switch_cooldown", 0.5)

        # ---------------- STATE ----------------
        self.last_switch_time = 0.0
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self._switch_start_time = None
        self._switching_to_tile = None

        # Track pending futures for cleanup
        self._pending_future = None

        # ---------------- SERVICES (lazy init) ----------------
        self._map_loader = None
        self._costmap_clear = None

        # Reusable request objects (avoid repeated allocations)
        self._costmap_clear_req = None

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(PoseStamped, "/robot_pose", self._on_pose, 10)
        self.create_subscription(Float32, "/robot_movement_yaw", self._on_yaw,
                                 10)

        # ---------------- BENCHMARK PUBLISHERS ----------------
        self.switch_time_pub = self.create_publisher(
            Float32, "/benchmark/switch_time_ms", 10)
        self.current_tile_pub = self.create_publisher(
            Int32, "/benchmark/current_tile", 10)

        # ---------------- TIMER FOR ZONE CHECKING ----------------
        self.create_timer(1.0 / check_rate, self._check_loop)

        # Publish initial tile
        self._publish_tile()

        # ---------------- LOG STARTUP ----------------
        self.get_logger().info("Tile Switcher started (memory optimized)")
        self.get_logger().info(
            f"  Tiles: {len(self.tiles)}, Zones: {len(self.trigger_zones)}")
        self.get_logger().info(
            f"  Check rate: {check_rate} Hz, Verbose: {self.verbose}")

    # ================== LAZY SERVICE GETTERS ==================
    @property
    def map_loader(self):
        if self._map_loader is None:
            self._map_loader = self.create_client(LoadMap,
                                                  "/map_server/load_map")
        return self._map_loader

    @property
    def costmap_clear(self):
        if self._costmap_clear is None:
            self._costmap_clear = self.create_client(
                ClearEntireCostmap, "/global_costmap/clear_entire_costmap")
        return self._costmap_clear

    @property
    def costmap_clear_req(self):
        """Reusable costmap clear request"""
        if self._costmap_clear_req is None:
            self._costmap_clear_req = ClearEntireCostmap.Request()
        return self._costmap_clear_req

    # ================== CONFIG LOADING ==================
    def _load_config(self, path):
        try:
            with open(path, "r") as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Config error: {e}")
            return None

    def _parse_tiles(self, tiles_cfg):
        tiles = {}
        maps_dir = os.path.join(self.pkg_share, "maps")
        for tid, info in tiles_cfg.items():
            tiles[int(tid)] = os.path.join(
                maps_dir, info.get("file", f"tile{tid:02d}.yaml"))
        return tiles

    def _parse_trigger_zones(self, zones_cfg):
        zones = []
        for z in zones_cfg:
            b = z.get("bounds", [0, 0, 0, 0])
            zones.append((
                b[0],
                b[1],
                b[2],
                b[3],
                z.get("from_tile"),
                z.get("to_tile"),
                z.get("heading"),
            ))
        return zones

    # ================== CALLBACKS ==================
    def _on_pose(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def _on_yaw(self, msg: Float32):
        self.current_yaw = msg.data

    # ================== TIMER CALLBACK ==================
    def _check_loop(self):
        if self.current_x is None or self.current_yaw is None:
            return

        if time.time() - self.last_switch_time < self.switch_cooldown:
            return

        x, y = self.current_x, self.current_y
        heading = self._yaw_to_heading(self.current_yaw)

        if self.verbose:
            self.get_logger().info(
                f"[CHECK] x={x:.2f} y={y:.2f} h={heading} tile={self.current_tile}"
            )

        for (
                x_min,
                x_max,
                y_min,
                y_max,
                from_tile,
                to_tile,
                req_heading,
        ) in self.trigger_zones:
            if from_tile != self.current_tile:
                continue
            if x_min <= x <= x_max and y_min <= y <= y_max and heading == req_heading:
                self.get_logger().info(
                    f"Trigger: ({x:.2f}, {y:.2f}) {heading} -> Tile {to_tile}")
                self._switch_tile(to_tile)
                return

    # ================== HEADING ==================
    def _yaw_to_heading(self, yaw):
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        if -0.785 <= yaw < 0.785:
            return "+x"
        elif 0.785 <= yaw < 2.356:
            return "+y"
        elif -2.356 <= yaw < -0.785:
            return "-y"
        else:
            return "-x"

    # ================== PUBLISHERS ==================
    def _publish_tile(self):
        msg = Int32()
        msg.data = self.current_tile
        self.current_tile_pub.publish(msg)

    def _publish_switch_time(self, ms):
        msg = Float32()
        msg.data = ms
        self.switch_time_pub.publish(msg)

    # ================== MAP SWITCHING ==================
    def _switch_tile(self, new_tile):
        if new_tile not in self.tiles:
            self.get_logger().error(f"Unknown tile: {new_tile}")
            return

        # Clean up any pending future from previous switch
        if self._pending_future is not None:
            self._pending_future = None

        self.get_logger().warn(f"Switching to TILE {new_tile}")

        if not self.map_loader.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("map_server unavailable")
            return

        # Start timing
        self._switch_start_time = time.perf_counter()
        self._switching_to_tile = new_tile

        # Async map load
        req = LoadMap.Request()
        req.map_url = self.tiles[new_tile]
        self._pending_future = self.map_loader.call_async(req)
        self._pending_future.add_done_callback(self._on_map_loaded)

        # Update state
        self.current_tile = new_tile
        self.last_switch_time = time.time()
        self._publish_tile()

    def _on_map_loaded(self, future):
        switch_ms = (time.perf_counter() - self._switch_start_time) * 1000

        try:
            future.result()
            self.get_logger().info(
                f"Tile {self._switching_to_tile} loaded in {switch_ms:.1f} ms")
        except Exception as e:
            self.get_logger().error(f"Map load failed: {e}")
            switch_ms = -1.0

        self._publish_switch_time(switch_ms)

        # Clear costmap AFTER load
        if self.costmap_clear.wait_for_service(timeout_sec=1.0):
            # Use reusable request object
            self.costmap_clear.call_async(self.costmap_clear_req)

        # Cleanup: clear future reference and hint GC
        self._pending_future = None
        gc.collect(0)  # Generation 0 only - fast


def main():
    rclpy.init()
    node = TileSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
