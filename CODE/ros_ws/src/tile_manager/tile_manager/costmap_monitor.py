#!/usr/bin/env python3
"""
Costmap Monitor

Verifies that costmaps properly resize when tiles are switched.
Monitors:
    - /map dimensions (from map_server)
    - /global_costmap/costmap dimensions
    - Memory usage correlation

Usage:
    ros2 run tile_manager costmap_monitor
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32


class CostmapMonitor(Node):
    def __init__(self):
        super().__init__("costmap_monitor")
        
        # Track dimensions
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        
        self.costmap_width = 0
        self.costmap_height = 0
        self.costmap_resolution = 0.0
        
        self.current_tile = 0
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self._on_map, 
            10
        )
        self.create_subscription(
            OccupancyGrid, 
            '/global_costmap/costmap', 
            self._on_costmap, 
            10
        )
        self.create_subscription(
            Int32,
            '/benchmark/current_tile',
            self._on_tile,
            10
        )
        
        # Timer for periodic report
        self.create_timer(2.0, self._report)
        
        self.get_logger().info("Costmap Monitor started")
        self.get_logger().info("Watching /map and /global_costmap/costmap dimensions")

    def _on_tile(self, msg: Int32):
        if msg.data != self.current_tile:
            self.get_logger().warn(f"=== TILE SWITCH: {self.current_tile} -> {msg.data} ===")
            self.current_tile = msg.data

    def _on_map(self, msg: OccupancyGrid):
        new_width = msg.info.width
        new_height = msg.info.height
        new_res = msg.info.resolution
        
        if new_width != self.map_width or new_height != self.map_height:
            self.get_logger().info(
                f"[/map] Changed: {self.map_width}x{self.map_height} -> "
                f"{new_width}x{new_height} @ {new_res}m/cell"
            )
            
            # Calculate memory
            pixels = new_width * new_height
            mem_kb = pixels / 1024
            self.get_logger().info(f"[/map] Pixels: {pixels:,} = {mem_kb:.1f} KB raw")
            
        self.map_width = new_width
        self.map_height = new_height
        self.map_resolution = new_res

    def _on_costmap(self, msg: OccupancyGrid):
        new_width = msg.info.width
        new_height = msg.info.height
        new_res = msg.info.resolution
        
        if new_width != self.costmap_width or new_height != self.costmap_height:
            self.get_logger().info(
                f"[/global_costmap] Changed: {self.costmap_width}x{self.costmap_height} -> "
                f"{new_width}x{new_height} @ {new_res}m/cell"
            )
            
            # Calculate memory (costmap typically has multiple layers)
            pixels = new_width * new_height
            # Costmap2D typically has: master + static + obstacle + inflation layers
            # Each layer is 1 byte per cell, plus some float layers
            estimated_mem_kb = (pixels * 5) / 1024  # ~5 bytes per cell estimate
            self.get_logger().info(
                f"[/global_costmap] Pixels: {pixels:,} = ~{estimated_mem_kb:.1f} KB estimated"
            )
            
        self.costmap_width = new_width
        self.costmap_height = new_height
        self.costmap_resolution = new_res

    def _report(self):
        if self.map_width == 0:
            return
            
        self.get_logger().info("-" * 50)
        self.get_logger().info(f"Tile: {self.current_tile}")
        self.get_logger().info(
            f"  /map:            {self.map_width}x{self.map_height} "
            f"({self.map_width * self.map_height:,} cells)"
        )
        self.get_logger().info(
            f"  /global_costmap: {self.costmap_width}x{self.costmap_height} "
            f"({self.costmap_width * self.costmap_height:,} cells)"
        )
        
        # Check if they match
        if self.costmap_width != self.map_width or self.costmap_height != self.map_height:
            self.get_logger().warn(
                f"  ⚠️  MISMATCH! Costmap dimensions don't match map!"
            )
            self.get_logger().warn(
                f"      Map: {self.map_width}x{self.map_height}, "
                f"Costmap: {self.costmap_width}x{self.costmap_height}"
            )
        else:
            self.get_logger().info(f"  ✓ Dimensions match")


def main():
    rclpy.init()
    node = CostmapMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
