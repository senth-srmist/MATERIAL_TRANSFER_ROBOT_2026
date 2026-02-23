#!/usr/bin/env python3
"""
Mission Controller Service

Provides /navigate_to_room service for room-to-room navigation.
Handles tile sequencing, waypoint generation, and Nav2 integration.

Key feature: Subscribes to /active_tile topic (published by TileCheckAction).
When tile switches, cancels current goal and moves to next waypoint.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

import yaml
import time
import math
from pathlib import Path
from collections import deque

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Int32

from mission_controller.srv import NavigateToRoom


class MissionController(Node):
    def __init__(self):
        super().__init__("mission_controller")

        # Parameters
        self.declare_parameter("config_file", "")
        self.declare_parameter("tile_check_interval", 0.2)

        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        self.tile_check_interval = (
            self.get_parameter("tile_check_interval").get_parameter_value().double_value
        )

        if not config_file:
            try:
                pkg_share = get_package_share_directory("tile_manager")
                config_file = str(Path(pkg_share) / "config" / "tiles_config.yaml")
            except Exception:
                self.get_logger().error("Could not find tile_manager package")

        # Load config
        self.tiles = {}
        self.connections = {}
        self.rooms = {}
        if config_file:
            self.load_config(config_file)

        # Callback groups - separate so they don't block each other
        self.service_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()

        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.service_callback_group
        )

        # Active tile state (from /active_tile topic published by TileCheckAction)
        self.current_tile = None
        self.tile_valid = False

        # Subscription in its own callback group
        # Use volatile durability to match BT plugin publisher
        tile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.tile_sub = self.create_subscription(
            Int32,
            "/active_tile",
            self.active_tile_callback,
            tile_qos,
            callback_group=self.subscription_callback_group,
        )

        # NavigateToRoom service
        self.srv = self.create_service(
            NavigateToRoom,
            "navigate_to_room",
            self.navigate_to_room_callback,
            callback_group=self.service_callback_group,
        )

        self.get_logger().info(
            f"MissionController ready: {len(self.tiles)} tiles, {len(self.rooms)} rooms"
        )
        self.get_logger().info("Subscribed to /active_tile topic")

    def load_config(self, config_path: str):
        """Load tiles, connections, and rooms from config."""
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            # Load tiles with neighbors and rooms
            for tile_id, tile_data in config.get("tiles", {}).items():
                tile_id = int(tile_id)
                self.tiles[tile_id] = {
                    "bounds": tile_data.get("bounds", []),
                    "neighbors": tile_data.get("neighbors", []),
                }

                # Extract rooms
                for room_name, room_data in tile_data.get("rooms", {}).items():
                    if room_name != "placeholder":
                        self.rooms[room_name] = {
                            "tile": tile_id,
                            "coordinates": room_data.get("coordinates", []),
                            "description": room_data.get("description", ""),
                        }

            # Load connections
            for conn_key, conn_data in config.get("connections", {}).items():
                self.connections[conn_key] = {
                    "overlap": conn_data.get("overlap", []),
                    "switch_points": conn_data.get("switch_points", {}),
                }

            self.get_logger().info(
                f"Loaded config: {len(self.tiles)} tiles, {len(self.rooms)} rooms"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

    def active_tile_callback(self, msg: Int32):
        old_tile = self.current_tile
        self.current_tile = msg.data
        self.tile_valid = True
        if old_tile != self.current_tile:
            self.get_logger().info(f"Active tile changed: {old_tile} -> {self.current_tile}")

    def find_tile_sequence(self, start_tile: int, goal_tile: int) -> list:
        """BFS to find shortest path between tiles."""
        if start_tile == goal_tile:
            return [start_tile]

        queue = deque([[start_tile]])
        visited = {start_tile}

        while queue:
            path = queue.popleft()
            current = path[-1]

            for neighbor in self.tiles.get(current, {}).get("neighbors", []):
                if neighbor == goal_tile:
                    return path + [neighbor]

                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(path + [neighbor])

        return []  # No path found

    def get_switch_point(self, from_tile: int, to_tile: int) -> list:
        """Get switch point for transition from from_tile to to_tile."""
        # Try both key formats
        key1 = f"{from_tile}-{to_tile}"
        key2 = f"{to_tile}-{from_tile}"

        if key1 in self.connections:
            sp_key = f"{from_tile}_to_{to_tile}"
            return self.connections[key1]["switch_points"].get(sp_key, [])
        elif key2 in self.connections:
            sp_key = f"{from_tile}_to_{to_tile}"
            return self.connections[key2]["switch_points"].get(sp_key, [])

        return []

    async def send_nav_goal_with_tile_monitor(
        self, x: float, y: float, target_tile: int, yaw: float = 0.0
    ) -> tuple:
        """
        Send navigation goal and monitor tile in real-time.

        Returns: (result, tile_switched)
            result: 'succeeded', 'canceled', 'failed'
            tile_switched: True if we're now in target_tile
        """
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            return "failed", False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(
            f"Sending nav goal: ({x:.2f}, {y:.2f}) -> target tile {target_tile}"
        )

        send_goal_future = await self.nav_client.send_goal_async(goal)

        if not send_goal_future.accepted:
            self.get_logger().warn("Nav goal rejected")
            return "failed", False

        goal_handle = send_goal_future

        # Get the result future
        result_future = goal_handle.get_result_async()

        # Monitor tile while navigating
        while not result_future.done():
            # Check if tile switched (updated by callback)
            if self.tile_valid and self.current_tile == target_tile:
                self.get_logger().info(
                    f"Tile switched to {target_tile}! Canceling nav goal."
                )
                goal_handle.cancel_goal_async()
                return "canceled", True

            # Short sleep - executor will process callbacks in between
            time.sleep(self.tile_check_interval)

        # Goal completed - check result
        result = result_future.result()
        current_tile = self.current_tile
        valid = self.tile_valid
        self.get_logger().info(
            f"Nav goal completed. Status: {result.status}, Current tile: {current_tile}, Target: {target_tile}"
        )

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            tile_switched = valid and current_tile == target_tile
            if not tile_switched:
                # Give tile switch a moment to process
                time.sleep(0.5)
                current_tile = self.current_tile
                tile_switched = self.tile_valid and current_tile == target_tile
                self.get_logger().info(
                    f"After delay - Current tile: {current_tile}, Switched: {tile_switched}"
                )
            return "succeeded", tile_switched
        else:
            return "failed", (valid and current_tile == target_tile)

    async def send_nav_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Send navigation goal to Nav2 (simple version for final destination). Returns True if succeeded."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f"Sending final nav goal: ({x:.2f}, {y:.2f})")

        goal_handle = await self.nav_client.send_goal_async(goal)

        if not goal_handle.accepted:
            self.get_logger().warn("Nav goal rejected")
            return False

        result = await goal_handle.get_result_async()

        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def navigate_to_room_callback(self, request, response):
        """Main service callback for room-to-room navigation."""
        start_time = time.time()
        room_name = request.room_name
        tiles_traversed = []

        self.get_logger().info(f"Navigation requested to: {room_name}")

        # 1. Look up room
        if room_name not in self.rooms:
            response.success = False
            response.message = f"Unknown room: {room_name}"
            response.tiles_traversed = []
            response.duration_seconds = 0.0
            return response

        room = self.rooms[room_name]
        goal_coords = room["coordinates"]
        goal_tile = room["tile"]

        # 2. Get current tile
        if not self.tile_valid or self.current_tile is None:
            response.success = False
            response.message = "No active tile received yet. Is /active_tile being published?"
            response.tiles_traversed = []
            response.duration_seconds = 0.0
            return response

        current_tile = self.current_tile
        tiles_traversed.append(current_tile)
        self.get_logger().info(f"Current tile: {current_tile}, Goal tile: {goal_tile}")

        # 3. Generate tile sequence
        tile_sequence = self.find_tile_sequence(current_tile, goal_tile)
        if not tile_sequence:
            response.success = False
            response.message = f"No path from tile {current_tile} to tile {goal_tile}"
            response.tiles_traversed = tiles_traversed
            response.duration_seconds = 0.0
            return response

        self.get_logger().info(f"Tile sequence: {tile_sequence}")

        # 4. Navigate through tile transitions
        for i in range(len(tile_sequence) - 1):
            from_tile = tile_sequence[i]
            to_tile = tile_sequence[i + 1]

            switch_point = self.get_switch_point(from_tile, to_tile)
            if not switch_point:
                self.get_logger().error(f"No switch point for {from_tile} -> {to_tile}")
                response.success = False
                response.message = f"No switch point for tile {from_tile} to {to_tile}"
                response.tiles_traversed = tiles_traversed
                response.duration_seconds = time.time() - start_time
                return response

            self.get_logger().info(
                f"Navigating to switch point: {switch_point} (tile {from_tile} -> {to_tile})"
            )

            # Send nav goal with tile monitoring
            result, tile_switched = await self.send_nav_goal_with_tile_monitor(
                switch_point[0], switch_point[1], to_tile
            )

            self.get_logger().info(f"Nav result: {result}, Tile switched: {tile_switched}")

            if tile_switched:
                # Successfully in next tile (either reached goal or tile switched early)
                tiles_traversed.append(to_tile)
                self.get_logger().info(f"Now in tile {to_tile}")
                continue

            # Tile didn't switch - navigation failed
            response.success = False
            response.message = f"Failed to navigate to tile {to_tile}"
            response.tiles_traversed = tiles_traversed
            response.duration_seconds = time.time() - start_time
            return response

        # 5. Navigate to final room coordinates
        self.get_logger().info(f"Navigating to room {room_name} at {goal_coords}")
        nav_success = await self.send_nav_goal(goal_coords[0], goal_coords[1])

        if not nav_success:
            response.success = False
            response.message = f"Failed to reach room {room_name}"
            response.tiles_traversed = tiles_traversed
            response.duration_seconds = time.time() - start_time
            return response

        # Success!
        response.success = True
        response.message = f"Reached {room_name}"
        response.tiles_traversed = tiles_traversed
        response.duration_seconds = time.time() - start_time

        self.get_logger().info(
            f"Navigation complete: {room_name} in {response.duration_seconds:.1f}s"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    # Use MultiThreadedExecutor so subscription callbacks run while service is processing
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
