#!/usr/bin/env python3
"""
Mission Controller Service

Provides /navigate_to_room service for room-to-room navigation.
Handles tile sequencing, waypoint generation, and Nav2 integration.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import yaml
import time
import math
from pathlib import Path
from collections import deque

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

from mission_controller.srv import NavigateToRoom
from tile_manager.srv import GetCurrentTile


class MissionController(Node):
    def __init__(self):
        super().__init__("mission_controller")

        # Parameters
        self.declare_parameter("config_file", "")

        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
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

        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()

        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.callback_group
        )

        # GetCurrentTile service client
        self.tile_client = self.create_client(
            GetCurrentTile, "get_current_tile", callback_group=self.callback_group
        )

        # NavigateToRoom service
        self.srv = self.create_service(
            NavigateToRoom,
            "navigate_to_room",
            self.navigate_to_room_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"MissionController ready: {len(self.tiles)} tiles, {len(self.rooms)} rooms"
        )

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

    def get_current_tile(self) -> tuple:
        """Call GetCurrentTile service. Returns (tile_id, x, y, yaw, valid)."""
        if not self.tile_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("GetCurrentTile service not available")
            return -1, 0.0, 0.0, 0.0, False

        request = GetCurrentTile.Request()
        future = self.tile_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            r = future.result()
            return r.tile_id, r.x, r.y, r.yaw, r.valid
        return -1, 0.0, 0.0, 0.0, False

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

    async def send_nav_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Send navigation goal to Nav2. Returns True if succeeded."""
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

        self.get_logger().info(f"Sending nav goal: ({x:.2f}, {y:.2f})")

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
        current_tile, cur_x, cur_y, cur_yaw, valid = self.get_current_tile()
        if not valid or current_tile == -1:
            response.success = False
            response.message = "Could not determine current tile"
            response.tiles_traversed = []
            response.duration_seconds = 0.0
            return response

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

            self.get_logger().info(f"Navigating to switch point: {switch_point}")

            # Send nav goal to switch point
            nav_success = await self.send_nav_goal(switch_point[0], switch_point[1])

            # Check if we're now in the target tile (regardless of nav result)
            new_tile, _, _, _, valid = self.get_current_tile()

            if valid and new_tile == to_tile:
                # Successfully in next tile
                tiles_traversed.append(to_tile)
                self.get_logger().info(f"Reached tile {to_tile}")
                continue

            # If nav failed and not in target tile, try alternate or fail
            if not nav_success:
                # One more check after a brief pause
                time.sleep(0.5)
                new_tile, _, _, _, valid = self.get_current_tile()

                if valid and new_tile == to_tile:
                    tiles_traversed.append(to_tile)
                    self.get_logger().info(
                        f"Reached tile {to_tile} (after delay check)"
                    )
                    continue

                response.success = False
                response.message = f"Failed to navigate to tile {to_tile}"
                response.tiles_traversed = tiles_traversed
                response.duration_seconds = time.time() - start_time
                return response

            tiles_traversed.append(to_tile)

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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
