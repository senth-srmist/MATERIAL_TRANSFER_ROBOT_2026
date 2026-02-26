#!/usr/bin/env python3
"""
Mission Controller Service (Optimized)

Provides /navigate_to_room service for room-to-room navigation.
Uses event-driven tile monitoring instead of polling.

Changes from original:
- Event-driven tile switching (no polling loop)
- Proper async/await (no time.sleep blocking)
- Optimized BFS with parent map
- Single server connection check
- Proper log levels (DEBUG/INFO/WARN/ERROR)
- Memory-efficient data structures
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

import yaml
import math
from pathlib import Path
from collections import deque
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Int32

from mission_controller.srv import NavigateToRoom


@dataclass(slots=True)
class RoomInfo:
    """Memory-efficient room data structure."""

    tile: int
    x: float
    y: float
    description: str


@dataclass(slots=True)
class TileInfo:
    """Memory-efficient tile data structure."""

    bounds: Tuple[float, float, float, float]
    neighbors: Tuple[int, ...]


class MissionController(Node):

    def __init__(self):
        super().__init__("mission_controller")

        # Parameters
        self.declare_parameter("config_file", "")
        config_file = (self.get_parameter(
            "config_file").get_parameter_value().string_value)

        if not config_file:
            try:
                pkg_share = get_package_share_directory("tile_manager")
                config_file = str(
                    Path(pkg_share) / "config" / "tiles_config.yaml")
            except Exception:
                self.get_logger().fatal("Could not find tile_manager package")
                raise RuntimeError("tile_manager package not found")

        # Data structures (optimized)
        self.tiles: Dict[int, TileInfo] = {}
        self.connections: Dict[str, Dict] = {}
        self.rooms: Dict[str, RoomInfo] = {}
        self._load_config(config_file)

        # Callback groups
        self._service_cb_group = ReentrantCallbackGroup()
        self._subscription_cb_group = MutuallyExclusiveCallbackGroup()

        # Nav2 action client
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self._service_cb_group,
        )
        self._nav_server_ready = False

        # Tile state (event-driven)
        self._current_tile: Optional[int] = None
        self._target_tile: Optional[int] = None
        self._active_goal_handle = None

        # Subscription
        tile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._tile_sub = self.create_subscription(
            Int32,
            "/active_tile",
            self._active_tile_callback,
            tile_qos,
            callback_group=self._subscription_cb_group,
        )

        # Service
        self._srv = self.create_service(
            NavigateToRoom,
            "navigate_to_room",
            self._navigate_to_room_callback,
            callback_group=self._service_cb_group,
        )

        self.get_logger().info(
            f"MissionController ready: {len(self.tiles)} tiles, {len(self.rooms)} rooms"
        )

    def _load_config(self, config_path: str) -> None:
        """Load tiles, connections, and rooms from config."""
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            # Load tiles with neighbors and rooms
            for tile_id, tile_data in config.get("tiles", {}).items():
                tile_id = int(tile_id)
                bounds = tile_data.get("bounds", [0, 0, 0, 0])
                neighbors = tile_data.get("neighbors", [])

                self.tiles[tile_id] = TileInfo(
                    bounds=tuple(bounds),
                    neighbors=tuple(neighbors),
                )

                # Extract rooms
                for room_name, room_data in tile_data.get("rooms", {}).items():
                    if room_name == "placeholder":
                        continue
                    coords = room_data.get("coordinates", [0.0, 0.0])
                    self.rooms[room_name] = RoomInfo(
                        tile=tile_id,
                        x=coords[0],
                        y=coords[1],
                        description=room_data.get("description", ""),
                    )

            # Load connections
            for conn_key, conn_data in config.get("connections", {}).items():
                self.connections[conn_key] = {
                    "overlap": tuple(conn_data.get("overlap", [])),
                    "switch_points": conn_data.get("switch_points", {}),
                }

            self.get_logger().debug(
                f"Config loaded: {len(self.tiles)} tiles, "
                f"{len(self.connections)} connections, {len(self.rooms)} rooms"
            )

        except FileNotFoundError:
            self.get_logger().fatal(f"Config file not found: {config_path}")
            raise
        except yaml.YAMLError as e:
            self.get_logger().fatal(f"Invalid YAML in config: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            raise

    def _active_tile_callback(self, msg: Int32) -> None:
        """
        Event-driven tile update callback.
        Cancels active navigation if target tile is reached.
        """
        new_tile = msg.data

        if new_tile == self._current_tile:
            return  # No change, skip processing

        old_tile = self._current_tile
        self._current_tile = new_tile

        self.get_logger().debug(f"Tile changed: {old_tile} -> {new_tile}")

        # If we reached target tile, cancel active navigation
        if self._target_tile is not None and new_tile == self._target_tile:
            self.get_logger().info(
                f"Target tile {new_tile} reached, canceling navigation")
            if self._active_goal_handle is not None:
                self._active_goal_handle.cancel_goal_async()

    def _find_tile_sequence(self, start_tile: int,
                            goal_tile: int) -> List[int]:
        """
        BFS with parent map (memory-efficient).
        Returns shortest path between tiles.
        """
        if start_tile == goal_tile:
            return [start_tile]

        if start_tile not in self.tiles:
            self.get_logger().error(f"Unknown start tile: {start_tile}")
            return []

        # BFS with parent tracking (no list copying)
        parent: Dict[int, int] = {start_tile: -1}
        queue = deque([start_tile])

        while queue:
            current = queue.popleft()
            tile_info = self.tiles.get(current)

            if tile_info is None:
                continue

            for neighbor in tile_info.neighbors:
                if neighbor in parent:
                    continue  # Already visited

                parent[neighbor] = current

                if neighbor == goal_tile:
                    # Reconstruct path
                    path = []
                    node = goal_tile
                    while node != -1:
                        path.append(node)
                        node = parent[node]
                    path.reverse()
                    return path

                queue.append(neighbor)

        self.get_logger().warn(
            f"No path found: tile {start_tile} -> {goal_tile}")
        return []

    def _get_switch_point(self, from_tile: int,
                          to_tile: int) -> Optional[Tuple[float, float]]:
        """Get switch point coordinates for tile transition."""
        # Try both key formats
        for key in (f"{from_tile}-{to_tile}", f"{to_tile}-{from_tile}"):
            if key in self.connections:
                sp_key = f"{from_tile}_to_{to_tile}"
                coords = self.connections[key]["switch_points"].get(sp_key)
                if coords:
                    return (coords[0], coords[1])

        self.get_logger().warn(f"No switch point for {from_tile} -> {to_tile}")
        return None

    async def _ensure_nav_server_ready(self) -> bool:
        """Check Nav2 server once, cache result."""
        if self._nav_server_ready:
            return True

        self.get_logger().debug("Waiting for Nav2 action server...")

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available")
            return False

        self._nav_server_ready = True
        self.get_logger().debug("Nav2 action server connected")
        return True

    def _create_pose_goal(self,
                          x: float,
                          y: float,
                          yaw: float = 0.0) -> NavigateToPose.Goal:
        """Create navigation goal message."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # Precompute sin/cos only once per goal
        half_yaw = yaw * 0.5
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)

        return goal

    async def _navigate_to_switch_point(self, x: float, y: float,
                                        target_tile: int) -> Tuple[str, bool]:
        """
        Navigate to switch point with event-driven tile monitoring.

        Returns: (result, tile_switched)
            result: 'succeeded', 'canceled', 'failed'
            tile_switched: True if now in target_tile
        """
        if not await self._ensure_nav_server_ready():
            return ("failed", False)

        # Set target for event-driven cancellation
        self._target_tile = target_tile
        self._tile_switched_event.clear()

        goal = self._create_pose_goal(x, y)

        self.get_logger().debug(
            f"Sending goal: ({x:.2f}, {y:.2f}) -> tile {target_tile}")

        # Send goal
        goal_future = await self._nav_client.send_goal_async(goal)

        if not goal_future.accepted:
            self.get_logger().warn("Navigation goal rejected")
            self._target_tile = None
            return ("failed", False)

        self._active_goal_handle = goal_future

        # Wait for result - callback handles tile switch detection
        result_future = goal_future.get_result_async()

        # Block until nav completes or callback cancels (no polling needed)
        # The _active_tile_callback will cancel goal if target tile reached
        result = await result_future

        self._active_goal_handle = None
        self._target_tile = None

        # Check result
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return ("succeeded", self._current_tile == target_tile)
        elif result.status == GoalStatus.STATUS_CANCELED:
            # Canceled by tile switch callback
            return ("canceled", self._current_tile == target_tile)
        else:
            self.get_logger().warn(
                f"Navigation failed with status: {result.status}")
            return ("failed", False)

    async def _navigate_to_final(self, x: float, y: float) -> bool:
        """Navigate to final destination (simple, no tile monitoring)."""
        if not await self._ensure_nav_server_ready():
            return False

        goal = self._create_pose_goal(x, y)

        self.get_logger().debug(f"Sending final goal: ({x:.2f}, {y:.2f})")

        goal_handle = await self._nav_client.send_goal_async(goal)

        if not goal_handle.accepted:
            self.get_logger().warn("Final goal rejected")
            return False

        result = await goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def _navigate_to_room_callback(self, request, response):
        """Service callback for room-to-room navigation."""
        room_name = request.room_name
        start_time = self.get_clock().now()
        tiles_traversed = []

        self.get_logger().info(f"Navigation requested: {room_name}")

        # 1. Validate room
        room = self.rooms.get(room_name)
        if room is None:
            self.get_logger().warn(f"Unknown room requested: {room_name}")
            response.success = False
            response.message = f"Unknown room: {room_name}"
            response.tiles_traversed = []
            response.duration_seconds = 0.0
            return response

        goal_tile = room.tile

        # 2. Validate current tile
        if self._current_tile is None:
            self.get_logger().error(
                "No active tile - is /active_tile being published?")
            response.success = False
            response.message = "No active tile received"
            response.tiles_traversed = []
            response.duration_seconds = 0.0
            return response

        current_tile = self._current_tile
        tiles_traversed.append(current_tile)

        self.get_logger().debug(
            f"Current tile: {current_tile}, Goal tile: {goal_tile}")

        # 3. Find tile sequence
        tile_sequence = self._find_tile_sequence(current_tile, goal_tile)
        if not tile_sequence:
            response.success = False
            response.message = f"No path from tile {current_tile} to {goal_tile}"
            response.tiles_traversed = tiles_traversed
            response.duration_seconds = 0.0
            return response

        self.get_logger().debug(f"Tile sequence: {tile_sequence}")

        # 4. Navigate through tiles
        for i in range(len(tile_sequence) - 1):
            from_tile = tile_sequence[i]
            to_tile = tile_sequence[i + 1]

            switch_point = self._get_switch_point(from_tile, to_tile)
            if switch_point is None:
                response.success = False
                response.message = f"No switch point: {from_tile} -> {to_tile}"
                response.tiles_traversed = tiles_traversed
                response.duration_seconds = self._elapsed_seconds(start_time)
                return response

            self.get_logger().info(
                f"Navigating: tile {from_tile} -> {to_tile}")

            result, tile_switched = await self._navigate_to_switch_point(
                switch_point[0], switch_point[1], to_tile)

            self.get_logger().debug(
                f"Result: {result}, switched: {tile_switched}")

            if tile_switched:
                tiles_traversed.append(to_tile)
                continue

            # Failed to switch
            self.get_logger().error(f"Failed to reach tile {to_tile}")
            response.success = False
            response.message = f"Failed to navigate to tile {to_tile}"
            response.tiles_traversed = tiles_traversed
            response.duration_seconds = self._elapsed_seconds(start_time)
            return response

        # 5. Navigate to final room
        self.get_logger().info(f"Navigating to room: {room_name}")

        if not await self._navigate_to_final(room.x, room.y):
            response.success = False
            response.message = f"Failed to reach room {room_name}"
            response.tiles_traversed = tiles_traversed
            response.duration_seconds = self._elapsed_seconds(start_time)
            return response

        # Success
        duration = self._elapsed_seconds(start_time)
        self.get_logger().info(
            f"Navigation complete: {room_name} in {duration:.1f}s")

        response.success = True
        response.message = f"Reached {room_name}"
        response.tiles_traversed = tiles_traversed
        response.duration_seconds = duration
        return response

    def _elapsed_seconds(self, start_time) -> float:
        """Calculate elapsed time in seconds."""
        return (self.get_clock().now() - start_time).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
