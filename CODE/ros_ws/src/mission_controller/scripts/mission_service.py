#!/usr/bin/env python3
"""
Mission Controller Service

Provides /navigate_to_room service for room-to-room navigation.
Uses event-driven tile monitoring instead of polling.

Features:
- Event-driven tile switching (no polling loop)
- Proper async/await (no blocking calls)
- Mission queue (multiple room goals processed sequentially)
- Retry logic for tile transitions
- Thread-safe goal handle access
- Graceful shutdown with active goal cancellation
- Nav2 server health checking (no stale cache)

Usage:
    ros2 run mission_controller mission_service
    ros2 service call /navigate_to_room mission_controller/srv/NavigateToRoom "{room_name: 'kitchen'}"
"""

import asyncio
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import yaml

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Int32

from mission_controller.srv import NavigateToRoom


# ============================================================================
# Data structures
# ============================================================================

@dataclass(slots=True)
class RoomInfo:
    """Room data."""
    tile: int
    x: float
    y: float
    description: str


@dataclass(slots=True)
class TileInfo:
    """Tile data."""
    bounds: Tuple[float, float, float, float]
    neighbors: Tuple[int, ...]


@dataclass(slots=True)
class ConnectionInfo:
    """Connection data between two tiles."""
    overlap: Tuple[float, ...]
    switch_points: Dict


# ============================================================================
# Mission Controller
# ============================================================================

class MissionController(Node):

    # Max retries per tile transition before aborting
    TILE_TRANSITION_MAX_RETRIES = 3

    def __init__(self):
        super().__init__("mission_controller")

        # Parameters
        self.declare_parameter("config_file", "")
        config_file = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )

        if not config_file:
            try:
                pkg_share = get_package_share_directory("tile_manager")
                config_file = str(
                    Path(pkg_share) / "config" / "tiles_config.yaml"
                )
            except Exception:
                self.get_logger().fatal("Could not find tile_manager package")
                raise RuntimeError("tile_manager package not found")

        # Data structures
        self.tiles: Dict[int, TileInfo] = {}
        self.connections: Dict[str, ConnectionInfo] = {}
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

        # Tile state (event-driven)
        self._current_tile: Optional[int] = None
        self._target_tile: Optional[int] = None

        # Thread-safe goal handle access
        self._goal_lock = threading.Lock()
        self._active_goal_handle = None

        # Mission queue
        self._mission_queue: deque = deque()
        self._mission_active = False
        self._mission_lock = threading.Lock()

        # Shutdown flag
        self._shutting_down = False

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
            f"MissionController ready: {len(self.tiles)} tiles, "
            f"{len(self.rooms)} rooms"
        )

    # ========================================================================
    # Config loading
    # ========================================================================

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
                self.connections[conn_key] = ConnectionInfo(
                    overlap=tuple(conn_data.get("overlap", [])),
                    switch_points=conn_data.get("switch_points", {}),
                )

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

    # ========================================================================
    # Tile monitoring (event-driven)
    # ========================================================================

    def _active_tile_callback(self, msg: Int32) -> None:
        """
        Event-driven tile update callback.
        Cancels active navigation if target tile is reached.
        """
        new_tile = msg.data

        if new_tile == self._current_tile:
            return

        old_tile = self._current_tile
        self._current_tile = new_tile

        self.get_logger().debug(f"Tile changed: {old_tile} -> {new_tile}")

        # If we reached target tile, cancel active navigation
        if self._target_tile is not None and new_tile == self._target_tile:
            self.get_logger().info(
                f"Target tile {new_tile} reached, canceling navigation"
            )
            with self._goal_lock:
                if self._active_goal_handle is not None:
                    cancel_future = (
                        self._active_goal_handle.cancel_goal_async()
                    )
                    cancel_future.add_done_callback(self._on_cancel_done)

    def _on_cancel_done(self, future) -> None:
        """Log cancel result for diagnostics."""
        try:
            result = future.result()
            if result is not None:
                self.get_logger().debug("Goal cancel acknowledged")
            else:
                self.get_logger().warn("Goal cancel returned None")
        except Exception as e:
            self.get_logger().warn(f"Goal cancel failed: {e}")

    # ========================================================================
    # Path finding
    # ========================================================================

    def _find_tile_sequence(
        self, start_tile: int, goal_tile: int
    ) -> List[int]:
        """
        BFS with parent map.
        Returns shortest path between tiles, or empty list if unreachable.
        """
        if start_tile == goal_tile:
            return [start_tile]

        if start_tile not in self.tiles:
            self.get_logger().error(f"Unknown start tile: {start_tile}")
            return []

        parent: Dict[int, int] = {start_tile: -1}
        queue = deque([start_tile])

        while queue:
            current = queue.popleft()
            tile_info = self.tiles.get(current)

            if tile_info is None:
                continue

            for neighbor in tile_info.neighbors:
                if neighbor in parent:
                    continue

                parent[neighbor] = current

                if neighbor == goal_tile:
                    # Reconstruct path
                    path = []
                    trace = goal_tile
                    while trace != -1:
                        path.append(trace)
                        trace = parent[trace]
                    path.reverse()
                    return path

                queue.append(neighbor)

        self.get_logger().warn(
            f"No path found: tile {start_tile} -> {goal_tile}"
        )
        return []

    def _get_switch_point(
        self, from_tile: int, to_tile: int
    ) -> Optional[Tuple[float, float]]:
        """Get switch point coordinates for tile transition."""
        for key in (f"{from_tile}-{to_tile}", f"{to_tile}-{from_tile}"):
            if key in self.connections:
                sp_key = f"{from_tile}_to_{to_tile}"
                coords = self.connections[key].switch_points.get(sp_key)
                if coords:
                    return (coords[0], coords[1])

        self.get_logger().warn(
            f"No switch point for {from_tile} -> {to_tile}"
        )
        return None

    # ========================================================================
    # Nav2 interaction
    # ========================================================================

    async def _ensure_nav_server_ready(self) -> bool:
        """
        Check Nav2 server availability (non-blocking).
        Checks every time — no stale cache after Nav2 restart.
        """
        if self._nav_client.server_is_ready():
            return True

        self.get_logger().debug("Waiting for Nav2 action server...")

        # Non-blocking poll loop (Humble compatible, no wait_for_server_async)
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=10)
        while self.get_clock().now() < deadline:
            if self._nav_client.server_is_ready():
                self.get_logger().debug("Nav2 action server connected")
                return True
            await asyncio.sleep(0.2)

        self.get_logger().error("Nav2 action server not available after 10s")
        return False

    def _create_pose_goal(
        self, x: float, y: float, yaw: float = 0.0
    ) -> NavigateToPose.Goal:
        """Create navigation goal message."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        half_yaw = yaw * 0.5
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)

        return goal

    async def _navigate_to_switch_point(
        self, x: float, y: float, target_tile: int
    ) -> Tuple[str, bool]:
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

        goal = self._create_pose_goal(x, y)

        self.get_logger().debug(
            f"Sending goal: ({x:.2f}, {y:.2f}) -> tile {target_tile}"
        )

        # Send goal
        goal_future = await self._nav_client.send_goal_async(goal)

        if not goal_future.accepted:
            self.get_logger().warn("Navigation goal rejected")
            self._target_tile = None
            return ("failed", False)

        with self._goal_lock:
            self._active_goal_handle = goal_future

        # Wait for result — callback handles tile switch detection & cancel
        try:
            result_future = goal_future.get_result_async()
            result = await result_future
        except Exception as e:
            self.get_logger().error(f"Navigation result error: {e}")
            with self._goal_lock:
                self._active_goal_handle = None
            self._target_tile = None
            return ("failed", False)

        with self._goal_lock:
            self._active_goal_handle = None
        self._target_tile = None

        # Check result
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return ("succeeded", self._current_tile == target_tile)
        elif result.status == GoalStatus.STATUS_CANCELED:
            return ("canceled", self._current_tile == target_tile)
        else:
            self.get_logger().warn(
                f"Navigation failed with status: {result.status}"
            )
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

        with self._goal_lock:
            self._active_goal_handle = goal_handle

        try:
            result = await goal_handle.get_result_async()
        except Exception as e:
            self.get_logger().error(f"Final navigation error: {e}")
            with self._goal_lock:
                self._active_goal_handle = None
            return False

        with self._goal_lock:
            self._active_goal_handle = None

        return result.status == GoalStatus.STATUS_SUCCEEDED

    # ========================================================================
    # Mission execution
    # ========================================================================

    async def _execute_mission(
        self, room_name: str
    ) -> Tuple[bool, str, List[int], float]:
        """
        Execute a single room navigation mission.

        Returns: (success, message, tiles_traversed, duration_seconds)
        """
        start_time = self.get_clock().now()
        tiles_traversed = []

        # 1. Validate room
        room = self.rooms.get(room_name)
        if room is None:
            self.get_logger().warn(f"Unknown room requested: {room_name}")
            return (False, f"Unknown room: {room_name}", [], 0.0)

        goal_tile = room.tile

        # 2. Validate current tile
        if self._current_tile is None:
            self.get_logger().error(
                "No active tile - is /active_tile being published?"
            )
            return (False, "No active tile received", [], 0.0)

        current_tile = self._current_tile
        tiles_traversed.append(current_tile)

        self.get_logger().debug(
            f"Current tile: {current_tile}, Goal tile: {goal_tile}"
        )

        # 3. Find tile sequence
        tile_sequence = self._find_tile_sequence(current_tile, goal_tile)
        if not tile_sequence:
            return (
                False,
                f"No path from tile {current_tile} to {goal_tile}",
                tiles_traversed,
                0.0,
            )

        self.get_logger().debug(f"Tile sequence: {tile_sequence}")

        # 4. Navigate through tiles with retry
        for i in range(len(tile_sequence) - 1):
            from_tile = tile_sequence[i]
            to_tile = tile_sequence[i + 1]

            switch_point = self._get_switch_point(from_tile, to_tile)
            if switch_point is None:
                return (
                    False,
                    f"No switch point: {from_tile} -> {to_tile}",
                    tiles_traversed,
                    self._elapsed_seconds(start_time),
                )

            # Retry loop for tile transitions
            transition_success = False
            for attempt in range(1, self.TILE_TRANSITION_MAX_RETRIES + 1):
                self.get_logger().info(
                    f"Navigating: tile {from_tile} -> {to_tile}"
                    + (f" (attempt {attempt})" if attempt > 1 else "")
                )

                result, tile_switched = await self._navigate_to_switch_point(
                    switch_point[0], switch_point[1], to_tile
                )

                self.get_logger().debug(
                    f"Result: {result}, switched: {tile_switched}"
                )

                if tile_switched:
                    tiles_traversed.append(to_tile)
                    transition_success = True
                    break

                if self._shutting_down:
                    return (
                        False,
                        "Shutdown requested",
                        tiles_traversed,
                        self._elapsed_seconds(start_time),
                    )

                if attempt < self.TILE_TRANSITION_MAX_RETRIES:
                    self.get_logger().warn(
                        f"Tile transition {from_tile} -> {to_tile} failed "
                        f"(attempt {attempt}/{self.TILE_TRANSITION_MAX_RETRIES})"
                        ", retrying..."
                    )
                    # Brief pause before retry
                    await asyncio.sleep(1.0)

            if not transition_success:
                self.get_logger().error(
                    f"Failed to reach tile {to_tile} after "
                    f"{self.TILE_TRANSITION_MAX_RETRIES} attempts"
                )
                return (
                    False,
                    f"Failed to navigate to tile {to_tile} after "
                    f"{self.TILE_TRANSITION_MAX_RETRIES} retries",
                    tiles_traversed,
                    self._elapsed_seconds(start_time),
                )

        # 5. Navigate to final room
        self.get_logger().info(f"Navigating to room: {room_name}")

        if not await self._navigate_to_final(room.x, room.y):
            return (
                False,
                f"Failed to reach room {room_name}",
                tiles_traversed,
                self._elapsed_seconds(start_time),
            )

        # Success
        duration = self._elapsed_seconds(start_time)
        self.get_logger().info(
            f"Navigation complete: {room_name} in {duration:.1f}s"
        )
        return (True, f"Reached {room_name}", tiles_traversed, duration)

    async def _navigate_to_room_callback(self, request, response):
        """
        Service callback for room-to-room navigation.
        Queues missions and processes them sequentially.
        """
        room_name = request.room_name
        self.get_logger().info(f"Navigation requested: {room_name}")

        # Quick validation before queueing
        if room_name not in self.rooms:
            response.success = False
            response.message = f"Unknown room: {room_name}"
            response.tiles_traversed = []
            response.duration_seconds = 0.0
            return response

        # Check if a mission is already active — queue this one
        with self._mission_lock:
            if self._mission_active:
                queue_pos = len(self._mission_queue) + 1
                self.get_logger().info(
                    f"Mission queued: {room_name} (position {queue_pos})"
                )

                # Create a future to wait on
                loop = asyncio.get_event_loop()
                result_future = loop.create_future()
                self._mission_queue.append((room_name, result_future))

                # Wait for our turn (this awaits without blocking the executor)
                success, message, tiles_traversed, duration = (
                    await result_future
                )

                response.success = success
                response.message = message
                response.tiles_traversed = tiles_traversed
                response.duration_seconds = duration
                return response

            # We're first — mark active
            self._mission_active = True

        # Execute our mission
        try:
            success, message, tiles_traversed, duration = (
                await self._execute_mission(room_name)
            )

            response.success = success
            response.message = message
            response.tiles_traversed = tiles_traversed
            response.duration_seconds = duration

        except Exception as e:
            self.get_logger().error(f"Mission failed with exception: {e}")
            response.success = False
            response.message = f"Internal error: {e}"
            response.tiles_traversed = []
            response.duration_seconds = 0.0

        # Process queued missions
        await self._process_queue()

        return response

    async def _process_queue(self) -> None:
        """Process queued missions sequentially."""
        while True:
            with self._mission_lock:
                if not self._mission_queue:
                    self._mission_active = False
                    return

                room_name, result_future = self._mission_queue.popleft()

            if self._shutting_down:
                if not result_future.done():
                    result_future.set_result(
                        (False, "Shutdown requested", [], 0.0)
                    )
                continue

            self.get_logger().info(
                f"Processing queued mission: {room_name}"
            )

            try:
                result = await self._execute_mission(room_name)
                if not result_future.done():
                    result_future.set_result(result)
            except Exception as e:
                self.get_logger().error(
                    f"Queued mission '{room_name}' failed: {e}"
                )
                if not result_future.done():
                    result_future.set_result(
                        (False, f"Internal error: {e}", [], 0.0)
                    )

    # ========================================================================
    # Utilities
    # ========================================================================

    def _elapsed_seconds(self, start_time) -> float:
        """Calculate elapsed time in seconds."""
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    # ========================================================================
    # Shutdown
    # ========================================================================

    def shutdown(self) -> None:
        """Graceful shutdown: cancel active goal, drain queue."""
        self._shutting_down = True

        # Cancel active navigation goal
        with self._goal_lock:
            if self._active_goal_handle is not None:
                self.get_logger().info("Canceling active navigation goal...")
                try:
                    self._active_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warn(f"Cancel during shutdown failed: {e}")

        # Drain mission queue
        with self._mission_lock:
            while self._mission_queue:
                _, result_future = self._mission_queue.popleft()
                if not result_future.done():
                    result_future.set_result(
                        (False, "Shutdown requested", [], 0.0)
                    )
            self._mission_active = False


# ============================================================================
# Main
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    # 3 threads: service handler, subscription callback, queue processing
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
