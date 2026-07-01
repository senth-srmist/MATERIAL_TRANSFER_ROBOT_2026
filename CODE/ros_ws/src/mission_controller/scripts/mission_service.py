#!/usr/bin/env python3
"""
Mission Controller Service

Provides /navigate_to_room service for room-to-room navigation.
Uses file-based tile state with inotify for instant change detection.

Features:
- File-based tile state (/tmp/current_tile.txt)
- inotify-based file watching (zero-latency tile change detection)
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
import ctypes
import os
import struct
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import yaml

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

from mission_controller.srv import NavigateToRoom, CancelMission
from mission_state import (
    save_mission_state,
    load_mission_state,
    clear_mission_state,
    MissionState,
)

# ============================================================================
# Constants
# ============================================================================

# Persistent tile state file - single source of truth (shared with tile_check_action)
TILE_STATE_FILE = Path("/tmp/current_tile.txt")

# inotify constants (from <sys/inotify.h>)
IN_MODIFY = 0x00000002
IN_CLOSE_WRITE = 0x00000008
IN_MOVED_TO = 0x00000080
IN_CREATE = 0x00000100

# Watch for any file content change
INOTIFY_MASK = IN_MODIFY | IN_CLOSE_WRITE | IN_MOVED_TO | IN_CREATE

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
# Inotify File Watcher
# ============================================================================


class TileFileWatcher:
    """
    Async inotify-based file watcher for /tmp/current_tile.txt.

    Provides zero-latency notification when the tile state file changes.
    Uses Linux inotify directly via ctypes (no external dependencies).
    """

    def __init__(self, filepath: Path, logger):
        self._filepath = filepath
        self._logger = logger
        self._inotify_fd: Optional[int] = None
        self._watch_fd: Optional[int] = None
        self._running = False
        self._callbacks: List[callable] = []
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def add_callback(self, callback: callable) -> None:
        """Add callback to be invoked on file change."""
        self._callbacks.append(callback)

    def start(self, loop: asyncio.AbstractEventLoop) -> bool:
        """Start watching the file."""
        self._loop = loop

        try:
            # Create inotify instance
            libc = ctypes.CDLL("libc.so.6", use_errno=True)
            self._inotify_fd = libc.inotify_init1(os.O_NONBLOCK)

            if self._inotify_fd < 0:
                self._logger.error(
                    f"inotify_init1 failed: {os.strerror(ctypes.get_errno())}"
                )
                return False

            # Ensure parent directory exists
            watch_path = str(self._filepath.parent)

            # Watch the directory (file might not exist yet)
            watch_path_bytes = watch_path.encode("utf-8")
            self._watch_fd = libc.inotify_add_watch(
                self._inotify_fd, watch_path_bytes, INOTIFY_MASK
            )

            if self._watch_fd < 0:
                self._logger.error(
                    f"inotify_add_watch failed: {os.strerror(ctypes.get_errno())}"
                )
                os.close(self._inotify_fd)
                return False

            self._running = True

            # Add fd to event loop for async reading
            loop.add_reader(self._inotify_fd, self._on_inotify_event)

            self._logger.debug(f"TileFileWatcher started on {watch_path}")
            return True

        except Exception as e:
            self._logger.error(f"Failed to start inotify: {e}")
            return False

    def stop(self) -> None:
        """Stop watching."""
        self._running = False

        if self._loop and self._inotify_fd is not None:
            try:
                self._loop.remove_reader(self._inotify_fd)
            except Exception:
                pass

        if self._inotify_fd is not None:
            try:
                os.close(self._inotify_fd)
            except Exception:
                pass
            self._inotify_fd = None

    def _on_inotify_event(self) -> None:
        """Handle inotify event (called by event loop)."""
        if not self._running or self._inotify_fd is None:
            return

        try:
            # Read all available events
            data = os.read(self._inotify_fd, 4096)

            # Parse events to check if our file was modified
            offset = 0
            target_name = self._filepath.name.encode("utf-8")

            while offset < len(data):
                # inotify_event struct: wd (4), mask (4), cookie (4), len (4), name (len)
                wd, mask, cookie, length = struct.unpack_from("iIII", data, offset)
                offset += 16

                if length > 0:
                    name = data[offset : offset + length].rstrip(b"\x00")
                    offset += length

                    # Check if this event is for our file
                    if name == target_name:
                        self._notify_callbacks()
                else:
                    offset += length

        except BlockingIOError:
            pass  # No data available
        except Exception as e:
            self._logger.warn(f"inotify read error: {e}")

    def _notify_callbacks(self) -> None:
        """Notify all registered callbacks."""
        for callback in self._callbacks:
            try:
                callback()
            except Exception as e:
                self._logger.warn(f"Callback error: {e}")


# ============================================================================
# Mission Controller
# ============================================================================


class MissionController(Node):
    # Max retries per tile transition before aborting
    TILE_TRANSITION_MAX_RETRIES = 3

    def __init__(self):
        super().__init__("mission_controller")

        # Create a dedicated event loop for async nav work
        self._async_loop = asyncio.new_event_loop()
        self._async_thread = threading.Thread(
            target=self._async_loop.run_forever, daemon=True
        )
        self._async_thread.start()
        self._mission_cancel_requested = False

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
                self.get_logger().fatal("Could not find tile_manager package")
                raise RuntimeError("tile_manager package not found")

        interrupted = load_mission_state()
        if interrupted:
            self.get_logger().warn(
                f"Found interrupted mission: {interrupted.room_name} "
                f"(step {interrupted.current_step}/{len(interrupted.tile_sequence)})"
            )
            clear_mission_state()

        # Data structures
        self.tiles: Dict[int, TileInfo] = {}
        self.connections: Dict[str, ConnectionInfo] = {}
        self.rooms: Dict[str, RoomInfo] = {}
        self._load_config(config_file)

        # Callback groups
        self._service_cb_group = ReentrantCallbackGroup()

        # Nav2 action client
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self._service_cb_group,
        )

        # Thread-safe goal handle access
        self._goal_lock = threading.Lock()
        self._active_goal_handle = None

        # Mission queue
        self._mission_queue: deque = deque()
        self._mission_active = False
        self._mission_lock = threading.Lock()

        # Shutdown flag
        self._shutting_down = False

        # Tile change event (set by inotify callback)
        self._tile_changed_event: Optional[asyncio.Event] = None
        self._event_loop: Optional[asyncio.AbstractEventLoop] = None

        # File watcher (initialized later when event loop is available)
        self._file_watcher = TileFileWatcher(TILE_STATE_FILE, self.get_logger())
        self._file_watcher.add_callback(self._on_tile_file_changed)
        self._file_watcher.start(self._async_loop)

        # Service
        self._srv = self.create_service(
            NavigateToRoom,
            "navigate_to_room",
            self._navigate_to_room_callback,
            callback_group=self._service_cb_group,
        )

        self._cancel_srv = self.create_service(
            CancelMission,
            "cancel_mission",
            self._cancel_mission_callback,
            callback_group=self._service_cb_group,
        )

        self.get_logger().info(
            f"MissionController ready: {len(self.tiles)} tiles, {len(self.rooms)} rooms"
        )

    def _on_tile_file_changed(self) -> None:
        """Callback invoked when tile file changes (from inotify)."""
        if self._tile_changed_event is not None:
            # Thread-safe set from inotify callback
            try:
                self._event_loop.call_soon_threadsafe(self._tile_changed_event.set)
            except Exception:
                pass

    def _cancel_mission_callback(self, request, response):
        """Cancel the currently active mission."""
        if not self._mission_active:
            response.accepted = False
            response.message = "No active mission to cancel"
            return response

        self.get_logger().info("Mission cancel requested by job_manager")
        self._mission_cancel_requested = True

        # Wake tile-change waiter so it doesn't hang
        if self._tile_changed_event is not None:
            try:
                self._async_loop.call_soon_threadsafe(self._tile_changed_event.set)
            except Exception:
                pass

        # Cancel active nav2 goal
        asyncio.run_coroutine_threadsafe(self._cancel_active_goal(), self._async_loop)

        response.accepted = True
        response.message = "Mission cancel accepted"
        return response

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
            self.get_logger().fatal(f"Failed to load config: {e}")
            raise

    # ========================================================================
    # File-based tile state
    # ========================================================================

    def _read_current_tile(self) -> Optional[int]:
        """
        Read current tile from persistent file.
        Returns tile ID if valid, None otherwise.
        """
        try:
            if not TILE_STATE_FILE.exists():
                self.get_logger().debug(f"Tile state file not found: {TILE_STATE_FILE}")
                return None

            content = TILE_STATE_FILE.read_text().strip()
            tile = int(content)

            if tile < 1:
                self.get_logger().warn(f"Invalid tile value in file: {tile}")
                return None

            # Validate tile exists in config
            if tile not in self.tiles:
                self.get_logger().warn(f"Persisted tile {tile} not in config, ignoring")
                return None

            return tile

        except ValueError as e:
            self.get_logger().warn(f"Failed to parse tile file: {e}")
            return None
        except Exception as e:
            self.get_logger().warn(f"Failed to read tile file: {e}")
            return None

    async def _wait_for_tile_change(
        self, target_tile: int, timeout: float = 120.0
    ) -> bool:
        """
        Wait for tile to change to target_tile using inotify.
        Returns True if target tile reached, False on timeout.
        """
        # Create fresh event for this wait
        self._tile_changed_event = asyncio.Event()

        start_time = time.time()

        try:
            while time.time() - start_time < timeout:
                if self._shutting_down:
                    return False

                # Check current tile
                current = self._read_current_tile()
                if current == target_tile:
                    self.get_logger().debug(f"Target tile {target_tile} reached")
                    return True

                # Wait for file change event (with timeout for periodic recheck)
                try:
                    await asyncio.wait_for(
                        self._tile_changed_event.wait(),
                        timeout=min(1.0, timeout - (time.time() - start_time)),
                    )
                    self._tile_changed_event.clear()
                except asyncio.TimeoutError:
                    # Periodic timeout - recheck tile and continue
                    pass

            return False

        finally:
            self._tile_changed_event = None

    # ========================================================================
    # Path planning
    # ========================================================================

    def _find_path(self, start_tile: int, end_tile: int) -> List[int]:
        """BFS to find shortest path between tiles."""
        if start_tile == end_tile:
            return [start_tile]

        queue = deque([(start_tile, [start_tile])])
        visited = {start_tile}

        while queue:
            current, path = queue.popleft()

            if current not in self.tiles:
                continue

            for neighbor in self.tiles[current].neighbors:
                if neighbor == end_tile:
                    return path + [neighbor]

                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        return []

    def _get_switch_point(
        self, from_tile: int, to_tile: int
    ) -> Optional[Tuple[float, float]]:
        """Get switch point coordinates for tile transition."""
        key = f"{min(from_tile, to_tile)}-{max(from_tile, to_tile)}"

        if key not in self.connections:
            return None

        conn = self.connections[key]
        switch_key = f"{from_tile}_to_{to_tile}"

        if switch_key not in conn.switch_points:
            return None

        coords = conn.switch_points[switch_key]
        return (coords[0], coords[1])

    # ========================================================================
    # Navigation helpers
    # ========================================================================

    async def _wait_for_nav2(self, timeout: float = 10.0) -> bool:
        """Wait for Nav2 action server with fresh connection check."""
        start = time.time()
        while time.time() - start < timeout:
            if self._shutting_down:
                return False
            if self._nav_client.server_is_ready():
                return True
            await asyncio.sleep(0.1)
        return False

    async def _send_nav_goal(self, x: float, y: float) -> bool:
        """Send navigation goal and wait for acceptance."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        send_goal_future = self._nav_client.send_goal_async(goal)

        try:
            goal_handle = await asyncio.wait_for(
                send_goal_future,
                timeout=5.0,
            )
        except asyncio.TimeoutError:
            self.get_logger().error("Goal send timed out")
            return False

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return False

        with self._goal_lock:
            self._active_goal_handle = goal_handle

        return True

    async def _wait_for_result(self, timeout: float = 120.0) -> Tuple[bool, int]:
        with self._goal_lock:
            goal_handle = self._active_goal_handle

        if goal_handle is None:
            return False, GoalStatus.STATUS_UNKNOWN

        result_future = goal_handle.get_result_async()

        try:
            result = await asyncio.wait_for(result_future, timeout=timeout)
            status = result.status
            success = status == GoalStatus.STATUS_SUCCEEDED
            return success, status
        except asyncio.TimeoutError:
            self.get_logger().error("Navigation timed out")
            return False, GoalStatus.STATUS_UNKNOWN
        finally:
            with self._goal_lock:
                self._active_goal_handle = None

    async def _cancel_active_goal(self) -> None:
        """Cancel the currently active navigation goal."""
        with self._goal_lock:
            goal_handle = self._active_goal_handle
            self._active_goal_handle = None

        if goal_handle is not None:
            try:
                await goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().debug(f"Cancel goal exception: {e}")

    async def _navigate_to_switch_point(
        self, x: float, y: float, target_tile: int
    ) -> Tuple[bool, bool]:
        """
        Navigate to switch point and monitor for tile switch via inotify.
        Returns (nav_success, tile_switched).
        """
        if not await self._send_nav_goal(x, y):
            return False, False

        # Race between navigation completion and tile switch
        nav_task = asyncio.create_task(self._wait_for_result(timeout=120.0))
        tile_task = asyncio.create_task(
            self._wait_for_tile_change(target_tile, timeout=120.0)
        )

        try:
            # Wait for either task to complete
            done, pending = await asyncio.wait(
                [nav_task, tile_task], return_when=asyncio.FIRST_COMPLETED
            )

            # Check which completed first
            if tile_task in done:
                tile_switched = tile_task.result()
                if tile_switched:
                    self.get_logger().info(
                        f"Tile switch detected: now on tile {target_tile}"
                    )
                    # Cancel navigation - we're done with this transition
                    nav_task.cancel()
                    await self._cancel_active_goal()
                    return True, True

            if nav_task in done:
                nav_success, status = nav_task.result()
                # Navigation completed, check if tile switched during nav
                current_tile = self._read_current_tile()
                tile_switched = current_tile == target_tile
                tile_task.cancel()
                return nav_success, tile_switched

            # Neither completed (shouldn't happen with FIRST_COMPLETED)
            return False, False

        except asyncio.CancelledError:
            nav_task.cancel()
            tile_task.cancel()
            raise

        finally:
            # Ensure both tasks are cleaned up
            for task in [nav_task, tile_task]:
                if not task.done():
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass

    async def _navigate_to_final(self, x: float, y: float) -> bool:
        """Navigate to final destination."""
        if not await self._send_nav_goal(x, y):
            return False

        success, _ = await self._wait_for_result()
        return success

    # ========================================================================
    # Mission execution
    # ========================================================================

    async def _execute_mission(
        self, room_name: str
    ) -> Tuple[bool, str, List[int], float]:
        """
        Execute navigation mission to room.
        Returns (success, message, tiles_traversed, duration_seconds).
        """
        self._mission_cancel_requested = False
        start_time = self.get_clock().now()
        tiles_traversed: List[int] = []

        # 1. Get room info
        if room_name not in self.rooms:
            return (False, f"Unknown room: {room_name}", [], 0.0)

        room = self.rooms[room_name]

        # 2. Get current tile from file
        current_tile = self._read_current_tile()
        if current_tile is None:
            # Default to tile 1 if no state file
            current_tile = 1
            self.get_logger().warn("No tile state found, assuming tile 1")

        tiles_traversed.append(current_tile)

        # 3. Plan path
        tile_sequence = self._find_path(current_tile, room.tile)

        if not tile_sequence:
            return (
                False,
                f"No path from tile {current_tile} to tile {room.tile}",
                tiles_traversed,
                self._elapsed_seconds(start_time),
            )

        self.get_logger().info(
            f"Mission: {room_name} | Path: {' -> '.join(map(str, tile_sequence))}"
        )

        # 4. Wait for Nav2
        if not await self._wait_for_nav2():
            return (
                False,
                "Nav2 server not available",
                tiles_traversed,
                self._elapsed_seconds(start_time),
            )

        # 5. Execute tile transitions
        for i in range(len(tile_sequence) - 1):
            from_tile = tile_sequence[i]
            to_tile = tile_sequence[i + 1]

            switch_point = self._get_switch_point(from_tile, to_tile)
            if not switch_point:
                return (
                    False,
                    f"No switch point for {from_tile} -> {to_tile}",
                    tiles_traversed,
                    self._elapsed_seconds(start_time),
                )

            if self._shutting_down:
                return (
                    False,
                    "Shutdown requested",
                    tiles_traversed,
                    self._elapsed_seconds(start_time),
                )

            if self._mission_cancel_requested:
                return (
                    False,
                    "Mission cancelled",
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

                self.get_logger().debug(f"Result: {result}, switched: {tile_switched}")

                if tile_switched:
                    tiles_traversed.append(to_tile)
                    transition_success = True
                    break

                save_mission_state(
                    MissionState(
                        room_name=room_name,
                        tile_sequence=tile_sequence,
                        current_step=i + 1,
                        tiles_completed=tiles_traversed.copy(),
                        start_timestamp=start_time.nanoseconds / 1e9,
                        last_update_timestamp=time.time(),
                    )
                )

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

        # 6. Navigate to final room
        if self._mission_cancel_requested:
            return (
                False,
                "Mission cancelled",
                tiles_traversed,
                self._elapsed_seconds(start_time),
            )

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
        self.get_logger().info(f"Navigation complete: {room_name} in {duration:.1f}s")
        clear_mission_state()
        return (True, f"Reached {room_name}", tiles_traversed, duration)

    def _navigate_to_room_callback(self, request, response):
        """Sync service callback — dispatches to async loop."""
        room_name = request.room_name
        self.get_logger().info(f"Navigation requested: {room_name}")

        if room_name not in self.rooms:
            response.success = False
            response.message = f"Unknown room: {room_name}"
            response.tiles_traversed = []
            response.duration_seconds = 0.0
            return response

        # Submit coroutine to the async loop and block until done
        future = asyncio.run_coroutine_threadsafe(
            self._execute_mission(room_name), self._async_loop
        )
        try:
            success, message, tiles_traversed, duration = future.result(timeout=300.0)
        except TimeoutError:
            self.get_logger().error("Mission timed out")
            success, message, tiles_traversed, duration = False, "Timeout", [], 0.0
        except Exception as e:
            self.get_logger().error(f"Mission failed with exception: {e}")
            success, message, tiles_traversed, duration = (
                False,
                f"Internal error: {e}",
                [],
                0.0,
            )

        response.success = success
        response.message = message
        response.tiles_traversed = tiles_traversed
        response.duration_seconds = duration
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
                    result_future.set_result((False, "Shutdown requested", [], 0.0))
                continue

            self.get_logger().info(f"Processing queued mission: {room_name}")

            try:
                result = await self._execute_mission(room_name)
                if not result_future.done():
                    result_future.set_result(result)
            except Exception as e:
                self.get_logger().error(f"Queued mission '{room_name}' failed: {e}")
                if not result_future.done():
                    result_future.set_result((False, f"Internal error: {e}", [], 0.0))

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
        """Graceful shutdown: stop watcher, cancel active goal, drain queue."""
        self._shutting_down = True

        # Stop file watcher
        self._file_watcher.stop()

        # Wake up any waiting tile change event
        if self._tile_changed_event is not None:
            try:
                self._event_loop.call_soon_threadsafe(self._tile_changed_event.set)
            except Exception:
                pass

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
                    result_future.set_result((False, "Shutdown requested", [], 0.0))
            self._mission_active = False


# ============================================================================
# Main
# ============================================================================


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    # 2 threads: service handler, queue processing
    executor = MultiThreadedExecutor(num_threads=2)
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
