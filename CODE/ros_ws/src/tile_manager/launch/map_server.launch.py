#!/usr/bin/env python3
"""
Tile Manager - Map Server Launch

Launches:
    - Nav2 Map Server
    - Nav2 Lifecycle Manager (map_server only)
    - Initial tile state file write (/tmp/current_tile.txt)

Default:
    tile1.yaml

Override:
    ros2 launch tile_manager map_server.launch.py map:=tile3.yaml

Tile State Persistence:
    Current tile is written to /tmp/current_tile.txt on startup.
    This file is the single source of truth for tile state, shared by:
    - tile_check_action (BT node) - reads on init, writes on tile change
    - mission_service - reads to determine current tile for path planning
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_tile_mgr = get_package_share_directory("tile_manager")

    # ---------------- LAUNCH ARGUMENTS ----------------
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="tile1.yaml",
        description="Map YAML file (relative to tile_manager/maps)",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")

    map_yaml = PathJoinSubstitution(
        [FindPackageShare("tile_manager"), "maps", LaunchConfiguration("map")]
    )

    # Extract the first run of digits from the map name (e.g., "tile1.yaml" -> "1").
    # Using regex is more robust than strip-based string replacements.
    initial_tile = PythonExpression(
        [
            "__import__('re').search(r'\\d+', '",
            LaunchConfiguration("map"),
            "').group()",
        ]
    )

    # ---------------- MAP SERVER ----------------
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="log",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": False,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ---------------- LIFECYCLE MANAGER ----------------
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="log",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ---------------- INITIAL TILE STATE FILE ----------------
    # Write initial tile to /tmp/current_tile.txt (overwrites existing)
    # This is the single source of truth for tile state
    write_tile_state = TimerAction(
        period=1.0,  # Small delay to ensure map_server is starting
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    PythonExpression(
                        [
                            "'echo ",
                            initial_tile,
                            " > /tmp/current_tile.txt && echo \"[map_server.launch] Wrote initial tile ",
                            initial_tile,
                            " to /tmp/current_tile.txt\"'",
                        ]
                    ),
                ],
                output="log",
            )
        ],
    )

    return LaunchDescription(
        [
            map_arg,
            log_level_arg,
            map_server,
            lifecycle_manager,
            write_tile_state,
        ]
    )
