#!/usr/bin/env python3
"""
Tile Manager - Map Server Launch

Launches:
    - Nav2 Map Server
    - Nav2 Lifecycle Manager (map_server only)
    - Initial /active_tile publish (one-shot)

Default:
    tile1.yaml

Override:
    ros2 launch tile_manager map_server.launch.py map:=tile3.yaml
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

    map_yaml = PathJoinSubstitution(
        [FindPackageShare("tile_manager"), "maps", LaunchConfiguration("map")]
    )

    # Extract tile number from map name (e.g., "tile1.yaml" -> "1", "tile3.yaml" -> "3")
    # Using PythonExpression to parse the tile number
    initial_tile = PythonExpression(
        ["'", LaunchConfiguration("map"), "'.replace('tile', '').replace('.yaml', '')"]
    )

    # ---------------- MAP SERVER ----------------
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": False,
            }
        ],
    )

    # ---------------- LIFECYCLE MANAGER ----------------
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    # ---------------- INITIAL ACTIVE TILE PUBLISH ----------------
    # Publish initial tile to /active_tile (one-shot, after 2 sec delay for nodes to start)
    initial_tile_pub = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--times",
                    "1",
                    "/active_tile",
                    "std_msgs/msg/Int32",
                    PythonExpression(["'{data: ", initial_tile, "}'"]),
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            map_arg,
            map_server,
            lifecycle_manager,
            initial_tile_pub,
        ]
    )
