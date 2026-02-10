#!/usr/bin/env python3
"""
Tile Manager - Map Server Launch

Launches:
    - Nav2 Map Server
    - Nav2 Lifecycle Manager (map_server only)

Default:
    tile01.yaml

Override:
    ros2 launch tile_manager map_server.launch.py map:=tile03.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_tile_mgr = get_package_share_directory("tile_manager")

    # ---------------- LAUNCH ARGUMENT ----------------
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="tile01.yaml",
        description="Map YAML file (relative to tile_manager/maps)",
    )

    map_yaml = PathJoinSubstitution(
        [FindPackageShare("tile_manager"), "maps", LaunchConfiguration("map")]
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

    return LaunchDescription(
        [
            map_arg,
            map_server,
            lifecycle_manager,
        ]
    )
