#!/usr/bin/env python3
"""
Teleop Launch File

Launches joystick teleoperation:
  - joy_node: Reads joystick input
  - teleop_node: Converts to cmd_vel_joy

Usage:
    ros2 launch teleop teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_teleop = get_package_share_directory("teleop")

    # Config file path
    teleop_config = os.path.join(pkg_teleop, "config", "teleop.yaml")

    # Joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    # Teleop node with remapping
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        output="screen",
        parameters=[teleop_config],
        remappings=[
            ("cmd_vel", "cmd_vel_joy"),
        ],
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
