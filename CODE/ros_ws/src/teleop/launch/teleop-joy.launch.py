#!/usr/bin/env python3
"""
Teleop Joy Launch File

Launches:
1. joy_node - Reads joystick input
2. teleop_node - Converts to cmd_vel_joy (starts after /joy topic is available)

QoS: Best effort, volatile, keep_last(1) for low latency
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_teleop = get_package_share_directory("teleop")
    teleop_config = os.path.join(pkg_teleop, "config", "teleop.yaml")

    # Log level argument
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")

    # Joy node - publishes /joy topic
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Wait for /joy topic to be available
    wait_for_joy = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'until ros2 topic info /joy 2>/dev/null | grep -q "Publisher count: 1"; do sleep 0.2; done',
        ],
        output="log",
    )

    # Teleop node - subscribes to /joy, publishes cmd_vel_joy
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        output="screen",
        parameters=[teleop_config],
        remappings=[("cmd_vel", "cmd_vel_joy")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Start teleop_node after /joy topic is ready
    start_teleop_after_joy = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_joy,
            on_exit=[teleop_node],
        )
    )

    return LaunchDescription(
        [
            log_level_arg,
            joy_node,
            wait_for_joy,
            start_teleop_after_joy,
        ]
    )
