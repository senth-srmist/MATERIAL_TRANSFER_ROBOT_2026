#!/usr/bin/env python3
"""
Teleop Joy Launch File

Launches:
1. joy_node - Reads joystick input
2. teleop_node - Converts to cmd_vel_joy (starts after /joy topic is available)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_teleop = get_package_share_directory("teleop")
    teleop_config = os.path.join(pkg_teleop, "config", "teleop.yaml")

    # Joy node - publishes /joy topic
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id": 0
        }],
    )

    # Wait for /joy topic to be available
    wait_for_joy = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 topic info /joy 2>/dev/null | grep -q "Publisher count: 1"; do sleep 0.2; done'],
        output='log',
    )

    # Teleop node - subscribes to /joy, publishes cmd_vel_joy
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        output="screen",
        parameters=[teleop_config],
        remappings=[("cmd_vel", "cmd_vel_joy")],
    )

    # Start teleop_node after /joy topic is ready
    start_teleop_after_joy = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_joy,
            on_exit=[teleop_node],
        )
    )

    return LaunchDescription([
        joy_node,
        wait_for_joy,
        start_teleop_after_joy,
    ])
