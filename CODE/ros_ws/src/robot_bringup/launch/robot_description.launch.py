#!/usr/bin/env python3
"""
robot_description.launch.py
Publishes robot URDF and broadcasts static TFs (including zed_camera_link → base_link)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")

    # Log level argument
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")

    # Path to URDF file
    urdf_file = os.path.join(pkg_robot_bringup, "urdf", "robot.urdf.xacro")

    # Process xacro to get URDF XML
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )

    # Robot State Publisher - broadcasts TFs from URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 10.0,
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([
        log_level_arg,
        robot_state_publisher,
    ])
