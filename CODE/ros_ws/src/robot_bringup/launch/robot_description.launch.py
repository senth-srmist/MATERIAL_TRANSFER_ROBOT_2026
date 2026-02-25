#!/usr/bin/env python3
"""
robot_description.launch.py
Publishes robot URDF and broadcasts static TFs (including zed_camera_link → base_link)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_robot_bringup, "urdf", "robot.urdf.xacro")
    
    # Process xacro to get URDF XML
    robot_description = Command(["xacro ", urdf_file])
    
    # Robot State Publisher - broadcasts TFs from URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 10.0,  # Match ZED rate
        }],
    )
    
    return LaunchDescription([
        robot_state_publisher,
    ])
