#!/usr/bin/env python3
"""
Robot Bringup — Production Mode

Launches always-on nodes + system supervisor + job manager.
On-demand nodes (ZED, odom, Nav2, mission_controller) are started
and stopped by the supervisor based on /system/nav_needed.

Always-on:
  - teleop + twist_mux (joystick control always available)
  - diff_drive / controller_node (motors)
  - robot_description (URDF + static TF)
  - map_server (tile maps for visualization)
  - system_supervisor (health monitoring + lifecycle)
  - job_manager (delivery job queue)

On-demand (supervisor-managed):
  - ZED camera
  - odom_base_publisher
  - Nav2 (navigation stack)
  - mission_controller

Usage:
  ros2 launch robot_bringup robot_bringup.launch.py
  ros2 launch robot_bringup robot_bringup.launch.py log_level:=debug
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    pkg_teleop = get_package_share_directory("teleop")
    pkg_tile_manager = get_package_share_directory("tile_manager")
    pkg_sabertooth_diff_drive = get_package_share_directory("sabertooth_diff_drive")
    pkg_supervisor = get_package_share_directory("system_supervisor")

    supervisor_config = os.path.join(pkg_supervisor, "config", "supervisor_config.yaml")

    # ==================== LAUNCH ARGUMENTS ====================
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")

    set_log_level = SetEnvironmentVariable("RCUTILS_LOGGING_MIN_SEVERITY", log_level)

    # ==================== TELEOP + DIFF DRIVE ====================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop, "launch", "teleop-joy.launch.py")
        ),
    )

    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_sabertooth_diff_drive,
                "launch",
                "controller_with_twist_mux.launch.py",
            )
        ),
    )

    # ==================== ROBOT DESCRIPTION (URDF) ====================
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, "launch", "robot_description.launch.py")
        ),
    )

    # ==================== MAP SERVER ====================
    tile_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tile_manager, "launch", "map_server.launch.py")
        ),
    )

    # ==================== SYSTEM SUPERVISOR ====================
    system_supervisor = Node(
        package="system_supervisor",
        executable="supervisor_node.py",
        name="system_supervisor",
        output="screen",
        parameters=[{"config_file": supervisor_config}],
    )

    # ==================== JOB MANAGER ====================
    job_manager = Node(
        package="job_manager",
        executable="job_manager_node.py",
        name="job_manager",
        output="screen",
    )

    # ==================== LAUNCH ====================
    return LaunchDescription(
        [
            log_level_arg,
            set_log_level,
            LogInfo(msg=""),
            LogInfo(msg="==========================================="),
            LogInfo(msg="  Robot Bringup — Production Mode"),
            LogInfo(msg="  Supervisor manages on-demand nodes"),
            LogInfo(msg="==========================================="),
            LogInfo(msg=""),
            # Always-on nodes
            teleop_launch,
            diff_drive_launch,
            robot_description_launch,
            tile_manager_launch,
            # Orchestration
            system_supervisor,
            job_manager,
        ]
    )
