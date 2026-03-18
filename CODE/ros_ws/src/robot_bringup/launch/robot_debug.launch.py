#!/usr/bin/env python3
"""
Robot Bringup — Debug Mode

Launches ALL nodes directly without the system supervisor or job manager.
Use this when you want to test individual components, manually send
Nav2 goals, or debug without the supervisor restarting/killing nodes.

All nodes start simultaneously. Dependencies resolve naturally:
  - Nav2 waits for TF + map via internal timeouts
  - Mission controller waits for /active_tile + Nav2 action server
  - odom_base_publisher retries URDF TF lookup until available
  - map_server publishes with transient local QoS

Usage:
  ros2 launch robot_bringup robot_debug.launch.py
  ros2 launch robot_bringup robot_debug.launch.py log_level:=debug
  ros2 launch robot_bringup robot_debug.launch.py use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

encoder_config = os.path.join(
    get_package_share_directory("wheel_encoder_driver"), "config", "encoder_params.yaml"
)


def generate_launch_description():
    pkg_zed_wrapper = get_package_share_directory("zed_wrapper")
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    pkg_teleop = get_package_share_directory("teleop")
    pkg_tile_manager = get_package_share_directory("tile_manager")
    pkg_robot_nav = get_package_share_directory("robot_navigation")
    pkg_diff_drive_controller = get_package_share_directory("diff_drive_controller")

    zed_config = os.path.join(pkg_robot_bringup, "config", "zedm.yaml")

    # ==================== LAUNCH ARGUMENTS ====================
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")

    set_log_level = SetEnvironmentVariable("RCUTILS_LOGGING_MIN_SEVERITY", log_level)

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false", description="Launch RViz"
    )

    # ==================== TELEOP + DIFF DRIVE ====================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop, "launch", "teleop-joy.launch.py")
        ),
        launch_arguments={"log_level": log_level}.items(),
    )

    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diff_drive_controller, "launch", "drive_controller.launch.py")
        ),
        launch_arguments={"log_level": log_level}.items(),
    )

    # ==================== WHEEL ENCODER PUBLISHER ====================
    encoder_publisher = Node(
        package="wheel_encoder_driver",
        executable="encoder_driver",
        name="encoder_driver",
        output="screen",
        parameters=[encoder_config],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ==================== ROBOT DESCRIPTION (URDF) ====================
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, "launch", "robot_description.launch.py")
        ),
        launch_arguments={"log_level": log_level}.items(),
    )

    # ==================== ZED CAMERA ====================
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model": "zedm",
            "ros_params_override_path": zed_config,
            "publish_tf": "false",
        }.items(),
    )

    # SDK 4.0 -> 5.1 topic compatibility relay
    topic_relay = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "sleep 5 && ros2 run topic_tools relay "
            "/zed/zed_node/rgb/image_rect_color "
            "/zed/zed_node/rgb/color/rect/image "
            "2>/dev/null || true",
        ],
        output="log",
    )

    # ==================== ODOM BASE PUBLISHER ====================
    odom_base_publisher = Node(
        package="odom_base_publisher",
        executable="odom_base_publisher",
        name="odom_base_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ==================== MAP SERVER ====================
    tile_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tile_manager, "launch", "map_server.launch.py")
        ),
        launch_arguments={"log_level": log_level}.items(),
    )

    # ==================== NAV2 ====================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_nav, "launch", "navigation.launch.py")
        ),
        launch_arguments={"log_level": log_level}.items(),
    )

    # ==================== MISSION CONTROLLER ====================
    mission_controller_node = Node(
        package="mission_controller",
        executable="mission_service.py",
        name="mission_controller",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ==================== JOB MANAGER ====================
    job_manager = Node(
        package="job_manager",
        executable="job_manager_node.py",
        name="job_manager",
        output="screen",
        parameters=[{"debug_mode": True}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ==================== LAUNCH ====================
    return LaunchDescription(
        [
            log_level_arg,
            set_log_level,
            use_rviz_arg,
            LogInfo(msg=""),
            LogInfo(msg="==========================================="),
            LogInfo(msg="  Robot Bringup — Debug Mode"),
            LogInfo(msg="  No supervisor, all nodes direct"),
            LogInfo(msg="==========================================="),
            LogInfo(msg=""),
            teleop_launch,
            diff_drive_launch,
            encoder_publisher,
            robot_description_launch,
            zed_launch,
            topic_relay,
            odom_base_publisher,
            tile_manager_launch,
            navigation_launch,
            mission_controller_node,
            job_manager,
        ]
    )
