#!/usr/bin/env python3
"""
Robot Bringup Launch File — Parallel Start

All nodes launch simultaneously. Dependencies resolve naturally:
  - Nav2 waits for TF + map via internal timeouts
  - Mission controller waits for /active_tile + Nav2 action server
  - odom_base_publisher retries URDF TF lookup until available
  - map_server publishes with transient local QoS (late subs get last map)

System Supervisor monitors health and handles failures:
  Category 1 (auto-restart): controller_node, map_server
  Category 2 (abort + restart): nav2, mission_controller
  Category 3 (cascade shutdown): zed, odom_base_publisher

No sequential wait_for processes. No shell-based health checks.
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    LogInfo,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_zed_wrapper = get_package_share_directory("zed_wrapper")
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    pkg_teleop = get_package_share_directory("teleop")
    pkg_tile_manager = get_package_share_directory("tile_manager")
    pkg_robot_nav = get_package_share_directory("robot_navigation")
    pkg_sabertooth_diff_drive = get_package_share_directory(
        "sabertooth_diff_drive"
    )

    zed_config = os.path.join(pkg_robot_bringup, "config", "zedm.yaml")

    # ==================== LAUNCH ARGUMENTS ====================
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false", description="Launch RViz"
    )

    # ==================== TELEOP + DIFF DRIVE ====================
    # Category 1 — respawn enabled
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
            os.path.join(
                pkg_robot_bringup, "launch", "robot_description.launch.py"
            )
        ),
    )

    # ==================== ODOM BASE PUBLISHER ====================
    # Category 3 — no respawn (supervisor handles cascade shutdown)
    odom_base_publisher = Node(
        package="odom_base_publisher",
        executable="odom_base_publisher",
        name="odom_base_publisher",
        output="screen",
    )

    # ==================== ZED CAMERA ====================
    # Category 3 — no respawn (supervisor handles cascade shutdown)
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
            "bash", "-c",
            "sleep 5 && ros2 run topic_tools relay "
            "/zed/zed_node/rgb/image_rect_color "
            "/zed/zed_node/rgb/color/rect/image "
            "2>/dev/null || true",
        ],
        output="log",
    )

    # ==================== MISSION CONTROLLER ====================
    # Category 2 — supervisor handles abort + restart
    mission_controller_node = Node(
        package="mission_controller",
        executable="mission_service.py",
        name="mission_controller",
        output="screen",
    )

    # ==================== MAP SERVER ====================
    # Category 1 — respawn enabled as first line of defense
    tile_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_tile_manager, "launch", "map_server.launch.py"
            )
        ),
    )

    # ==================== NAV2 ====================
    # Category 2 — supervisor handles abort + restart
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_nav, "launch", "navigation.launch.py")
        ),
    )

    # ==================== SYSTEM SUPERVISOR ====================
    # Must stay alive — never respawned by anything else
    system_supervisor = Node(
        package="system_supervisor",
        executable="system_supervisor.py",
        name="system_supervisor",
        output="screen",
    )

    # ==================== LAUNCH ====================
    return LaunchDescription(
        [
            use_rviz_arg,
            LogInfo(msg=""),
            LogInfo(msg="==========================================="),
            LogInfo(msg="  Robot Bringup — Parallel Start"),
            LogInfo(msg="  System Supervisor monitors all nodes"),
            LogInfo(msg="==========================================="),
            LogInfo(msg=""),
            # Everything launches simultaneously
            # Dependencies resolve naturally via ROS2 mechanisms
            teleop_launch,
            diff_drive_launch,
            robot_description_launch,
            odom_base_publisher,
            zed_launch,
            topic_relay,
            mission_controller_node,
            tile_manager_launch,
            navigation_launch,
            system_supervisor,
        ]
    )
