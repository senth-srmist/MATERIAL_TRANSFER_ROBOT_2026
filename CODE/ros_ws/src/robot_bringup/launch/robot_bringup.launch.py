#!/usr/bin/env python3
"""
Robot Bringup Launch File

Launches everything needed for autonomous navigation:
  - ZED Camera
  - Robot Localization (TF, pose monitor, tile position service)
  - Teleop (joy + teleop_twist_joy)
  - Sabertooth Diff Drive (placeholder)
  - Tile Manager (map server)
  - Navigation (Nav2)
  - Mission Controller

Usage:
    ros2 launch robot_bringup robot_bringup.launch.py
    ros2 launch robot_bringup robot_bringup.launch.py use_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_zed_wrapper = get_package_share_directory("zed_wrapper")
    pkg_robot_loc = get_package_share_directory("robot_localization")
    pkg_teleop = get_package_share_directory("teleop")
    pkg_tile_manager = get_package_share_directory("tile_manager")
    pkg_robot_nav = get_package_share_directory("robot_navigation")
    pkg_sabertooth_diff_drive = get_package_share_directory(
        "sabertooth_diff_drive")

    # ==================== LAUNCH ARGUMENT ====================
    use_rviz_arg = DeclareLaunchArgument("use_rviz",
                                         default_value="false",
                                         description="Launch RViz")

    # ==================== 2. ZED CAMERA ====================
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, "launch", "zed_camera.launch.py")),
        launch_arguments={"camera_model": "zedm"}.items(),
    )

    # ==================== 3. ROBOT LOCALIZATION ====================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_loc, "launch",
                         "robot_localization.launch.py")),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
    )

    # ==================== 7. MISSION CONTROLLER ====================
    mission_controller_node = Node(
        package="mission_controller",
        executable="mission_service.py",
        name="mission_controller",
        output="log",
    )

    # ==================== 6. NAVIGATION (Nav2) ====================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_nav, "launch", "navigation.launch.py")), )

    # ==================== 1. TILE MANAGER (Map Server) ====================
    tile_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tile_manager, "launch",
                         "map_server.launch.py")), )

    # ==================== 4. TELEOP ====================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop, "launch", "teleop-joy.launch.py")), )

    # ==================== 5. SABERTOOTH DIFF DRIVE (placeholder) ====================
    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_sabertooth_diff_drive,
                "launch",
                "controller_with_twist_mux.launch.py",
            )), )

    # ==================== RETURN ====================
    return LaunchDescription([
        use_rviz_arg,
        LogInfo(msg="========== ROBOT BRINGUP =========="),
        zed_launch,
        localization_launch,
        teleop_launch,
        diff_drive_launch,
        tile_manager_launch,
        navigation_launch,
        mission_controller_node,
        LogInfo(msg="==================================="),
    ])
