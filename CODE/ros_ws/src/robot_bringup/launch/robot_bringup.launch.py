#!/usr/bin/env python3
"""
Robot Bringup Launch File

Startup order:
1. ZED Camera
2. Localization
3. Teleop + Diff Drive
4. Mission Controller (needs to be ready before /active_tile is published)
5. Tile Manager / Map Server (publishes /active_tile after 2s)
6. Navigation (needs map_server ready)
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_zed_wrapper = get_package_share_directory("zed_wrapper")
    pkg_robot_loc = get_package_share_directory("robot_localization")
    pkg_teleop = get_package_share_directory("teleop")
    pkg_tile_manager = get_package_share_directory("tile_manager")
    pkg_robot_nav = get_package_share_directory("robot_navigation")
    pkg_sabertooth_diff_drive = get_package_share_directory("sabertooth_diff_drive")

    # ==================== LAUNCH ARGUMENT ====================
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz"
    )

    # ==================== 1. ZED CAMERA ====================
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={"camera_model": "zedm"}.items(),
    )

    # ==================== 2. LOCALIZATION ====================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_loc, "launch", "robot_localization.launch.py")
        ),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
    )

    # ==================== 3. TELEOP ====================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop, "launch", "teleop-joy.launch.py")
        ),
    )

    # ==================== 4. DIFF DRIVE ====================
    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sabertooth_diff_drive, "launch", "controller_with_twist_mux.launch.py")
        ),
    )

    # ==================== 5. MISSION CONTROLLER ====================
    # Starts early so it's ready to receive /active_tile
    mission_controller_node = Node(
        package="mission_controller",
        executable="mission_service.py",
        name="mission_controller",
        output="screen",
    )

    # ==================== 6. TILE MANAGER (Map Server) ====================
    # Delayed 2s to let mission_controller initialize
    tile_manager_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_tile_manager, "launch", "map_server.launch.py")
                ),
            )
        ],
    )

    # ==================== 7. NAVIGATION ====================
    # Delayed 6s to let map_server fully activate (it needs ~4s after tile_manager starts)
    navigation_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot_nav, "launch", "navigation.launch.py")
                ),
            )
        ],
    )

    return LaunchDescription([
        use_rviz_arg,
        LogInfo(msg="========== ROBOT BRINGUP =========="),
        
        # Immediate starts
        zed_launch,
        localization_launch,
        teleop_launch,
        diff_drive_launch,
        mission_controller_node,
        
        # Delayed starts
        tile_manager_launch,      # +2s
        navigation_launch,        # +6s
        
        LogInfo(msg="==================================="),
    ])
