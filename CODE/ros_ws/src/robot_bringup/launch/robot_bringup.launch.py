#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_zed_wrapper = get_package_share_directory("zed_wrapper")
    pkg_robot_loc = get_package_share_directory("robot_localization")
    pkg_teleop = get_package_share_directory("teleop")
    pkg_tile_manager = get_package_share_directory("tile_manager")
    pkg_robot_nav = get_package_share_directory("robot_navigation")
    pkg_sabertooth_diff_drive = get_package_share_directory(
        "sabertooth_diff_drive")

    use_rviz_arg = DeclareLaunchArgument("use_rviz",
                                         default_value="false",
                                         description="Launch RViz")

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, "launch", "zed_camera.launch.py")),
        launch_arguments={"camera_model": "zedm"}.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_loc, "launch",
                         "robot_localization.launch.py")),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
    )

    mission_controller_node = Node(
        package="mission_controller",
        executable="mission_service.py",
        name="mission_controller",
        output="log",
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_nav, "launch", "navigation.launch.py")), )

    tile_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tile_manager, "launch",
                         "map_server.launch.py")), )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop, "launch", "teleop-joy.launch.py")), )

    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_sabertooth_diff_drive,
                "launch",
                "controller_with_twist_mux.launch.py",
            )), )

    # Wait for mission_controller to start, then give it 1s to initialize
    # before bringing up tile_manager (map server)
    delayed_tile_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=mission_controller_node,
            on_start=[TimerAction(period=1.0, actions=[tile_manager_launch])],
        ))

    return LaunchDescription([
        use_rviz_arg,
        LogInfo(msg="========== ROBOT BRINGUP =========="),
        zed_launch,
        localization_launch,
        teleop_launch,
        diff_drive_launch,
        mission_controller_node,  # starts first
        delayed_tile_manager,  # tile_manager starts 1s after mission_controller
        navigation_launch,
        mission_controller_node,
        LogInfo(msg="==================================="),
    ])
