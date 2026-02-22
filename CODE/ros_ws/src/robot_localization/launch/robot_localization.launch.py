#!/usr/bin/env python3
"""
Robot Localization Launch File

Launches:
    - ZED Camera (odometry source)
    - Static TF: zed_camera_link → base_link
    - Static TF: map → odom (identity until ArUco)
    - Pose Monitor
    - RViz (optional)

TF Tree:
    map
     └── odom
         └── zed_camera_link
             └── base_link

Usage:
    ros2 launch robot_localization robot_localization.launch.py
    ros2 launch robot_localization robot_localization.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_robot_loc = get_package_share_directory("robot_localization")

    # ---------------- LAUNCH ARGUMENTS ----------------
    use_rviz_arg = DeclareLaunchArgument("use_rviz",
                                         default_value="true",
                                         description="Launch RViz")

    # ---------------- 1) STATIC TF: camera → base_link ----------------
    static_tf_camera_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_base_tf",
        arguments=[
            "-0.15",
            "0",
            "-0.70",
            "0",
            "-0.523",
            "0",
            "zed_camera_link",
            "base_link",
        ],
        output="screen",
    )

    # ---------------- 2) STATIC TF: map → odom ----------------
    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # ---------------- 3) POSE MONITOR ----------------
    pose_monitor = Node(
        package="robot_localization",
        executable="pose_monitor",
        name="pose_monitor",
        output="screen",
    )

    # ---------------- 4) TILE POSITION SERVICE ----------------

    tile_position_service = Node(
        package="robot_localization",
        executable="tile_position_service",
        name="tile_position_service",
        output="screen",
    )

    # ---------------- 5) RVIZ ----------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_robot_loc, "rviz", "visualization.rviz")
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ---------------- RETURN ----------------
    return LaunchDescription([
        use_rviz_arg,
        static_tf_camera_base,
        static_tf_map_odom,
        pose_monitor,
        tile_position_service,
        rviz,
    ])
