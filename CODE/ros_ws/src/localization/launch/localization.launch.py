#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("localization")
    zed_launch_dir = get_package_share_directory("zed_wrapper")

    robot_description = ParameterValue(
        Command(["xacro ",
                 os.path.join(pkg, "urdf", "robot.urdf.xacro")]),
        value_type=str,
    )

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_launch_dir, "launch", "zed_camera.launch.py")),
        launch_arguments={
            "publish_tf": "false",
            "camera_model": "zedm",
            "ros_params_override_path": os.path.join(pkg, "config",
                                                     "zedm.yaml"),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 10.0
        }],
    )

    zed_odom_relay = Node(
        package="localization",
        executable="zed_odom_relay",
        name="zed_odom_relay",
        output="screen",
    )

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[os.path.join(pkg, "config", "ekf_local_params.yaml")],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[os.path.join(pkg, "config", "ekf_global_params.yaml")],
        remappings=[("odometry/filtered", "odometry/filtered/global")],
    )

    return LaunchDescription([
        zed,
        robot_state_publisher,
        zed_odom_relay,
        ekf_local,
        ekf_global,
    ])
