#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    twist_mux_config = os.path.join(
        get_package_share_directory("diff_drive_controller"), "config",
        "twist_mux.yaml")

    encoder_params = os.path.join(
        get_package_share_directory("wheel_encoder_driver"),
        "config",
        "encoder_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            parameters=[twist_mux_config],
            output="log",
        ),
        Node(
            package="wheel_encoder_driver",
            executable="encoder_driver",
            name="encoder_driver",
            parameters=[encoder_params],
            output="log",
        ),
        Node(
            package="diff_drive_controller",
            executable="pid_controller",
            name="pid_controller",
            output="log",
        ),
        Node(
            package="diff_drive_controller",
            executable="motor_driver",
            name="motor_driver",
            output="log",
        ),
    ])
