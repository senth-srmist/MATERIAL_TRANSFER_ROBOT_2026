#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("wheel_encoder_driver"), "config", "encoder_params.yaml"
    )

    encoder_driver = Node(
        package="wheel_encoder_driver",
        executable="encoder_driver",
        name="encoder_driver",
        output="screen",
        parameters=[params],
    )

    wheel_odometry = Node(
        package="wheel_encoder_driver",
        executable="wheel_odometry",
        name="wheel_odometry",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([encoder_driver, wheel_odometry])
