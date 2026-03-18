from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("diff_drive_controller")

    return LaunchDescription([
        # =========================
        # Twist Mux
        # =========================
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "twist_mux.yaml")],
        ),
        # =========================
        # Motor Driver (serial I/O)
        # =========================
        Node(
            package="diff_drive_controller",
            executable="motor_driver",
            name="motor_driver",
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "drive_params.yaml")],
        ),
    ])
