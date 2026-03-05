from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("diff_drive_controller")

    return LaunchDescription([
        # =========================
        # Twist Mux Node
        # =========================
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "twist_mux.yaml")],
        ),
        # =========================
        # Base Controller Node
        # =========================
        Node(
            package="sabertooth_diff_drive",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[
                os.path.join(pkg_dir, "config", "controller_params.yaml")
            ],
        ),
    ])
