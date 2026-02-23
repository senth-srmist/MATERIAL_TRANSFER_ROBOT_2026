from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    twist_mux_params = (
        "/workspace/ros_ws/install/sabertooth_diff_drive/share/sabertooth_diff_drive/config/twist_mux.yaml"
    )

    return LaunchDescription([

        # =========================
        # Twist Mux Node
        # =========================
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[twist_mux_params],
        ),

        # =========================
        # Base Controller Node
        # =========================
        Node(
            package="sabertooth_diff_drive",
            executable="controller_node",
            name="controller_node",
            output="screen",
        ),
    ])