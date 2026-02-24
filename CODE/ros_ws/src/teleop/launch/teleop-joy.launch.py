from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_teleop = get_package_share_directory("teleop")
    teleop_config = os.path.join(pkg_teleop, "config", "teleop.yaml")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id": 0
        }],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        output="screen",
        parameters=[teleop_config],
        remappings=[("cmd_vel", "cmd_vel_joy")],
    )

    # Wait for joy_node to start before launching teleop_node
    delayed_teleop = RegisterEventHandler(
        OnProcessStart(
            target_action=joy_node,
            on_start=[TimerAction(period=1.0, actions=[teleop_node])],
        ))

    return LaunchDescription([
        joy_node,
        delayed_teleop,
    ])
