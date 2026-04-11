"""
Tuning Session Launch File

Starts the tuning infrastructure on top of your existing Nav2 + PID stack.

Usage:
  ros2 launch nav2_tuning_tools tuning_session.launch.py

  # With specific nodes to monitor:
  ros2 launch nav2_tuning_tools tuning_session.launch.py \
    monitor_nodes:="['controller_server','pid_controller','local_costmap']"

Your existing launch files start Nav2 + PID. This launch adds:
  - tuning_metrics node
  - system_monitor node

Then in separate terminals you run:
  - param_tuner (interactive CLI)
  - test_runner (sends goals)
  - PlotJuggler (visualization)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "monitor_nodes",
                default_value="['controller_server','planner_server','pid_controller','bt_navigator']",
                description="List of node names to monitor for memory/CPU",
            ),
            DeclareLaunchArgument(
                "leak_threshold",
                default_value="0.5",
                description="Memory leak threshold in MB/minute",
            ),
            # Tuning metrics node
            Node(
                package="nav2_tuning_tools",
                executable="metrics_node",
                name="tuning_metrics",
                output="screen",
            ),
            # System monitor
            Node(
                package="nav2_tuning_tools",
                executable="system_monitor",
                name="system_monitor",
                output="screen",
                parameters=[
                    {
                        "monitor_nodes": LaunchConfiguration("monitor_nodes"),
                        "leak_threshold_mb_per_min": LaunchConfiguration(
                            "leak_threshold"
                        ),
                        "publish_rate": 1.0,
                    }
                ],
            ),
        ]
    )
