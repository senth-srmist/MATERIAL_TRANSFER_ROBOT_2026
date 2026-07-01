#!/usr/bin/env python3
"""
Navigation Launch File

Launches Nav2 with custom params and behavior tree for tile switching.

Prerequisites:
    - map_server running (ros2 launch tile_manager map_server.launch.py)
    - localization running (ros2 launch robot_localization robot_localization.launch.py)

Usage:
    ros2 launch robot_navigation navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package paths
    pkg_robot_nav = FindPackageShare("robot_navigation")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    bt_xml_file = LaunchConfiguration("bt_xml_file")
    log_level = LaunchConfiguration("log_level")

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically start Nav2 nodes"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [pkg_robot_nav, "config", "nav2_params.yaml"]
        ),
        description="Nav2 parameters file",
    )

    declare_bt_xml_file = DeclareLaunchArgument(
        "bt_xml_file",
        default_value=PathJoinSubstitution(
            [pkg_robot_nav, "behavior_trees", "navigate_with_tiles.xml"]
        ),
        description="Behavior tree XML file",
    )

    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )

    # Rewrite params with use_sim_time
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )

    # Lifecycle nodes to manage
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    # Controller Server
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="log",
        parameters=[configured_params],
        remappings=[("cmd_vel", "cmd_vel_nav2")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Planner Server
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="log",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Behavior Server
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="log",
        parameters=[configured_params],
        remappings=[("cmd_vel", "cmd_vel_nav2")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # BT Navigator
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="log",
        parameters=[
            configured_params,
            {
                "default_nav_to_pose_bt_xml": bt_xml_file,
                "default_nav_through_poses_bt_xml": bt_xml_file,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="log",
        parameters=[
            {
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_autostart,
            declare_params_file,
            declare_bt_xml_file,
            declare_log_level,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]
    )
