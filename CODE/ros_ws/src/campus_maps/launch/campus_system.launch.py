#!/usr/bin/env python3
"""
Campus Navigation Launch File

TF Tree Structure:
    map
     └── odom          (static: map = odom until ArUco correction)
         └── zed_camera_link   (from ZED odometry)
             └── base_link     (static: camera = robot body)

What this launches:
    ✓ ZED Camera (defines odometry)
    ✓ Static TF: zed_camera_link → base_link
    ✓ Static TF: map → odom (identity until ArUco)
    ✓ Map Server (lifecycle managed)
    ✓ Nav2 Navigation Stack (NO AMCL)
    ✓ Tile Switcher
    ✓ RViz (optional)

What this does NOT launch (intentionally):
    ✗ AMCL
    ✗ Fake odometry publishers
    ✗ Static odom → base_link
    ✗ initialpose
    ✗ Wheel encoders
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

    pkg_share = get_package_share_directory('campus_maps')

    # ---------------- LAUNCH ARGUMENTS ----------------
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    # ---------------- 1) ZED CAMERA ----------------
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': 'zedm',
        }.items()
    )

    # ---------------- 2) STATIC TF: camera → base_link ----------------
    static_tf_camera_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'zed_camera_link', 'base_link'],
        output='screen'
    )

    # ---------------- 3) STATIC TF: map → odom ----------------
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # ---------------- 4) MAP SERVER ----------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(pkg_share, 'maps', 'tile01.yaml'),
            'use_sim_time': False,
        }]
    )

    # ---------------- 5) LIFECYCLE MANAGER (map_server only) ----------------
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    # ---------------- 6) NAV2 NAVIGATION (NO AMCL) ----------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items()
    )

    # ---------------- 7) TILE SWITCHER ----------------
    tile_switcher = Node(
        package='campus_maps',
        executable='tile_switcher',
        name='tile_switcher',
        output='screen',
    )

    # ---------------- 8) RVIZ (optional) ----------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'campus_nav.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # ---------------- RETURN LAUNCH DESCRIPTION ----------------
    return LaunchDescription([
        use_rviz_arg,
        zed_launch,
        static_tf_camera_base,
        static_tf_map_odom,
        map_server,
        lifecycle_manager_map,
        nav2_launch,
        tile_switcher,
        rviz,
    ])
