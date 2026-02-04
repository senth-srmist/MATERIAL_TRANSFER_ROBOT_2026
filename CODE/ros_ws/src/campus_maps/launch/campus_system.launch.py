from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ------------------------------------------------
    # 1) ZED MINI CAMERA
    # ros2 launch zed_wrapper zed_camera.launch.py
    # ------------------------------------------------
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        )
    )

    # ------------------------------------------------
    # 2) STATIC TRANSFORM (camera → base_link)
    # ros2 run tf2_ros static_transform_publisher ...
    # ------------------------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'zed_camera_link', 'base_link'],
        output='screen'
    )

    # ------------------------------------------------
    # 3) MAP SERVER (tile01.yaml)
    # ros2 run nav2_map_server map_server ...
    # ------------------------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': '/workspace/ros_ws/src/campus_maps/maps/tile01.yaml'
        }]
    )

    # ------------------------------------------------
    # 4 & 6) NAV2 LIFECYCLE MANAGER
    # replaces all ros2 lifecycle set commands
    # ------------------------------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # ------------------------------------------------
    # 5) AMCL
    # ros2 run nav2_amcl amcl
    # ------------------------------------------------
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen'
    )

    # ------------------------------------------------
    # 7) NAV2 BRINGUP
    # ros2 launch nav2_bringup navigation_launch.py
    # ------------------------------------------------
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
            'autostart': 'true'
        }.items()
    )

    # ------------------------------------------------
    # 8) RVIZ
    # ros2 run rviz2 rviz2
    # ------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # ------------------------------------------------
    # 9) TILE SWITCHER
    # ros2 run campus_maps tile_switcher
    # ------------------------------------------------
    tile_switcher = Node(
        package='campus_maps',
        executable='tile_switcher',
        name='tile_switcher',
        output='screen'
    )

    return LaunchDescription([
        zed_launch,
        static_tf,
        map_server,
        amcl,
        lifecycle_manager,
        nav2_launch,
        rviz,
        tile_switcher
    ])
