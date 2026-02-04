from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ------------------------------------------------
    # 1) ZED MINI CAMERA
    # ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
    # ------------------------------------------------
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': 'zedm'
        }.items()
    )

    # ------------------------------------------------
    # 2) STATIC TRANSFORM (camera → base_link)
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
    # 4) AMCL
    # ------------------------------------------------
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen'
    )

    # ------------------------------------------------
    # 5) LIFECYCLE MANAGER (map_server + amcl)
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
    # 6) NAV2 BRINGUP
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
    # 7) RVIZ
    # ------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # ------------------------------------------------
    # 8) TILE SWITCHER
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
