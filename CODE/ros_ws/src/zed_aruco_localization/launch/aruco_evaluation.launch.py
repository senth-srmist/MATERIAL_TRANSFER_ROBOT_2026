import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    aruco_package_dir = get_package_share_directory('zed_aruco_localization')
    
    # Path to config files
    params_file = PathJoinSubstitution([
        FindPackageShare('zed_aruco_localization'),
        'config',
        'aruco_loc.yaml'
    ])
    
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('zed_aruco_localization'),
        'rviz2',
        'aruco.rviz'
    ])
    
    # Launch arguments
    camera_model = LaunchConfiguration('camera_model', default='zed2i')
    
    # Declare launch arguments
    declare_camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model (zed, zed2, zed2i, zedm, zedx, zedxm)'
    )
    
    # Include ZED camera launch file
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'publish_tf': 'true',
            'publish_map_tf': 'true',
        }.items()
    )
    
    # ArUco evaluation node
    aruco_eval_node = Node(
        package='zed_aruco_localization',
        executable='aruco_evaluation',
        name='aruco_evaluation',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    
    return LaunchDescription([
        declare_camera_model,
        zed_launch,
        aruco_eval_node,
        rviz_node,
    ])
