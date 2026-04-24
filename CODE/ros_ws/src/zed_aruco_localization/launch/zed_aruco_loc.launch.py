import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

default_config_aruco = os.path.join(
    get_package_share_directory("zed_aruco_localization"), "config", "aruco_loc.yaml"
)


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration("camera_name")
    camera_model = LaunchConfiguration("camera_model")
    aruco_node_name = LaunchConfiguration("aruco_node_name")
    config_path_aruco = LaunchConfiguration("config_path_aruco")
    start_rviz = LaunchConfiguration("rviz")
    log_level = LaunchConfiguration("log_level")

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    aruco_node_name_val = aruco_node_name.perform(context)

    if camera_name_val == "":
        camera_name_val = "zed"

    # Determine ZED node name for topic remapping
    zed_node_name_val = "zed_node"

    config_rviz2 = os.path.join(
        get_package_share_directory("zed_aruco_localization"), "rviz2", "aruco.rviz"
    )

    # RViz2 node
    rviz2_node = Node(
        condition=IfCondition(start_rviz),
        package="rviz2",
        executable="rviz2",
        name=camera_model_val + "_rviz2",
        output="screen",
        arguments=[["-d"], [config_rviz2]],
    )

    # ArUco component in its own container
    zed_aruco_component = ComposableNode(
        package="zed_aruco_localization",
        namespace=camera_name_val,
        plugin="stereolabs::ZedArucoLoc",
        name=aruco_node_name,
        parameters=[config_path_aruco],
        remappings=[
            ("in/zed_image", zed_node_name_val + "/rgb/image_rect_color"),
            ("/" + camera_name_val + "/in/camera_info",
             "/" + camera_name_val + "/" + zed_node_name_val + "/rgb/camera_info"),
            ("set_pose", zed_node_name_val + "/set_pose"),
        ],
    )

    container = ComposableNodeContainer(
        name="aruco_localization_container",
        namespace=camera_name_val,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[zed_aruco_component],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [rviz2_node, container]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value=TextSubstitution(text="zed"),
                description="The name of the camera. Used as node namespace.",
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value="zedm",
                description="ZED camera model (zed, zedm, zed2, zed2i, zedx, zedxm)",
            ),
            DeclareLaunchArgument(
                "aruco_node_name",
                default_value="aruco_node",
                description="The name of the ArUco detection node.",
            ),
            DeclareLaunchArgument(
                "config_path_aruco",
                default_value=TextSubstitution(text=default_config_aruco),
                description="Path to the YAML configuration file for the ArUco detector.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 with ArUco detection visualization",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logger level: debug, info, warn, error, fatal",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
