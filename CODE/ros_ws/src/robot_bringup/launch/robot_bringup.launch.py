#!/usr/bin/env python3
"""
Robot Bringup Launch File - Event-Driven Startup

Startup sequence with dependency checks:
1. Teleop + Diff Drive (immediate - no dependencies)
2. ZED Camera (immediate) - uses custom optimized config
   └─> Wait for /zed/zed_node/rgb/color/rect/image topic
3. Mission Controller (after ZED ready)
   └─> Wait for /navigate_to_room service
4. Map Server (after Mission Controller ready)
   └─> Wait for /map topic
5. Localization + Nav2 (after Map Server ready)
   └─> Wait for /navigate_to_pose action

Then ready to accept goals!
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    LogInfo,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_zed_wrapper = get_package_share_directory("zed_wrapper")
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    pkg_robot_loc = get_package_share_directory("robot_localization")
    pkg_teleop = get_package_share_directory("teleop")
    pkg_tile_manager = get_package_share_directory("tile_manager")
    pkg_robot_nav = get_package_share_directory("robot_navigation")
    pkg_sabertooth_diff_drive = get_package_share_directory("sabertooth_diff_drive")

    # Custom ZED config (optimized for navigation)
    zed_config = os.path.join(pkg_robot_bringup, "config", "zedm.yaml")

    # ==================== LAUNCH ARGUMENT ====================
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false", description="Launch RViz"
    )

    # ==================== STAGE 0: IMMEDIATE START ====================
    # Teleop and Diff Drive - no dependencies
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop, "launch", "teleop-joy.launch.py")
        ),
    )

    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_sabertooth_diff_drive,
                "launch",
                "controller_with_twist_mux.launch.py",
            )
        ),
    )

    # ==================== STAGE 1: ZED CAMERA ====================
    # Using custom optimized config from robot_bringup/config/zedm.yaml
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed_wrapper, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model": "zedm",
            "ros_params_override_path": zed_config,
        }.items(),
    )

    # Wait for ZED to be ready (correct topic name)
    wait_for_zed = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'echo "[BRINGUP] Waiting for ZED camera..." && '
            'until ros2 topic info /zed/zed_node/rgb/color/rect/image 2>/dev/null | grep -q "Publisher count: 1"; do sleep 0.5; done && '
            'echo "[BRINGUP] ✓ ZED Camera READY"',
        ],
        output="screen",
    )

    # ==================== STAGE 2: MISSION CONTROLLER ====================
    mission_controller_node = Node(
        package="mission_controller",
        executable="mission_service.py",
        name="mission_controller",
        output="screen",
    )

    # Wait for Mission Controller service to be ready
    wait_for_mission = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'echo "[BRINGUP] Waiting for Mission Controller..." && '
            'until ros2 service list 2>/dev/null | grep -q "/navigate_to_room"; do sleep 0.5; done && '
            'echo "[BRINGUP] ✓ Mission Controller READY"',
        ],
        output="screen",
    )

    # ==================== STAGE 3: MAP SERVER ====================
    tile_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tile_manager, "launch", "map_server.launch.py")
        ),
    )

    # Wait for Map Server to publish /map
    wait_for_map = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'echo "[BRINGUP] Waiting for Map Server..." && '
            'until ros2 topic info /map 2>/dev/null | grep -q "Publisher count: 1"; do sleep 0.5; done && '
            'echo "[BRINGUP] ✓ Map Server READY"',
        ],
        output="screen",
    )

    # ==================== STAGE 4: LOCALIZATION + NAV2 ====================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_loc, "launch", "robot_localization.launch.py")
        ),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_nav, "launch", "navigation.launch.py")
        ),
    )

    # Wait for Nav2 to be ready
    wait_for_nav2 = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'echo "[BRINGUP] Waiting for Nav2..." && '
            'until ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; do sleep 0.5; done && '
            'echo "[BRINGUP] ✓ Nav2 READY"',
        ],
        output="screen",
    )

    # Final ready message
    all_ready = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'echo "" && '
            'echo "============================================" && '
            'echo "[BRINGUP] ✓✓✓ ALL SYSTEMS READY ✓✓✓" && '
            'echo "[BRINGUP] You can now send navigation goals!" && '
            'echo "============================================" && '
            'echo ""',
        ],
        output="screen",
    )

    # ==================== EVENT HANDLERS - CHAIN THE STARTUP ====================

    # After ZED wait completes → Start Mission Controller
    start_mission_after_zed = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_zed,
            on_exit=[mission_controller_node, wait_for_mission],
        )
    )

    # After Mission Controller wait completes → Start Map Server
    start_map_after_mission = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_mission,
            on_exit=[tile_manager_launch, wait_for_map],
        )
    )

    # After Map Server wait completes → Start Localization + Nav2
    start_nav_after_map = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map,
            on_exit=[localization_launch, navigation_launch, wait_for_nav2],
        )
    )

    # After Nav2 wait completes → Print ready message
    ready_after_nav = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav2,
            on_exit=[all_ready],
        )
    )

    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription(
        [
            use_rviz_arg,
            LogInfo(msg=""),
            LogInfo(msg="============================================"),
            LogInfo(msg="[BRINGUP] Starting Robot Bringup Sequence..."),
            LogInfo(msg="============================================"),
            LogInfo(msg=""),
            # Stage 0: Immediate starts (no dependencies)
            teleop_launch,
            diff_drive_launch,
            # Stage 1: ZED Camera
            zed_launch,
            wait_for_zed,
            # Event handlers for chained startup
            start_mission_after_zed,
            start_map_after_mission,
            start_nav_after_map,
            ready_after_nav,
        ]
    )
