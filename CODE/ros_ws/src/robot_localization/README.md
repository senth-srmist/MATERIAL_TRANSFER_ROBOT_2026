# robot_localization

ROS2 package for robot pose monitoring and localization abstraction.

## Overview

This package provides a TF-based localization for the downstream consumers. It publishes full 6DOF pose data without making assumptions about the robot's environment.

## Nodes

### pose_monitor

Monitors TF transforms and publishes robot pose.

**Subscriptions:**
- TF: `map` → `zed_camera_link`

**Publications:**
| Topic | Type | Description |
|-------|------|-------------|
| `/robot_pose` | `geometry_msgs/PoseStamped` | Full 6DOF pose |
| `/robot_movement_yaw` | `std_msgs/Float32` | Movement direction in radians |
| `/path_taken` | `visualization_msgs/Marker` | Path visualization for RViz |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `source_frame` | `zed_camera_link` | Source TF frame |
| `target_frame` | `map` | Target TF frame |
| `publish_rate` | `20.0` | Publishing rate in Hz |
| `movement_threshold` | `0.02` | Minimum movement (m) to update heading |

## Launch Files

### robot_localization.launch.py

Launches the complete localization stack:
- ZED Camera
- Static TFs (camera→base_link, map→odom)
- Map Server
- Nav2 Navigation
- Pose Monitor
- RViz (optional)

**Usage:**
```bash
# With RViz
ros2 launch robot_localization robot_localization.launch.py

# Without RViz
ros2 launch robot_localization robot_localization.launch.py use_rviz:=false
```

## TF Tree

```
map
 └── odom (static identity transform)
     └── zed_camera_link (from ZED odometry)
         └── base_link (static identity transform)
```

## Architecture

```
┌─────────────────────┐
│     ZED Camera      │
│  (TF: odom → cam)   │
└──────────┬──────────┘
           │ TF
           ▼
┌─────────────────────┐
│    pose_monitor     │
│  - TF lookup        │
│  - Movement calc    │
│  - Visualization    │
└──────────┬──────────┘
           │
           ├──► /robot_pose (PoseStamped)
           ├──► /robot_movement_yaw (Float32)
           └──► /path_taken (Marker)
```

## Dependencies

- `rclpy`
- `tf2_ros`
- `geometry_msgs`
- `std_msgs`
- `visualization_msgs`
- `zed_wrapper`
- `nav2_bringup`
- `nav2_map_server`
- `nav2_lifecycle_manager`

## Installation

```bash
cd ~/ros_ws
colcon build --packages-select robot_localization
source install/setup.bash
```
