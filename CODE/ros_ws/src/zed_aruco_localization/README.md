# ZED ArUco Localization

ArUco marker-based localization and evaluation for ZED stereo cameras. Three tools in one package: generate printable markers, localize the camera at runtime, and evaluate marker performance under controlled conditions.

## Quick Start

```bash
# Generate markers
ros2 run zed_aruco_localization aruco_marker_generator --ros-args -p marker_size:=0.07

# Start ZED camera first (required)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm

# Then run localization
ros2 launch zed_aruco_localization zed_aruco_loc.launch.py camera_model:=zedm

# Or run evaluation
ros2 launch zed_aruco_localization aruco_evaluation.launch.py
ros2 service call /start_evaluation zed_aruco_localization/srv/StartArucoEvaluation \
  "{distance_m: 2.0, angle_deg: 0.0, num_samples: 300}"
```

## Package Structure

```
zed_aruco_localization/
├── CMakeLists.txt
├── config/
│   └── aruco_loc.yaml              # Marker config (auto-generated + manual edits)
├── eval_metrics/
│   └── stats.csv                   # Evaluation results (auto-appended)
├── launch/
│   ├── aruco_evaluation.launch.py  # Evaluation pipeline
│   └── zed_aruco_loc.launch.py     # Runtime localization
├── markers/
│   └── 6x6_1000-0.png             # Generated marker images
├── rviz2/
│   └── aruco.rviz                  # RViz config for visualization
├── src/
│   ├── aruco_marker_eval.cpp       # Evaluation node
│   ├── aruco_marker_generator.cpp  # Marker generator node
│   ├── component/
│   │   ├── include/
│   │   │   ├── aruco.hpp
│   │   │   └── zed_aruco_localization_component.hpp
│   │   └── src/
│   │       ├── aruco.cpp
│   │       └── zed_aruco_localization_component.cpp
│   └── include/
│       └── aruco_loc_visibility_control.hpp
├── srv/
│   └── StartArucoEvaluation.srv
└── videos/
    └── aruco.mp4
```

## Prerequisites

The ZED camera must be running before launching the localization or evaluation nodes. Start it separately:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
```

The localization node subscribes to ZED image topics and calls ZED's `set_pose` service. If ZED is not running, the node will wait for topics and the service to become available.

## Dependencies

- OpenCV 4.x (core, imgproc, calib3d, aruco module)
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) (zed_wrapper, zed_components, zed_msgs)
- image_transport
- tf2, tf2_ros, tf2_geometry_msgs
- yaml-cpp

Supports all ZED camera models: zed, zedm, zed2, zed2i, zedx, zedxm.

## Nodes

### 1. Marker Generator (`aruco_marker_generator`)

Generates printable ArUco markers at real-world physical size and updates `aruco_loc.yaml` with marker definitions.

```bash
ros2 run zed_aruco_localization aruco_marker_generator --ros-args \
  -p marker_count:=3 \
  -p marker_size:=0.07 \
  -p dpi:=300
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `marker_count` | 1 | Number of markers to generate (IDs: 0 to N-1) |
| `marker_size` | 0.07 | Physical marker size in meters |
| `dpi` | 300 | Print resolution |
| `output_format` | png | Output format (png, jpg, svg) |
| `maximum_distance` | 2.0 | Max detection distance in meters |
| `detection_rate` | 0.5 | Detection frequency in Hz |
| `camera_name` | zed | Camera namespace |
| `world_frame_id` | map | World frame for marker poses |
| `refine_detection` | false | Enable sub-pixel corner refinement |
| `debug_level` | 1 | 0=none, 1=markers, 2=full |

Markers are saved to `markers/` directory. Dictionary: `DICT_6X6_1000`.

> Use **image-magick** to display images at actual size. When printing, ensure scaling is set to 100% / "Actual Size".

### 2. Localization Node (`zed_aruco_loc_node`)

Detects ArUco markers in the ZED camera feed, estimates camera pose relative to known marker positions, and resets the ZED pose in the global map frame via the `set_pose` service. Runs as a composable node in its own container.

**Topics:**

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `<cam>/zed_node/rgb/color/rect/image` | sensor_msgs/Image | ZED rectified RGB image |
| Subscribe | `<cam>/zed_node/rgb/color/rect/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |
| Publish | `<cam>/aruco_node/out/aruco_result` | sensor_msgs/Image | Detection visualization |

**Services (client):**

| Service | Type | Description |
|---------|------|-------------|
| `<cam>/zed_node/set_pose` | zed_msgs/SetPose | Resets ZED camera pose when marker detected |

**TF Broadcasts:**

| Parent Frame | Child Frame | Description |
|-------------|-------------|-------------|
| `<world_frame_id>` | `aruco_marker_<id>` | Known marker positions in world frame |

**Behavior:**
- Only the nearest valid marker within `maximum_distance` is used
- Pose estimation via `estimatePoseSingleMarkers`
- Detection frequency throttled by `detection_rate`
- Requires the `<camera_name>_left_camera_frame` → `<camera_name>_camera_link` TF (published by ZED's state publisher)

**Launch:**

```bash
# ZED must be running first
ros2 launch zed_aruco_localization zed_aruco_loc.launch.py camera_model:=zedm
```

**Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_model` | zedm | zed, zedm, zed2, zed2i, zedx, zedxm |
| `camera_name` | zed | Camera namespace |
| `aruco_node_name` | aruco_node | Name of the ArUco node |
| `config_path_aruco` | (package default) | Path to aruco_loc.yaml |
| `rviz` | true | Launch RViz with ArUco config |

### 3. Evaluation Node (`aruco_evaluation`)

Evaluates ArUco marker detection accuracy under controlled conditions. Idle by default — triggered via service call to collect pose statistics at a specific distance and angle.

This node does **not** reset pose, publish TF, or interfere with localization.

**Topics:**

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/zed/zed_node/rgb/color/rect/image` | sensor_msgs/Image | ZED rectified RGB |
| Subscribe | (camera_info from same transport) | sensor_msgs/CameraInfo | Camera intrinsics |

**Services (server):**

| Service | Type | Description |
|---------|------|-------------|
| `/start_evaluation` | StartArucoEvaluation | Start a measurement run |

**Service Request:**

| Field | Type | Description |
|-------|------|-------------|
| `distance_m` | float32 | Physical distance between camera and marker |
| `angle_deg` | float32 | Viewing angle of marker |
| `num_samples` | int32 | Number of frames to process |

**Launch:**

```bash
# ZED must be running first
ros2 launch zed_aruco_localization aruco_evaluation.launch.py
ros2 launch zed_aruco_localization aruco_evaluation.launch.py camera_model:=zedm
```

**Computed Statistics (per run):**
- Mean and standard deviation of x, y, z, yaw
- Detection ratio (detections / attempted frames)
- Results appended to `eval_metrics/stats.csv`

**CSV Columns:**

```
timestamp, marker_size_m, distance_m, angle_deg, samples_collected,
frames_attempted, detection_ratio, mean_x_m, std_x_m, mean_y_m,
std_y_m, mean_z_m, std_z_m, mean_yaw_deg, std_yaw_deg
```

## Configuration

All parameters are loaded from `config/aruco_loc.yaml`. This file is auto-generated by the marker generator but can be edited manually.

```yaml
/**:
  ros__parameters:
    general:
      marker_count: 1
      marker_size: 0.030
      maximum_distance: 2.000
      detection_rate: 0.500
      camera_name: zed
      world_frame_id: map
      refine_detection: false
    debug:
      level: 1                          # 0=none, 1=markers, 2=full
    marker_000:
      aruco_id: 0
      position: [0.000, 0.000, 0.000]  # x, y, z in world frame
      orientation: [0.000, 0.000, 0.000] # roll, pitch, yaw in radians
```

> For every marker, update `position` and `orientation` with respect to the world frame to ensure correct localization. Default values are `[0, 0, 0]`.

## Evaluation Workflow

1. Print marker at target size using the generator
2. Mount marker flat on a wall
3. Place camera at a known distance and angle
4. Start ZED camera
5. Launch the evaluation node
6. Call the evaluation service with distance, angle, and sample count
7. Wait for completion — results are logged to terminal and CSV
8. Move to next configuration and repeat

Each service call produces one row in `eval_metrics/stats.csv`. The file is append-only, never overwritten.

## Integration Status

The localization node is planned for integration into the robot's production runtime via the system supervisor. Currently it runs standalone for testing and evaluation. Future integration will allow the supervisor to trigger ArUco-based relocalization on demand, for example after a ZED camera crash or localization drift.
