# ZED ArUco Localization & Evaluation (ROS 2)

This package provides tools for:

1. **ArUco-based camera localization** using a ZED camera
2. **ArUco marker generation** for printing and deployment
3. **Experimental evaluation of ArUco markers** to measure:

   * Standoff distance limits
   * Angular resolution
   * Pose stability and detection reliability

The evaluation pipeline is designed to support **marker sizing, placement, and segmentation decisions** before deploying localization in a real environment.

---

## Package Overview

```
ros_ws/src/zed_aruco_localization
├── CMakeLists.txt
├── config
│   └── aruco_loc.yaml  # Auto-Updated using aruco_marker_generator node
├── eval_metrics
│   └── stats.csv   # Auto-generated and updated by aruco_evaluation node
├── launch
│   ├── aruco_evaluation.launch.py
│   └── zed_aruco_loc.launch.py
├── markers
│   └── 6x6_1000-0.png   # Generated using aruco_marker_generator node
├── package.xml
├── README.md
├── rviz2
│   └── aruco.rviz
├── src
│   ├── aruco_marker_eval.cpp
│   ├── aruco_marker_generator.cpp
│   ├── component
│   │   ├── include
│   │   │   ├── aruco.hpp
│   │   │   └── zed_aruco_localization_component.hpp
│   │   └── src
│   │       ├── aruco.cpp
│   │       └── zed_aruco_localization_component.cpp
│   └── include
│       └── aruco_loc_visibility_control.hpp
└── srv
    └── StartArucoEvaluation.srv
```

---

## 1. ArUco Marker Generator

### Purpose

Generate printable ArUco markers at real-world physical size along with aruco_loc config file with necessary parameters for localisation.

> Use **image-magick** to display the images in their actual sizes. Ensure image isn't scaled to screen while displaying on a system and are printed in actual sizes.

### Node

```
aruco_marker_generator
```

### Usage

```bash
ros2 run zed_aruco_localization aruco_marker_generator --ros-args -p marker_size:=0.07
```

* Marker dictionary: `DICT_6X6_1000`
* Generated images are placed in the **markers** directory

### Parameters

* marker_count: Number of markers in the World (Default: 1)
* marker_size: Width/Height of the ArUco tags in meters (Default: 0.07)
* maximum_distance: Maximum distance of the target from the camera to consider it valid in meters (Default: 2.0)
* detection_rate: Maximum detection frequency for camera pose update in Hz (Default: 0.5)
* refine_detection: If enabled the corners of the detected markers will be processed to obtain sub-pixel precision (Default: false)
* debug_level: Set debug messages level (Default: 1)
    * debug_level==0: No debug messages
    * debug_level==1: Show only for detected markers and it's pose estimation
    * debug_level==2: Full debug logs

> The parameters define the aruco marker characteristics and data requried for successful localisation. The parameters file is updated by automatically by the node in config/aruco_loc.yaml. If required you may manually modify it by editing the yaml file.
---

## 2. ArUco-Based Localization

### Node

```
zed_aruco_loc_node
```

### Purpose

* Detect ArUco markers
* Estimate camera pose relative to known markers
* Reset the ZED pose in the global map frame

This node is intended for **runtime localization**.

---

### Configuration

All parameters are loaded from:

```
config/aruco_loc.yaml
```

Example:

```yaml
general:
  marker_count: 1
  marker_size: 0.07
  maximum_distance: 2.0
  detection_rate: 0.5
  camera_name: zed
  world_frame_id: map
  refine_detection: false

marker_000:
  aruco_id: 0
  position: [0.0, 0.0, 0.0]
  orientation: [0.0, 0.0, 0.0]
```

> For every marker update the position and orientation with respect to the world frame in yaml file to ensure the camera localises itself properly. By default every marker's position and orientation is set to [0.0, 0.0, 0.0] and [0.0, 0.0, 0.0] respectively.
---

### Launch

```bash
ros2 launch zed_aruco_localization zed_aruco_loc.launch.py camera_model:=<camera_model>
```
> By default the node launches with an rviz interface. To disable use rviz:=false

### Behavior Summary

* Only the **nearest valid marker within the general.maximum_distance mentioned** is used
* Pose is estimated using `estimatePoseSingleMarkers`
* ZED pose is reset using the `set_pose` service
* Detection frequency is throttled by `general.detection_rate`

---

## 3. ArUco Marker Evaluation

### Node

```
aruco_evaluation
```

### Purpose

This node is used to **experimentally evaluate ArUco markers** under controlled conditions.

It answers questions like:

* How far can this marker be detected?
* At what angles does pose estimation become unstable?
* How noisy are `x`, `y`, `z`, and `yaw`?

This node **does NOT**:

* Reset pose
* Publish TF
* Interfere with localization

---

### Start the Evaluation Node

Run the evaluation node directly using `ros2 run`:

```bash
ros2 launch zed_aruco_localization aruco_evaluation.launch.py
```

What this does:

* Starts the **ArucoEvaluationNode**
* Loads marker parameters from `aruco_loc.yaml`
* Subscribes to:

  * ZED rectified RGB image
  * Camera info

* Waits **idle** for service calls

You should see logs indicating:

* Parameters loaded
* Image subscription active

---

## Evaluation Control (Service-Call-Based)

The evaluation node is **idle by default** and only runs when triggered via a ROS 2 service.

---

### Service Name

```
/start_evaluation
```

### Service Type

```
zed_aruco_localization/srv/StartArucoEvaluation
```

---

### Service Request

| Field        | Description                                 |
| ------------ | ------------------------------------------- |
| `distance_m` | Physical distance between camera and marker |
| `angle_deg`  | Viewing angle of marker                     |
| `samples`    | Number of valid pose samples to collect     |

Example:

```bash
ros2 service call /start_evaluation zed_aruco_localization/srv/StartArucoEvaluation "{distance_m: 2.0, angle_deg: 30.0, samples: 300}"
```

---

## Evaluation Parameters

The evaluator **reuses the same YAML file** as localization:

```
config/aruco_loc.yaml
```

Required parameter:

```yaml
general:
  marker_size: 0.07
```

The node will **abort immediately** if this parameter is missing, zero, or negative.

---

## Evaluation Workflow

For each service call:

1. Internal buffers are reset
2. Frames are subscribed from the ZED camera
3. ArUco markers are detected
4. Pose is estimated (camera w.r.t marker)
5. `(x, y, z, yaw)` are stored per valid detection
6. Collection stops after `samples` valid detections
7. Statistics are computed
8. Results are logged to CSV
9. Node returns to idle

---

## Computed Statistics (Per Experiment)

For the collected samples:

* Mean:

  * `x`, `y`, `z`, `yaw`
* Standard deviation:

  * `x`, `y`, `z`, `yaw`
* Detection ratio:

  ```
  detections / attempted_frames
  ```

Yaw is reported in **degrees**.

---

## CSV Logging

### Location

```
eval_metrics/stats.csv
```

* File is auto-created if missing
* Results are **appended**, never overwritten
* One row per service call

---

### CSV Columns

```
timestamp
ground_truth_distance_m,
ground_truth_angle_deg,
samples_collected,
frames_attempted,
detection_ratio,
mean_x_m,
std_x_m,
mean_y_m,
std_y_m,
mean_z_m,
std_z_m,
mean_yaw_deg,
std_yaw_deg
```

This file is intended for:

* Plotting
* Threshold selection
* Marker placement decisions

---

## Recommended Experiment Procedure

1. Print marker at target size
2. Mount marker flat on wall
3. Place camera at known distance
4. Set viewing angle
5. Call evaluation service
6. Wait for completion
7. Move to next configuration
8. Repeat

Each experiment produces **one row** in the CSV.

---

## When to Use What

| Tool              | Use Case                  |
| ----------------- | ------------------------- |
| Marker Generator  | Printing markers          |
| Localization Node | Runtime pose estimation   |
| Evaluation Node   | Marker design & placement |

---
