# nav2_depth_obstacle_layer

A Nav2 costmap layer plugin that projects depth image points as obstacles with optional human detection masking. Also provides BehaviorTree nodes for human-aware navigation.

## Overview

### Costmap Layer

Projects depth images onto the costmap as `LETHAL_OBSTACLE`. Points are filtered by height above ground to exclude floor and overhead obstacles.

**Human Detection Masking (Optional):** When enabled with ZED camera messages, regions containing detected humans are excluded from obstacle marking. This allows the BT nodes to handle human encounters (stop and wait) without triggering path replanning.

### BT Nodes

- **HumanBlockingPath**: Condition node that returns SUCCESS when a human is blocking the planned path within `human_stop_distance` of the robot.
- **WaitUntilHumanClears**: Action node that stops the robot and waits until the human moves away, optionally calling a speech service.

## Costmap Layer Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `depth_topic` | string | `/depth/image_rect` | Depth image topic |
| `camera_info_topic` | string | `/depth/camera_info` | Camera info topic |
| `ground_frame` | string | `base_link` | Frame for height calculation |
| `stride` | int | `2` | Pixel stride for processing |
| `obstacle_range` | double | `3.0` | Maximum range in meters |
| `min_obstacle_height` | double | `0.05` | Minimum height above ground (m) |
| `max_obstacle_height` | double | `1.5` | Maximum height above ground (m) |
| `human_mask_enabled` | bool | `false` | Enable human detection masking |
| `human_topic` | string | `""` | Human detection topic (ZED objects) |
| `human_mask_padding` | int | `20` | Padding around human bbox (pixels) |
| `human_persistence` | double | `0.5` | Time to keep human detection (s) |

## BT Node Ports

### HumanBlockingPath

| Port | Type | Default | Description |
|------|------|---------|-------------|
| `path` | input | - | Current navigation path |
| `human_stop_distance` | input | `1.5` | Only stop if human within this distance (m) |
| `path_width` | input | `0.3` | Human must be within this of path (m) |
| `human_topic` | input | `/zed/zed_node/obj_det/objects` | Human detection topic |
| `global_frame` | input | `map` | Global frame |
| `robot_frame` | input | `base_link` | Robot frame |

### WaitUntilHumanClears

| Port | Type | Default | Description |
|------|------|---------|-------------|
| `path` | input | - | Current navigation path |
| `human_stop_distance` | input | `1.5` | Distance threshold (m) |
| `path_width` | input | `0.3` | Path corridor width (m) |
| `human_topic` | input | `/zed/zed_node/obj_det/objects` | Human detection topic |
| `cmd_vel_topic` | input | `/cmd_vel` | Velocity command topic |
| `speak_service` | input | `/speak` | Speech service (empty to disable) |
| `speak_interval` | input | `5.0` | Seconds between announcements |

## Usage

### nav2_params.yaml

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["depth_obstacle_layer", "inflation_layer"]

      depth_obstacle_layer:
        plugin: "nav2_depth_obstacle_layer::DepthObstacleLayer"
        enabled: true
        depth_topic: "/zed/zed_node/depth/depth_registered"
        camera_info_topic: "/zed/zed_node/depth/camera_info"
        ground_frame: "ground"
        stride: 2
        obstacle_range: 3.0
        min_obstacle_height: 0.05
        max_obstacle_height: 1.5
        human_mask_enabled: true
        human_topic: "/zed/zed_node/obj_det/objects"
        human_mask_padding: 20
        human_persistence: 0.5

bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_depth_obstacle_layer_bt_nodes
      # ... other plugins

controller_server:
  ros__parameters:
    progress_checker:
      movement_time_allowance: 86400.0  # 24hr - allows indefinite human wait
```

### BT XML

```xml
<ReactiveSequence>
  <Inverter>
    <Sequence>
      <HumanBlockingPath path="{path}" human_stop_distance="1.5"/>
      <WaitUntilHumanClears path="{path}" speak_service="/speak"/>
    </Sequence>
  </Inverter>
  <FollowPath path="{path}"/>
</ReactiveSequence>
```

## Building

```bash
colcon build --packages-select nav2_depth_obstacle_layer
```

The build automatically detects if `zed_msgs` is available. Without it:
- Costmap layer works but human masking is disabled
- BT nodes will not function (return FAILURE)

## Libraries

- `nav2_depth_obstacle_layer` - Costmap layer plugin
- `nav2_depth_obstacle_layer_bt_nodes` - BT nodes plugin

## License

Apache-2.0
