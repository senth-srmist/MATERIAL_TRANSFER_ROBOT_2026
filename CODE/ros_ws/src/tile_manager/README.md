# tile_manager

ROS2 package for dynamic map tile switching based on robot position.

## Overview

This package enables seamless navigation across large environments by dynamically switching between map tiles as the robot moves through different areas. All map configuration is externalized to YAML files for easy modification without code changes.

## Nodes

### tile_switcher

Manages map tile transitions based on robot position and movement direction.

**Subscriptions:**
| Topic | Type | Description |
|-------|------|-------------|
| `/robot_pose` | `geometry_msgs/PoseStamped` | Robot position |
| `/robot_movement_yaw` | `std_msgs/Float32` | Movement direction |

**Services Called:**
| Service | Type | Description |
|---------|------|-------------|
| `/map_server/load_map` | `nav2_msgs/LoadMap` | Load new map tile |
| `/global_costmap/clear_entire_costmap` | `nav2_msgs/ClearEntireCostmap` | Clear costmap after switch |

## Configuration

All tile and trigger zone settings are defined in `config/tiles_config.yaml`. See that file for:
- Available tiles and their map files
- Trigger zone boundaries and directions
- Initial tile and cooldown settings

## Usage

```bash
ros2 run tile_manager tile_switcher
```

## Architecture

```
┌─────────────────────────────────────────┐
│          robot_localization             │
└──────────────────┬──────────────────────┘
                   │
                   ├──► /robot_pose
                   └──► /robot_movement_yaw
                              │
                              ▼
┌─────────────────────────────────────────┐
│            tile_switcher                │
│  - Loads config from YAML               │
│  - Position + heading check             │
│  - Map switching via Nav2               │
└──────────────────┬──────────────────────┘
                   │
                   └──► /map_server/load_map
```

## File Structure

```
tile_manager/
├── config/
│   └── tiles_config.yaml   # All tile & trigger zone config
├── maps/
│   ├── tile01.pgm
│   ├── tile01.yaml
│   ├── ...
│   └── raw_maps/           # Source files
├── tile_manager/
│   ├── __init__.py
│   └── tile_switcher.py
├── package.xml
└── setup.py
```

## Dependencies

- `rclpy`
- `geometry_msgs`
- `std_msgs`
- `nav2_msgs`
- `robot_localization` (for pose data)
