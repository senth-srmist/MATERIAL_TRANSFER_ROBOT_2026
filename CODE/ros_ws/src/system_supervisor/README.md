# System Supervisor

Health monitoring and lifecycle management for the Material Transfer Robot. Manages always-on node monitoring, on-demand navigation stack startup/shutdown, categorized failure recovery, and system resource tracking.

## State Machine

```
                    ┌──────────────────────────────────────────────┐
                    │                                              │
  ┌─────────┐   all ready   ┌──────┐  nav_needed  ┌────────────┐ │ all up  ┌────────┐
  │ BOOTING ├──────────────►│ IDLE ├─────────────►│ ACTIVATING ├─┼────────►│ ACTIVE │
  └─────────┘               └──┬───┘              └─────┬───────┘ │        └───┬────┘
                               │  ▲                     │         │            │
                               │  │    abort/fail       │         │  !nav_needed
                               │  └─────────────────────┘         │            │
                               │  ▲                               │  ┌─────────▼──────┐
                               │  └───────────────────────────────┼──┤ DEACTIVATING   │
                               │                                  │  └────────────────┘
                               │          cat3 cascade            │
                               └──────────────────────────────────┘
```

**BOOTING** — Waits for all always-on nodes to come up. Uses progress-based detection. A 60-second boot delay prevents the supervisor from ticking before other nodes have had time to spawn, avoiding CPU/memory pressure on startup. Publishes `/system/ready` when all always-on nodes are RUNNING, then transitions to IDLE.

**IDLE** — All always-on nodes monitored. Waiting for `/system/nav_needed` from the job manager. When a delivery job arrives, transitions to ACTIVATING.

**ACTIVATING** — Starts on-demand nodes sequentially with health gates: ZED → odom_bridge → Nav2 → pid_controller → mission. Each node must pass its health check before the next starts. If any node fails, aborts activation, cleans up, and returns to IDLE.

**ACTIVE** — Full monitoring of all nodes. Categorized failure recovery runs here. Publishes `/system/nav_ready` on entry. Transitions to DEACTIVATING when nav is no longer needed.

**DEACTIVATING** — Shuts down on-demand nodes in reverse order: mission → pid_controller → Nav2 → odom_bridge → ZED. Publishes `/system/nav_shutdown` before stopping nodes. Returns to IDLE.

## Boot Detection

The supervisor uses progress-based detection rather than fixed boot timeouts. Each node tracks a `last_progress` timestamp that resets when milestones are hit:

- **PID found** — Process is at least running (resets stall timer)
- **Heartbeat received** — Node is fully operational (boot complete)

Two thresholds per node:

- `stall_timeout` — No progress for this many seconds means the node is stuck
- `max_boot_timeout` — Absolute safety net regardless of progress

For `pid_only` nodes, PID appearing is sufficient — no heartbeat topic required.

## Node Categories

Each monitored node has a failure category that determines recovery behavior:

**Category 1 (Auto Restart)** — Automatically restarts the node up to `max_restarts` times with a cooldown between attempts. Used for always-on nodes like motors, teleop, and encoders.

**Category 2 (Abort + Restart)** — Sends an e-stop command, then restarts the node. Used for navigation-critical nodes like Nav2, PID controller, and mission controller. If max restarts exhausted, marks system as DEGRADED and disables autonomous operation.

**Category 3 (Cascade Shutdown)** — Sends e-stop, kills all on-demand nodes, publishes nav_shutdown, and returns to IDLE. Used for localization-critical nodes like ZED camera and odom bridge. Losing these means localization is gone — no point keeping Nav2 running.

## Always-On vs On-Demand Nodes

**Always-on** (monitored in all states, launched by robot_bringup):

| Node | Process | Detection | Category |
|------|---------|-----------|----------|
| motors | motor_driver | PID only | Cat 1 |
| robot_description | robot_state_publisher | PID only | Cat 1 |
| map_server | map_server | PID only | Cat 1 |
| teleop | joy_node | PID only | Cat 1 |
| encoders | encoder_driver | PID only | Cat 1 |

**On-demand** (started/stopped by supervisor based on navigation need):

| Node | Process | Detection | Category | Startup Order |
|------|---------|-----------|----------|--------------|
| zed | zed_wrapper | Topic: `/zed/zed_node/status/heartbeat` | Cat 3 | 1 |
| odom_bridge | odom_base_publisher | Topic: `/odom/base_link` | Cat 3 | 2 |
| nav2 | controller_server | PID only | Cat 2 | 3 |
| pid_controller | pid_controller | PID only | Cat 2 | 4 |
| mission | mission_service | PID only | Cat 2 | 5 |

## Topics

### Published

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/robot_health` | `system_supervisor/RobotHealth` | RELIABLE, TRANSIENT_LOCAL, 1Hz | Full system status: per-node health, CPU, memory, supervisor state |
| `/system/ready` | `std_msgs/Empty` | RELIABLE, TRANSIENT_LOCAL, once | All always-on nodes are up — robot ready to accept jobs |
| `/system/nav_ready` | `std_msgs/Empty` | RELIABLE, TRANSIENT_LOCAL, on event | Nav stack activation complete |
| `/system/nav_shutdown` | `std_msgs/Empty` | RELIABLE, TRANSIENT_LOCAL, on event | Nav stack shutting down |
| `/cmd_vel_estop` | `geometry_msgs/Twist` | RELIABLE, VOLATILE | Emergency stop — zero velocity, routed through twist_mux at highest priority |

### Subscribed

| Topic | Type | QoS | Source |
|-------|------|-----|--------|
| `/system/nav_needed` | `std_msgs/Bool` | RELIABLE, TRANSIENT_LOCAL | job_manager — true when navigation stack is needed |
| Heartbeat topics | Configured per node | BEST_EFFORT, VOLATILE | Various — dynamic subscriptions from config |

## Messages

### RobotHealth.msg

```
std_msgs/Header header
float32 cpu_percent
uint8 cpu_count
float32 memory_used_mb
float32 memory_total_mb
system_supervisor/NodeHealth[] nodes
uint8 system_state
uint8 supervisor_state
string system_message
bool autonomous_enabled

uint8 NOMINAL=0
uint8 DEGRADED=1
uint8 LOCALIZATION_LOST=2
uint8 EMERGENCY_STOP=3

uint8 SV_BOOTING=0
uint8 SV_IDLE=1
uint8 SV_ACTIVATING=2
uint8 SV_ACTIVE=3
uint8 SV_DEACTIVATING=4
```

### NodeHealth.msg

```
string name
uint8 status
float32 cpu_percent
float32 memory_mb
float32 last_heartbeat_age
uint32 restart_count
uint32 error_count
string last_error
string message

uint8 WAITING=0
uint8 RUNNING=1
uint8 STALE=2
uint8 DEAD=3
uint8 RESTARTING=4
uint8 SHUTDOWN=5
uint8 OFF=6
```

## Configuration

All node definitions are in `config/supervisor_config.yaml`. The supervisor loads this at startup via the `config_file` parameter.

### Config Fields (per node)

| Field | Type | Description |
|-------|------|-------------|
| `category` | int | Failure category: 1, 2, or 3 |
| `process_name` | string | Process name for `pgrep` PID detection |
| `heartbeat_topic` | string | Topic to subscribe for liveness (omit for pid_only) |
| `heartbeat_msg_type` | string | Python message type string, e.g. `nav_msgs.msg.Odometry` |
| `pid_only` | bool | If true, PID detection is sufficient — no heartbeat needed |
| `stall_timeout` | float | Seconds without progress before declaring node stuck |
| `max_boot_timeout` | float | Absolute boot time limit |
| `heartbeat_timeout` | float | Seconds without heartbeat before STALE (running nodes) |
| `max_restarts` | int | Maximum restart attempts before giving up |
| `restart_cooldown` | float | Minimum seconds between restart attempts |
| `start_cmd` | list | Command to start the node (used by supervisor for on-demand nodes) |

On-demand nodes additionally have:

| Field | Type | Description |
|-------|------|-------------|
| `startup_order` | int | Sequence number for activation (lower starts first) |
| `wait_type` | string | `"topic"` or `"pid"` — what to wait for during activation |

## Resource Monitoring

The supervisor tracks system-wide and per-node resource usage at 0.5 Hz:

- **System CPU** — Read from `/proc/stat`, averaged across all cores
- **System Memory** — Read from `/proc/meminfo` (total and available)
- **Per-node CPU** — Read from `/proc/<pid>/stat` (user + system ticks)
- **Per-node Memory** — RSS from `/proc/<pid>/status`

All values are published in the `/robot_health` message.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_file` | (install path) | Path to supervisor_config.yaml |
| `boot_delay` | 60.0 | Seconds to wait after init before starting health checks |

## Package Structure

```
system_supervisor/
├── config/
│   └── supervisor_config.yaml
├── msg/
│   ├── NodeHealth.msg
│   └── RobotHealth.msg
└── scripts/
    └── system_supervisor.py
```

## Monitoring

```bash
# Watch system health
ros2 topic echo /robot_health

# Wait for robot ready
ros2 topic echo /system/ready --once

# Check supervisor state only
ros2 topic echo /robot_health --field supervisor_state
```
