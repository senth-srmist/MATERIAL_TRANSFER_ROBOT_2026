# Job Manager

Delivery job orchestration for the Material Transfer Robot. Accepts delivery requests, maintains a priority queue, coordinates with the system supervisor for navigation stack lifecycle, and executes jobs sequentially through the mission controller.

## Job Lifecycle

```
QUEUED → WAITING_FOR_NAV → PICKUP_NAV → PICKUP_ARRIVED → DROPOFF_NAV → DROPOFF_ARRIVED → RETURNING → COMPLETE
                                              │                              │
                                         timeout/abort                  timeout/abort
                                              │                              │
                                          CANCELLED                      COMPLETE
                                                                     (item left at drop)
```

**QUEUED** — Job accepted, waiting in the priority queue.

**WAITING_FOR_NAV** — First job triggers `nav_needed=true`. Job waits up to 120s for the supervisor to bring up the navigation stack and signal `/system/nav_ready`.

**PICKUP_NAV** — Robot navigating to the pickup room via `/navigate_to_room` service.

**PICKUP_ARRIVED** — Robot at pickup location. Publishes on `/job/awaiting_confirmation` and waits for `/job/confirm` service call. If no confirmation within 5 minutes or `proceed: false`, the job is cancelled.

**DROPOFF_NAV** — Robot navigating to the dropoff room.

**DROPOFF_ARRIVED** — Robot at dropoff location. Same confirmation flow as pickup, but timeout or abort results in COMPLETE (item left at dropoff, not cancelled).

**RETURNING** — All jobs done, robot returning to home position. `nav_needed` stays true until return completes.

**COMPLETE** — Delivery finished successfully.

**FAILED** — Navigation error or nav stack unavailable.

**CANCELLED** — Cancelled by user via `/cancel_job`, pickup timeout, or pickup abort.

## Priority Queue

Jobs are stored in a Python `deque`. Normal priority (0) jobs are appended to the back. High priority (1+) jobs are inserted at the front via `appendleft`, jumping ahead of all normal-priority jobs but behind any currently executing job.

The active job cannot be preempted — it runs to completion (or cancellation). High priority only affects queue ordering.

## Topics

### Published

| Topic | Type | QoS | Rate | Description |
|-------|------|-----|------|-------------|
| `/job_status` | `job_manager/JobStatus` | RELIABLE, TRANSIENT_LOCAL | On change | Current job state, room names, progress message |
| `/job_queue` | `std_msgs/String` | RELIABLE, TRANSIENT_LOCAL | 1 Hz | JSON snapshot of all queued jobs (not including active) |
| `/system/active_jobs` | `std_msgs/Int32` | RELIABLE, TRANSIENT_LOCAL | 1 Hz | Count of queued + active jobs |
| `/system/nav_needed` | `std_msgs/Bool` | RELIABLE, TRANSIENT_LOCAL | 1 Hz | True when nav stack is needed (jobs pending or returning home) |
| `/job/awaiting_confirmation` | `std_msgs/String` | RELIABLE, TRANSIENT_LOCAL | On event | Published at pickup/dropoff with stage, location, command, and timeout |

### Subscribed

| Topic | Type | QoS | Source |
|-------|------|-----|--------|
| `/system/nav_ready` | `std_msgs/Empty` | RELIABLE, TRANSIENT_LOCAL | system_supervisor — nav stack is up |
| `/system/nav_shutdown` | `std_msgs/Empty` | RELIABLE, TRANSIENT_LOCAL | system_supervisor — nav stack going down |

## Services

### /request_delivery (DeliveryJob.srv)

Submit a new delivery job.

```
# Request
string pickup_room
string dropoff_room
uint8 priority          # 0 = normal, 1+ = high (jumps queue)

# Response
string job_id           # e.g. "job_001"
bool accepted
string message
```

### /cancel_job (CancelJob.srv)

Cancel a job by ID or cancel the current active job.

```
# Request
string job_id           # Empty string = cancel current active job

# Response
bool success
string message
```

Queued jobs are removed immediately. Active jobs set a cancel flag checked between navigation steps — the robot stops at the next safe point.

### /job/confirm (ConfirmJob.srv)

Confirm pickup or dropoff to proceed.

```
# Request
bool proceed            # true = continue, false = abort

# Response
bool success
string message
```

### /navigate_to_room (client)

Called by the job manager to drive the robot. Provided by the mission controller.

```
# Request
string room_name

# Response
bool success
string message
```

## Messages

### JobStatus.msg

```
std_msgs/Header header
string job_id
string pickup_room
string dropoff_room
uint8 priority
uint8 state
string state_name
string message

uint8 QUEUED=0
uint8 WAITING_FOR_NAV=1
uint8 PICKUP_NAV=2
uint8 PICKUP_ARRIVED=3
uint8 DROPOFF_NAV=4
uint8 DROPOFF_ARRIVED=5
uint8 RETURNING=6
uint8 COMPLETE=7
uint8 FAILED=8
uint8 CANCELLED=9
```

## Queue Topic Format

The `/job_queue` topic publishes a JSON string at 1 Hz containing all queued (not active) jobs:

```json
{
  "jobs": [
    {
      "job_id": "job_002",
      "pickup_room": "H204",
      "dropoff_room": "H210",
      "priority": 0,
      "state": 0,
      "message": ""
    },
    {
      "job_id": "job_003",
      "pickup_room": "H201",
      "dropoff_room": "H207",
      "priority": 1,
      "state": 0,
      "message": ""
    }
  ]
}
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `home_room` | `"home"` | Room name for return-home navigation |
| `pickup_timeout` | `300.0` | Seconds to wait for pickup confirmation |
| `dropoff_timeout` | `300.0` | Seconds to wait for dropoff confirmation |
| `debug_mode` | `false` | Skip nav_ready wait (for testing without supervisor) |

## Interaction with Supervisor

The job manager and system supervisor coordinate through a simple signal protocol:

1. Job arrives → job manager sets `nav_needed = true`
2. Supervisor sees `nav_needed`, starts on-demand nodes (ZED, Nav2, etc.)
3. Supervisor publishes `/system/nav_ready` when all on-demand nodes are up
4. Job manager proceeds with navigation
5. All jobs done, robot returns home
6. Job manager sets `nav_needed = false`
7. Supervisor sees `nav_needed = false`, shuts down on-demand nodes

If the supervisor needs to shut down the nav stack (Cat3 cascade), it publishes `/system/nav_shutdown`. The job manager clears its `nav_ready` flag and any running navigation will fail, causing the job to fail.

## Threading Model

The job manager uses a `MultiThreadedExecutor` with 4 threads to handle concurrent service calls while executing jobs:

- **Executor thread** — Background daemon thread running `_job_executor_loop`, processes jobs sequentially from the queue
- **Service callbacks** — Handled by the reentrant callback group, can accept new jobs and confirmations while a job is executing
- **Timer callback** — Publishes system status at 1 Hz
- **Thread lock** — Protects shared state (queue, active_job, nav_needed, cancel flag)

## Package Structure

```
job_manager/
├── msg/
│   └── JobStatus.msg
├── scripts/
│   └── job_manager_node.py
└── srv/
    ├── DeliveryJob.srv
    ├── CancelJob.srv
    └── ConfirmJob.srv
```

## Quick Commands

```bash
# Submit a delivery
ros2 service call /request_delivery job_manager/srv/DeliveryJob \
  "{pickup_room: 'H201', dropoff_room: 'H207', priority: 0}"

# High priority delivery
ros2 service call /request_delivery job_manager/srv/DeliveryJob \
  "{pickup_room: 'H203', dropoff_room: 'H212', priority: 1}"

# Confirm pickup/dropoff
ros2 service call /job/confirm job_manager/srv/ConfirmJob "{proceed: true}"

# Abort pickup/dropoff
ros2 service call /job/confirm job_manager/srv/ConfirmJob "{proceed: false}"

# Cancel current job
ros2 service call /cancel_job job_manager/srv/CancelJob "{job_id: ''}"

# Cancel specific job
ros2 service call /cancel_job job_manager/srv/CancelJob "{job_id: 'job_002'}"

# Watch job status
ros2 topic echo /job_status

# Watch queue
ros2 topic echo /job_queue
```
