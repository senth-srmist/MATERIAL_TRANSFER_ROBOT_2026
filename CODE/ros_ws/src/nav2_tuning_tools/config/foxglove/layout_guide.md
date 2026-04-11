# =============================================================================
# PlotJuggler Layout Guide — Nav2 Tuning
# =============================================================================
#
# Install: sudo apt install ros-humble-plotjuggler-ros2
# Launch:  ros2 run plotjuggler plotjuggler
#
# Connect: Click "Streaming" → "ROS2 Topic Subscriber" → Start
# Then drag topics from the left panel into plot areas.
#
# Save layouts: File → Save Layout (one per level)
#
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
# LEVEL 1: Controller + PID Tuning
# ─────────────────────────────────────────────────────────────────────────────
#
# Panel Layout (recommended 3 rows × 2 columns):
#
# ┌──────────────────────────────┬──────────────────────────────────────────┐
# │  LINEAR VELOCITY CHAIN       │  ANGULAR VELOCITY CHAIN                  │
# │                              │                                          │
# │  /pid/debug.data[1]          │  /pid/debug.data[2]                      │
# │    = nav2 raw cmd v          │    = nav2 raw cmd w                      │
# │  /pid/debug.data[3]          │  /pid/debug.data[4]                      │
# │    = rate-limited v          │    = rate-limited w                      │
# │  /pid/debug.data[21]         │  /pid/debug.data[22]                     │
# │    = pid output v            │    = pid output w                        │
# │  /metrics/velocity_tracking  │  /metrics/velocity_tracking              │
# │    .data[4] = odom actual v  │    .data[5] = odom actual w              │
# │                              │                                          │
# │  4 lines overlaid. Gaps      │  Same 4 lines for angular.               │
# │  between them show where     │  During rotation tests, this is          │
# │  the chain breaks.           │  the primary panel.                      │
# ├──────────────────────────────┼──────────────────────────────────────────┤
# │  LEFT WHEEL PID              │  RIGHT WHEEL PID                         │
# │                              │                                          │
# │  /pid/debug.data[9]          │  /pid/debug.data[15]                     │
# │    = left error              │    = right error                         │
# │  /pid/debug.data[10]         │  /pid/debug.data[16]                     │
# │    = left P term             │    = right P term                        │
# │  /pid/debug.data[11]         │  /pid/debug.data[17]                     │
# │    = left I term             │    = right I term                        │
# │  /pid/debug.data[12]         │  /pid/debug.data[18]                     │
# │    = left D term             │    = right D term                        │
# │                              │                                          │
# │  Error should → 0.           │  Compare L vs R for motor asymmetry.     │
# │  I term grows for steady-    │  If one wheel consistently has more      │
# │  state correction.           │  error, that motor may be weaker.        │
# │  D term should be smooth.    │                                          │
# ├──────────────────────────────┼──────────────────────────────────────────┤
# │  PATH TRACKING               │  SYSTEM HEALTH                           │
# │                              │                                          │
# │  /metrics/path_tracking      │  /metrics/system.data[0]                 │
# │    .data[0] = lateral dev    │    = total CPU %                         │
# │    .data[1] = heading error  │  /metrics/system.data[1]                 │
# │    .data[2] = dist to goal   │    = RAM used MB                         │
# │    .data[3] = completion %   │  /metrics/system.data[3]                 │
# │                              │    = RAM %                               │
# │  Lateral dev < 0.05m for     │                                          │
# │  straight line = PASS        │  Watch for trending up = leak            │
# └──────────────────────────────┴──────────────────────────────────────────┘
#
# INTERPRETING THE GRAPHS:
#
# Scenario: RPP commands 0.6 m/s but robot only achieves 0.3 m/s
#   Look at: pid/debug[3] (rate-limited) vs pid/debug[21] (PID output)
#   - If rate-limited = 0.6 but PID output = 0.3 → PID gains too low
#   - If PID output = 0.6 but odom = 0.3 → motor can't deliver, check FF gain
#   - If rate-limited = 0.3 → rate limiter is clamping, check max_linear_accel
#
# Scenario: Robot oscillates during straight line
#   Look at: pid/debug[10] (P term) — is it swinging +/-?
#   - Large P oscillation → Kp too high, reduce it
#   - I term growing large → may be windup, check max_integral
#   - D term spiky → encoder noise, increase Kd carefully or filter
#
# Scenario: Robot overshoots rotation target
#   Look at: pid/debug[4] (rate-limited w) — does it ramp down in time?
#   - If rate-limited w stays high too long → max_angular_decel too low
#   - If PID output w overshoots → angular PID gains need tuning
#   - If heading_error crosses zero and oscillates → Kp too high for angular

# ─────────────────────────────────────────────────────────────────────────────
# LEVEL 2: Costmap Tuning
# ─────────────────────────────────────────────────────────────────────────────
#
# Keep all L1 panels, ADD:
#
# ┌──────────────────────────────┐
# │  OBSTACLE DISTANCE           │
# │                              │
# │  (Need to add min_obstacle   │
# │   distance to metrics_node   │
# │   when obstacle_layer is     │
# │   enabled — subscribe to     │
# │   /local_costmap/costmap)    │
# │                              │
# │  For now: use RViz costmap   │
# │  visualization alongside     │
# │  PlotJuggler.                │
# └──────────────────────────────┘
#
# Also open RViz with:
#   - Map display (/map)
#   - Local costmap (/local_costmap/costmap)
#   - Global costmap (/global_costmap/costmap)
#   - Robot model
#   - Path (/plan)
#   - Local plan (/local_plan)
#
# The combination of PlotJuggler (time-series) + RViz (spatial) gives
# you full visibility.

# ─────────────────────────────────────────────────────────────────────────────
# LEVEL 2.5: Recovery Behavior
# ─────────────────────────────────────────────────────────────────────────────
#
# Primary panel: velocity commands over time
#   /pid/debug.data[1] (nav2 cmd v) — watch for it dropping to 0 (stuck)
#   /pid/debug.data[2] (nav2 cmd w) — watch for spin recovery
#
# Also monitor /rosout in a terminal:
#   ros2 topic echo /rosout --filter "bt_navigator OR recovery"
#
# Key timing to measure:
#   Time from "robot stops moving" to "recovery behavior starts"
#   = progress_checker.movement_time_allowance (currently 45s, tune to 15-20s)

# ─────────────────────────────────────────────────────────────────────────────
# LEVEL 3-4: Full System
# ─────────────────────────────────────────────────────────────────────────────
#
# Keep all previous panels, ADD:
#
# ┌──────────────────────────────┐
# │  MEMORY PER NODE             │
# │                              │
# │  /metrics/node_memory        │
# │    .data[0] = controller RSS │
# │    .data[3] = planner RSS    │
# │    .data[6] = pid RSS        │
# │    .data[9] = bt_nav RSS     │
# │                              │
# │  (every 3 values = one node: │
# │   rss, cpu%, trend)          │
# │                              │
# │  For L4: watch during tile   │
# │  transitions for spikes.     │
# └──────────────────────────────┘
#
# Node memory index mapping (matches monitor_nodes order):
#   controller_server: data[0], data[1], data[2]  (rss, cpu, trend)
#   planner_server:    data[3], data[4], data[5]
#   pid_controller:    data[6], data[7], data[8]
#   bt_navigator:      data[9], data[10], data[11]
#   local_costmap:     data[12], data[13], data[14]  (if monitored)
#   global_costmap:    data[15], data[16], data[17]  (if monitored)
#   tuning_metrics:    data[18], data[19], data[20]  (if monitored)

# ─────────────────────────────────────────────────────────────────────────────
# QUICK START CHECKLIST
# ─────────────────────────────────────────────────────────────────────────────
#
# 1. Launch your Nav2 + PID stack (your existing launch file)
# 2. Launch tuning tools:
#      ros2 launch nav2_tuning_tools tuning_session.launch.py
# 3. Open PlotJuggler:
#      ros2 run plotjuggler plotjuggler
#    → Streaming → ROS2 Topic Subscriber → Start
#    → Drag topics into panels per the layout above
#    → Save layout: File → Save Layout → "L1_controller.xml"
# 4. Open param_tuner in another terminal:
#      ros2 run nav2_tuning_tools param_tuner
# 5. For L1, disable obstacle layers:
#      tuner> set local_costmap human_layer.enabled false
# 6. Run a test:
#      ros2 run nav2_tuning_tools test_runner -s L1_straight
#    Or with custom goals:
#      ros2 run nav2_tuning_tools test_runner -s L1_straight -g "2.0,0,0"
# 7. Watch PlotJuggler, tune live in param_tuner
# 8. When happy, export:
#      tuner> export L1_winner.yaml
# 9. Move to next scenario/level
