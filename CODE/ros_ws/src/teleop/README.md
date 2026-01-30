# Teleoperation with Game Controller (ROS 2)

This package enables **manual teleoperation of the robot using a game controller** via ROS 2.

It uses:

* `joy` → to read joystick input
* `teleop_twist_joy` → to convert joystick input into `/cmd_vel`

---
## Step 1: Start the joystick driver

This node reads raw joystick data and publishes it on `/joy`.

```bash
ros2 run joy joy_node
```

### Verify joystick input

In a new terminal:

```bash
ros2 topic echo /joy
```

You should see output like:

```yaml
axes:
- 0.0
- -0.4
buttons:
- 0
- 1
```

If axes/buttons change when you move the controller → **joystick is working**.

---

## Step 2: Start teleop (joystick → velocity)

This node converts `/joy` messages into `/cmd_vel` using a YAML config.

```bash
ros2 run teleop_twist_joy teleop_node \
  --ros-args \
  --params-file src/teleop/config/teleop.yaml
```
---

## Step 3: Verify velocity output

In another terminal:

```bash
ros2 topic echo /cmd_vel
```

Expected behavior:

* Hold the **deadman button** → `/cmd_vel` updates
* Release the deadman → `/cmd_vel` goes to zero
* Left stick forward/back → `linear.x`
* Left stick left/right → `angular.z`

---

## Control Mapping (Xbox-style)

| Control               | Action                          |
| --------------------- | ------------------------------- |
| Left stick up/down    | Forward / Backward (`linear.x`) |
| Left stick left/right | Turn Left / Right (`angular.z`) |
| Right bumper (RB)     | Deadman (must be held to move)  |

---

## Useful Debug Commands

Check joystick data:

```bash
ros2 topic echo /joy
```

Check velocity commands:

```bash
ros2 topic echo /cmd_vel
```

Check running nodes:

```bash
ros2 node list
```
---
