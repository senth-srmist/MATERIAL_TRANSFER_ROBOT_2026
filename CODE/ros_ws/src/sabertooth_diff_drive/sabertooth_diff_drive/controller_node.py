#!/usr/bin/env python3
"""
Motor Controller Node for Sabertooth motor driver.

Subscribes to /cmd_vel_out and sends serial commands to Sabertooth.
Includes rate limiting, watchdog timeout, serial health monitoring,
and graceful shutdown.

Hardware: Sabertooth 2x25 (or similar) in simplified serial mode.
Odometry: Provided externally by ZED camera (not this node).
This node publishes expected wheel velocities as diagnostics so
external tools can compare against ZED odometry for drift detection.

Changes from original:
- Proper try/finally for serial cleanup (P0)
- Serial health tracking with reconnect (P0)
- Ramped watchdog deceleration instead of hard stop (P0)
- Fixed rate limiter for direction reversals (P1)
- Safe dt handling (clamp, no zero/negative) (P1)
- send_stop failure tracking (P1)
- Motor scaling dead zone fix (P2)
- Wheel clamping warning (P2)
- All constants as ROS parameters (P3)
- Diagnostic velocity publisher for odom cross-check (P4)
- Separate angular decel limit (P4)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import serial
import math
import time


class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        # ==================================================================
        # ROS Parameters (all former module-level constants)
        # ==================================================================
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baud", 9600)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("base_length", 0.4)
        self.declare_parameter("min_linear_vel", -0.2)
        self.declare_parameter("max_linear_vel", 0.78)
        self.declare_parameter("min_angular_vel", -2.0)
        self.declare_parameter("max_angular_vel", 2.0)
        self.declare_parameter("max_linear_accel", 0.3)
        self.declare_parameter("max_linear_decel", 0.6)
        self.declare_parameter("max_angular_accel", 1.5)
        self.declare_parameter("max_angular_decel", 1.5)
        self.declare_parameter("control_dt", 0.1)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("max_wheel_rad_s", 10.0)
        self.declare_parameter("serial_reconnect_interval", 2.0)
        self.declare_parameter("watchdog_decel_rate", 1.5)

        self.serial_port = self._p("serial_port")
        self.serial_baud = self._p("serial_baud")
        self.wheel_radius = self._p("wheel_radius")
        self.base_length = self._p("base_length")
        self.min_linear_vel = self._p("min_linear_vel")
        self.max_linear_vel = self._p("max_linear_vel")
        self.min_angular_vel = self._p("min_angular_vel")
        self.max_angular_vel = self._p("max_angular_vel")
        self.max_linear_accel = self._p("max_linear_accel")
        self.max_linear_decel = self._p("max_linear_decel")
        self.max_angular_accel = self._p("max_angular_accel")
        self.max_angular_decel = self._p("max_angular_decel")
        self.control_dt = self._p("control_dt")
        self.cmd_timeout = self._p("cmd_timeout")
        self.max_wheel_rad_s = self._p("max_wheel_rad_s")
        self.serial_reconnect_interval = self._p("serial_reconnect_interval")
        self.watchdog_decel_rate = self._p("watchdog_decel_rate")

        # ==================================================================
        # Subscription
        # ==================================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Twist, "/cmd_vel_out", self.cmd_vel_callback, qos
        )

        # ==================================================================
        # Diagnostic publisher
        # Publishes [v_current, w_current, omega_left, omega_right]
        # External nodes can compare against ZED odom for drift detection
        # ==================================================================
        self.diag_pub = self.create_publisher(
            Float32MultiArray, "/motor_controller/diagnostics", 10
        )

        # ==================================================================
        # Control timer
        # ==================================================================
        self.control_timer = self.create_timer(
            self.control_dt, self.control_loop
        )

        # Desired command (latest received)
        self.v_target = 0.0
        self.w_target = 0.0

        # Actual applied command (rate limited)
        self.v_current = 0.0
        self.w_current = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_control_time = self.get_clock().now()

        self.is_stopped = True
        self.watchdog_active = False  # True when ramping down due to timeout

        # ==================================================================
        # Serial port with health tracking
        # ==================================================================
        self.motor: serial.Serial = None
        self.serial_healthy = False
        self.last_reconnect_attempt = 0.0  # monotonic time

        self._connect_serial()

    def _p(self, name: str):
        """Shorthand for getting a parameter value."""
        return self.get_parameter(name).value

    # ======================================================================
    # Serial port management
    # ======================================================================

    def _connect_serial(self) -> bool:
        """Attempt to open the serial port. Returns True on success."""
        try:
            if self.motor is not None:
                try:
                    self.motor.close()
                except Exception:
                    pass

            self.motor = serial.Serial(
                self.serial_port, self.serial_baud, timeout=1
            )
            self.serial_healthy = True
            self.last_reconnect_attempt = time.monotonic()
            self.get_logger().info(
                f"Connected to Sabertooth on {self.serial_port}"
            )
            return True

        except Exception as e:
            self.motor = None
            self.serial_healthy = False
            self.last_reconnect_attempt = time.monotonic()
            self.get_logger().error(f"Serial port open failed: {e}")
            return False

    def _try_reconnect(self) -> bool:
        """
        Attempt serial reconnect with rate limiting.
        Returns True if connected (or already healthy).
        """
        if self.serial_healthy:
            return True

        now = time.monotonic()
        if now - self.last_reconnect_attempt < self.serial_reconnect_interval:
            return False

        self.get_logger().info("Attempting serial reconnect...")
        return self._connect_serial()

    def _mark_serial_failed(self) -> None:
        """Mark serial as unhealthy after a write failure."""
        if self.serial_healthy:
            self.serial_healthy = False
            self.get_logger().error(
                "Serial port failed — stopping commands until reconnect"
            )
        # Zero out state so we don't resume at old velocity after reconnect
        self.v_current = 0.0
        self.w_current = 0.0
        self.v_target = 0.0
        self.w_target = 0.0
        self.is_stopped = True

    def _close_serial(self) -> None:
        """Close serial port safely."""
        if self.motor is not None:
            try:
                # Send stop before closing
                if self.serial_healthy:
                    self.motor.write(bytes([64, 192]))
            except Exception:
                pass
            try:
                self.motor.close()
            except Exception:
                pass
            self.motor = None
            self.serial_healthy = False

    # ======================================================================
    # CMD_VEL callback
    # ======================================================================

    def cmd_vel_callback(self, msg: Twist):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.get_logger().warn(
                "Received invalid cmd_vel (NaN or Inf). Ignoring."
            )
            return

        self.last_cmd_time = self.get_clock().now()

        self.v_target = max(
            self.min_linear_vel, min(self.max_linear_vel, msg.linear.x)
        )
        self.w_target = max(
            self.min_angular_vel, min(self.max_angular_vel, msg.angular.z)
        )

        # If watchdog was ramping down, new command cancels it
        self.watchdog_active = False

    # ======================================================================
    # Rate limiter
    # ======================================================================

    @staticmethod
    def _limit_rate(target: float, current: float, accel: float,
                    decel: float, dt: float) -> float:
        """
        Rate limit with proper direction-change handling.

        Decel is used when:
          - Moving toward zero (reducing magnitude)
          - Crossing zero (must decel to zero first, then accel)
        Accel is used when:
          - Increasing magnitude in the same direction
        """
        delta = target - current

        # Determine if we're accelerating or decelerating
        same_sign = (current >= 0 and target >= 0) or (
            current <= 0 and target <= 0
        )

        if same_sign and abs(target) >= abs(current):
            # Accelerating (increasing magnitude, same direction)
            limit = accel * dt
        else:
            # Decelerating (reducing magnitude, or crossing zero)
            limit = decel * dt

        return current + max(-limit, min(limit, delta))

    # ======================================================================
    # Control loop
    # ======================================================================

    def control_loop(self):
        now = self.get_clock().now()

        # Safe dt — clamp to [1ms, 500ms] to prevent zero/negative/huge values
        dt_raw = (now - self.last_control_time).nanoseconds * 1e-9
        dt = max(0.001, min(0.5, dt_raw))
        self.last_control_time = now

        # Serial health check — attempt reconnect if needed
        if not self._try_reconnect():
            return  # Can't do anything without serial

        # ==================================================================
        # Watchdog — ramp down instead of hard stop
        # ==================================================================
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9

        if elapsed > self.cmd_timeout:
            if not self.is_stopped:
                if not self.watchdog_active:
                    self.get_logger().warn("CMD_VEL timeout — ramping down")
                    self.watchdog_active = True

                # Ramp to zero using watchdog decel rate
                self.v_current = self._limit_rate(
                    0.0, self.v_current,
                    self.max_linear_accel, self.watchdog_decel_rate, dt
                )
                self.w_current = self._limit_rate(
                    0.0, self.w_current,
                    self.max_angular_accel, self.watchdog_decel_rate, dt
                )

                # Check if fully stopped
                if abs(self.v_current) < 1e-3 and abs(self.w_current) < 1e-3:
                    self._send_stop()
                    self.watchdog_active = False
                    return

                # Still ramping — send intermediate command
                self._send_velocity()
            return

        # ==================================================================
        # Normal rate limiting
        # ==================================================================
        self.v_current = self._limit_rate(
            self.v_target, self.v_current,
            self.max_linear_accel, self.max_linear_decel, dt
        )
        self.w_current = self._limit_rate(
            self.w_target, self.w_current,
            self.max_angular_accel, self.max_angular_decel, dt
        )

        # ==================================================================
        # Stop condition
        # ==================================================================
        if abs(self.v_current) < 1e-3 and abs(self.w_current) < 1e-3:
            if not self.is_stopped:
                self._send_stop()
            return

        self._send_velocity()

    # ======================================================================
    # Velocity to motor commands
    # ======================================================================

    def _send_velocity(self) -> None:
        """Convert current velocity to motor commands and send."""
        self.is_stopped = False

        # Differential drive kinematics
        v_r = self.v_current + (self.base_length / 2.0) * self.w_current
        v_l = self.v_current - (self.base_length / 2.0) * self.w_current

        omega_r = v_r / self.wheel_radius
        omega_l = v_l / self.wheel_radius

        # Normalize to [-1, 1] with clamping warning
        raw_right = omega_r / self.max_wheel_rad_s
        raw_left = omega_l / self.max_wheel_rad_s

        if abs(raw_right) > 1.0 or abs(raw_left) > 1.0:
            self.get_logger().debug(
                f"Wheel velocity clamped: L={raw_left:.2f} R={raw_right:.2f}"
                " — actual turn radius may differ from planned"
            )

        right = max(-1.0, min(1.0, raw_right))
        left = max(-1.0, min(1.0, raw_left))

        left_cmd = self._scale_motor_command(left, left_motor=True)
        right_cmd = self._scale_motor_command(right, left_motor=False)

        self._send_serial(left_cmd, right_cmd)

        # Publish diagnostics for external odom comparison
        self._publish_diagnostics(omega_l, omega_r)

        self.get_logger().debug(
            f"v={self.v_current:.2f} w={self.w_current:.2f} | "
            f"L={left_cmd} R={right_cmd}"
        )

    # ======================================================================
    # Motor scaling (Sabertooth simplified serial)
    #
    # Left motor:  1 (full reverse) — 64 (stop) — 127 (full forward)
    # Right motor: 128 (full reverse) — 192 (stop) — 255 (full forward)
    #
    # Dead zone: values in [-MOTOR_DEAD_ZONE, +MOTOR_DEAD_ZONE] map to
    # stop command. Prevents tiny inputs from jumping to min motor cmd.
    #
    # Symmetric scaling: both forward and reverse get 63 steps each.
    # Forward: stop+1 to stop+63, Reverse: stop-1 to stop-63
    # ======================================================================

    MOTOR_DEAD_ZONE = 0.02

    @classmethod
    def _scale_motor_command(cls, value: float, left_motor: bool) -> int:
        """
        Scale normalized [-1.0, 1.0] value to Sabertooth command byte.
        Dead zone around zero prevents discontinuity at zero crossing.
        """
        stop = 64 if left_motor else 192

        if abs(value) < cls.MOTOR_DEAD_ZONE:
            return stop

        if left_motor:
            if value > 0:
                # Forward: 65 — 127 (63 steps)
                cmd = int(64 + value * 63)
                return max(65, min(127, cmd))
            else:
                # Reverse: 63 — 1 (63 steps)
                cmd = int(64 + value * 63)
                return max(1, min(63, cmd))
        else:
            if value > 0:
                # Forward: 193 — 255 (63 steps)
                cmd = int(192 + value * 63)
                return max(193, min(255, cmd))
            else:
                # Reverse: 191 — 128 (63 steps, was 129 before)
                cmd = int(192 + value * 63)
                return max(128, min(191, cmd))

    # ======================================================================
    # Serial I/O with health tracking
    # ======================================================================

    def _send_serial(self, left_cmd: int, right_cmd: int) -> None:
        """Send motor commands with failure tracking. No recursive calls."""
        if not self.serial_healthy or self.motor is None:
            return

        try:
            self.motor.write(bytes([left_cmd, right_cmd]))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            self._mark_serial_failed()

    def _send_stop(self) -> None:
        """Send stop command. Marks serial failed if write errors."""
        self.v_current = 0.0
        self.w_current = 0.0
        self.v_target = 0.0
        self.w_target = 0.0
        self.is_stopped = True

        if not self.serial_healthy or self.motor is None:
            return

        try:
            self.motor.write(bytes([64, 192]))
        except Exception as e:
            self.get_logger().error(f"Failed to send STOP: {e}")
            self._mark_serial_failed()

    # ======================================================================
    # Diagnostics
    # ======================================================================

    def _publish_diagnostics(self, omega_left: float,
                             omega_right: float) -> None:
        """
        Publish expected wheel velocities for external comparison.

        Data: [v_current, w_current, omega_left, omega_right]

        Since we have no wheel encoders, this represents what the robot
        *should* be doing based on commands sent. External nodes can
        compare against ZED camera odometry to detect:
        - Wheel slip (ZED shows less movement than commanded)
        - Motor stall (ZED shows zero movement despite commands)
        - Drift (ZED odom diverges over time from integrated commands)
        """
        msg = Float32MultiArray()
        msg.data = [
            float(self.v_current),
            float(self.w_current),
            float(omega_left),
            float(omega_right),
        ]
        self.diag_pub.publish(msg)

    # ======================================================================
    # Shutdown
    # ======================================================================

    def shutdown(self) -> None:
        """Graceful shutdown — stop motors and close serial."""
        self.get_logger().info("Shutting down controller...")

        # Cancel timer to stop control loop
        if self.control_timer is not None:
            self.control_timer.cancel()

        self._close_serial()


# ==========================================================================
# Main
# ==========================================================================

def main():
    rclpy.init()
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received")
    except Exception as e:
        node.get_logger().fatal(f"Unhandled exception: {e}")
    finally:
        # ALWAYS runs — stops motors and closes serial port
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
