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

All robot-specific parameters are loaded from a YAML config file.
The node will not start without a valid config.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import serial
import math
import time

# Required parameters — node will not start without these
REQUIRED_PARAMS = [
    "serial_port",
    "serial_baud",
    "wheel_radius",
    "base_length",
    "min_linear_vel",
    "max_linear_vel",
    "min_angular_vel",
    "max_angular_vel",
    "max_linear_accel",
    "max_linear_decel",
    "max_angular_accel",
    "max_angular_decel",
    "control_dt",
    "cmd_timeout",
    "max_wheel_rad_s",
    "serial_reconnect_interval",
    "watchdog_decel_rate",
    "motor_dead_zone",
]


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # ==================================================================
        # Load and validate parameters (no defaults — config file required)
        # ==================================================================
        self._declare_params()
        if not self._validate_params():
            self.get_logger().fatal(
                "Missing required parameters. "
                "Ensure controller_params.yaml is loaded. Shutting down."
            )
            raise SystemExit(1)

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
        self.motor_dead_zone = self._p("motor_dead_zone")

        self.get_logger().info(
            f"Config loaded — wheel_r={self.wheel_radius}, "
            f"base_l={self.base_length}, "
            f"v=[{self.min_linear_vel}, {self.max_linear_vel}], "
            f"port={self.serial_port}"
        )

        # ==================================================================
        # Subscription
        # ==================================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Twist, "/cmd_vel_out", self.cmd_vel_callback, qos)

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
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        # Desired command (latest received)
        self.v_target = 0.0
        self.w_target = 0.0

        # Actual applied command (rate limited)
        self.v_current = 0.0
        self.w_current = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_control_time = self.get_clock().now()

        self.is_stopped = True
        self.watchdog_active = False

        # ==================================================================
        # Serial port with health tracking
        # ==================================================================
        self.motor: serial.Serial = None
        self.serial_healthy = False
        self.last_reconnect_attempt = 0.0

        self._connect_serial()

    # ==================================================================
    # Parameter handling
    # ==================================================================

    def _declare_params(self):
        """Declare all parameters without defaults."""
        for name in REQUIRED_PARAMS:
            if not self.has_parameter(name):
                self.declare_parameter(name)

    def _validate_params(self) -> bool:
        """Check all required params are set (not None)."""
        missing = []
        for name in REQUIRED_PARAMS:
            val = self.get_parameter(name).value
            if val is None:
                missing.append(name)

        if missing:
            self.get_logger().fatal(f"Missing parameters: {missing}")
            return False
        return True

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

            self.motor = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
            self.serial_healthy = True
            self.last_reconnect_attempt = time.monotonic()
            self.get_logger().info(f"Connected to Sabertooth on {self.serial_port}")
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
        self.v_current = 0.0
        self.w_current = 0.0
        self.v_target = 0.0
        self.w_target = 0.0
        self.is_stopped = True

    def _close_serial(self) -> None:
        """Close serial port safely."""
        if self.motor is not None:
            try:
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
            self.get_logger().warn("Received invalid cmd_vel (NaN or Inf). Ignoring.")
            return

        self.last_cmd_time = self.get_clock().now()

        self.v_target = max(self.min_linear_vel, min(self.max_linear_vel, msg.linear.x))
        self.w_target = max(
            self.min_angular_vel, min(self.max_angular_vel, msg.angular.z)
        )

        # If watchdog was ramping down, new command cancels it
        self.watchdog_active = False

    # ======================================================================
    # Rate limiter
    # ======================================================================

    @staticmethod
    def _limit_rate(
        target: float, current: float, accel: float, decel: float, dt: float
    ) -> float:
        """
        Rate limit with proper direction-change handling.

        Decel is used when:
          - Moving toward zero (reducing magnitude)
          - Crossing zero (must decel to zero first, then accel)
        Accel is used when:
          - Increasing magnitude in the same direction
        """
        delta = target - current

        same_sign = (current >= 0 and target >= 0) or (current <= 0 and target <= 0)

        if same_sign and abs(target) >= abs(current):
            limit = accel * dt
        else:
            limit = decel * dt

        return current + max(-limit, min(limit, delta))

    # ======================================================================
    # Control loop
    # ======================================================================

    def control_loop(self):
        now = self.get_clock().now()

        # Safe dt — clamp to [1ms, 500ms]
        dt_raw = (now - self.last_control_time).nanoseconds * 1e-9
        dt = max(0.001, min(0.5, dt_raw))
        self.last_control_time = now

        # Serial health check
        if not self._try_reconnect():
            return

        # ==================================================================
        # Watchdog — ramp down instead of hard stop
        # ==================================================================
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9

        if elapsed > self.cmd_timeout:
            if not self.is_stopped:
                if not self.watchdog_active:
                    self.get_logger().warn("CMD_VEL timeout — ramping down")
                    self.watchdog_active = True

                self.v_current = self._limit_rate(
                    0.0,
                    self.v_current,
                    self.max_linear_accel,
                    self.watchdog_decel_rate,
                    dt,
                )
                self.w_current = self._limit_rate(
                    0.0,
                    self.w_current,
                    self.max_angular_accel,
                    self.watchdog_decel_rate,
                    dt,
                )

                if abs(self.v_current) < 1e-3 and abs(self.w_current) < 1e-3:
                    self._send_stop()
                    self.watchdog_active = False
                    return

                self._send_velocity()
            return

        # ==================================================================
        # Normal rate limiting
        # ==================================================================
        self.v_current = self._limit_rate(
            self.v_target,
            self.v_current,
            self.max_linear_accel,
            self.max_linear_decel,
            dt,
        )
        self.w_current = self._limit_rate(
            self.w_target,
            self.w_current,
            self.max_angular_accel,
            self.max_angular_decel,
            dt,
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
        v_r = self.v_current - (self.base_length / 2.0) * self.w_current
        v_l = self.v_current + (self.base_length / 2.0) * self.w_current

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
    # Dead zone: values in [-motor_dead_zone, +motor_dead_zone] map to
    # stop command. Prevents tiny inputs from jumping to min motor cmd.
    #
    # Symmetric scaling: both forward and reverse get 63 steps each.
    # Forward: stop+1 to stop+63, Reverse: stop-1 to stop-63
    # ======================================================================

    def _scale_motor_command(self, value: float, left_motor: bool) -> int:
        """
        Scale normalized [-1.0, 1.0] value to Sabertooth command byte.
        Dead zone around zero prevents discontinuity at zero crossing.
        """
        stop = 64 if left_motor else 192

        if abs(value) < self.motor_dead_zone:
            return stop

        if left_motor:
            if value > 0:
                cmd = int(64 + value * 63)
                return max(65, min(127, cmd))
            else:
                cmd = int(64 + value * 63)
                return max(1, min(63, cmd))
        else:
            if value > 0:
                cmd = int(192 + value * 63)
                return max(193, min(255, cmd))
            else:
                cmd = int(192 + value * 63)
                return max(128, min(191, cmd))

    # ======================================================================
    # Serial I/O with health tracking
    # ======================================================================

    def _send_serial(self, left_cmd: int, right_cmd: int) -> None:
        """Send motor commands with failure tracking."""
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

    def _publish_diagnostics(self, omega_left: float, omega_right: float) -> None:
        """
        Publish expected wheel velocities for external comparison.
        Data: [v_current, w_current, omega_left, omega_right]
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
    except SystemExit:
        pass
    except Exception as e:
        node.get_logger().fatal(f"Unhandled exception: {e}")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
