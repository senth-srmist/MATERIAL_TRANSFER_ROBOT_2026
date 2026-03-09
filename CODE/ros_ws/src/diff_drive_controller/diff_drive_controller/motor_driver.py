#!/usr/bin/env python3
"""
Motor Driver Node for Sabertooth motor driver.

Subscribes to /cmd_vel_out (from twist_mux) and sends serial commands
to Sabertooth. This is a dumb pipe — no rate limiting, no PID.
Rate limiting is handled by the PID controller (for Nav2) or is
unnecessary (for joystick, where the human is the controller).

Includes watchdog timeout, serial health monitoring, and graceful shutdown.

Subscribes:
  /cmd_vel_out (Twist) — from twist_mux (joystick or PID-corrected Nav2)

Publishes:
  /motor_controller/diagnostics (Float32MultiArray) — [v, w, omega_l, omega_r]

All robot-specific parameters are loaded from drive_params.yaml.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import serial
import math
import time


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")

        self._declare_params()
        self._load_params()

        self.get_logger().info(
            f"Motor driver loaded — "
            f"wheel_r={self.wheel_radius}, base_l={self.base_length}, "
            f"port={self.serial_port}"
        )

        # Subscription
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Twist, "/cmd_vel_out", self._cmd_vel_callback, cmd_qos)

        # Diagnostics publisher
        diag_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.diag_pub = self.create_publisher(
            Float32MultiArray, "/motor_controller/diagnostics", diag_qos
        )

        # Control timer
        self.control_timer = self.create_timer(self.control_dt, self._control_loop)

        # State
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.v_current = 0.0
        self.w_current = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_control_time = self.get_clock().now()
        self.is_stopped = True
        self.watchdog_active = False

        # Serial
        self.motor: serial.Serial = None
        self.serial_healthy = False
        self.last_reconnect_attempt = 0.0
        self._connect_serial()

    # ==================================================================
    # Parameters
    # ==================================================================

    def _declare_params(self):
        self.declare_parameter("serial_port", Parameter.Type.STRING)
        self.declare_parameter("serial_baud", Parameter.Type.INTEGER)
        self.declare_parameter("wheel_radius", Parameter.Type.DOUBLE)
        self.declare_parameter("base_length", Parameter.Type.DOUBLE)
        self.declare_parameter("max_wheel_rad_s", Parameter.Type.DOUBLE)
        self.declare_parameter("motor_dead_zone", Parameter.Type.DOUBLE)
        self.declare_parameter("control_dt", Parameter.Type.DOUBLE)
        self.declare_parameter("cmd_timeout", Parameter.Type.DOUBLE)
        self.declare_parameter("serial_reconnect_interval", Parameter.Type.DOUBLE)
        self.declare_parameter("watchdog_decel_rate", Parameter.Type.DOUBLE)
        self.declare_parameter("max_linear_accel", Parameter.Type.DOUBLE)
        self.declare_parameter("max_angular_accel", Parameter.Type.DOUBLE)

    def _load_params(self):
        self.serial_port = self.get_parameter("serial_port").value
        self.serial_baud = self.get_parameter("serial_baud").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.base_length = self.get_parameter("base_length").value
        self.max_wheel_rad_s = self.get_parameter("max_wheel_rad_s").value
        self.motor_dead_zone = self.get_parameter("motor_dead_zone").value
        self.control_dt = self.get_parameter("control_dt").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.serial_reconnect_interval = self.get_parameter(
            "serial_reconnect_interval"
        ).value
        self.watchdog_decel_rate = self.get_parameter("watchdog_decel_rate").value
        self.max_linear_accel = self.get_parameter("max_linear_accel").value
        self.max_angular_accel = self.get_parameter("max_angular_accel").value

        if self.wheel_radius is None or self.wheel_radius <= 0:
            self.get_logger().fatal("wheel_radius must be > 0")
            raise SystemExit(1)
        if self.base_length is None or self.base_length <= 0:
            self.get_logger().fatal("base_length must be > 0")
            raise SystemExit(1)
        if not self.serial_port:
            self.get_logger().fatal("serial_port must be set")
            raise SystemExit(1)

    # ==================================================================
    # Serial management
    # ==================================================================

    def _connect_serial(self):
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

    def _try_reconnect(self):
        if self.serial_healthy:
            return True
        now = time.monotonic()
        if now - self.last_reconnect_attempt < self.serial_reconnect_interval:
            return False
        self.get_logger().info("Attempting serial reconnect...")
        return self._connect_serial()

    def _mark_serial_failed(self):
        if self.serial_healthy:
            self.serial_healthy = False
            self.get_logger().error("Serial port failed — stopping until reconnect")
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.v_current = 0.0
        self.w_current = 0.0
        self.is_stopped = True

    def _close_serial(self):
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

    # ==================================================================
    # Command callback
    # ==================================================================

    def _cmd_vel_callback(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.get_logger().warn("Invalid cmd_vel (NaN/Inf). Ignoring.")
            return

        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_active = False

    # ==================================================================
    # Rate limiter (only used for watchdog ramp-down)
    # ==================================================================

    @staticmethod
    def _limit_rate(target, current, accel, dt):
        delta = target - current
        limit = accel * dt
        return current + max(-limit, min(limit, delta))

    # ==================================================================
    # Control loop
    # ==================================================================

    def _control_loop(self):
        now = self.get_clock().now()
        dt_raw = (now - self.last_control_time).nanoseconds * 1e-9
        dt = max(0.001, min(0.5, dt_raw))
        self.last_control_time = now

        if not self._try_reconnect():
            return

        # Watchdog — ramp down if no commands received
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            if not self.is_stopped:
                if not self.watchdog_active:
                    self.get_logger().warn("CMD_VEL timeout — ramping down")
                    self.watchdog_active = True

                self.v_current = self._limit_rate(
                    0.0, self.v_current, self.watchdog_decel_rate, dt
                )
                self.w_current = self._limit_rate(
                    0.0, self.w_current, self.watchdog_decel_rate, dt
                )

                if abs(self.v_current) < 1e-3 and abs(self.w_current) < 1e-3:
                    self._send_stop()
                    self.watchdog_active = False
                    return

                self._send_velocity(self.v_current, self.w_current)
            return

        # Normal operation — send commanded velocity directly
        self.v_current = self.v_cmd
        self.w_current = self.w_cmd

        if abs(self.v_cmd) < 1e-3 and abs(self.w_cmd) < 1e-3:
            if not self.is_stopped:
                self._send_stop()
            return

        self._send_velocity(self.v_cmd, self.w_cmd)

    # ==================================================================
    # Velocity to motor commands
    # ==================================================================

    def _send_velocity(self, v, w):
        self.is_stopped = False

        v_r = v - (self.base_length / 2.0) * w
        v_l = v + (self.base_length / 2.0) * w

        omega_r = v_r / self.wheel_radius
        omega_l = v_l / self.wheel_radius

        raw_right = omega_r / self.max_wheel_rad_s
        raw_left = omega_l / self.max_wheel_rad_s

        if abs(raw_right) > 1.0 or abs(raw_left) > 1.0:
            self.get_logger().debug(
                f"Wheel velocity clamped: L={raw_left:.2f} R={raw_right:.2f}"
            )

        right = max(-1.0, min(1.0, raw_right))
        left = max(-1.0, min(1.0, raw_left))

        left_cmd = self._scale_motor_command(left, left_motor=True)
        right_cmd = self._scale_motor_command(right, left_motor=False)

        self._send_serial(left_cmd, right_cmd)
        self._publish_diagnostics(v, w, omega_l, omega_r)

        self.get_logger().debug(f"v={v:.2f} w={w:.2f} | L={left_cmd} R={right_cmd}")

    # ==================================================================
    # Motor scaling (Sabertooth simplified serial)
    # ==================================================================

    def _scale_motor_command(self, value, left_motor):
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

    # ==================================================================
    # Serial I/O
    # ==================================================================

    def _send_serial(self, left_cmd, right_cmd):
        if not self.serial_healthy or self.motor is None:
            return
        try:
            self.motor.write(bytes([left_cmd, right_cmd]))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            self._mark_serial_failed()

    def _send_stop(self):
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.v_current = 0.0
        self.w_current = 0.0
        self.is_stopped = True

        if not self.serial_healthy or self.motor is None:
            return
        try:
            self.motor.write(bytes([64, 192]))
        except Exception as e:
            self.get_logger().error(f"Failed to send STOP: {e}")
            self._mark_serial_failed()

    # ==================================================================
    # Diagnostics
    # ==================================================================

    def _publish_diagnostics(self, v, w, omega_left, omega_right):
        msg = Float32MultiArray()
        msg.data = [float(v), float(w), float(omega_left), float(omega_right)]
        self.diag_pub.publish(msg)

    # ==================================================================
    # Shutdown
    # ==================================================================

    def shutdown(self):
        self.get_logger().info("Shutting down motor driver...")
        if self.control_timer is not None:
            self.control_timer.cancel()
        self._close_serial()


# ======================================================================
# Main
# ======================================================================


def main():
    rclpy.init()
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
