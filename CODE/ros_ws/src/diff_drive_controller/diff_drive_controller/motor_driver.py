#!/usr/bin/env python3
"""
Motor Driver Node for Sabertooth (v7 - Per-Wheel Speed Input)

Subscribes to /wheel_speeds (Float32MultiArray [omega_left_rad_s, omega_right_rad_s])
from the wheel speed controller (pid_controller). Applies per-wheel acceleration
limiting and sends commands to the Sabertooth via simplified serial.

Kinematics are handled upstream. This node is a pure hardware interface.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from std_msgs.msg import Float32MultiArray

import serial


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")

        self._declare_params()
        self._load_params()

        self._inv_max_wheel_rad_s = 1.0 / self._max_wheel_rad_s

        # Pre-allocated serial buffer (2 bytes: left, right)
        self._serial_buf = bytearray(2)

        # Pre-allocated diagnostic message
        self._diag_msg = Float32MultiArray()
        self._diag_msg.data = [0.0, 0.0, 0.0, 0.0]

        self.get_logger().info(
            f"Motor driver v7 — wheel_r={self._wheel_radius} "
            f"max_wheel_rad_s={self._max_wheel_rad_s:.2f} max_wheel_accel={self._max_wheel_accel:.2f}"
        )

        # Subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Float32MultiArray, "/wheel_speeds", self._wheel_speeds_callback, qos
        )

        self._diag_pub = self.create_publisher(
            Float32MultiArray, "/motor_controller/diagnostics", qos
        )

        # Control timer
        self._control_timer = self.create_timer(self._control_dt, self._control_loop)

        # Per-wheel state
        self._omega_l_target = 0.0
        self._omega_r_target = 0.0
        self._omega_l_current = 0.0
        self._omega_r_current = 0.0
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds
        self._last_control_time_ns = self.get_clock().now().nanoseconds
        self._is_stopped = True
        self._watchdog_active = False

        # Serial
        self._motor: serial.Serial = None
        self._serial_healthy = False
        self._last_reconnect_time = 0.0
        self._connect_serial()

    # ==================================================================
    # Parameters
    # ==================================================================

    def _declare_params(self):
        self.declare_parameter("serial_port", Parameter.Type.STRING)
        self.declare_parameter("serial_baud", Parameter.Type.INTEGER)
        self.declare_parameter("wheel_radius", Parameter.Type.DOUBLE)
        self.declare_parameter("max_wheel_rad_s", Parameter.Type.DOUBLE)
        self.declare_parameter("max_wheel_accel", Parameter.Type.DOUBLE)
        self.declare_parameter("motor_dead_zone", Parameter.Type.DOUBLE)
        self.declare_parameter("control_dt", Parameter.Type.DOUBLE)
        self.declare_parameter("cmd_timeout", Parameter.Type.DOUBLE)
        self.declare_parameter("serial_reconnect_interval", Parameter.Type.DOUBLE)
        self.declare_parameter("watchdog_decel_rate", Parameter.Type.DOUBLE)
        self.declare_parameter("serial_write_timeout", 0.1)

    def _load_params(self):
        self._serial_port = self.get_parameter("serial_port").value
        self._serial_baud = self.get_parameter("serial_baud").value
        self._wheel_radius = self.get_parameter("wheel_radius").value
        self._max_wheel_rad_s = self.get_parameter("max_wheel_rad_s").value
        self._max_wheel_accel = self.get_parameter("max_wheel_accel").value
        self._motor_dead_zone = self.get_parameter("motor_dead_zone").value
        self._control_dt = self.get_parameter("control_dt").value
        self._cmd_timeout = self.get_parameter("cmd_timeout").value
        self._serial_reconnect_interval = self.get_parameter(
            "serial_reconnect_interval"
        ).value
        self._watchdog_decel_rate = self.get_parameter("watchdog_decel_rate").value
        self._serial_write_timeout = self.get_parameter("serial_write_timeout").value

        # Validations
        if self._wheel_radius <= 0:
            self.get_logger().fatal("wheel_radius must be > 0")
            raise SystemExit(1)
        if self._max_wheel_rad_s <= 0:
            self.get_logger().fatal("max_wheel_rad_s must be > 0")
            raise SystemExit(1)
        if not self._serial_port:
            self.get_logger().fatal("serial_port must be set")
            raise SystemExit(1)

    # ==================================================================
    # Serial management
    # ==================================================================

    def _connect_serial(self) -> bool:
        try:
            if self._motor is not None:
                try:
                    self._motor.close()
                except Exception:
                    pass

            self._motor = serial.Serial(
                self._serial_port,
                self._serial_baud,
                timeout=1,
                write_timeout=self._serial_write_timeout,
            )
            self._serial_healthy = True
            self._last_reconnect_time = time.monotonic()
            self.get_logger().info(f"Connected to Sabertooth on {self._serial_port}")
            return True
        except Exception as e:
            self._motor = None
            self._serial_healthy = False
            self._last_reconnect_time = time.monotonic()
            self.get_logger().error(f"Serial port open failed: {e}")
            return False

    def _try_reconnect(self) -> bool:
        if self._serial_healthy:
            return True
        now = time.monotonic()
        if now - self._last_reconnect_time < self._serial_reconnect_interval:
            return False
        self.get_logger().info("Attempting serial reconnect...")
        return self._connect_serial()

    def _mark_serial_failed(self):
        if self._serial_healthy:
            self._serial_healthy = False
            self.get_logger().error("Serial port failed — stopping until reconnect")
        self._omega_l_target = 0.0
        self._omega_r_target  = 0.0
        self._omega_l_current = 0.0
        self._omega_r_current = 0.0
        self._is_stopped = True

    def _close_serial(self):
        if self._motor is not None:
            try:
                if self._serial_healthy:
                    self._serial_buf[0] = 64
                    self._serial_buf[1] = 192
                    self._motor.write(self._serial_buf)
            except Exception:
                pass
            try:
                self._motor.close()
            except Exception:
                pass
            self._motor = None
            self._serial_healthy = False

    # ==================================================================
    # Command callback – receive per-wheel speeds from wheel speed controller
    # ==================================================================

    def _wheel_speeds_callback(self, msg: Float32MultiArray):
        m = self._max_wheel_rad_s
        self._omega_l_target = max(-m, min(m, msg.data[0]))
        self._omega_r_target  = max(-m, min(m, msg.data[1]))
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds
        self._watchdog_active = False

        # Send stop immediately — don't wait up to control_dt (100 ms) for the timer to fire.
        if abs(self._omega_l_target) < 1e-4 and abs(self._omega_r_target) < 1e-4:
            self._send_stop()

    # ==================================================================
    # Rate limiter (acceleration limiting)
    # ==================================================================

    @staticmethod
    def _limit_rate(target: float, current: float, accel: float, dt: float) -> float:
        delta = target - current
        limit = accel * dt
        if delta > limit:
            return current + limit
        elif delta < -limit:
            return current - limit
        return target

    # ==================================================================
    # Control loop
    # ==================================================================

    def _control_loop(self):
        now_ns = self.get_clock().now().nanoseconds
        dt_ns = now_ns - self._last_control_time_ns
        dt = max(0.001, min(0.5, dt_ns * 1e-9))
        self._last_control_time_ns = now_ns

        if not self._try_reconnect():
            return

        # Watchdog — ramp down if no commands received (communication lost)
        elapsed_ns = now_ns - self._last_cmd_time_ns
        elapsed = elapsed_ns * 1e-9

        if elapsed > self._cmd_timeout:
            if not self._is_stopped:
                if not self._watchdog_active:
                    self.get_logger().warning("Wheel speeds timeout — ramping down")
                    self._watchdog_active = True

                self._omega_l_current = self._limit_rate(
                    0.0, self._omega_l_current, self._watchdog_decel_rate, dt
                )
                self._omega_r_current = self._limit_rate(
                    0.0, self._omega_r_current, self._watchdog_decel_rate, dt
                )

                if abs(self._omega_l_current) < 1e-3 and abs(self._omega_r_current) < 1e-3:
                    self._send_stop()
                    self._watchdog_active = False
                    return

                self._send_velocity(self._omega_l_current, self._omega_r_current)
            return

        # Instant stop — if both wheel targets are zero, stop immediately without ramping.
        if abs(self._omega_l_target) < 1e-4 and abs(self._omega_r_target) < 1e-4:
            if not self._is_stopped:
                self._omega_l_current = 0.0
                self._omega_r_current = 0.0
                self._send_stop()
            return

        # Ramp per-wheel velocities toward target (acceleration limiting)
        self._omega_l_current = self._limit_rate(
            self._omega_l_target, self._omega_l_current, self._max_wheel_accel, dt
        )
        self._omega_r_current = self._limit_rate(
            self._omega_r_target, self._omega_r_current, self._max_wheel_accel, dt
        )

        self._send_velocity(self._omega_l_current, self._omega_r_current)

    # ==================================================================
    # Velocity to motor commands
    # ==================================================================

    def _send_velocity(self, omega_l: float, omega_r: float):
        self._is_stopped = False

        # Normalize to [-1, 1] — inputs are already wheel rad/s, no kinematics needed
        raw_left  = max(-1.0, min(1.0, omega_l * self._inv_max_wheel_rad_s))
        raw_right = max(-1.0, min(1.0, omega_r * self._inv_max_wheel_rad_s))

        # Scale to motor commands (Sabertooth simplified serial)
        left_cmd  = self._scale_motor_command(raw_left,  left_motor=True)
        right_cmd = self._scale_motor_command(raw_right, left_motor=False)

        # Send to hardware
        self._send_serial(left_cmd, right_cmd)

        # Publish diagnostics
        self._publish_diagnostics(omega_l, omega_r)

        self.get_logger().debug(f"omegaL={omega_l:.3f} omegaR={omega_r:.3f} | L={left_cmd} R={right_cmd}")

    # ==================================================================
    # Motor scaling (Sabertooth simplified serial)
    # ==================================================================
    #
    # Sabertooth simplified serial protocol:
    #   Motor 1 (Left):  1-127  (1=full reverse, 64=stop, 127=full forward)
    #   Motor 2 (Right): 128-255 (128=full reverse, 192=stop, 255=full forward)
    #
    # Input: value in [-1.0, 1.0]
    # Output: command byte

    def _scale_motor_command(self, value: float, left_motor: bool) -> int:
        if left_motor:
            stop_cmd = 64
            half_range = 63
            if abs(value) < self._motor_dead_zone:
                return stop_cmd
            cmd = int(stop_cmd + value * half_range)
            return max(1, min(127, cmd))
        else:
            stop_cmd = 192
            half_range = 63
            if abs(value) < self._motor_dead_zone:
                return stop_cmd
            cmd = int(stop_cmd + value * half_range)
            return max(128, min(255, cmd))

    # ==================================================================
    # Serial I/O
    # ==================================================================

    def _send_serial(self, left_cmd: int, right_cmd: int):
        if not self._serial_healthy or self._motor is None:
            return
        try:
            self._serial_buf[0] = left_cmd
            self._serial_buf[1] = right_cmd
            self._motor.write(self._serial_buf)
        except serial.SerialTimeoutException:
            self.get_logger().warning("Serial write timeout")
            self._mark_serial_failed()
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            self._mark_serial_failed()

    def _send_stop(self):
        self._omega_l_target  = 0.0
        self._omega_r_target  = 0.0
        self._omega_l_current = 0.0
        self._omega_r_current = 0.0
        self._is_stopped = True

        if not self._serial_healthy or self._motor is None:
            return
        try:
            self._serial_buf[0] = 64
            self._serial_buf[1] = 192
            self._motor.write(self._serial_buf)
        except Exception as e:
            self.get_logger().error(f"Failed to send STOP: {e}")
            self._mark_serial_failed()

    # ==================================================================
    # Diagnostics
    # ==================================================================

    def _publish_diagnostics(self, omega_left: float, omega_right: float):
        self._diag_msg.data[0] = omega_left
        self._diag_msg.data[1] = omega_right
        self._diag_msg.data[2] = omega_left  * self._wheel_radius  # linear speed left (m/s)
        self._diag_msg.data[3] = omega_right * self._wheel_radius  # linear speed right (m/s)
        self._diag_pub.publish(self._diag_msg)

    # ==================================================================
    # Shutdown
    # ==================================================================

    def shutdown(self):
        self.get_logger().info("Shutting down motor driver...")
        if self._control_timer is not None:
            self._control_timer.cancel()
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
