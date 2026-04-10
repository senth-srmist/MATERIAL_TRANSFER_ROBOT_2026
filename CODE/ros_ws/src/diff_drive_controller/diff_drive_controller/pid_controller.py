#!/usr/bin/env python3
"""
PID Velocity Controller Node (v2 - Optimized)

Closes the velocity loop between Nav2 and the motor driver using
encoder feedback.

Optimizations from v1:
  - Pre-allocated Twist message (reused each publish)
  - Derivative-on-measurement (avoids derivative kick on setpoint change)
  - Consistent ROS clock usage throughout
  - Cached kinematic constants

Subscribes:
  /cmd_vel_nav2       (Twist)           — desired body velocity from Nav2
  /encoder/velocity   (Float32MultiArray) — [stamp_s, stamp_ns, left_rad_s, right_rad_s]

Publishes:
  /cmd_vel_pid        (Twist)           — corrected body velocity for twist_mux

Architecture:
  Nav2 → /cmd_vel_nav2 → [PID controller] → /cmd_vel_pid → twist_mux
                              ↑
                        /encoder/velocity

The PID runs per-wheel in rad/s space:
  1. Convert desired body velocity (v, w) to desired wheel velocities
  2. Feedforward: output = ff_gain * desired_wheel_vel
  3. PID correction using derivative-on-measurement
  4. Convert corrected wheel velocities back to body velocity (v, w)
  5. Rate limit and publish on /cmd_vel_pid
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class PIDController:
    """
    PID with anti-windup and derivative-on-measurement.

    Derivative-on-measurement avoids the "derivative kick" that occurs
    when the setpoint changes suddenly. Instead of d(error)/dt, we use
    -d(measurement)/dt.
    """

    __slots__ = (
        "kp",
        "ki",
        "kd",
        "max_integral",
        "_integral",
        "_prev_measurement",
        "_first_run",
    )

    def __init__(self, kp: float, ki: float, kd: float, max_integral: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral

        self._integral = 0.0
        self._prev_measurement = 0.0
        self._first_run = True

    def compute(self, desired: float, actual: float, dt: float) -> float:
        error = desired - actual

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        if self._integral > self.max_integral:
            self._integral = self.max_integral
        elif self._integral < -self.max_integral:
            self._integral = -self.max_integral
        i_term = self.ki * self._integral

        # Derivative on measurement (not error) — avoids derivative kick
        if self._first_run:
            d_term = 0.0
            self._first_run = False
        elif dt > 0:
            # Negative because we want -d(measurement)/dt
            d_term = -self.kd * (actual - self._prev_measurement) / dt
        else:
            d_term = 0.0

        self._prev_measurement = actual

        return p_term + i_term + d_term

    def reset(self):
        self._integral = 0.0
        self._prev_measurement = 0.0
        self._first_run = True


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__("pid_controller")

        self._declare_params()
        self._load_params()

        # Cached kinematic constants
        self._half_base = self._base_length / 2.0
        self._inv_wheel_radius = 1.0 / self._wheel_radius

        # Create PID instances (one per wheel)
        self._left_pid = PIDController(self._kp, self._ki, self._kd, self._max_integral)
        self._right_pid = PIDController(
            self._kp, self._ki, self._kd, self._max_integral
        )

        # State
        self._desired_v = 0.0
        self._desired_w = 0.0
        self._actual_left_vel = 0.0
        self._actual_right_vel = 0.0
        self._rate_limited_v = 0.0
        self._rate_limited_w = 0.0
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds
        self._last_control_time_ns = self.get_clock().now().nanoseconds
        self._is_stopped = True

        # Pre-allocated message (reused each publish)
        self._cmd_msg = Twist()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe to Nav2 velocity commands
        self.create_subscription(
            Twist, "/cmd_vel_nav2", self._cmd_vel_callback, reliable_qos
        )

        # Subscribe to encoder velocity
        self.create_subscription(
            Float32MultiArray,
            "/encoder/velocity",
            self._encoder_callback,
            best_effort_qos,
        )

        # Publisher for corrected velocity
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_pid", reliable_qos)

        # Control loop timer
        period = 1.0 / self._control_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"PID controller v2 started — Kp={self._kp} Ki={self._ki} Kd={self._kd} FF={self._ff_gain} rate={self._control_rate}Hz"
        )

    # ==================================================================
    # Parameters
    # ==================================================================

    def _declare_params(self):
        self.declare_parameter("wheel_radius", Parameter.Type.DOUBLE)
        self.declare_parameter("base_length", Parameter.Type.DOUBLE)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("min_linear_vel", -0.2)
        self.declare_parameter("max_linear_vel", 0.78)
        self.declare_parameter("min_angular_vel", -2.0)
        self.declare_parameter("max_angular_vel", 2.0)
        self.declare_parameter("max_linear_accel", 0.3)
        self.declare_parameter("max_linear_decel", 0.6)
        self.declare_parameter("max_angular_accel", 1.5)
        self.declare_parameter("max_angular_decel", 1.5)
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("ki", 0.2)
        self.declare_parameter("kd", 0.01)
        self.declare_parameter("max_integral", 2.0)
        self.declare_parameter("feedforward_gain", 1.0)
        self.declare_parameter("cmd_timeout", 0.5)

    def _load_params(self):
        self._wheel_radius = self.get_parameter("wheel_radius").value
        self._base_length = self.get_parameter("base_length").value
        self._control_rate = self.get_parameter("control_rate").value
        self._min_linear_vel = self.get_parameter("min_linear_vel").value
        self._max_linear_vel = self.get_parameter("max_linear_vel").value
        self._min_angular_vel = self.get_parameter("min_angular_vel").value
        self._max_angular_vel = self.get_parameter("max_angular_vel").value
        self._max_linear_accel = self.get_parameter("max_linear_accel").value
        self._max_linear_decel = self.get_parameter("max_linear_decel").value
        self._max_angular_accel = self.get_parameter("max_angular_accel").value
        self._max_angular_decel = self.get_parameter("max_angular_decel").value
        self._kp = self.get_parameter("kp").value
        self._ki = self.get_parameter("ki").value
        self._kd = self.get_parameter("kd").value
        self._max_integral = self.get_parameter("max_integral").value
        self._ff_gain = self.get_parameter("feedforward_gain").value
        self._cmd_timeout = self.get_parameter("cmd_timeout").value

        if self._wheel_radius is None or self._wheel_radius <= 0:
            self.get_logger().fatal("wheel_radius must be > 0")
            raise SystemExit(1)
        if self._base_length is None or self._base_length <= 0:
            self.get_logger().fatal("base_length must be > 0")
            raise SystemExit(1)

    # ==================================================================
    # Callbacks
    # ==================================================================

    def _cmd_vel_callback(self, msg: Twist):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            return

        # Clamp to limits
        v = msg.linear.x
        w = msg.angular.z

        if v > self._max_linear_vel:
            v = self._max_linear_vel
        elif v < self._min_linear_vel:
            v = self._min_linear_vel

        if w > self._max_angular_vel:
            w = self._max_angular_vel
        elif w < self._min_angular_vel:
            w = self._min_angular_vel

        self._desired_v = v
        self._desired_w = w
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds

    def _encoder_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self._actual_left_vel = msg.data[2]
            self._actual_right_vel = msg.data[3]

    # ==================================================================
    # Rate limiter
    # ==================================================================

    def _limit_rate(
        self, target: float, current: float, accel: float, decel: float, dt: float
    ) -> float:
        delta = target - current

        # Determine if accelerating or decelerating
        same_sign = (current >= 0 and target >= 0) or (current <= 0 and target <= 0)

        if same_sign and abs(target) >= abs(current):
            limit = accel * dt
        else:
            limit = decel * dt

        if delta > limit:
            return current + limit
        elif delta < -limit:
            return current - limit
        return target

    # ==================================================================
    # Differential drive kinematics (inlined for performance)
    # ==================================================================

    def _body_to_wheel(self, v: float, w: float) -> tuple:
        """Convert body velocity (v, w) to wheel angular velocities (left, right) in rad/s."""
        v_left = v + self._half_base * w
        v_right = v - self._half_base * w
        return v_left * self._inv_wheel_radius, v_right * self._inv_wheel_radius

    def _wheel_to_body(self, omega_left: float, omega_right: float) -> tuple:
        """Convert wheel angular velocities (rad/s) back to body velocity (v, w)."""
        v_left = omega_left * self._wheel_radius
        v_right = omega_right * self._wheel_radius
        v = (v_left + v_right) * 0.5
        w = (v_left - v_right) / self._base_length
        return v, w

    # ==================================================================
    # Control loop
    # ==================================================================

    def _control_loop(self):
        now_ns = self.get_clock().now().nanoseconds
        dt_ns = now_ns - self._last_control_time_ns
        dt = max(0.001, min(0.5, dt_ns * 1e-9))
        self._last_control_time_ns = now_ns

        # Check for Nav2 command timeout
        elapsed_ns = now_ns - self._last_cmd_time_ns
        elapsed = elapsed_ns * 1e-9

        if elapsed > self._cmd_timeout:
            if not self._is_stopped:
                # Ramp down
                self._rate_limited_v = self._limit_rate(
                    0.0,
                    self._rate_limited_v,
                    self._max_linear_accel,
                    self._max_linear_decel,
                    dt,
                )
                self._rate_limited_w = self._limit_rate(
                    0.0,
                    self._rate_limited_w,
                    self._max_angular_accel,
                    self._max_angular_decel,
                    dt,
                )

                if (
                    abs(self._rate_limited_v) < 1e-3
                    and abs(self._rate_limited_w) < 1e-3
                ):
                    self._rate_limited_v = 0.0
                    self._rate_limited_w = 0.0
                    self._is_stopped = True
                    self._left_pid.reset()
                    self._right_pid.reset()
                    self._publish_cmd(0.0, 0.0)
                    return

                # Still ramping down — run PID with current rate-limited target
                self._run_pid_and_publish(dt)
            return

        self._is_stopped = False

        # Rate limit the desired velocity
        self._rate_limited_v = self._limit_rate(
            self._desired_v,
            self._rate_limited_v,
            self._max_linear_accel,
            self._max_linear_decel,
            dt,
        )
        self._rate_limited_w = self._limit_rate(
            self._desired_w,
            self._rate_limited_w,
            self._max_angular_accel,
            self._max_angular_decel,
            dt,
        )

        self._run_pid_and_publish(dt)

    def _run_pid_and_publish(self, dt: float):
        # Convert desired body velocity to wheel velocities
        desired_left, desired_right = self._body_to_wheel(
            self._rate_limited_v, self._rate_limited_w
        )

        # Feedforward
        ff_left = self._ff_gain * desired_left
        ff_right = self._ff_gain * desired_right

        # PID correction
        pid_left = self._left_pid.compute(desired_left, self._actual_left_vel, dt)
        pid_right = self._right_pid.compute(desired_right, self._actual_right_vel, dt)

        # Total output per wheel
        output_left = ff_left + pid_left
        output_right = ff_right + pid_right

        # Convert back to body velocity
        v_out, w_out = self._wheel_to_body(output_left, output_right)

        # Clamp to velocity limits
        if v_out > self._max_linear_vel:
            v_out = self._max_linear_vel
        elif v_out < self._min_linear_vel:
            v_out = self._min_linear_vel

        if w_out > self._max_angular_vel:
            w_out = self._max_angular_vel
        elif w_out < self._min_angular_vel:
            w_out = self._min_angular_vel

        self._publish_cmd(v_out, w_out)

    def _publish_cmd(self, v: float, w: float):
        """Publish command velocity (reuses pre-allocated message)."""
        self._cmd_msg.linear.x = v
        self._cmd_msg.angular.z = w
        self._cmd_pub.publish(self._cmd_msg)


# ======================================================================
# Main
# ======================================================================


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
