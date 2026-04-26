#!/usr/bin/env python3
"""
Wheel Speed Controller Node (v5 - Feedforward Only)

Changes from v4:
  - PID removed entirely. Pure feedforward control.
  - Per-wheel FF gains still compensate for wiring asymmetry during linear motion.
  - During pure rotation (v ≈ 0), a single symmetric FF gain is used for both
    wheels so |omega_left| == |omega_right| is guaranteed by construction.
  - Encoder subscription kept (for diagnostics / future use), but no feedback path.

Subscribes:
  /cmd_vel_out        (Twist)             — selected body velocity from twist_mux
  /encoder/velocity   (Float32MultiArray) — [stamp_s, stamp_ns, left_rad_s, right_rad_s]

Publishes:
  /wheel_speeds       (Float32MultiArray) — [omega_left_rad_s, omega_right_rad_s]
  /pid/debug          (Float32MultiArray) — diagnostic data (see layout below)

/pid/debug layout (12 floats):
  [ 0] dt
  [ 1] raw_v
  [ 2] raw_w
  [ 3] rate_limited_v
  [ 4] rate_limited_w
  [ 5] desired_left_wheel   — after kinematics, before FF
  [ 6] desired_right_wheel
  [ 7] ff_left              — FF-scaled output
  [ 8] ff_right
  [ 9] actual_left_wheel    — from encoders (diagnostic only)
  [10] actual_right_wheel
  [11] ff_gain_used         — 0=per-wheel asymmetric gains, 1=symmetric rotation gain

Architecture:
  twist_mux → /cmd_vel_out → [rate limiter] → [FF scale] → /wheel_speeds → motor_driver
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__("pid_controller")

        self._declare_params()
        self._load_params()

        # Cached kinematic constants
        self._half_base = self._base_length / 2.0
        self._inv_wheel_radius = 1.0 / self._wheel_radius

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

        # Pre-allocated messages
        self._wheel_msg = Float32MultiArray()
        self._wheel_msg.data = [0.0, 0.0]
        self._debug_msg = Float32MultiArray()
        self._debug_msg.data = [0.0] * 12

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Twist, "/cmd_vel_out", self._cmd_vel_callback, best_effort_qos
        )
        self.create_subscription(
            Float32MultiArray,
            "/encoder/velocity",
            self._encoder_callback,
            best_effort_qos,
        )

        self._cmd_pub = self.create_publisher(
            Float32MultiArray, "/wheel_speeds", best_effort_qos
        )
        self._debug_pub = self.create_publisher(
            Float32MultiArray, "/pid/debug", best_effort_qos
        )

        self.add_on_set_parameters_callback(self._on_parameter_change)

        period = 1.0 / self._control_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"Wheel speed controller v5 (FF-only) started — "
            f"ff_left={self._ff_gain_left} ff_right={self._ff_gain_right} "
            f"ff_rotation={self._ff_gain_rotation} "
            f"rotation_threshold={self._rotation_threshold} m/s | "
            f"rate={self._control_rate}Hz [dynamic reconfig enabled]"
        )

    # ==================================================================
    # Parameters
    # ==================================================================

    def _declare_params(self):
        self.declare_parameter("wheel_radius", Parameter.Type.DOUBLE)
        self.declare_parameter("base_length", Parameter.Type.DOUBLE)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("min_linear_vel", -0.5)
        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("min_angular_vel", -0.6)
        self.declare_parameter("max_angular_vel", 0.6)
        self.declare_parameter("max_linear_accel", 0.15)
        self.declare_parameter("max_linear_decel", 0.15)
        self.declare_parameter("max_angular_accel", 1.2)
        self.declare_parameter("max_angular_decel", 1.2)
        # FF gains for mixed (linear + angular) motion — compensates wiring asymmetry
        self.declare_parameter("feedforward_gain_left", 1.17)
        self.declare_parameter("feedforward_gain_right", 1.15)
        # Single symmetric FF gain used during pure rotation (v ≈ 0)
        # Both wheels use this gain so |omega_L| == |omega_R| by construction.
        # Start with the average of left/right and tune from there.
        self.declare_parameter("feedforward_gain_rotation", 1.16)
        # Linear velocity threshold below which we treat motion as pure rotation
        self.declare_parameter("rotation_threshold", 0.02)  # m/s
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
        self._ff_gain_left = self.get_parameter("feedforward_gain_left").value
        self._ff_gain_right = self.get_parameter("feedforward_gain_right").value
        self._ff_gain_rotation = self.get_parameter("feedforward_gain_rotation").value
        self._rotation_threshold = self.get_parameter("rotation_threshold").value
        self._cmd_timeout = self.get_parameter("cmd_timeout").value

        if self._wheel_radius is None or self._wheel_radius <= 0:
            self.get_logger().fatal("wheel_radius must be > 0")
            raise SystemExit(1)
        if self._base_length is None or self._base_length <= 0:
            self.get_logger().fatal("base_length must be > 0")
            raise SystemExit(1)

    def _on_parameter_change(self, params) -> SetParametersResult:
        for param in params:
            name = param.name
            val = param.value

            if name == "feedforward_gain_left":
                self._ff_gain_left = val
                self.get_logger().info(f"ff_gain_left → {val}")
            elif name == "feedforward_gain_right":
                self._ff_gain_right = val
                self.get_logger().info(f"ff_gain_right → {val}")
            elif name == "feedforward_gain_rotation":
                self._ff_gain_rotation = val
                self.get_logger().info(f"ff_gain_rotation → {val}")
            elif name == "rotation_threshold":
                self._rotation_threshold = val
                self.get_logger().info(f"rotation_threshold → {val}")
            elif name == "min_linear_vel":
                self._min_linear_vel = val
            elif name == "max_linear_vel":
                self._max_linear_vel = val
            elif name == "min_angular_vel":
                self._min_angular_vel = val
            elif name == "max_angular_vel":
                self._max_angular_vel = val
            elif name == "max_linear_accel":
                self._max_linear_accel = val
            elif name == "max_linear_decel":
                self._max_linear_decel = val
            elif name == "max_angular_accel":
                self._max_angular_accel = val
            elif name == "max_angular_decel":
                self._max_angular_decel = val
            elif name == "cmd_timeout":
                self._cmd_timeout = val
            elif name in ("wheel_radius", "base_length", "control_rate"):
                self.get_logger().warn(
                    f"Cannot change {name} at runtime — restart required"
                )
                return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)

    # ==================================================================
    # Callbacks
    # ==================================================================

    def _cmd_vel_callback(self, msg: Twist):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            return

        v = max(self._min_linear_vel, min(self._max_linear_vel, msg.linear.x))
        w = max(self._min_angular_vel, min(self._max_angular_vel, msg.angular.z))

        self._desired_v = v
        self._desired_w = w
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds

        # Explicit zero → stop immediately, don't wait for rate limiter
        if abs(v) < 1e-4 and abs(w) < 1e-4:
            self._rate_limited_v = 0.0
            self._rate_limited_w = 0.0
            self._is_stopped = True
            self._wheel_msg.data[0] = 0.0
            self._wheel_msg.data[1] = 0.0
            self._cmd_pub.publish(self._wheel_msg)

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
        same_sign = (current >= 0 and target >= 0) or (current <= 0 and target <= 0)
        limit = (accel if (same_sign and abs(target) >= abs(current)) else decel) * dt

        if delta > limit:
            return current + limit
        elif delta < -limit:
            return current - limit
        return target

    # ==================================================================
    # Differential drive kinematics
    # ==================================================================

    def _body_to_wheel(self, v: float, w: float) -> tuple:
        # Signs swapped: physical wiring has left port → right wheel, right port → left wheel
        v_left = (v + self._half_base * w) * self._inv_wheel_radius
        v_right = (v - self._half_base * w) * self._inv_wheel_radius
        return v_left, v_right

    # ==================================================================
    # Control loop
    # ==================================================================

    def _control_loop(self):
        now_ns = self.get_clock().now().nanoseconds
        dt_ns = now_ns - self._last_control_time_ns
        dt = max(0.001, min(0.5, dt_ns * 1e-9))
        self._last_control_time_ns = now_ns

        elapsed = (now_ns - self._last_cmd_time_ns) * 1e-9
        if elapsed > self._cmd_timeout:
            if not self._is_stopped:
                self.get_logger().warning("CMD_VEL timeout — stopping")
                self._rate_limited_v = 0.0
                self._rate_limited_w = 0.0
                self._is_stopped = True
                self._wheel_msg.data[0] = 0.0
                self._wheel_msg.data[1] = 0.0
                self._cmd_pub.publish(self._wheel_msg)
                self._publish_debug(dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
            return

        self._is_stopped = False

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

        self._compute_and_publish(dt)

    def _compute_and_publish(self, dt: float):
        v = self._rate_limited_v
        w = self._rate_limited_w

        # Kinematics: body → wheel rad/s (equal & opposite for pure rotation)
        desired_left, desired_right = self._body_to_wheel(v, w)

        # Feedforward scaling.
        #
        # For pure rotation (|v| < rotation_threshold):
        #   Use a single symmetric gain for both wheels.
        #   This guarantees |ff_left| == |ff_right| regardless of hardware asymmetry,
        #   so the robot pivots cleanly in place.
        #
        # For linear or mixed motion:
        #   Use per-wheel gains to compensate the wiring asymmetry (left port is
        #   physically slower than right port at the same command byte).
        #
        if abs(v) < self._rotation_threshold:
            ff_left = self._ff_gain_rotation * desired_left
            ff_right = self._ff_gain_rotation * desired_right
            gain_mode = 1  # symmetric
        else:
            ff_left = self._ff_gain_left * desired_left
            ff_right = self._ff_gain_right * desired_right
            gain_mode = 0  # per-wheel asymmetric

        # Clamp to physical wheel speed limit
        max_w = self._max_linear_vel / self._wheel_radius
        ff_left = max(-max_w, min(max_w, ff_left))
        ff_right = max(-max_w, min(max_w, ff_right))

        self._wheel_msg.data[0] = ff_left
        self._wheel_msg.data[1] = ff_right
        self._cmd_pub.publish(self._wheel_msg)

        self._publish_debug(
            dt,
            self._desired_v,
            self._desired_w,
            v,
            w,
            desired_left,
            desired_right,
            ff_left,
            ff_right,
            gain_mode,
        )

    def _publish_debug(
        self, dt, raw_v, raw_w, rl_v, rl_w, des_l, des_r, ff_l, ff_r, gain_mode
    ):
        d = self._debug_msg.data
        d[0] = dt
        d[1] = raw_v
        d[2] = raw_w
        d[3] = rl_v
        d[4] = rl_w
        d[5] = des_l
        d[6] = des_r
        d[7] = ff_l
        d[8] = ff_r
        d[9] = self._actual_left_vel
        d[10] = self._actual_right_vel
        d[11] = float(gain_mode)
        self._debug_pub.publish(self._debug_msg)


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
