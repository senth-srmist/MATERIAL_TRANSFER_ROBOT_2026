#!/usr/bin/env python3
"""
Encoder Driver Node

Reads quadrature encoders via Jetson GPIO interrupts and publishes
wheel tick counts and velocities.

Publishes:
  /encoder/ticks    (Int32MultiArray) — [left_ticks, right_ticks] raw counts
  /encoder/velocity (Float32MultiArray) — [stamp_sec, stamp_nsec, left_rad_s, right_rad_s]

Interrupt strategy:
  Channel A only (BOTH edges), read B for direction.
  204 interrupts per revolution — half the load of full quadrature.
  Resolution is still sufficient for velocity estimation at 15-50Hz.

Hardware:
  Left wheel:  board pins 19 (ch A), 21 (ch B)
  Right wheel: board pins 24 (ch A), 22 (ch B)
  Ticks per revolution: 204 (channel A, both edges)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from std_msgs.msg import Int32MultiArray, Float32MultiArray

try:
    import Jetson.GPIO as GPIO

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class EncoderDriver(Node):

    def __init__(self):
        super().__init__("encoder_driver")

        # Load and validate params
        self._declare_params()
        self._load_params()
        self._validate_params()

        # Ticks to radians
        self._ticks_to_rad = (2.0 * math.pi) / self._ticks_per_rev

        # Max physically possible ticks per publish cycle
        # Used to filter impossible tick jumps from noise/EMI
        self._max_wheel_rad_s = 20.0  # generous upper bound
        self._max_ticks_per_sec = self._max_wheel_rad_s / self._ticks_to_rad

        # Tick counters (updated by ISR, read by timer)
        self._right_ticks = 0
        self._left_ticks = 0

        # Previous values for velocity computation
        self._prev_right_ticks = 0
        self._prev_left_ticks = 0
        self._prev_time = time.monotonic()

        # Moving average buffers
        self._right_vel_buf = []
        self._left_vel_buf = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Publishers
        self._ticks_pub = self.create_publisher(Int32MultiArray,
                                                "/encoder/ticks", qos)
        self._vel_pub = self.create_publisher(Float32MultiArray,
                                              "/encoder/velocity", qos)

        # Setup GPIO
        if not GPIO_AVAILABLE:
            self.get_logger().fatal("Jetson.GPIO not available")
            raise SystemExit(1)

        self._setup_gpio()

        # Publish timer
        period = 1.0 / self._publish_rate
        self.create_timer(period, self._publish_callback)

        self.get_logger().info(
            f"Encoder driver started — "
            f"L pins: {self._left_pin_a}/{self._left_pin_b}, "
            f"R pins: {self._right_pin_a}/{self._right_pin_b}, "
            f"TPR: {self._ticks_per_rev}, "
            f"rate: {self._publish_rate}Hz")

    # ==================================================================
    # Parameter handling
    # ==================================================================

    def _declare_params(self):
        self.declare_parameter("right_encoder_pin_a", Parameter.Type.INTEGER)
        self.declare_parameter("right_encoder_pin_b", Parameter.Type.INTEGER)
        self.declare_parameter("left_encoder_pin_a", Parameter.Type.INTEGER)
        self.declare_parameter("left_encoder_pin_b", Parameter.Type.INTEGER)
        self.declare_parameter("ticks_per_revolution", Parameter.Type.INTEGER)
        self.declare_parameter("wheel_radius", Parameter.Type.DOUBLE)
        self.declare_parameter("publish_rate", Parameter.Type.DOUBLE)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("velocity_window", 5)

    def _load_params(self):
        self._right_pin_a = self.get_parameter("right_encoder_pin_a").value
        self._right_pin_b = self.get_parameter("right_encoder_pin_b").value
        self._left_pin_a = self.get_parameter("left_encoder_pin_a").value
        self._left_pin_b = self.get_parameter("left_encoder_pin_b").value
        self._ticks_per_rev = self.get_parameter("ticks_per_revolution").value
        self._wheel_radius = self.get_parameter("wheel_radius").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._invert_right = self.get_parameter("invert_right").value
        self._invert_left = self.get_parameter("invert_left").value
        self._velocity_window = self.get_parameter("velocity_window").value

    def _validate_params(self):
        errors = []

        for name in (
                "right_encoder_pin_a",
                "right_encoder_pin_b",
                "left_encoder_pin_a",
                "left_encoder_pin_b",
        ):
            if self.get_parameter(name).value is None:
                errors.append(f"{name} not set")

        if self._ticks_per_rev is None or self._ticks_per_rev <= 0:
            errors.append(
                f"ticks_per_revolution must be > 0 (got {self._ticks_per_rev})"
            )

        if self._wheel_radius is None or self._wheel_radius <= 0:
            errors.append(
                f"wheel_radius must be > 0 (got {self._wheel_radius})")

        if self._publish_rate is None or self._publish_rate <= 0:
            errors.append(
                f"publish_rate must be > 0 (got {self._publish_rate})")

        if errors:
            for e in errors:
                self.get_logger().fatal(f"Config error: {e}")
            raise SystemExit(1)

    # ==================================================================
    # GPIO setup
    # ==================================================================

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        pins = [
            self._right_pin_a,
            self._right_pin_b,
            self._left_pin_a,
            self._left_pin_b,
        ]
        for pin in pins:
            GPIO.setup(pin, GPIO.IN)

        # Channel A only — BOTH edges, read B for direction
        # 204 interrupts per revolution (half of full quadrature)
        GPIO.add_event_detect(
            self._right_pin_a,
            GPIO.BOTH,
            callback=self._right_a_callback,
        )
        GPIO.add_event_detect(
            self._left_pin_a,
            GPIO.BOTH,
            callback=self._left_a_callback,
        )

        self.get_logger().info(
            "GPIO interrupts configured (channel A only, both edges)")

    # ==================================================================
    # Interrupt handlers — MINIMAL
    #
    # Channel A edge only, read B for direction:
    #   A edge: if A == B -> forward, else -> reverse
    # ==================================================================

    def _right_a_callback(self, channel):
        a = GPIO.input(self._right_pin_a)
        b = GPIO.input(self._right_pin_b)
        if a == b:
            self._right_ticks += 1
        else:
            self._right_ticks -= 1

    def _left_a_callback(self, channel):
        a = GPIO.input(self._left_pin_a)
        b = GPIO.input(self._left_pin_b)
        if a == b:
            self._left_ticks += 1
        else:
            self._left_ticks -= 1

    # ==================================================================
    # Publish callback
    # ==================================================================

    def _publish_callback(self):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()

        # Snapshot ticks
        right_ticks = self._right_ticks
        left_ticks = self._left_ticks

        # Apply inversion
        if self._invert_right:
            right_ticks = -right_ticks
        if self._invert_left:
            left_ticks = -left_ticks

        # Compute dt
        dt = now - self._prev_time
        if dt < 1e-6:
            return
        self._prev_time = now

        # Tick deltas
        d_right = right_ticks - self._prev_right_ticks
        d_left = left_ticks - self._prev_left_ticks
        self._prev_right_ticks = right_ticks
        self._prev_left_ticks = left_ticks

        # Impossible tick filter — reject EMI/noise spikes
        max_ticks = self._max_ticks_per_sec * dt
        if abs(d_right) > max_ticks:
            self.get_logger().warn(
                f"Right encoder spike filtered: {d_right} ticks in {dt:.3f}s")
            d_right = 0
        if abs(d_left) > max_ticks:
            self.get_logger().warn(
                f"Left encoder spike filtered: {d_left} ticks in {dt:.3f}s")
            d_left = 0

        # Raw velocity in rad/s
        right_vel = (d_right * self._ticks_to_rad) / dt
        left_vel = (d_left * self._ticks_to_rad) / dt

        # Moving average filter
        self._right_vel_buf.append(right_vel)
        self._left_vel_buf.append(left_vel)
        if len(self._right_vel_buf) > self._velocity_window:
            self._right_vel_buf.pop(0)
        if len(self._left_vel_buf) > self._velocity_window:
            self._left_vel_buf.pop(0)

        right_vel_filtered = sum(self._right_vel_buf) / len(
            self._right_vel_buf)
        left_vel_filtered = sum(self._left_vel_buf) / len(self._left_vel_buf)

        # Publish ticks
        ticks_msg = Int32MultiArray()
        ticks_msg.data = [left_ticks, right_ticks]
        self._ticks_pub.publish(ticks_msg)

        # Publish velocity with timestamp
        # data: [stamp_sec, stamp_nsec, left_rad_s, right_rad_s]
        vel_msg = Float32MultiArray()
        vel_msg.data = [
            float(stamp.sec),
            float(stamp.nanosec),
            float(left_vel_filtered),
            float(right_vel_filtered),
        ]
        self._vel_pub.publish(vel_msg)

    # ==================================================================
    # Cleanup
    # ==================================================================

    def destroy_node(self):
        self.get_logger().info("Cleaning up GPIO...")
        try:
            GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()


# ======================================================================
# Main
# ======================================================================


def main(args=None):
    rclpy.init(args=args)
    node = EncoderDriver()
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
