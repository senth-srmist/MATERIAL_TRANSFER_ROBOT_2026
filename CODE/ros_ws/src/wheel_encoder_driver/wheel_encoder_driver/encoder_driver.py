#!/usr/bin/env python3
"""
Encoder Driver Node (v2 - Optimized)

Reads quadrature encoders via Jetson GPIO interrupts and publishes
wheel tick counts and velocities.

Optimizations from v1:
  - collections.deque for O(1) moving average (was O(n) list.pop(0))
  - Pre-allocated messages (reused each publish)
  - Thread-safe tick access via simple locking
  - Cached constants

Publishes:
  /encoder/ticks    (Int32MultiArray) — [left_ticks, right_ticks] raw counts
  /encoder/velocity (Float32MultiArray) — [stamp_sec, stamp_nsec, left_rad_s, right_rad_s]

Interrupt strategy:
  Channel A only (BOTH edges), read B for direction.
  204 interrupts per revolution — half the load of full quadrature.

Hardware:
  Left wheel:  board pins 19 (ch A), 21 (ch B)
  Right wheel: board pins 24 (ch A), 22 (ch B)
  Ticks per revolution: 204 (channel A, both edges)
"""

import math
import threading
from collections import deque

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

        # Cached constants
        self._ticks_to_rad = (2.0 * math.pi) / self._ticks_per_rev
        self._max_wheel_rad_s = 20.0  # Generous upper bound
        self._max_ticks_per_sec = self._max_wheel_rad_s / self._ticks_to_rad

        # Tick counters (updated by ISR, read by timer)
        # Use simple lock for thread safety
        self._tick_lock = threading.Lock()
        self._right_ticks = 0
        self._left_ticks = 0

        # Previous values for velocity computation
        self._prev_right_ticks = 0
        self._prev_left_ticks = 0
        self._prev_time_ns = self.get_clock().now().nanoseconds

        # Moving average buffers using deque for O(1) operations
        self._right_vel_buf = deque(maxlen=self._velocity_window)
        self._left_vel_buf = deque(maxlen=self._velocity_window)

        # Pre-allocated messages (reused each publish)
        self._ticks_msg = Int32MultiArray()
        self._ticks_msg.data = [0, 0]  # Pre-size
        self._vel_msg = Float32MultiArray()
        self._vel_msg.data = [0.0, 0.0, 0.0, 0.0]  # Pre-size

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Publishers
        self._ticks_pub = self.create_publisher(Int32MultiArray, "/encoder/ticks", qos)
        self._vel_pub = self.create_publisher(
            Float32MultiArray, "/encoder/velocity", qos
        )

        # Setup GPIO
        if not GPIO_AVAILABLE:
            self.get_logger().fatal("Jetson.GPIO not available")
            raise SystemExit(1)

        self._setup_gpio()

        # Publish timer
        period = 1.0 / self._publish_rate
        self.create_timer(period, self._publish_callback)

        self.get_logger().info(
            "Encoder driver v2 started — L:%d/%d R:%d/%d TPR:%d rate:%.0fHz",
            self._left_pin_a,
            self._left_pin_b,
            self._right_pin_a,
            self._right_pin_b,
            self._ticks_per_rev,
            self._publish_rate,
        )

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
        self.declare_parameter("min_velocity_threshold", 0.04)
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
        self._min_vel_threshold = self.get_parameter("min_velocity_threshold").value
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
            errors.append(f"wheel_radius must be > 0 (got {self._wheel_radius})")

        if self._publish_rate is None or self._publish_rate <= 0:
            errors.append(f"publish_rate must be > 0 (got {self._publish_rate})")

        if errors:
            for e in errors:
                self.get_logger().fatal("Config error: %s", e)
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

        self.get_logger().info("GPIO interrupts configured (channel A, both edges)")

    # ==================================================================
    # Interrupt handlers — MINIMAL, thread-safe
    # ==================================================================

    def _right_a_callback(self, channel):
        a = GPIO.input(self._right_pin_a)
        b = GPIO.input(self._right_pin_b)
        with self._tick_lock:
            if a == b:
                self._right_ticks += 1
            else:
                self._right_ticks -= 1

    def _left_a_callback(self, channel):
        a = GPIO.input(self._left_pin_a)
        b = GPIO.input(self._left_pin_b)
        with self._tick_lock:
            if a == b:
                self._left_ticks += 1
            else:
                self._left_ticks -= 1

    # ==================================================================
    # Publish callback
    # ==================================================================

    def _publish_callback(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        stamp = now.to_msg()

        # Snapshot ticks atomically
        with self._tick_lock:
            right_ticks = self._right_ticks
            left_ticks = self._left_ticks

        # Apply inversion
        if self._invert_right:
            right_ticks = -right_ticks
        if self._invert_left:
            left_ticks = -left_ticks

        # Compute dt in seconds
        dt_ns = now_ns - self._prev_time_ns
        if dt_ns < 1000:  # < 1 microsecond
            return
        dt = dt_ns * 1e-9
        self._prev_time_ns = now_ns

        # Tick deltas
        d_right = right_ticks - self._prev_right_ticks
        d_left = left_ticks - self._prev_left_ticks
        self._prev_right_ticks = right_ticks
        self._prev_left_ticks = left_ticks

        # Impossible tick filter — reject EMI/noise spikes
        max_ticks = self._max_ticks_per_sec * dt
        if abs(d_right) > max_ticks:
            self.get_logger().warning(
                "Right encoder spike: %d ticks in %.3fs", d_right, dt
            )
            d_right = 0
        if abs(d_left) > max_ticks:
            self.get_logger().warning(
                "Left encoder spike: %d ticks in %.3fs", d_left, dt
            )
            d_left = 0

        # Raw velocity in rad/s
        right_vel = (d_right * self._ticks_to_rad) / dt
        left_vel = (d_left * self._ticks_to_rad) / dt

        # Moving average filter (deque auto-evicts oldest)
        self._right_vel_buf.append(right_vel)
        self._left_vel_buf.append(left_vel)

        # Compute filtered velocities
        right_vel_filtered = sum(self._right_vel_buf) / len(self._right_vel_buf)
        left_vel_filtered = sum(self._left_vel_buf) / len(self._left_vel_buf)

        # Zero out noise
        if abs(right_vel_filtered) < self._min_vel_threshold:
            right_vel_filtered = 0.0
        if abs(left_vel_filtered) < self._min_vel_threshold:
            left_vel_filtered = 0.0

        # Publish ticks (reuse message)
        self._ticks_msg.data[0] = left_ticks
        self._ticks_msg.data[1] = right_ticks
        self._ticks_pub.publish(self._ticks_msg)

        # Publish velocity with timestamp (reuse message)
        self._vel_msg.data[0] = float(stamp.sec)
        self._vel_msg.data[1] = float(stamp.nanosec)
        self._vel_msg.data[2] = float(left_vel_filtered)
        self._vel_msg.data[3] = float(right_vel_filtered)
        self._vel_pub.publish(self._vel_msg)

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
