#!/usr/bin/env python3
"""
Encoder Driver Node

Reads quadrature encoders via Jetson GPIO interrupts and publishes
wheel tick counts and velocities.

Publishes:
  /encoder/ticks    (Int32MultiArray) — [left_ticks, right_ticks] raw counts
  /encoder/velocity (Float32MultiArray) — [left_rad_s, right_rad_s] wheel angular velocities

GPIO interrupt handlers are kept minimal (increment/decrement only).
Velocity is computed in the publish timer from tick deltas.

Hardware:
  Right wheel: board pins 19 (ch A), 21 (ch B)
  Left wheel:  board pins 22 (ch A), 24 (ch B)
  Full quadrature decoding: 408 ticks per revolution
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray

try:
    import Jetson.GPIO as GPIO

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

REQUIRED_PARAMS = [
    "right_encoder_pin_a",
    "right_encoder_pin_b",
    "left_encoder_pin_a",
    "left_encoder_pin_b",
    "ticks_per_revolution",
    "wheel_radius",
    "publish_rate",
]


class EncoderDriver(Node):

    def __init__(self):
        super().__init__("encoder_driver")

        # Load params
        self._declare_params()
        if not self._validate_params():
            self.get_logger().fatal(
                "Missing required parameters. Shutting down.")
            raise SystemExit(1)

        self._right_pin_a = self._p("right_encoder_pin_a")
        self._right_pin_b = self._p("right_encoder_pin_b")
        self._left_pin_a = self._p("left_encoder_pin_a")
        self._left_pin_b = self._p("left_encoder_pin_b")
        self._ticks_per_rev = self._p("ticks_per_revolution")
        self._wheel_radius = self._p("wheel_radius")
        self._publish_rate = self._p("publish_rate")
        self._invert_right = self._p("invert_right") or False
        self._invert_left = self._p("invert_left") or False
        self._velocity_window = self._p("velocity_window") or 5

        # Ticks to radians
        self._ticks_to_rad = (2.0 * math.pi) / self._ticks_per_rev

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
            f"R pins: {self._right_pin_a}/{self._right_pin_b}, "
            f"L pins: {self._left_pin_a}/{self._left_pin_b}, "
            f"CPR: {self._ticks_per_rev}, "
            f"rate: {self._publish_rate}Hz")

    # ==================================================================
    # Parameter handling
    # ==================================================================

    def _declare_params(self):
        for name in REQUIRED_PARAMS:
            if not self.has_parameter(name):
                self.declare_parameter(name)
        if not self.has_parameter("invert_right"):
            self.declare_parameter("invert_right", False)
        if not self.has_parameter("invert_left"):
            self.declare_parameter("invert_left", False)
        if not self.has_parameter("velocity_window"):
            self.declare_parameter("velocity_window", 5)

    def _validate_params(self):
        missing = []
        for name in REQUIRED_PARAMS:
            if self.get_parameter(name).value is None:
                missing.append(name)
        if missing:
            self.get_logger().fatal(f"Missing parameters: {missing}")
            return False
        return True

    def _p(self, name):
        return self.get_parameter(name).value

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

        # Right encoder — interrupt on both channels, both edges
        GPIO.add_event_detect(
            self._right_pin_a,
            GPIO.BOTH,
            callback=self._right_a_callback,
        )
        GPIO.add_event_detect(
            self._right_pin_b,
            GPIO.BOTH,
            callback=self._right_b_callback,
        )

        # Left encoder — interrupt on both channels, both edges
        GPIO.add_event_detect(
            self._left_pin_a,
            GPIO.BOTH,
            callback=self._left_a_callback,
        )
        GPIO.add_event_detect(
            self._left_pin_b,
            GPIO.BOTH,
            callback=self._left_b_callback,
        )

        self.get_logger().info("GPIO interrupts configured")

    # ==================================================================
    # Interrupt handlers — KEEP THESE MINIMAL
    #
    # Full quadrature decoding:
    #   Channel A edge: if A == B → forward, else → reverse
    #   Channel B edge: if A == B → reverse, else → forward
    # ==================================================================

    def _right_a_callback(self, channel):
        a = GPIO.input(self._right_pin_a)
        b = GPIO.input(self._right_pin_b)
        if a == b:
            self._right_ticks += 1
        else:
            self._right_ticks -= 1

    def _right_b_callback(self, channel):
        a = GPIO.input(self._right_pin_a)
        b = GPIO.input(self._right_pin_b)
        if a == b:
            self._right_ticks -= 1
        else:
            self._right_ticks += 1

    def _left_a_callback(self, channel):
        a = GPIO.input(self._left_pin_a)
        b = GPIO.input(self._left_pin_b)
        if a == b:
            self._left_ticks += 1
        else:
            self._left_ticks -= 1

    def _left_b_callback(self, channel):
        a = GPIO.input(self._left_pin_a)
        b = GPIO.input(self._left_pin_b)
        if a == b:
            self._left_ticks -= 1
        else:
            self._left_ticks += 1

    # ==================================================================
    # Publish callback
    # ==================================================================

    def _publish_callback(self):
        now = time.monotonic()

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

        # Publish velocity
        vel_msg = Float32MultiArray()
        vel_msg.data = [
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
