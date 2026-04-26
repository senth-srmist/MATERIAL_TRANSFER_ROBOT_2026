#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# Robot constants
WHEEL_RADIUS = 0.05  # m
WHEELBASE = 0.4  # m

# Feedforward gain
FF_GAIN = 1.16

# PI gains — tune these
KP_V = 0.3
KI_V = 0.05
KP_W = 0.3
KI_W = 0.05

# Integral windup limit
INTEGRAL_LIMIT = 0.5


class PIDController(Node):

    def __init__(self):
        super().__init__("pid_controller")

        # PI state
        self._integral_v = 0.0
        self._integral_w = 0.0
        self._actual_v = 0.0
        self._actual_w = 0.0
        self._prev_time = self.get_clock().now()
        self._lock = threading.Lock()

        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Twist, "/cmd_vel_out", self.cmd_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            "/encoder/velocity",
            self.encoder_callback,
            best_effort_qos,
        )

        self._pub = self.create_publisher(Twist, "/cmd_vel_pid", 10)

        self.get_logger().info("PID controller node initialized.")

    def encoder_callback(self, msg: Float32MultiArray):
        left_rad_s = msg.data[2]
        right_rad_s = msg.data[3]

        with self._lock:
            self._actual_v = (left_rad_s + right_rad_s) / 2.0 * WHEEL_RADIUS
            self._actual_w = (right_rad_s -
                              left_rad_s) / WHEELBASE * WHEEL_RADIUS

    def cmd_callback(self, msg: Twist):
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Reset integrals when stopped
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self._integral_v = 0.0
            self._integral_w = 0.0

        with self._lock:
            actual_v = self._actual_v
            actual_w = self._actual_w

        # Feedforward does the heavy lifting
        ff_v = msg.linear.x * FF_GAIN
        ff_w = msg.angular.z * FF_GAIN

        # PI corrects residual error
        error_v = msg.linear.x - actual_v
        error_w = msg.angular.z - actual_w

        self._integral_v = max(
            -INTEGRAL_LIMIT,
            min(INTEGRAL_LIMIT, self._integral_v + error_v * dt))
        self._integral_w = max(
            -INTEGRAL_LIMIT,
            min(INTEGRAL_LIMIT, self._integral_w + error_w * dt))

        out = Twist()
        out.linear.x = ff_v + KP_V * error_v + KI_V * self._integral_v
        out.angular.z = ff_w + KP_W * error_w + KI_W * self._integral_w

        self._pub.publish(out)

        self.get_logger().debug(
            f"err_v:{error_v:.3f} err_w:{error_w:.3f} | "
            f"out_v:{out.linear.x:.3f} out_w:{out.angular.z:.3f}")


def main():
    rclpy.init()
    node = PIDController()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
