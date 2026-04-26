#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver_raw")

        self.ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            "MotorDriver node initialized. Listening to /cmd_vel_out"
        )

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 1.0:  # 300ms timeout
            self.ser.write(bytes([0, 128]))
            self.get_logger().warn(
                "Watchdog: no cmd received, sending stop", throttle_duration_sec=1.0
            )

    def map_left(self, value):
        if value > 0:
            byte_val = int(65 + value * 62)
            byte_val = max(65, min(127, byte_val))
            return byte_val
        elif value < 0:
            byte_val = int((-value) * 63)
            byte_val = max(1, min(63, byte_val))
            return byte_val
        else:
            return 0

    def map_right(self, value):
        if value > 0:
            byte_val = int(128 + value * 63)
            byte_val = max(129, min(191, byte_val))
            return byte_val
        elif value < 0:
            byte_val = int(193 + (-value) * 62)
            byte_val = max(193, min(255, byte_val))
            return byte_val
        else:
            return 128

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive (RAW, no clamp)
        left = v - w
        right = v + w

        cmd_l = self.map_left(left)
        cmd_r = self.map_right(right)

        # Send raw bytes (no bounds check)
        self.ser.write(bytes([cmd_l, cmd_r]))

        # Debug
        self.get_logger().debug(
            f"v:{v:.2f} w:{w:.2f} | L:{left:.2f}->{cmd_l} R:{right:.2f}->{cmd_r}"
        )


def main():
    rclpy.init()
    node = MotorDriver()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
