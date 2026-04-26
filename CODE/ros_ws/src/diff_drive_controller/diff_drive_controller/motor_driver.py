#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading


class MotorDriver(Node):

    def __init__(self):
        super().__init__("motor_driver_raw")

        self.ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
        self.v_lim = 0.5
        self.w_lim = 1.2
        self.feedforward_gain = 1.16

        self._watchdog = None

        self.create_subscription(Twist, "/cmd_vel_out", self.cmd_callback, 10)
        self._reset_watchdog()

        self.get_logger().info(
            "MotorDriver node initialized. Listening to /cmd_vel_out")

    def _reset_watchdog(self):
        if self._watchdog is not None:
            self._watchdog.cancel()
        self._watchdog = threading.Timer(0.3, self._stop)
        self._watchdog.daemon = True
        self._watchdog.start()

    def _stop(self):
        self.ser.write(bytes([0, 128]))
        self.get_logger().warn("Watchdog timeout: sending stop")

    def feedforward(self, v, w):
        return v * self.feedforward_gain, w * self.feedforward_gain

    def normalize(self, v, w):
        v_norm = max(-1.0, min(1.0, v / self.v_lim))
        w_norm = max(-1.0, min(1.0, w / self.w_lim))
        return v_norm, w_norm

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
        self._reset_watchdog()

        v_ff, w_ff = self.feedforward(msg.linear.x, msg.angular.z)
        v_norm, w_norm = self.normalize(v_ff, w_ff)

        left = v_norm - w_norm
        right = v_norm + w_norm

        cmd_l = self.map_left(left)
        cmd_r = self.map_right(right)

        self.ser.write(bytes([cmd_l, cmd_r]))

        self.get_logger().debug(
            f"v:{v_norm:.2f} w:{w_norm:.2f} | L:{left:.2f}->{cmd_l} R:{right:.2f}->{cmd_r}"
        )


def main():
    rclpy.init()
    node = MotorDriver()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
