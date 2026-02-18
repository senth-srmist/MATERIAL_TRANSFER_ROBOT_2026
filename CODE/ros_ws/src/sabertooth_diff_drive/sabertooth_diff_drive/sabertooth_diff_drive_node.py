import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 9600


class SabertoothDiffDrive(Node):
    def __init__(self):
        super().__init__("sabertooth_diff_drive")

        # Use QoS with depth=1 to always get latest command
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, qos
        )

        self.is_stopped = True

        try:
            self.motor = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to Sabertooth on {PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

    def cmd_vel_callback(self, msg):
        self.motor.reset_input_buffer()

        v = msg.linear.x
        w = msg.angular.z

        # Stop condition
        if v == 0.0 and w == 0.0:
            if not self.is_stopped:
                data = bytes([64, 192])

                # Print decimal
                print(f"SENDING (INT): [{64}, {192}]")

                # Print binary (8 bits per byte)
                binary_str = " ".join(format(b, "08b") for b in data)
                print(f"SENDING (BIN): {binary_str}")

                self.motor.write(data)
                self.motor.flush()

                self.is_stopped = True
            return

        self.is_stopped = False

        left = v - w
        right = v + w

        # Clamp input to [-1, 1] just to be safe
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # ---------------- LEFT MOTOR (0–127) ----------------
        if left >= 0:
            # Forward side (unchanged)
            left_cmd = int(64 + left * 63)
        else:
            # Reverse side (flipped magnitude)
            left_cmd = int(1 + (-left) * 62)

        left_cmd = max(1, min(127, left_cmd))

        # ---------------- RIGHT MOTOR (128–255) ----------------
        if right >= 0:
            # Forward side (unchanged)
            right_cmd = int(192 + right * 63)
        else:
            # Reverse side (flipped magnitude)
            right_cmd = int(129 + (-right) * 62)

        right_cmd = max(128, min(255, right_cmd))

        data = bytes([left_cmd, right_cmd])

        # Print decimal
        print(f"SENDING (INT): [{left_cmd}, {right_cmd}]")

        # Print binary (8 bits per byte)
        binary_str = " ".join(format(b, "08b") for b in data)
        print(f"SENDING (BIN): {binary_str}")

        try:
            self.motor.write(data)
            self.motor.flush()
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main():
    rclpy.init()
    node = SabertoothDiffDrive()
    rclpy.spin(node)

    node.motor.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
