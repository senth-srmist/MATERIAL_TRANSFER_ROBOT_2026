import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


PORT = "/dev/ttyUSB0"
BAUD = 9600


class SabertoothDiffDrive(Node):

    def __init__(self):
        super().__init__('sabertooth_diff_drive')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
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
        v = msg.linear.x
        w = msg.angular.z

        # Zero velocity - send stop only once
        if v == 0.0 and w == 0.0:
            if not self.is_stopped:
                self.motor.write(bytes([64, 192]))
                self.get_logger().info("STOP")
                self.is_stopped = True
            return

        self.is_stopped = False

        left = v - w
        right = v + w

        left_cmd = max(1, min(127, int(64 + left * 63)))
        right_cmd = max(128, min(255, int(192 + right * 63)))

        self.get_logger().info(
            f"CMD_VEL → v={v:.2f}, w={w:.2f} | "
            f"LEFT={left_cmd} RIGHT={right_cmd}"
        )

        try:
            self.motor.write(bytes([left_cmd, right_cmd]))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main():
    rclpy.init()
    node = SabertoothDiffDrive()
    rclpy.spin(node)

    node.motor.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
