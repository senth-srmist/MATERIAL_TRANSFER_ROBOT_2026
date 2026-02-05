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

        # ROS subscription
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Open serial port
        try:
            self.motor = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to Sabertooth on {PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

    def cmd_vel_callback(self, msg):
        # Extract velocities
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive equations
        left = v - w
        right = v + w

        # Convert to Sabertooth simplified serial
        left_cmd = int(64 + left * 63)
        right_cmd = int(192 + right * 63)

        # Clamp to valid ranges
        left_cmd = max(1, min(127, left_cmd))
        right_cmd = max(128, min(255, right_cmd))

        data = bytes([left_cmd, right_cmd])

        # 🔍 DEBUG LOG (this proves signals are sent)
        self.get_logger().info(
            f"CMD_VEL → v={v:.2f}, w={w:.2f} | "
            f"LEFT={left_cmd} RIGHT={right_cmd} | "
            f"BYTES={[hex(left_cmd), hex(right_cmd)]}"
        )

        # Send to Sabertooth
        try:
            self.motor.write(data)
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main():
    rclpy.init()
    node = SabertoothDiffDrive()
    rclpy.spin(node)

    # Cleanup
    node.motor.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
