import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 9600

def map_range(value, from_low, from_high, to_low, to_high):
    normalized = (value - from_low) / (from_high - to_high)
    mapped = round(to_low + normalized * (to_high - to_low))
    return max(to_low, min(to_high, mapped))

class SabertoothDiffDrive(Node):

    def __init__(self):
        super().__init__('sabertooth_diff_drive')
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.motor = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        self.get_logger().info("Sabertooth Differential Drive Node Running")

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        left = v - w
        right = v + w

        left_cmd = int(64 + left * 63)
        right_cmd = int(192 + right * 63)

        left_cmd = max(1, min(127, left_cmd))
        right_cmd = max(128, min(255, right_cmd))

        self.motor.write(bytes([left_cmd, right_cmd]))

def main():
    rclpy.init()
    node = SabertoothDiffDrive()
    rclpy.spin(node)
    node.motor.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
