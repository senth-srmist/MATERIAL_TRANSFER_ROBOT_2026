import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("base_length", 0.4)

        self._wheel_radius = self.get_parameter("wheel_radius").value
        self._base_length = self.get_parameter("base_length").value

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_stamp_ns = None

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._odom_pub = self.create_publisher(Odometry, "/wheel_odom", 10)
        self.create_subscription(Float32MultiArray, "/encoder/velocity", self._velocity_cb, qos)

        self.get_logger().info(
            f"Wheel odometry started — radius: {self._wheel_radius}m, base: {self._base_length}m"
        )

    def _velocity_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        stamp_sec = int(msg.data[0])
        stamp_nsec = int(msg.data[1])
        left_rad_s = float(msg.data[2])
        right_rad_s = float(msg.data[3])

        stamp_ns = stamp_sec * 1_000_000_000 + stamp_nsec

        if self._last_stamp_ns is None:
            self._last_stamp_ns = stamp_ns
            return

        dt = (stamp_ns - self._last_stamp_ns) * 1e-9
        self._last_stamp_ns = stamp_ns

        if dt <= 0.0 or dt > 0.5:
            return

        v_left = left_rad_s * self._wheel_radius
        v_right = right_rad_s * self._wheel_radius

        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self._base_length

        if abs(omega) < 1e-6:
            self._x += v * math.cos(self._theta) * dt
            self._y += v * math.sin(self._theta) * dt
        else:
            r = v / omega
            self._x += r * (math.sin(self._theta + omega * dt) - math.sin(self._theta))
            self._y += r * (math.cos(self._theta) - math.cos(self._theta + omega * dt))
        self._theta += omega * dt
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        odom = Odometry()
        odom.header.stamp.sec = stamp_sec
        odom.header.stamp.nanosec = stamp_nsec
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # Conservative covariance — wheel slip and encoder noise
        odom.pose.covariance[0] = 0.1    # x variance
        odom.pose.covariance[7] = 0.1    # y variance
        odom.pose.covariance[35] = 0.05  # yaw variance
        odom.twist.covariance[0] = 0.01  # vx variance
        odom.twist.covariance[35] = 0.05  # vyaw variance

        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
