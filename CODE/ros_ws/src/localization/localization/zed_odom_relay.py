import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

# Diagonal indices of a flat 6x6 covariance matrix (row-major)
_DIAG6 = (0, 7, 14, 21, 28, 35)
# Diagonal indices of a flat 3x3 covariance matrix (row-major)
_DIAG3 = (0, 4, 8)

MIN_TWIST_COV = 0.1   # floor for ZED odom twist covariance
MIN_POSE_COV  = 0.5   # floor for ZED pose_with_covariance
MIN_IMU_COV   = 1e-4  # floor for ZED IMU angular velocity and linear acceleration covariance

# Use BEST_EFFORT QoS to match ZED's sensor data topics
_ZED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ZedOdomRelay(Node):
    def __init__(self):
        super().__init__("zed_odom_relay")

        self._odom_pub = self.create_publisher(Odometry, "/zed/odom_safe", 10)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/zed/pose_safe", 10)
        self._imu_pub  = self.create_publisher(Imu, "/zed/imu_safe", 10)

        self.create_subscription(Odometry, "/zed/zed_node/odom", self._odom_cb, _ZED_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/zed/zed_node/pose_with_covariance",
            self._pose_cb,
            _ZED_QOS,
        )
        self.create_subscription(Imu, "/zed/zed_node/imu/data", self._imu_cb, _ZED_QOS)

        self.get_logger().info(
            "ZED relay started — twist floor: %.3f  pose floor: %.3f  imu floor: %.2e"
            % (MIN_TWIST_COV, MIN_POSE_COV, MIN_IMU_COV)
        )

    def _odom_cb(self, msg: Odometry):
        for i in _DIAG6:
            if msg.twist.covariance[i] < MIN_TWIST_COV:
                msg.twist.covariance[i] = MIN_TWIST_COV
        self._odom_pub.publish(msg)

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        for i in _DIAG6:
            if msg.pose.covariance[i] < MIN_POSE_COV:
                msg.pose.covariance[i] = MIN_POSE_COV
        self._pose_pub.publish(msg)

    def _imu_cb(self, msg: Imu):
        for i in _DIAG3:
            if msg.angular_velocity_covariance[i] < MIN_IMU_COV:
                msg.angular_velocity_covariance[i] = MIN_IMU_COV
            if msg.linear_acceleration_covariance[i] < MIN_IMU_COV:
                msg.linear_acceleration_covariance[i] = MIN_IMU_COV
        self._imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ZedOdomRelay())
    rclpy.shutdown()
