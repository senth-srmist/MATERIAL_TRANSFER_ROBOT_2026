import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

import serial
import math

# ================= SERIAL CONFIG =================
PORT = "/dev/ttyUSB0"
BAUD = 9600

# ================= ROBOT GEOMETRY =================
WHEEL_RADIUS = 0.05  # meters
BASE_LENGTH = 0.4  # meters

# ================= VELOCITY LIMITS =================
MIN_LINEAR_VEL = -0.2
MAX_LINEAR_VEL = 0.78
MIN_ANGULAR_VEL = -2.0
MAX_ANGULAR_VEL = 2.0

# ================= DYNAMICS LIMITS =================
MAX_LINEAR_ACCEL = 0.3
MAX_LINEAR_DECEL = 0.6
MAX_ANGULAR_ACCEL = 1.5

# ================= CONTROL =================
CONTROL_DT = 0.1
CMD_TIMEOUT = 0.5

# ================= MOTOR SCALING =================
MAX_WHEEL_RAD_S = 10.0


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Twist, "/cmd_vel_out", self.cmd_vel_callback, qos)

        self.control_timer = self.create_timer(CONTROL_DT, self.control_loop)

        # Desired command (latest received)
        self.v_target = 0.0
        self.w_target = 0.0

        # Actual applied command (rate limited)
        self.v_current = 0.0
        self.w_current = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_control_time = self.get_clock().now()

        self.is_stopped = True

        try:
            self.motor = serial.Serial(PORT, BAUD, timeout=1)
            self.get_logger().info(f"Connected to Sabertooth on {PORT}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            raise

    # ================= CMD_VEL CALLBACK =================
    def cmd_vel_callback(self, msg: Twist):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.get_logger().warn("Received invalid cmd_vel (NaN or Inf). Ignoring.")
            return

        self.last_cmd_time = self.get_clock().now()

        v = max(MIN_LINEAR_VEL, min(MAX_LINEAR_VEL, msg.linear.x))
        w = max(MIN_ANGULAR_VEL, min(MAX_ANGULAR_VEL, msg.angular.z))

        self.v_target = v
        self.w_target = w

    # ================= RATE LIMITER =================
    def limit_rate(self, target, current, accel_limit, dt):
        max_delta = accel_limit * dt
        delta = target - current
        delta = max(-max_delta, min(max_delta, delta))
        return current + delta

    # ================= CONTROL LOOP =================
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now

        # ================= WATCHDOG =================
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9

        if elapsed > CMD_TIMEOUT:
            if not self.is_stopped:
                self.get_logger().warn("CMD_VEL timeout. Stopping robot.")
                self.send_stop()
            return

        # ================= RATE LIMITING =================
        if abs(self.v_target) > abs(self.v_current):
            self.v_current = self.limit_rate(
                self.v_target, self.v_current, MAX_LINEAR_ACCEL, dt
            )
        else:
            self.v_current = self.limit_rate(
                self.v_target, self.v_current, MAX_LINEAR_DECEL, dt
            )

        self.w_current = self.limit_rate(
            self.w_target, self.w_current, MAX_ANGULAR_ACCEL, dt
        )

        # ================= STOP CONDITION =================
        if abs(self.v_current) < 1e-3 and abs(self.w_current) < 1e-3:
            if not self.is_stopped:
                self.send_stop()
            return

        self.is_stopped = False

        # ================= KINEMATICS =================
        v_r = self.v_current + (BASE_LENGTH / 2.0) * self.w_current
        v_l = self.v_current - (BASE_LENGTH / 2.0) * self.w_current

        omega_r = v_r / WHEEL_RADIUS
        omega_l = v_l / WHEEL_RADIUS

        # ================= NORMALIZATION =================
        right = max(-1.0, min(1.0, omega_r / MAX_WHEEL_RAD_S))
        left = max(-1.0, min(1.0, omega_l / MAX_WHEEL_RAD_S))

        left_cmd = self.scale_motor_command(left, left_motor=True)
        right_cmd = self.scale_motor_command(right, left_motor=False)

        self.send_serial(left_cmd, right_cmd)

        self.get_logger().debug(
            f"v={self.v_current:.2f} w={self.w_current:.2f} | L={left_cmd} R={right_cmd}"
        )

    # ================= MOTOR SCALING =================
    def scale_motor_command(self, value, left_motor=True):
        if left_motor:
            if value >= 0:
                cmd = int(64 + value * 63)
            else:
                cmd = int(1 + (-value) * 62)
            return max(1, min(127, cmd))
        else:
            if value >= 0:
                cmd = int(192 + value * 63)
            else:
                cmd = int(129 + (-value) * 62)
            return max(128, min(255, cmd))

    # ================= SERIAL SEND =================
    def send_serial(self, left_cmd, right_cmd):
        try:
            self.motor.write(bytes([left_cmd, right_cmd]))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            self.send_stop()

    def send_stop(self):
        try:
            self.motor.write(bytes([64, 192]))
        except Exception as e:
            self.get_logger().error(f"Failed to send STOP: {e}")

        self.v_current = 0.0
        self.w_current = 0.0
        self.v_target = 0.0
        self.w_target = 0.0
        self.is_stopped = True


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)

    node.motor.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
