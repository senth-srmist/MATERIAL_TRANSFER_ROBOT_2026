import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

import serial
import time
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
CONTROL_DT = 0.1  # 10 Hz
CMD_TIMEOUT = 0.5  # 100 ms watchdog

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

        # Watchdog timer
        self.timer = self.create_timer(CONTROL_DT, self.watchdog_callback)

        self.v_prev = 0.0
        self.w_prev = 0.0
        self.is_stopped = True

        self.last_cmd_time = self.get_clock().now()

        try:
            self.motor = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to Sabertooth on {PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

    # ================= RATE LIMITER =================
    def limit_rate(self, target, prev, accel_limit):
        max_delta = accel_limit * CONTROL_DT
        delta = target - prev
        delta = max(-max_delta, min(max_delta, delta))
        return prev + delta

    # ================= WATCHDOG =================
    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9

        if elapsed > CMD_TIMEOUT and not self.is_stopped:
            self.get_logger().warn("CMD_VEL timeout! Stopping robot.")

            data = bytes([64, 192])  # STOP
            try:
                self.motor.write(data)
                self.motor.flush()
            except Exception as e:
                self.get_logger().error(f"Serial write failed: {e}")

            self.v_prev = 0.0
            self.w_prev = 0.0
            self.is_stopped = True

    # ================= CMD_VEL CALLBACK =================
    def cmd_vel_callback(self, msg):
        self.motor.reset_input_buffer()

        # Update watchdog timestamp
        self.last_cmd_time = self.get_clock().now()

        v_cmd = msg.linear.x
        w_cmd = msg.angular.z

        # Clamp velocities
        v_cmd = max(MIN_LINEAR_VEL, min(MAX_LINEAR_VEL, v_cmd))
        w_cmd = max(MIN_ANGULAR_VEL, min(MAX_ANGULAR_VEL, w_cmd))

        # Apply dynamics
        if abs(v_cmd) > abs(self.v_prev):
            v = self.limit_rate(v_cmd, self.v_prev, MAX_LINEAR_ACCEL)
        else:
            v = self.limit_rate(v_cmd, self.v_prev, MAX_LINEAR_DECEL)

        w = self.limit_rate(w_cmd, self.w_prev, MAX_ANGULAR_ACCEL)

        self.v_prev = v
        self.w_prev = w

        # Stop condition
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            if not self.is_stopped:
                data = bytes([64, 192])
                self.motor.write(data)
                self.motor.flush()
                self.is_stopped = True
            return

        self.is_stopped = False

        # ================= KINEMATICS =================
        v_r = v + (BASE_LENGTH / 2.0) * w
        v_l = v - (BASE_LENGTH / 2.0) * w

        omega_r = v_r / WHEEL_RADIUS
        omega_l = v_l / WHEEL_RADIUS

        # ================= NORMALIZATION =================
        right = omega_r / MAX_WHEEL_RAD_S
        left = omega_l / MAX_WHEEL_RAD_S

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # ================= SABERTOOTH SERIAL =================
        if left >= 0:
            left_cmd = int(64 + left * 63)
        else:
            left_cmd = int(1 + (-left) * 62)
        left_cmd = max(1, min(127, left_cmd))

        if right >= 0:
            right_cmd = int(192 + right * 63)
        else:
            right_cmd = int(129 + (-right) * 62)
        right_cmd = max(128, min(255, right_cmd))

        data = bytes([left_cmd, right_cmd])

        print(f"v={v:.2f} m/s  w={w:.2f} rad/s | L={left_cmd} R={right_cmd}")

        try:
            self.motor.write(data)
            self.motor.flush()
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)

    node.motor.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
