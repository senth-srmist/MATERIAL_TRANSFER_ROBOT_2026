#!/usr/bin/env python3
"""
Nav2 Tuning Metrics Node

Subscribes to raw navigation topics and computes derived metrics
for live visualization in PlotJuggler/Foxglove.

Publishes all metrics on /metrics/* topics as Float32MultiArray.

Topic Layout:
  /metrics/velocity_tracking (8 floats):
    [0] nav2_cmd_v       — what RPP commands
    [1] nav2_cmd_w
    [2] pid_output_v     — what PID sends to motors
    [3] pid_output_w
    [4] odom_actual_v    — what encoders report
    [5] odom_actual_w
    [6] linear_error     — nav2_cmd_v - odom_actual_v (full chain error)
    [7] angular_error    — nav2_cmd_w - odom_actual_w

  /metrics/path_tracking (4 floats):
    [0] lateral_deviation    — perpendicular distance from planned path
    [1] heading_error        — robot heading vs path tangent (rad)
    [2] distance_to_goal     — euclidean distance to current goal
    [3] path_completion_pct  — % of path traversed

  /metrics/test_status (4 floats):
    [0] test_active       — 1.0 if a test is running, 0.0 otherwise
    [1] test_level        — current test level (1-4)
    [2] elapsed_time      — seconds since test started
    [3] scenario_id       — numeric ID of current scenario
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32MultiArray, String


class MetricsNode(Node):
    def __init__(self):
        super().__init__("tuning_metrics")

        # State
        self._nav2_cmd_v = 0.0
        self._nav2_cmd_w = 0.0
        self._pid_out_v = 0.0
        self._pid_out_w = 0.0
        self._odom_v = 0.0
        self._odom_w = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        # Path tracking
        self._plan_points = []
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._has_plan = False
        self._has_goal = False

        # Test status
        self._test_active = False
        self._test_level = 0
        self._test_start_time = 0.0
        self._scenario_id = 0

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.create_subscription(
            Twist, "/cmd_vel_nav2", self._nav2_cmd_cb, reliable_qos
        )
        self.create_subscription(
            Twist, "/cmd_vel_pid", self._pid_out_cb, reliable_qos
        )
        self.create_subscription(
            Odometry, "/odom/base_link", self._odom_cb, best_effort_qos
        )
        self.create_subscription(Path, "/plan", self._plan_cb, reliable_qos)
        self.create_subscription(
            String, "/test/active", self._test_status_cb, reliable_qos
        )

        # Publishers
        self._vel_track_pub = self.create_publisher(
            Float32MultiArray, "/metrics/velocity_tracking", best_effort_qos
        )
        self._path_track_pub = self.create_publisher(
            Float32MultiArray, "/metrics/path_tracking", best_effort_qos
        )
        self._test_status_pub = self.create_publisher(
            Float32MultiArray, "/metrics/test_status", best_effort_qos
        )

        # Pre-allocate messages
        self._vel_msg = Float32MultiArray()
        self._vel_msg.data = [0.0] * 8
        self._path_msg = Float32MultiArray()
        self._path_msg.data = [0.0] * 4
        self._status_msg = Float32MultiArray()
        self._status_msg.data = [0.0] * 4

        # Publish at 10Hz
        self.create_timer(0.1, self._publish_metrics)

        self.get_logger().info("Tuning metrics node started")

    # ==================================================================
    # Subscribers
    # ==================================================================

    def _nav2_cmd_cb(self, msg: Twist):
        self._nav2_cmd_v = msg.linear.x
        self._nav2_cmd_w = msg.angular.z

    def _pid_out_cb(self, msg: Twist):
        self._pid_out_v = msg.linear.x
        self._pid_out_w = msg.angular.z

    def _odom_cb(self, msg: Odometry):
        self._odom_v = msg.twist.twist.linear.x
        self._odom_w = msg.twist.twist.angular.z
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _plan_cb(self, msg: Path):
        if len(msg.poses) < 2:
            self._has_plan = False
            return
        self._plan_points = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._has_plan = True
        last = msg.poses[-1]
        self._goal_x = last.pose.position.x
        self._goal_y = last.pose.position.y
        self._has_goal = True

    def _test_status_cb(self, msg: String):
        """Parse: 'START:L1:straight:1' or 'STOP'."""
        text = msg.data.strip()
        if text.startswith("START:"):
            parts = text.split(":")
            self._test_active = True
            self._test_level = int(parts[1].replace("L", "")) if len(parts) > 1 else 0
            self._scenario_id = int(parts[3]) if len(parts) > 3 else 0
            self._test_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(f"Test started: {text}")
        elif text == "STOP":
            self._test_active = False
            self.get_logger().info("Test stopped")

    # ==================================================================
    # Path tracking computation
    # ==================================================================

    def _compute_lateral_deviation(self) -> float:
        if not self._has_plan or len(self._plan_points) < 2:
            return 0.0

        rx, ry = self._odom_x, self._odom_y
        min_dist = float("inf")

        for i in range(len(self._plan_points) - 1):
            ax, ay = self._plan_points[i]
            bx, by = self._plan_points[i + 1]
            abx, aby = bx - ax, by - ay
            apx, apy = rx - ax, ry - ay
            ab_sq = abx * abx + aby * aby

            if ab_sq < 1e-12:
                dist = math.sqrt(apx * apx + apy * apy)
            else:
                t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
                proj_x = ax + t * abx
                proj_y = ay + t * aby
                dx, dy = rx - proj_x, ry - proj_y
                dist = math.sqrt(dx * dx + dy * dy)

            if dist < min_dist:
                min_dist = dist

        return min_dist if min_dist < float("inf") else 0.0

    def _compute_heading_error(self) -> float:
        if not self._has_plan or len(self._plan_points) < 2:
            return 0.0

        rx, ry = self._odom_x, self._odom_y
        min_dist = float("inf")
        nearest_idx = 0
        for i in range(len(self._plan_points) - 1):
            ax, ay = self._plan_points[i]
            dist = (rx - ax) ** 2 + (ry - ay) ** 2
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        ax, ay = self._plan_points[nearest_idx]
        bx, by = self._plan_points[min(nearest_idx + 1, len(self._plan_points) - 1)]
        path_yaw = math.atan2(by - ay, bx - ax)

        error = path_yaw - self._odom_yaw
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi
        return error

    def _compute_path_completion(self) -> float:
        if not self._has_plan or len(self._plan_points) < 2:
            return 0.0

        rx, ry = self._odom_x, self._odom_y
        min_dist = float("inf")
        nearest_idx = 0
        for i, (px, py) in enumerate(self._plan_points):
            dist = (rx - px) ** 2 + (ry - py) ** 2
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        traversed = 0.0
        for i in range(1, nearest_idx + 1):
            dx = self._plan_points[i][0] - self._plan_points[i - 1][0]
            dy = self._plan_points[i][1] - self._plan_points[i - 1][1]
            traversed += math.sqrt(dx * dx + dy * dy)

        total = 0.0
        for i in range(1, len(self._plan_points)):
            dx = self._plan_points[i][0] - self._plan_points[i - 1][0]
            dy = self._plan_points[i][1] - self._plan_points[i - 1][1]
            total += math.sqrt(dx * dx + dy * dy)

        if total < 1e-6:
            return 100.0
        return min(100.0, (traversed / total) * 100.0)

    # ==================================================================
    # Publish
    # ==================================================================

    def _publish_metrics(self):
        d = self._vel_msg.data
        d[0] = self._nav2_cmd_v
        d[1] = self._nav2_cmd_w
        d[2] = self._pid_out_v
        d[3] = self._pid_out_w
        d[4] = self._odom_v
        d[5] = self._odom_w
        d[6] = self._nav2_cmd_v - self._odom_v
        d[7] = self._nav2_cmd_w - self._odom_w
        self._vel_track_pub.publish(self._vel_msg)

        d = self._path_msg.data
        d[0] = self._compute_lateral_deviation()
        d[1] = self._compute_heading_error()
        if self._has_goal:
            dx = self._goal_x - self._odom_x
            dy = self._goal_y - self._odom_y
            d[2] = math.sqrt(dx * dx + dy * dy)
        else:
            d[2] = 0.0
        d[3] = self._compute_path_completion()
        self._path_track_pub.publish(self._path_msg)

        d = self._status_msg.data
        d[0] = 1.0 if self._test_active else 0.0
        d[1] = float(self._test_level)
        if self._test_active:
            d[2] = self.get_clock().now().nanoseconds * 1e-9 - self._test_start_time
        else:
            d[2] = 0.0
        d[3] = float(self._scenario_id)
        self._test_status_pub.publish(self._status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
