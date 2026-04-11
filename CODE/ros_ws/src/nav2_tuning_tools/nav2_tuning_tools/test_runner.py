#!/usr/bin/env python3
"""
Test Runner for Nav2 Tuning

Loads scenario YAML files, sends navigation goals, and manages
background rosbag recording.

Usage:
  ros2 run nav2_tuning_tools test_runner --scenario L1_straight
  ros2 run nav2_tuning_tools test_runner --scenario L1_straight --goals "1.0,0,0;2.0,0,0"
  ros2 run nav2_tuning_tools test_runner --list

The --goals flag overrides the YAML goals with custom positions.
Format: "x,y,yaw;x,y,yaw;..."

Scenarios are defined in config/scenarios/*.yaml with this format:
  scenario:
    name: L1_straight
    level: 1
    id: 1
    description: "Straight line 3m forward"
    goals:
      - {x: 3.0, y: 0.0, yaw: 0.0}
    record_topics:
      - /cmd_vel_nav2
      - /pid/debug
      ...
"""

import os
import sys
import math
import signal
import subprocess
import time
from datetime import datetime

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

# Default scenario directory — override with SCENARIOS_DIR env var
DEFAULT_SCENARIOS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config",
    "scenarios",
)


def quaternion_from_yaw(yaw: float):
    """Convert yaw angle to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def load_scenario(name: str, scenarios_dir: str = None) -> dict:
    """Load scenario YAML file."""
    if scenarios_dir is None:
        scenarios_dir = os.environ.get("SCENARIOS_DIR", DEFAULT_SCENARIOS_DIR)

    # Try exact name, then with .yaml extension
    candidates = [
        os.path.join(scenarios_dir, name),
        os.path.join(scenarios_dir, name + ".yaml"),
        os.path.join(scenarios_dir, name + ".yml"),
    ]

    for path in candidates:
        if os.path.exists(path):
            with open(path) as f:
                return yaml.safe_load(f)

    raise FileNotFoundError(
        f"Scenario '{name}' not found in {scenarios_dir}. "
        f"Tried: {[os.path.basename(c) for c in candidates]}"
    )


def list_scenarios(scenarios_dir: str = None):
    """List available scenarios."""
    if scenarios_dir is None:
        scenarios_dir = os.environ.get("SCENARIOS_DIR", DEFAULT_SCENARIOS_DIR)

    if not os.path.isdir(scenarios_dir):
        print(f"  Scenarios directory not found: {scenarios_dir}")
        return

    files = sorted(os.listdir(scenarios_dir))
    if not files:
        print("  No scenarios found.")
        return

    print(f"\n  Available scenarios in {scenarios_dir}:\n")
    for f in files:
        if f.endswith((".yaml", ".yml")):
            path = os.path.join(scenarios_dir, f)
            try:
                with open(path) as fh:
                    data = yaml.safe_load(fh)
                scenario = data.get("scenario", {})
                name = scenario.get("name", f)
                desc = scenario.get("description", "")
                level = scenario.get("level", "?")
                print(f"    L{level} | {name:30s} | {desc}")
            except Exception:
                print(f"    ? | {f}")
    print()


def parse_goals_string(goals_str: str) -> list:
    """Parse goals from 'x,y,yaw;x,y,yaw;...' format."""
    goals = []
    for part in goals_str.split(";"):
        part = part.strip()
        if not part:
            continue
        vals = [float(v.strip()) for v in part.split(",")]
        if len(vals) == 3:
            goals.append({"x": vals[0], "y": vals[1], "yaw": vals[2]})
        elif len(vals) == 2:
            goals.append({"x": vals[0], "y": vals[1], "yaw": 0.0})
        else:
            print(f"  ⚠ Invalid goal format: {part} (expected x,y,yaw)")
    return goals


class TestRunnerNode(Node):
    def __init__(self, scenario: dict, custom_goals: list = None):
        super().__init__("test_runner")

        self._scenario = scenario.get("scenario", scenario)
        self._goals = custom_goals or self._scenario.get("goals", [])
        self._current_goal_idx = 0
        self._bag_process = None
        self._start_time = None

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publisher for test status
        self._status_pub = self.create_publisher(
            String, "/test/active", reliable_qos
        )

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info(
            f"Test runner initialized — {self._scenario.get('name', 'unknown')} "
            f"with {len(self._goals)} goal(s)"
        )

        # Print goals for verification
        for i, g in enumerate(self._goals):
            self.get_logger().info(
                f"  Goal {i + 1}: x={g['x']:.2f}, y={g['y']:.2f}, yaw={g.get('yaw', 0.0):.2f}"
            )

    def start(self):
        """Start the test — bag recording + send first goal."""
        self._start_time = time.time()

        # Start bag recording
        record_topics = self._scenario.get("record_topics", [])
        if record_topics:
            self._start_bag(record_topics)

        # Publish test start status
        level = self._scenario.get("level", 0)
        name = self._scenario.get("name", "unknown")
        sid = self._scenario.get("id", 0)
        msg = String()
        msg.data = f"START:L{level}:{name}:{sid}"
        self._status_pub.publish(msg)

        # Wait for Nav2 action server
        self.get_logger().info("Waiting for Nav2 action server...")
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available!")
            return False

        # Send first goal
        self._send_next_goal()
        return True

    def _send_next_goal(self):
        if self._current_goal_idx >= len(self._goals):
            self._finish()
            return

        goal = self._goals[self._current_goal_idx]
        self.get_logger().info(
            f"Sending goal {self._current_goal_idx + 1}/{len(self._goals)}: "
            f"x={goal['x']:.2f}, y={goal['y']:.2f}, yaw={goal.get('yaw', 0.0):.2f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal["x"])
        goal_msg.pose.pose.position.y = float(goal["y"])

        yaw = float(goal.get("yaw", 0.0))
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Goal {self._current_goal_idx + 1} rejected!"
            )
            self._current_goal_idx += 1
            self._send_next_goal()
            return

        self.get_logger().info(
            f"Goal {self._current_goal_idx + 1} accepted, navigating..."
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        remaining = fb.distance_remaining
        # Log every ~1 second (action feedback comes at nav rate)
        if hasattr(self, "_last_fb_log"):
            if time.time() - self._last_fb_log < 2.0:
                return
        self._last_fb_log = time.time()
        self.get_logger().info(f"  Distance remaining: {remaining:.2f}m")

    def _result_cb(self, future):
        result = future.result()
        status = result.status

        elapsed = time.time() - self._start_time
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"✓ Goal {self._current_goal_idx + 1} reached! "
                f"(elapsed: {elapsed:.1f}s)"
            )
        else:
            self.get_logger().warn(
                f"✗ Goal {self._current_goal_idx + 1} failed with status {status} "
                f"(elapsed: {elapsed:.1f}s)"
            )

        self._current_goal_idx += 1
        self._send_next_goal()

    def _finish(self):
        elapsed = time.time() - self._start_time
        self.get_logger().info(
            f"\n{'=' * 50}\n"
            f"Test complete: {self._scenario.get('name', 'unknown')}\n"
            f"Total time: {elapsed:.1f}s\n"
            f"Goals: {self._current_goal_idx}/{len(self._goals)} attempted\n"
            f"{'=' * 50}"
        )

        # Publish stop
        msg = String()
        msg.data = "STOP"
        self._status_pub.publish(msg)

        # Stop bag
        self._stop_bag()

    def _start_bag(self, topics: list):
        name = self._scenario.get("name", "test")
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = os.environ.get("BAG_DIR", os.path.join(os.getcwd(), "bags"))
        os.makedirs(bag_dir, exist_ok=True)
        bag_path = os.path.join(bag_dir, f"{name}_{ts}")

        cmd = ["ros2", "bag", "record", "-o", bag_path] + topics
        self.get_logger().info(f"Recording bag: {bag_path}")
        self._bag_process = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def _stop_bag(self):
        if self._bag_process:
            self._bag_process.send_signal(signal.SIGINT)
            try:
                self._bag_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._bag_process.kill()
            self.get_logger().info("Bag recording stopped.")
            self._bag_process = None


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Nav2 Tuning Test Runner")
    parser.add_argument("--scenario", "-s", type=str, help="Scenario name (without .yaml)")
    parser.add_argument(
        "--goals",
        "-g",
        type=str,
        default=None,
        help='Override goals: "x,y,yaw;x,y,yaw;..." (overrides YAML goals)',
    )
    parser.add_argument("--list", "-l", action="store_true", help="List available scenarios")
    parser.add_argument(
        "--scenarios-dir",
        type=str,
        default=None,
        help="Override scenarios directory",
    )

    args = parser.parse_args()

    if args.list:
        list_scenarios(args.scenarios_dir)
        return

    if not args.scenario:
        parser.print_help()
        print("\n  Error: --scenario is required (or use --list to see options)")
        return

    # Load scenario
    try:
        scenario_data = load_scenario(args.scenario, args.scenarios_dir)
    except FileNotFoundError as e:
        print(f"  Error: {e}")
        return

    # Parse custom goals if provided
    custom_goals = None
    if args.goals:
        custom_goals = parse_goals_string(args.goals)
        if not custom_goals:
            print("  Error: No valid goals parsed from --goals argument")
            return
        print(f"  Using {len(custom_goals)} custom goal(s) (overriding YAML)")

    # Run
    rclpy.init()
    node = TestRunnerNode(scenario_data, custom_goals)

    if node.start():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\n  Test interrupted.")
            msg = String()
            msg.data = "STOP"
            node._status_pub.publish(msg)
            node._stop_bag()
    else:
        print("  Failed to start test.")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
