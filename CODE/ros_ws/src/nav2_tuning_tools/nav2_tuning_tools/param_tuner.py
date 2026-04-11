#!/usr/bin/env python3
"""
Interactive Parameter Tuner

Live-update Nav2 and PID parameters without restarting nodes.
Logs all changes with timestamps for reproducibility.

Usage:
  ros2 run nav2_tuning_tools param_tuner

Commands:
  set <node> <param> <value>   — Set parameter on a node
  get <node> <param>           — Get current parameter value
  history                      — Show all parameter changes
  undo                         — Revert last change
  export <filename.yaml>       — Export current winning params
  nodes                        — List monitored nodes
  params <node>                — List all params on a node
  help                         — Show this help
  quit                         — Exit

Shortcuts (common nodes):
  set pid kp 0.8             → ros2 param set /pid_controller kp 0.8
  set rpp desired_linear_vel 0.5 → ros2 param set /controller_server FollowPath.desired_linear_vel 0.5
  set costmap inflation_radius 0.4 → ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.4

The tuner resolves shorthand node names and parameter prefixes automatically.
"""

import sys
import os
import time
import json
import subprocess
import readline  # enables arrow keys and history in input()
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node


# Node name shortcuts
NODE_SHORTCUTS = {
    "pid": "/pid_controller",
    "rpp": "/controller_server",
    "controller": "/controller_server",
    "planner": "/planner_server",
    "local_costmap": "/local_costmap/local_costmap",
    "global_costmap": "/global_costmap/global_costmap",
    "bt": "/bt_navigator",
    "behavior": "/behavior_server",
}

# Parameter prefix shortcuts for common nodes
PARAM_PREFIXES = {
    "/controller_server": {
        # RPP parameters need FollowPath. prefix
        "desired_linear_vel": "FollowPath.desired_linear_vel",
        "max_linear_vel": "FollowPath.max_linear_vel",
        "min_linear_vel": "FollowPath.min_linear_vel",
        "max_linear_accel": "FollowPath.max_linear_accel",
        "max_linear_decel": "FollowPath.max_linear_decel",
        "max_angular_vel": "FollowPath.max_angular_vel",
        "min_angular_vel": "FollowPath.min_angular_vel",
        "min_lookahead_dist": "FollowPath.min_lookahead_dist",
        "max_lookahead_dist": "FollowPath.max_lookahead_dist",
        "lookahead_time": "FollowPath.lookahead_time",
        "rotate_to_heading_angular_vel": "FollowPath.rotate_to_heading_angular_vel",
        "rotate_to_heading_min_angle": "FollowPath.rotate_to_heading_min_angle",
        "regulated_linear_scaling_min_radius": "FollowPath.regulated_linear_scaling_min_radius",
        "regulated_linear_scaling_min_speed": "FollowPath.regulated_linear_scaling_min_speed",
        "min_approach_linear_velocity": "FollowPath.min_approach_linear_velocity",
        "approach_velocity_scaling_dist": "FollowPath.approach_velocity_scaling_dist",
        "max_allowed_time_to_collision_up_to_carrot": "FollowPath.max_allowed_time_to_collision_up_to_carrot",
        "max_robot_pose_search_dist": "FollowPath.max_robot_pose_search_dist",
        "transform_tolerance": "FollowPath.transform_tolerance",
    },
    "/local_costmap/local_costmap": {
        "inflation_radius": "inflation_layer.inflation_radius",
        "cost_scaling_factor": "inflation_layer.cost_scaling_factor",
        "human_inflation_radius": "human_layer.inflation_radius",
        "human_persistence_time": "human_layer.persistence_time",
    },
    "/global_costmap/global_costmap": {
        "inflation_radius": "inflation_layer.inflation_radius",
        "cost_scaling_factor": "inflation_layer.cost_scaling_factor",
    },
}


class ChangeRecord:
    def __init__(self, node: str, param: str, old_value, new_value, timestamp: str):
        self.node = node
        self.param = param
        self.old_value = old_value
        self.new_value = new_value
        self.timestamp = timestamp

    def __repr__(self):
        return (
            f"[{self.timestamp}] {self.node} :: {self.param}: "
            f"{self.old_value} → {self.new_value}"
        )


def ros2_param_get(node_name: str, param_name: str) -> Optional[str]:
    """Get parameter value via ros2 CLI."""
    try:
        result = subprocess.run(
            ["ros2", "param", "get", node_name, param_name],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            # Parse output like "Double value is: 0.5"
            output = result.stdout.strip()
            if "value is:" in output:
                return output.split("value is:")[-1].strip()
            return output
        return None
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def ros2_param_set(node_name: str, param_name: str, value: str) -> bool:
    """Set parameter via ros2 CLI."""
    try:
        result = subprocess.run(
            ["ros2", "param", "set", node_name, param_name, value],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def ros2_param_list(node_name: str) -> list:
    """List parameters on a node."""
    try:
        result = subprocess.run(
            ["ros2", "param", "list", node_name],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            return [line.strip() for line in result.stdout.splitlines() if line.strip()]
        return []
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []


def resolve_node(shortcut: str) -> str:
    """Resolve node shortcut to full name."""
    if shortcut in NODE_SHORTCUTS:
        return NODE_SHORTCUTS[shortcut]
    if not shortcut.startswith("/"):
        return "/" + shortcut
    return shortcut


def resolve_param(node_name: str, param: str) -> str:
    """Resolve parameter shortcut to full name."""
    prefixes = PARAM_PREFIXES.get(node_name, {})
    if param in prefixes:
        return prefixes[param]
    return param


class ParamTuner:
    def __init__(self):
        self.history = []
        self.log_file = f"param_changes_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

    def cmd_set(self, args: list):
        if len(args) < 3:
            print("  Usage: set <node> <param> <value>")
            return

        node = resolve_node(args[0])
        param = resolve_param(node, args[1])
        value = args[2]

        # Get old value first
        old_value = ros2_param_get(node, param)
        if old_value is None:
            print(f"  ⚠ Cannot read {node}::{param} — node may not be running")
            resp = input("  Try setting anyway? [y/N]: ").strip().lower()
            if resp != "y":
                return
            old_value = "unknown"

        # Set new value
        if ros2_param_set(node, param, value):
            timestamp = datetime.now().strftime("%H:%M:%S")
            record = ChangeRecord(node, param, old_value, value, timestamp)
            self.history.append(record)
            self._log(record)
            print(f"  ✓ {node}::{param}: {old_value} → {value}")
        else:
            print(f"  ✗ Failed to set {node}::{param}")

    def cmd_get(self, args: list):
        if len(args) < 2:
            print("  Usage: get <node> <param>")
            return

        node = resolve_node(args[0])
        param = resolve_param(node, args[1])
        value = ros2_param_get(node, param)
        if value is not None:
            print(f"  {node}::{param} = {value}")
        else:
            print(f"  ⚠ Cannot read {node}::{param}")

    def cmd_history(self):
        if not self.history:
            print("  No changes yet.")
            return
        print(f"  {'─' * 60}")
        for i, rec in enumerate(self.history):
            print(f"  {i + 1:3d}. {rec}")
        print(f"  {'─' * 60}")
        print(f"  Log file: {self.log_file}")

    def cmd_undo(self):
        if not self.history:
            print("  Nothing to undo.")
            return

        last = self.history[-1]
        if last.old_value == "unknown":
            print(f"  ⚠ Cannot undo — original value was unknown")
            return

        if ros2_param_set(last.node, last.param, last.old_value):
            self.history.pop()
            print(f"  ↶ Reverted {last.node}::{last.param} → {last.old_value}")
            self._log_raw(f"UNDO: {last}")
        else:
            print(f"  ✗ Failed to revert {last.node}::{last.param}")

    def cmd_export(self, args: list):
        if len(args) < 1:
            filename = f"winning_params_{datetime.now().strftime('%Y%m%d_%H%M%S')}.yaml"
        else:
            filename = args[0]

        # Build YAML from history — latest value per (node, param) wins
        current_params = {}
        for rec in self.history:
            key = (rec.node, rec.param)
            current_params[key] = rec.new_value

        if not current_params:
            print("  No changes to export.")
            return

        lines = [
            f"# Nav2 Tuning Export — {datetime.now().isoformat()}",
            f"# {len(current_params)} parameter(s) changed from baseline",
            "",
        ]

        # Group by node
        by_node = {}
        for (node, param), value in current_params.items():
            by_node.setdefault(node, []).append((param, value))

        for node, params in sorted(by_node.items()):
            lines.append(f"# Node: {node}")
            for param, value in sorted(params):
                lines.append(f"#   {param}: {value}")
            lines.append("")

        # Also write a shell script to replay
        lines.append("# Replay commands:")
        for (node, param), value in sorted(current_params.items()):
            lines.append(f"# ros2 param set {node} {param} {value}")

        with open(filename, "w") as f:
            f.write("\n".join(lines) + "\n")

        print(f"  ✓ Exported {len(current_params)} params to {filename}")

    def cmd_nodes(self):
        print("  Shortcuts:")
        for short, full in sorted(NODE_SHORTCUTS.items()):
            print(f"    {short:20s} → {full}")

    def cmd_params(self, args: list):
        if len(args) < 1:
            print("  Usage: params <node>")
            return

        node = resolve_node(args[0])
        params = ros2_param_list(node)
        if params:
            print(f"  Parameters on {node}:")
            for p in params:
                print(f"    {p}")
        else:
            print(f"  ⚠ No params found on {node} — node may not be running")

    def cmd_help(self):
        print("""
  ┌─────────────────────────────────────────────────────────────┐
  │ Nav2 Parameter Tuner — Live Reconfiguration                │
  ├─────────────────────────────────────────────────────────────┤
  │ set <node> <param> <value>  Set parameter live             │
  │ get <node> <param>          Get current value              │
  │ history                     Show all changes               │
  │ undo                        Revert last change             │
  │ export [filename]           Export winning params to YAML   │
  │ nodes                       Show node shortcuts            │
  │ params <node>               List params on a node          │
  │ help                        Show this help                 │
  │ quit / exit                 Exit tuner                     │
  ├─────────────────────────────────────────────────────────────┤
  │ Node shortcuts: pid, rpp, controller, planner,             │
  │   local_costmap, global_costmap, bt, behavior              │
  ├─────────────────────────────────────────────────────────────┤
  │ Examples:                                                  │
  │   set pid kp 0.8                                           │
  │   set pid feedforward_gain 0.9                             │
  │   set rpp desired_linear_vel 0.5                           │
  │   set rpp min_lookahead_dist 0.3                           │
  │   set local_costmap inflation_radius 0.4                   │
  │   get pid ki                                               │
  │   export L1_winner.yaml                                    │
  └─────────────────────────────────────────────────────────────┘
""")

    def _log(self, record: ChangeRecord):
        with open(self.log_file, "a") as f:
            f.write(f"{record}\n")

    def _log_raw(self, text: str):
        with open(self.log_file, "a") as f:
            f.write(f"[{datetime.now().strftime('%H:%M:%S')}] {text}\n")

    def run(self):
        print("\n  Nav2 Parameter Tuner v1.0")
        print("  Type 'help' for commands.\n")

        while True:
            try:
                raw = input("  tuner> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n  Bye.")
                break

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd == "set":
                self.cmd_set(args)
            elif cmd == "get":
                self.cmd_get(args)
            elif cmd == "history":
                self.cmd_history()
            elif cmd == "undo":
                self.cmd_undo()
            elif cmd == "export":
                self.cmd_export(args)
            elif cmd == "nodes":
                self.cmd_nodes()
            elif cmd == "params":
                self.cmd_params(args)
            elif cmd == "help":
                self.cmd_help()
            elif cmd in ("quit", "exit", "q"):
                if self.history:
                    resp = input("  Export changes before quitting? [Y/n]: ").strip().lower()
                    if resp != "n":
                        self.cmd_export([])
                print("  Bye.")
                break
            else:
                print(f"  Unknown command: {cmd}. Type 'help' for commands.")


def main(args=None):
    tuner = ParamTuner()
    tuner.run()


if __name__ == "__main__":
    main()
