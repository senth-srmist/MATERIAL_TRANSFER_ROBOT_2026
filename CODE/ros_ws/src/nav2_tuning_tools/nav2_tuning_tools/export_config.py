#!/usr/bin/env python3
"""
Config Export Tool

Reads param_tuner change logs and merges winning parameters
back into nav2_params.yaml and pid_params.yaml.

Usage:
  # Export from a single log file:
  ros2 run nav2_tuning_tools export_config \
    --log param_changes_20260410_143022.log \
    --nav2-base /path/to/nav2_params.yaml \
    --pid-base /path/to/pid_params.yaml \
    --output-dir ./final_configs/

  # Merge multiple level logs:
  ros2 run nav2_tuning_tools export_config \
    --log L1_winner.yaml L2_winner.yaml L3_winner.yaml \
    --nav2-base /path/to/nav2_params.yaml \
    --pid-base /path/to/pid_params.yaml

The tool parses the log, groups changes by node, and applies them
to the base YAML files. Later entries override earlier ones (last wins).
"""

import os
import sys
import re
import copy
import argparse
from datetime import datetime

import yaml


# Map ros2 param paths back to YAML structure paths
NODE_TO_YAML_PATH = {
    # PID controller
    "/pid_controller": {
        "yaml_file": "pid",
        "yaml_root": ["pid_controller", "ros__parameters"],
    },
    # Nav2 controller server → RPP params
    "/controller_server": {
        "yaml_file": "nav2",
        "yaml_root": ["controller_server", "ros__parameters"],
        "prefix_map": {
            "FollowPath.": ["FollowPath"],
            "progress_checker.": ["progress_checker"],
            "general_goal_checker.": ["general_goal_checker"],
        },
    },
    # Planner
    "/planner_server": {
        "yaml_file": "nav2",
        "yaml_root": ["planner_server", "ros__parameters"],
    },
    # Local costmap
    "/local_costmap/local_costmap": {
        "yaml_file": "nav2",
        "yaml_root": ["local_costmap", "local_costmap", "ros__parameters"],
        "prefix_map": {
            "inflation_layer.": ["inflation_layer"],
            "human_layer.": ["human_layer"],
            "obstacle_layer.": ["obstacle_layer"],
        },
    },
    # Global costmap
    "/global_costmap/global_costmap": {
        "yaml_file": "nav2",
        "yaml_root": ["global_costmap", "global_costmap", "ros__parameters"],
        "prefix_map": {
            "inflation_layer.": ["inflation_layer"],
            "static_layer.": ["static_layer"],
        },
    },
    # BT navigator
    "/bt_navigator": {
        "yaml_file": "nav2",
        "yaml_root": ["bt_navigator", "ros__parameters"],
    },
    # Behavior server
    "/behavior_server": {
        "yaml_file": "nav2",
        "yaml_root": ["behavior_server", "ros__parameters"],
    },
}


def parse_log_file(log_path: str) -> list:
    """
    Parse param_tuner log files.
    
    Handles two formats:
    1. Change log: [HH:MM:SS] /node :: param: old_value → new_value
    2. Export YAML: # ros2 param set /node param value
    """
    changes = []

    with open(log_path) as f:
        for line in f:
            line = line.strip()

            # Format 1: [HH:MM:SS] /node :: param: old → new
            m = re.match(
                r"\[[\d:]+\]\s+(/[\w/]+)\s+::\s+([\w.]+):\s+\S+\s+→\s+(\S+)", line
            )
            if m:
                changes.append(
                    {"node": m.group(1), "param": m.group(2), "value": m.group(3)}
                )
                continue

            # Format 2: # ros2 param set /node param value
            m = re.match(r"#?\s*ros2\s+param\s+set\s+(/[\w/]+)\s+([\w.]+)\s+(\S+)", line)
            if m:
                changes.append(
                    {"node": m.group(1), "param": m.group(2), "value": m.group(3)}
                )
                continue

    return changes


def cast_value(val_str: str):
    """Convert string value to appropriate Python type."""
    if val_str.lower() == "true":
        return True
    if val_str.lower() == "false":
        return False
    try:
        if "." in val_str:
            return float(val_str)
        return int(val_str)
    except ValueError:
        return val_str


def set_nested(d: dict, keys: list, value):
    """Set a value in a nested dict, creating intermediate dicts as needed."""
    for key in keys[:-1]:
        if key not in d:
            d[key] = {}
        d = d[key]
    d[keys[-1]] = value


def apply_changes(base_yaml: dict, changes: list, node_config: dict) -> dict:
    """Apply parameter changes to a YAML structure."""
    result = copy.deepcopy(base_yaml)

    for change in changes:
        param = change["param"]
        value = cast_value(change["value"])

        # Build the full key path
        root_path = list(node_config["yaml_root"])
        prefix_map = node_config.get("prefix_map", {})

        # Check if param has a known prefix
        param_path = None
        for prefix, sub_path in prefix_map.items():
            if param.startswith(prefix):
                param_name = param[len(prefix):]
                param_path = root_path + sub_path + [param_name]
                break

        if param_path is None:
            param_path = root_path + [param]

        set_nested(result, param_path, value)

    return result


def main():
    parser = argparse.ArgumentParser(description="Export tuning results to YAML configs")
    parser.add_argument(
        "--log",
        nargs="+",
        required=True,
        help="Param tuner log file(s) to process",
    )
    parser.add_argument(
        "--nav2-base",
        type=str,
        default=None,
        help="Base nav2_params.yaml to merge into",
    )
    parser.add_argument(
        "--pid-base",
        type=str,
        default=None,
        help="Base pid_params.yaml to merge into",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="./final_configs",
        help="Output directory for merged configs",
    )

    args = parser.parse_args()

    # Parse all log files
    all_changes = []
    for log_path in args.log:
        if not os.path.exists(log_path):
            print(f"  ⚠ Log file not found: {log_path}")
            continue
        changes = parse_log_file(log_path)
        print(f"  Parsed {len(changes)} changes from {log_path}")
        all_changes.extend(changes)

    if not all_changes:
        print("  No changes found in log files.")
        return

    # Deduplicate: last value wins per (node, param)
    deduped = {}
    for c in all_changes:
        key = (c["node"], c["param"])
        deduped[key] = c

    print(f"\n  {len(deduped)} unique parameter changes:")
    for (node, param), c in sorted(deduped.items()):
        print(f"    {node} :: {param} = {c['value']}")

    # Group by target YAML file
    nav2_changes = {}  # node -> [changes]
    pid_changes = {}

    for (node, param), c in deduped.items():
        config = NODE_TO_YAML_PATH.get(node)
        if config is None:
            print(f"  ⚠ Unknown node {node}, skipping {param}")
            continue

        target = config["yaml_file"]
        if target == "nav2":
            nav2_changes.setdefault(node, []).append(c)
        elif target == "pid":
            pid_changes.setdefault(node, []).append(c)

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Merge into nav2_params.yaml
    if nav2_changes:
        if args.nav2_base and os.path.exists(args.nav2_base):
            with open(args.nav2_base) as f:
                nav2_yaml = yaml.safe_load(f)
        else:
            print("  ⚠ No nav2 base YAML provided, generating changes-only file")
            nav2_yaml = {}

        for node, changes in nav2_changes.items():
            config = NODE_TO_YAML_PATH[node]
            nav2_yaml = apply_changes(nav2_yaml, changes, config)

        output_path = os.path.join(args.output_dir, f"nav2_params_tuned_{ts}.yaml")
        with open(output_path, "w") as f:
            f.write(f"# Nav2 Parameters — Tuned {datetime.now().isoformat()}\n")
            f.write(f"# Merged from: {', '.join(args.log)}\n")
            f.write(f"# {len(sum(nav2_changes.values(), []))} changes applied\n\n")
            yaml.dump(nav2_yaml, f, default_flow_style=False, sort_keys=False)
        print(f"\n  ✓ Nav2 config: {output_path}")

    # Merge into pid_params.yaml
    if pid_changes:
        if args.pid_base and os.path.exists(args.pid_base):
            with open(args.pid_base) as f:
                pid_yaml = yaml.safe_load(f)
        else:
            print("  ⚠ No PID base YAML provided, generating changes-only file")
            pid_yaml = {}

        for node, changes in pid_changes.items():
            config = NODE_TO_YAML_PATH[node]
            pid_yaml = apply_changes(pid_yaml, changes, config)

        output_path = os.path.join(args.output_dir, f"pid_params_tuned_{ts}.yaml")
        with open(output_path, "w") as f:
            f.write(f"# PID Parameters — Tuned {datetime.now().isoformat()}\n")
            f.write(f"# Merged from: {', '.join(args.log)}\n")
            f.write(f"# {len(sum(pid_changes.values(), []))} changes applied\n\n")
            yaml.dump(pid_yaml, f, default_flow_style=False, sort_keys=False)
        print(f"  ✓ PID config: {output_path}")

    # Also write a replay script
    replay_path = os.path.join(args.output_dir, f"replay_params_{ts}.sh")
    with open(replay_path, "w") as f:
        f.write("#!/bin/bash\n")
        f.write(f"# Parameter replay script — {datetime.now().isoformat()}\n")
        f.write(f"# Apply all tuned parameters via ros2 param set\n\n")
        for (node, param), c in sorted(deduped.items()):
            f.write(f"ros2 param set {node} {param} {c['value']}\n")
    os.chmod(replay_path, 0o755)
    print(f"  ✓ Replay script: {replay_path}")

    print(f"\n  Done. {len(deduped)} parameters exported.")


if __name__ == "__main__":
    main()
