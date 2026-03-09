# task_store.py
# Manages task persistence using a JSON file.
# When ROS is integrated, this layer stays — ROS bridge will update statuses here.

import json
import os
import threading
from datetime import datetime
from typing import List, Optional, Dict, Any

from models import Task

# ── Config ────────────────────────────────────────────────────────────────
# tasks.json lives inside UI/data/ in the repo
TASKS_FILE = os.getenv(
    "TASKS_FILE",
    os.path.join(
        os.path.expanduser("~"),
        "MATERIAL_TRANSFER_ROBOT_2026/CODE/UI/data/tasks.json"
    )
)
QUEUE_DEPTH = int(os.getenv("QUEUE_DEPTH", "10"))

# Priority order — lower number = higher priority (processed first)
# High → first in queue, Low → last in queue
# Same priority → FIFO (submission order)
#PRIORITY_ORDER = {"High": 1, "Medium": 2, "Low": 3}

# Thread lock — prevents race conditions if multiple users submit at once
_lock = threading.Lock()


# ── File Helpers ──────────────────────────────────────────────────────────

def _ensure_file() -> None:
    """Creates tasks.json with empty structure if it doesn't exist."""
    os.makedirs(os.path.dirname(TASKS_FILE), exist_ok=True)
    if not os.path.exists(TASKS_FILE):
        _write({"counter": 0, "tasks": []})
        print(f"[task_store] Created new tasks file at {TASKS_FILE}")


def _read() -> Dict[str, Any]:
    """Reads and returns the full tasks.json content."""
    with open(TASKS_FILE, "r") as f:
        return json.load(f)


def _write(data: Dict[str, Any]) -> None:
    """Writes data to tasks.json atomically."""
    tmp = TASKS_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, TASKS_FILE)


'''def _sort_by_priority(tasks: List[Dict]) -> List[Dict]:
    """
    Sorts tasks by priority first, then by timestamp (FIFO within same priority).
    High → Medium → Low
    Same priority → whoever submitted first goes first.
    """
    return sorted(
        tasks,
        key=lambda t: (
            PRIORITY_ORDER.get(t["priority"], 99),  # 1=High, 2=Medium, 3=Low
            t["timestamp"],                          # FIFO within same priority
        )
    )
'''

# ── Task ID Generator ─────────────────────────────────────────────────────

def _next_task_id(counter: int) -> str:
    """Generates REQ-01, REQ-02, ... style IDs."""
    return f"REQ-{counter:02d}"


# ── Public API ────────────────────────────────────────────────────────────

def init() -> None:
    """Called at FastAPI startup to ensure storage is ready."""
    _ensure_file()
    data = _read()
    count = len([t for t in data["tasks"] if t["status"] in ("queued", "in_progress")])
    print(f"[task_store] Loaded — {count} active tasks found")


def add_task(pickup: str, drop: str, priority: str, requested_by: str, requester_ip: str = "") -> Task:
    """
    Creates a new task, appends to tasks.json, returns the Task object.
    Raises ValueError if queue is already at max depth.
    """
    with _lock:
        data = _read()

        queued = [t for t in data["tasks"] if t["status"] == "queued"]

        if len(queued) >= QUEUE_DEPTH:
            raise ValueError(
                f"Queue is full ({QUEUE_DEPTH} tasks pending). "
                "Please wait for the robot to complete current tasks."
            )

        data["counter"] += 1
        task_id = _next_task_id(data["counter"])

        task = {
            "task_id": task_id,
            "pickup": pickup,
            "drop": drop,
            "priority": priority,
            "status": "queued",
            "requested_by": requested_by,
            "requester_ip": requester_ip,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
        }

        data["tasks"].append(task)
        _write(data)

        print(f"[task_store] New task added: {task_id} | {pickup} → {drop} | {priority}")
        return Task(**task)


def get_current_task() -> Optional[Task]:
    """Returns the single in_progress task, or None if robot is idle."""
    data = _read()
    for t in data["tasks"]:
        if t["status"] == "in_progress":
            return Task(**t)
    return None


def get_queue(limit: int = QUEUE_DEPTH) -> List[Task]:
    """
    Returns up to `limit` queued tasks sorted by priority.
    High → Medium → Low. Same priority = FIFO order.
    This mirrors the ROS topic queue depth.
    """
    data = _read()
    queued = [t for t in data["tasks"] if t["status"] == "queued"]
    #sorted_tasks = _sort_by_priority(queued)
    #return [Task(**t) for t in sorted_tasks[:limit]]
    return [Task(**t) for t in queued[:limit]]


def cancel_task(task_id: str, requester_ip: str = "") -> Optional[Task]:
    """
    Marks a queued task as cancelled.
    Only the original requester (by IP) can cancel.
    Raises ValueError if task is in_progress or IP doesn't match.
    """
    with _lock:
        data = _read()

        for t in data["tasks"]:
            if t["task_id"] == task_id:
                # ── IP ownership check ─────────────────────────────────
                if requester_ip and t.get("requester_ip") and t["requester_ip"] != requester_ip:
                    raise ValueError(
                        f"You are not allowed to cancel task {task_id}. "
                        "Only the person who submitted this request can cancel it."
                    )

                if t["status"] == "in_progress":
                    raise ValueError(
                        f"Task {task_id} is already in progress. "
                        "Cannot cancel a task the robot is currently executing."
                    )
                if t["status"] in ("done", "cancelled"):
                    raise ValueError(f"Task {task_id} is already {t['status']}.")

                t["status"] = "cancelled"
                _write(data)
                print(f"[task_store] Task cancelled: {task_id} by IP {requester_ip}")
                return Task(**t)

        return None


def get_all_tasks() -> List[Task]:
    """Returns all tasks (used for admin/debug endpoint)."""
    data = _read()
    return [Task(**t) for t in data["tasks"]]


def get_queue_stats() -> Dict[str, int]:
    """Returns a summary of task counts by status."""
    data = _read()
    stats = {"queued": 0, "in_progress": 0, "done": 0, "cancelled": 0}
    for t in data["tasks"]:
        if t["status"] in stats:
            stats[t["status"]] += 1
    stats["queue_depth"] = QUEUE_DEPTH
    return stats
