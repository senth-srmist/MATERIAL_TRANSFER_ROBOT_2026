#!/usr/bin/env python3
"""
Job Manager Node (v2 - Optimized)

Accepts delivery requests via /request_delivery service, maintains a
priority queue, and executes jobs sequentially.

Optimizations from v1:
  - Pre-allocated messages (Bool, Int32, JobStatus reused)
  - Reduced JSON serialization overhead (only on actual changes)
  - Consistent ROS clock usage
  - Simplified state persistence

Publishes:
  /system/nav_needed (Bool) — True when nav stack needed
  /system/active_jobs (Int32) — count of queued + active jobs
  /job_status (JobStatus) — current job state
  /job/awaiting_confirmation (String) — waiting at pickup/dropoff

Services:
  /request_delivery (DeliveryJob) — submit a new delivery job
  /cancel_job (CancelJob) — cancel a job by ID or cancel current
  /job/confirm (ConfirmJob) — confirm pickup/dropoff to proceed

Job lifecycle:
  QUEUED -> WAITING_FOR_NAV -> PICKUP_NAV -> PICKUP_ARRIVED ->
  DROPOFF_NAV -> DROPOFF_ARRIVED -> RETURNING -> COMPLETE
"""

import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32, Empty, Bool, String
from job_manager.msg import JobStatus
from job_manager.srv import DeliveryJob, CancelJob, ConfirmJob
from mission_controller.srv import NavigateToRoom


# ============================================================================
# Constants
# ============================================================================

STATE_FILE = "/tmp/job_manager_state.json"

# State name mapping (cached)
STATE_NAMES = {
    JobStatus.QUEUED: "QUEUED",
    JobStatus.WAITING_FOR_NAV: "WAITING_FOR_NAV",
    JobStatus.PICKUP_NAV: "PICKUP_NAV",
    JobStatus.PICKUP_ARRIVED: "PICKUP_ARRIVED",
    JobStatus.DROPOFF_NAV: "DROPOFF_NAV",
    JobStatus.DROPOFF_ARRIVED: "DROPOFF_ARRIVED",
    JobStatus.RETURNING: "RETURNING",
    JobStatus.COMPLETE: "COMPLETE",
    JobStatus.FAILED: "FAILED",
    JobStatus.CANCELLED: "CANCELLED",
}


# ============================================================================
# Job data
# ============================================================================


@dataclass
class Job:
    job_id: str
    pickup_room: str
    dropoff_room: str
    priority: int = 0
    state: int = JobStatus.QUEUED
    message: str = ""
    created_at: float = 0.0
    pickup_confirmed: bool = False
    interrupted: bool = False
    resume_state: int = JobStatus.QUEUED

    def to_dict(self) -> dict:
        return {
            "job_id": self.job_id,
            "pickup_room": self.pickup_room,
            "dropoff_room": self.dropoff_room,
            "priority": self.priority,
            "state": self.state,
            "message": self.message,
            "created_at": self.created_at,
            "pickup_confirmed": self.pickup_confirmed,
            "interrupted": self.interrupted,
            "resume_state": self.resume_state,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Job":
        return cls(
            job_id=data["job_id"],
            pickup_room=data["pickup_room"],
            dropoff_room=data["dropoff_room"],
            priority=data.get("priority", 0),
            state=data.get("state", JobStatus.QUEUED),
            message=data.get("message", ""),
            created_at=data.get("created_at", 0.0),
            pickup_confirmed=data.get("pickup_confirmed", False),
            interrupted=data.get("interrupted", False),
            resume_state=data.get("resume_state", JobStatus.QUEUED),
        )

    @property
    def has_item(self) -> bool:
        """True if robot is carrying an item for this job."""
        return self.pickup_confirmed


# ============================================================================
# Job Manager
# ============================================================================


class JobManager(Node):
    HOME_ROOM = "home"
    PICKUP_TIMEOUT = 300.0
    DROPOFF_TIMEOUT = 300.0

    def __init__(self):
        super().__init__("job_manager")

        # Parameters
        self.declare_parameter("home_room", self.HOME_ROOM)
        self.declare_parameter("pickup_timeout", self.PICKUP_TIMEOUT)
        self.declare_parameter("dropoff_timeout", self.DROPOFF_TIMEOUT)
        self.declare_parameter("debug_mode", False)
        self.declare_parameter("state_file", STATE_FILE)

        self.HOME_ROOM = self.get_parameter("home_room").value
        self.PICKUP_TIMEOUT = self.get_parameter("pickup_timeout").value
        self.DROPOFF_TIMEOUT = self.get_parameter("dropoff_timeout").value
        self._debug_mode = self.get_parameter("debug_mode").value
        self._state_file = Path(self.get_parameter("state_file").value)

        self._cb_group = ReentrantCallbackGroup()

        # Job queue and state
        self._queue: Deque[Job] = deque()
        self._active_job: Optional[Job] = None
        self._all_jobs: Dict[str, Job] = {}
        self._job_counter = 0
        self._lock = threading.Lock()
        self._cancel_requested = False
        self._nav_ready = False
        self._nav_ready_event = threading.Event()
        self._nav_needed = False

        # Confirmation state
        self._confirm_event = threading.Event()
        self._confirm_proceed = False

        # Track if queue changed (for JSON serialization optimization)
        self._queue_changed = True
        self._last_queue_json = ""

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Pre-allocated messages
        self._nav_needed_msg = Bool()
        self._active_jobs_msg = Int32()
        self._queue_msg = String()
        self._awaiting_msg = String()
        self._status_msg = JobStatus()

        # Publishers
        self._nav_needed_pub = self.create_publisher(Bool, "/system/nav_needed", qos)
        self._active_jobs_pub = self.create_publisher(Int32, "/system/active_jobs", qos)
        self._status_pub = self.create_publisher(JobStatus, "/job_status", qos)
        self._queue_pub = self.create_publisher(String, "/job_queue", qos)
        self._awaiting_pub = self.create_publisher(String, "/job/awaiting_confirmation", qos)

        # Services
        self.create_service(
            DeliveryJob,
            "/request_delivery",
            self._handle_delivery_request,
            callback_group=self._cb_group,
        )
        self.create_service(
            CancelJob,
            "/cancel_job",
            self._handle_cancel_request,
            callback_group=self._cb_group,
        )
        self.create_service(
            ConfirmJob,
            "/job/confirm",
            self._handle_confirm,
            callback_group=self._cb_group,
        )

        # Subscriptions
        self.create_subscription(
            Empty,
            "/system/nav_ready",
            self._nav_ready_callback,
            qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Empty,
            "/system/nav_shutdown",
            self._nav_shutdown_callback,
            qos,
            callback_group=self._cb_group,
        )

        # Service client
        self._nav_client = self.create_client(
            NavigateToRoom,
            "/navigate_to_room",
            callback_group=self._cb_group,
        )

        # Status publisher timer (1Hz)
        self.create_timer(1.0, self._publish_system_status)

        # Job executor thread
        self._executor_thread = threading.Thread(target=self._job_executor, daemon=True)
        self._running = True

        # Restore state
        self._restore_state()

        self._executor_thread.start()

        self.get_logger().info(f"Job manager v2 started — home={self.HOME_ROOM} pickup_timeout={self.PICKUP_TIMEOUT}s")

    # ==================================================================
    # State persistence
    # ==================================================================

    def _save_state(self):
        """Persist job queue to file."""
        try:
            with self._lock:
                data = {
                    "job_counter": self._job_counter,
                    "queue": [j.to_dict() for j in self._queue],
                    "active_job": self._active_job.to_dict() if self._active_job else None,
                }
            self._state_file.write_text(json.dumps(data, indent=2))
        except Exception as e:
            self.get_logger().warning(f"Failed to save state: {e}")

    def _restore_state(self):
        """Restore job queue from file."""
        if not self._state_file.exists():
            return

        try:
            data = json.loads(self._state_file.read_text())
            self._job_counter = data.get("job_counter", 0)

            for job_dict in data.get("queue", []):
                job = Job.from_dict(job_dict)
                job.interrupted = True
                self._queue.append(job)
                self._all_jobs[job.job_id] = job

            if data.get("active_job"):
                job = Job.from_dict(data["active_job"])
                job.interrupted = True
                
                # Determine resume point
                if job.pickup_confirmed:
                    job.resume_state = JobStatus.DROPOFF_NAV
                    job.message = "Resuming: item on robot, heading to dropoff"
                else:
                    job.resume_state = JobStatus.PICKUP_NAV
                    job.message = "Resuming: heading to pickup"

                self._queue.appendleft(job)
                self._all_jobs[job.job_id] = job

            self._queue_changed = True

            if self._queue:
                self.get_logger().info(f"Restored {len(self._queue)} jobs from state file")
        except Exception as e:
            self.get_logger().warning(f"Failed to restore state: {e}")

    # ==================================================================
    # Service handlers
    # ==================================================================

    def _handle_delivery_request(self, request, response):
        """Handle new delivery job request."""
        with self._lock:
            self._job_counter += 1
            job_id = f"job_{self._job_counter:04d}"

            job = Job(
                job_id=job_id,
                pickup_room=request.pickup_room,
                dropoff_room=request.dropoff_room,
                priority=request.priority,
                created_at=time.time(),
            )

            # Insert by priority (higher priority first)
            inserted = False
            for i, existing in enumerate(self._queue):
                if job.priority > existing.priority:
                    self._queue.insert(i, job)
                    inserted = True
                    break
            if not inserted:
                self._queue.append(job)

            self._all_jobs[job_id] = job
            self._queue_changed = True

        self._save_state()

        response.success = True
        response.job_id = job_id
        response.message = f"Job {job_id} queued: {request.pickup_room} -> {request.dropoff_room}"

        self.get_logger().info(response.message)
        return response

    def _handle_cancel_request(self, request, response):
        """Handle job cancellation request."""
        job_id = request.job_id

        with self._lock:
            # Cancel specific job
            if job_id:
                job = self._all_jobs.get(job_id)
                if job is None:
                    response.success = False
                    response.message = f"Job {job_id} not found"
                    return response

                if job.has_item:
                    response.success = False
                    response.message = f"Cannot cancel {job_id}: item already picked up"
                    return response

                if job == self._active_job:
                    self._cancel_requested = True
                    response.message = f"Cancelling active job {job_id}"
                else:
                    self._queue.remove(job)
                    job.state = JobStatus.CANCELLED
                    response.message = f"Removed {job_id} from queue"

                self._queue_changed = True
                response.success = True

            # Cancel current job
            else:
                if self._active_job is None:
                    response.success = False
                    response.message = "No active job to cancel"
                    return response

                if self._active_job.has_item:
                    response.success = False
                    response.message = "Cannot cancel: item already picked up"
                    return response

                self._cancel_requested = True
                response.success = True
                response.message = f"Cancelling {self._active_job.job_id}"

        self._save_state()
        self.get_logger().info(response.message)
        return response

    def _handle_confirm(self, request, response):
        """Handle pickup/dropoff confirmation."""
        self._confirm_proceed = request.proceed
        self._confirm_event.set()

        response.success = True
        response.message = "Confirmed" if request.proceed else "Aborted"
        return response

    # ==================================================================
    # Subscription callbacks
    # ==================================================================

    def _nav_ready_callback(self, msg):
        self._nav_ready = True
        self._nav_ready_event.set()
        self.get_logger().info("Nav stack ready")

    def _nav_shutdown_callback(self, msg):
        self._nav_ready = False
        self._nav_ready_event.clear()
        self.get_logger().info("Nav stack shutdown")

    # ==================================================================
    # Job executor thread
    # ==================================================================

    def _job_executor(self):
        """Main job execution loop (runs in separate thread)."""
        while self._running:
            job = None

            with self._lock:
                if self._queue and self._active_job is None:
                    job = self._queue.popleft()
                    self._active_job = job
                    self._nav_needed = True
                    self._queue_changed = True

            if job is None:
                time.sleep(0.5)
                continue

            self.get_logger().info(f"[{job.job_id}] Starting job: {job.pickup_room} -> {job.dropoff_room}")

            # Execute job
            success = self._execute_job(job)

            with self._lock:
                self._active_job = None
                self._cancel_requested = False
                self._queue_changed = True

                # Check if more jobs or need to return home
                if not self._queue:
                    self._return_home()

            self._save_state()

            if success:
                self.get_logger().info(f"[{job.job_id}] Job complete")
            else:
                self.get_logger().warning(f"[{job.job_id}] Job failed/cancelled")

    def _execute_job(self, job: Job) -> bool:
        """Execute a single job. Returns True if successful."""
        # Determine starting point (for recovery)
        if job.interrupted and job.resume_state == JobStatus.DROPOFF_NAV:
            start_phase = "dropoff"
        else:
            start_phase = "pickup"

        # Wait for nav stack
        job.state = JobStatus.WAITING_FOR_NAV
        job.message = "Waiting for navigation stack"
        self._publish_job_status(job)

        if not self._wait_for_nav(60.0):
            job.state = JobStatus.FAILED
            job.message = "Nav stack not available"
            self._publish_job_status(job)
            return False

        # Pickup phase
        if start_phase == "pickup":
            job.state = JobStatus.PICKUP_NAV
            job.message = f"Navigating to pickup: {job.pickup_room}"
            self._publish_job_status(job)
            self._save_state()

            if self._cancel_requested:
                job.state = JobStatus.CANCELLED
                return False

            if not self._navigate_to(job.pickup_room):
                job.state = JobStatus.FAILED
                job.message = "Failed to reach pickup"
                return False

            # Wait for pickup confirmation
            job.state = JobStatus.PICKUP_ARRIVED
            job.message = f"Waiting for pickup confirmation at {job.pickup_room}"
            self._publish_job_status(job)

            if not self._wait_for_confirmation(job.pickup_room, self.PICKUP_TIMEOUT, is_pickup=True):
                job.state = JobStatus.CANCELLED
                job.message = "Pickup not confirmed"
                return False

            job.pickup_confirmed = True
            self._save_state()

        # Dropoff phase
        job.state = JobStatus.DROPOFF_NAV
        job.message = f"Navigating to dropoff: {job.dropoff_room}"
        self._publish_job_status(job)
        self._save_state()

        if not self._navigate_to(job.dropoff_room):
            job.state = JobStatus.FAILED
            job.message = "Failed to reach dropoff"
            return False

        # Wait for dropoff confirmation
        job.state = JobStatus.DROPOFF_ARRIVED
        job.message = f"Waiting for dropoff confirmation at {job.dropoff_room}"
        self._publish_job_status(job)

        # Dropoff confirmation is optional — proceed anyway on timeout
        self._wait_for_confirmation(job.dropoff_room, self.DROPOFF_TIMEOUT, is_pickup=False)

        job.pickup_confirmed = False
        job.state = JobStatus.COMPLETE
        job.message = "Delivery complete"
        self._publish_job_status(job)
        return True

    def _wait_for_confirmation(self, location: str, timeout: float, is_pickup: bool) -> bool:
        """Wait for confirmation or timeout."""
        self._confirm_event.clear()
        self._confirm_proceed = False

        stage = "PICKUP" if is_pickup else "DROPOFF"

        # Publish what we're waiting for
        self._awaiting_msg.data = (
            f"{stage} at {location} | "
            f"Call: ros2 service call /job/confirm "
            f"job_manager/srv/ConfirmJob '{{proceed: true}}' | "
            f"Timeout: {timeout:.0f}s"
        )
        self._awaiting_pub.publish(self._awaiting_msg)

        self.get_logger().info(f"Awaiting {stage} confirmation at {location} (timeout: {timeout}s)")

        got_response = self._confirm_event.wait(timeout=timeout)

        if got_response and self._confirm_proceed:
            self.get_logger().info(f"{stage} confirmed at {location}")
            return True
        elif got_response:
            self.get_logger().warning(f"{stage} aborted at {location}")
            return False
        else:
            self.get_logger().warning(f"{stage} timeout at {location}")
            return False

    def _return_home(self):
        """Return to home position after all jobs complete."""
        self.get_logger().info("Queue empty, returning home")

        if not self._nav_ready:
            self.get_logger().warning("Nav not ready, skipping return home")
            self._nav_needed = False
            return

        if self._navigate_to(self.HOME_ROOM):
            self.get_logger().info("Returned home")
        else:
            self.get_logger().warning("Failed to return home")

        self._nav_needed = False

    # ==================================================================
    # Navigation helper
    # ==================================================================

    def _navigate_to(self, room_name: str) -> bool:
        """Call /navigate_to_room service. Blocks until complete."""
        if not self._nav_client.service_is_ready():
            self.get_logger().warning("navigate_to_room not ready, waiting 10s...")
            if not self._nav_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("navigate_to_room not available")
                return False

        request = NavigateToRoom.Request()
        request.room_name = room_name

        self.get_logger().info(f"Navigating to: {room_name}")

        future = self._nav_client.call_async(request)

        # Wait for result, checking for cancellation
        while not future.done():
            if self._cancel_requested and self._active_job and not self._active_job.has_item:
                self.get_logger().warning("Cancel during navigation")
                return False
            time.sleep(0.1)

        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"Reached {room_name}")
                return True
            else:
                self.get_logger().warning(f"Navigation failed: {result.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            return False

    def _wait_for_nav(self, timeout: float) -> bool:
        """Wait for supervisor to signal nav stack is ready."""
        if self._debug_mode:
            self.get_logger().info("Debug mode — skipping nav_ready wait")
            return True

        if self._nav_ready:
            return True

        self.get_logger().info(f"Waiting for nav stack (up to {timeout}s)...")
        return self._nav_ready_event.wait(timeout=timeout)

    # ==================================================================
    # Publishers
    # ==================================================================

    def _publish_system_status(self):
        """Publish nav_needed, active_jobs and job_queue at 1Hz."""
        with self._lock:
            count = len(self._queue)
            if self._active_job is not None:
                count += 1
            nav_needed = self._nav_needed
            queue_changed = self._queue_changed
            self._queue_changed = False

            # Only rebuild JSON if queue changed
            if queue_changed:
                queue_snapshot = [
                    {
                        "job_id": job.job_id,
                        "pickup_room": job.pickup_room,
                        "dropoff_room": job.dropoff_room,
                        "priority": job.priority,
                        "state": job.state,
                        "state_name": STATE_NAMES.get(job.state, "UNKNOWN"),
                        "message": job.message,
                        "has_item": job.has_item,
                        "interrupted": job.interrupted,
                    }
                    for job in self._queue
                ]

                active_snapshot = None
                if self._active_job:
                    active_snapshot = {
                        "job_id": self._active_job.job_id,
                        "pickup_room": self._active_job.pickup_room,
                        "dropoff_room": self._active_job.dropoff_room,
                        "priority": self._active_job.priority,
                        "state": self._active_job.state,
                        "state_name": STATE_NAMES.get(self._active_job.state, "UNKNOWN"),
                        "message": self._active_job.message,
                        "has_item": self._active_job.has_item,
                        "pickup_confirmed": self._active_job.pickup_confirmed,
                    }

                self._last_queue_json = json.dumps({
                    "active_job": active_snapshot,
                    "queue": queue_snapshot,
                })

        # Nav needed (reuse message)
        self._nav_needed_msg.data = nav_needed
        self._nav_needed_pub.publish(self._nav_needed_msg)

        # Active jobs count (reuse message)
        self._active_jobs_msg.data = count
        self._active_jobs_pub.publish(self._active_jobs_msg)

        # Queue JSON (reuse message, only update if changed)
        self._queue_msg.data = self._last_queue_json
        self._queue_pub.publish(self._queue_msg)

    def _publish_job_status(self, job: Job):
        """Publish current job status (reuses message)."""
        msg = self._status_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.job_id = job.job_id
        msg.pickup_room = job.pickup_room
        msg.dropoff_room = job.dropoff_room
        msg.priority = job.priority
        msg.state = job.state
        msg.message = job.message
        msg.state_name = STATE_NAMES.get(job.state, "UNKNOWN")

        self._status_pub.publish(msg)

    # ==================================================================
    # Cleanup
    # ==================================================================

    def destroy_node(self):
        self._running = False
        if self._executor_thread.is_alive():
            self._executor_thread.join(timeout=2.0)
        super().destroy_node()


# ============================================================================
# Main
# ============================================================================


def main(args=None):
    rclpy.init(args=args)
    node = JobManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
