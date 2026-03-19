#!/usr/bin/env python3
"""
Job Manager Node

Accepts delivery requests via /request_delivery service, maintains a
priority queue, and executes jobs sequentially by calling the
mission_controller's /navigate_to_room service.

Publishes:
  /system/nav_needed (Bool) — True when nav stack needed (jobs or
    returning home). Supervisor watches this for lifecycle.
  /system/active_jobs (Int32) — honest count of queued + active jobs.
  /job_status (JobStatus) — current job state for monitoring.
  /job/awaiting_confirmation (String) — published when waiting at
    pickup/dropoff, contains instructions for confirming.

Services:
  /request_delivery (DeliveryJob) — submit a new delivery job.
  /cancel_job (CancelJob) — cancel a job by ID or cancel current.
  /job/confirm (ConfirmJob) — confirm pickup/dropoff to proceed.

Requires:
  /navigate_to_room (mission_controller) — must be available when
    executing jobs. Supervisor brings it up when nav_needed=True.
  /system/nav_ready (std_msgs/Empty) — supervisor publishes this
    when the full nav stack is up and ready.
  /system/nav_shutdown (std_msgs/Empty) — supervisor publishes this
    when nav stack is going down.

Job lifecycle:
  QUEUED -> WAITING_FOR_NAV -> PICKUP_NAV -> PICKUP_ARRIVED ->
  DROPOFF_NAV -> DROPOFF_ARRIVED -> RETURNING -> COMPLETE

  At PICKUP_ARRIVED / DROPOFF_ARRIVED, the robot waits for
  confirmation via /job/confirm service or timeout:
    - Pickup timeout (no confirmation) = job cancelled
    - Pickup abort (proceed=false) = job cancelled
    - Dropoff timeout (no confirmation) = complete anyway
    - Dropoff abort (proceed=false) = complete anyway

  If queue is empty after a job, robot returns home.
  nav_needed stays True until return home completes.

State Persistence & Recovery:
  Job queue is persisted to /tmp/job_manager_state.json on every change.
  On startup, state is restored from file if it exists.

  Smart recovery based on last known state:
    - Before pickup confirmation: restart from pickup navigation
    - After pickup confirmation: resume from dropoff navigation
      (item is on robot, MUST complete delivery)
    - At confirmation wait: re-prompt for confirmation
"""

import threading
import json
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

# State name mapping for logging
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
    pickup_confirmed: bool = False  # True once item is loaded on robot
    interrupted: bool = False  # Was this job interrupted by a crash?
    resume_state: int = JobStatus.QUEUED  # State to resume from after recovery

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
    PICKUP_TIMEOUT = 300.0  # 5 min timeout at pickup
    DROPOFF_TIMEOUT = 300.0  # 5 min timeout at dropoff

    def __init__(self):
        super().__init__("job_manager")

        # Declare parameters
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

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self._nav_needed_pub = self.create_publisher(Bool,
                                                     "/system/nav_needed", qos)
        self._active_jobs_pub = self.create_publisher(Int32,
                                                      "/system/active_jobs",
                                                      qos)
        self._status_pub = self.create_publisher(JobStatus, "/job_status", qos)
        self._queue_pub = self.create_publisher(String, "/job_queue", qos)
        self._awaiting_pub = self.create_publisher(
            String, "/job/awaiting_confirmation", qos)

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
            self._handle_cancel,
            callback_group=self._cb_group,
        )
        self.create_service(
            ConfirmJob,
            "/job/confirm",
            self._handle_confirm,
            callback_group=self._cb_group,
        )

        # Subscription: supervisor signals nav stack is ready
        self.create_subscription(
            Empty,
            "/system/nav_ready",
            self._nav_ready_cb,
            qos,
        )

        # Subscription: supervisor signals nav stack shutting down
        self.create_subscription(
            Empty,
            "/system/nav_shutdown",
            self._nav_shutdown_cb,
            qos,
        )

        # Navigate to room client
        self._nav_client = self.create_client(
            NavigateToRoom,
            "/navigate_to_room",
            callback_group=self._cb_group,
        )

        # Restore state from file (before starting executor thread)
        self._restore_state()

        # Publish status at 1Hz
        self.create_timer(1.0, self._publish_system_status)

        # Job executor thread
        self._executor_thread = threading.Thread(
            target=self._job_executor_loop, daemon=True)
        self._executor_thread.start()

        self.get_logger().info(f"Job Manager started (home: {self.HOME_ROOM})")

    # ==================================================================
    # State persistence
    # ==================================================================

    def _save_state(self):
        """
        Save current state to file for crash recovery.
        Called after any change to queue or active job.
        """
        try:
            with self._lock:
                state = {
                    "job_counter": self._job_counter,
                    "nav_needed": self._nav_needed,
                    "queue": [job.to_dict() for job in self._queue],
                    "active_job": self._active_job.to_dict() if self._active_job else None,
                }

            # Write atomically via temp file
            temp_file = self._state_file.with_suffix(".tmp")
            temp_file.write_text(json.dumps(state, indent=2))
            temp_file.rename(self._state_file)

            self.get_logger().debug("State saved to file")

        except Exception as e:
            self.get_logger().error(f"Failed to save state: {e}")

    def _restore_state(self):
        """
        Restore state from file on startup.
        Jobs that were mid-execution are marked as interrupted and
        their resume_state is set based on where they were.
        """
        if not self._state_file.exists():
            self.get_logger().info("No state file found, starting fresh")
            return

        try:
            state = json.loads(self._state_file.read_text())

            self._job_counter = state.get("job_counter", 0)

            # Restore queued jobs
            queue_data = state.get("queue", [])
            for job_data in queue_data:
                job = Job.from_dict(job_data)
                self._queue.append(job)
                self._all_jobs[job.job_id] = job

            # Handle active job that was interrupted
            active_data = state.get("active_job")
            if active_data:
                job = Job.from_dict(active_data)
                job.interrupted = True

                # Determine resume point
                job.resume_state = self._determine_resume_state(job)

                # Update state and message for logging
                original_state = job.state
                job.state = JobStatus.QUEUED
                job.message = self._get_recovery_message(job, original_state)

                # Re-queue at front (highest priority for recovery)
                self._queue.appendleft(job)
                self._all_jobs[job.job_id] = job

                self.get_logger().warn(
                    f"Recovered interrupted job: {job.job_id} "
                    f"({job.pickup_room} -> {job.dropoff_room})"
                )
                self.get_logger().warn(
                    f"  Was at: {STATE_NAMES.get(original_state, 'UNKNOWN')}, "
                    f"Will resume from: {STATE_NAMES.get(job.resume_state, 'UNKNOWN')}, "
                    f"Has item: {job.has_item}"
                )

            # If we have jobs, set nav_needed
            if self._queue:
                self._nav_needed = True

            restored_count = len(self._queue)
            if restored_count > 0:
                self.get_logger().info(
                    f"Restored {restored_count} job(s) from state file"
                )

                # Log each restored job
                for job in self._queue:
                    if job.interrupted:
                        self.get_logger().info(
                            f"  - {job.job_id}: {job.pickup_room} -> {job.dropoff_room} "
                            f"[INTERRUPTED, resume from {STATE_NAMES.get(job.resume_state, 'UNKNOWN')}]"
                        )
                    else:
                        self.get_logger().info(
                            f"  - {job.job_id}: {job.pickup_room} -> {job.dropoff_room}"
                        )

        except Exception as e:
            self.get_logger().error(f"Failed to restore state: {e}")
            self.get_logger().warn("Starting with empty queue")

    def _determine_resume_state(self, job: Job) -> int:
        """
        Determine which state to resume from based on where the job was
        interrupted and whether pickup was confirmed.

        Recovery logic:
          - If pickup NOT confirmed: restart from PICKUP_NAV
            (no item on robot, need to go get it)
          - If pickup confirmed: resume from DROPOFF_NAV
            (item IS on robot, must complete delivery)
          - Special cases for confirmation waits
        """
        state = job.state

        if job.pickup_confirmed:
            # Item is on robot — MUST go to dropoff
            if state == JobStatus.DROPOFF_ARRIVED:
                # Was waiting for dropoff confirmation, re-prompt
                return JobStatus.DROPOFF_ARRIVED
            else:
                # Navigate to dropoff (or re-navigate if interrupted mid-nav)
                return JobStatus.DROPOFF_NAV

        else:
            # No item on robot — start from pickup
            if state == JobStatus.PICKUP_ARRIVED:
                # Was waiting for pickup confirmation, re-prompt
                return JobStatus.PICKUP_ARRIVED
            elif state in (JobStatus.QUEUED, JobStatus.WAITING_FOR_NAV):
                # Hadn't really started yet
                return JobStatus.WAITING_FOR_NAV
            else:
                # Was navigating to pickup, restart navigation
                return JobStatus.PICKUP_NAV

    def _get_recovery_message(self, job: Job, original_state: int) -> str:
        """Generate a human-readable recovery message."""
        if job.has_item:
            return (
                f"Interrupted with item on robot (was: {STATE_NAMES.get(original_state, 'UNKNOWN')}), "
                f"resuming delivery to {job.dropoff_room}"
            )
        else:
            return (
                f"Interrupted (was: {STATE_NAMES.get(original_state, 'UNKNOWN')}), "
                f"resuming from pickup at {job.pickup_room}"
            )

    def _clear_state_file(self):
        """Remove state file after clean shutdown or when queue empties."""
        try:
            if self._state_file.exists():
                self._state_file.unlink()
                self.get_logger().debug("State file cleared")
        except Exception as e:
            self.get_logger().error(f"Failed to clear state file: {e}")

    # ==================================================================
    # Service handlers
    # ==================================================================

    def _handle_delivery_request(self, request, response):
        pickup = request.pickup_room
        dropoff = request.dropoff_room
        priority = request.priority

        if not pickup or not dropoff:
            response.accepted = False
            response.message = "pickup_room and dropoff_room required"
            response.job_id = ""
            return response

        if pickup == dropoff:
            response.accepted = False
            response.message = "pickup and dropoff cannot be the same room"
            response.job_id = ""
            return response

        with self._lock:
            self._job_counter += 1
            job_id = f"job_{self._job_counter:03d}"

            job = Job(
                job_id=job_id,
                pickup_room=pickup,
                dropoff_room=dropoff,
                priority=priority,
                created_at=time.monotonic(),
            )

            # High priority goes to front of queue
            if priority > 0 and len(self._queue) > 0:
                self._queue.appendleft(job)
                position = 1
            else:
                self._queue.append(job)
                position = len(self._queue)

            self._all_jobs[job_id] = job
            total = len(self._queue) + (1 if self._active_job else 0)
            self._nav_needed = True

        # Save state after adding job
        self._save_state()

        self.get_logger().info(
            f"Job accepted: {job_id} ({pickup} -> {dropoff}), "
            f"queue position {position}, total jobs {total}")

        response.job_id = job_id
        response.accepted = True
        response.message = f"Queued at position {position}"
        return response

    def _handle_cancel(self, request, response):
        job_id = request.job_id

        with self._lock:
            # Cancel current active job
            if not job_id or (self._active_job
                              and self._active_job.job_id == job_id):
                if self._active_job:
                    # Check if item is on robot
                    if self._active_job.has_item:
                        response.success = False
                        response.message = (
                            f"Cannot cancel {self._active_job.job_id}: "
                            f"item is on robot, must complete delivery to {self._active_job.dropoff_room}"
                        )
                        self.get_logger().warn(response.message)
                        return response

                    self._cancel_requested = True
                    response.success = True
                    response.message = (
                        f"Cancelling active job {self._active_job.job_id}")
                    self.get_logger().warn(
                        f"Cancel requested: {self._active_job.job_id}")
                else:
                    response.success = False
                    response.message = "No active job to cancel"
                return response

            # Cancel queued job
            for i, job in enumerate(self._queue):
                if job.job_id == job_id:
                    # Check if this is a recovered job with item on robot
                    if job.has_item:
                        response.success = False
                        response.message = (
                            f"Cannot cancel {job_id}: "
                            f"item is on robot, must complete delivery"
                        )
                        return response

                    self._queue.remove(job)
                    job.state = JobStatus.CANCELLED
                    job.message = "Cancelled while queued"
                    response.success = True
                    response.message = f"Removed {job_id} from queue"
                    self.get_logger().info(f"Cancelled queued job: {job_id}")
                    self._publish_job_status(job)
                    # Save state after removing job
                    self._save_state()
                    return response

        response.success = False
        response.message = f"Job {job_id} not found"
        return response

    def _handle_confirm(self, request, response):
        """Handle pickup/dropoff confirmation."""
        if not self._confirm_event.is_set():
            # Robot is waiting for confirmation
            self._confirm_proceed = request.proceed
            self._confirm_event.set()
            if request.proceed:
                response.success = True
                response.message = "Confirmed, proceeding"
            else:
                response.success = True
                response.message = "Aborted by user"
            self.get_logger().info(
                f"Confirmation received: {'proceed' if request.proceed else 'abort'}"
            )
        else:
            response.success = False
            response.message = "Not awaiting confirmation"
        return response

    # ==================================================================
    # Nav ready signal from supervisor
    # ==================================================================

    def _nav_ready_cb(self, msg):
        self._nav_ready = True
        self._nav_ready_event.set()
        self.get_logger().info("Nav stack ready (supervisor signal)")

    def _nav_shutdown_cb(self, msg):
        self._nav_ready = False
        self._nav_ready_event.clear()
        self.get_logger().info("Nav stack shutdown (supervisor signal)")

    # ==================================================================
    # Job executor (background thread)
    # ==================================================================

    def _job_executor_loop(self):
        """Continuously processes jobs from the queue."""
        while rclpy.ok():
            job = None

            with self._lock:
                if self._queue:
                    job = self._queue.popleft()
                    self._active_job = job

            if job is None:
                time.sleep(0.5)
                continue

            # Save state when job becomes active
            self._save_state()

            if job.interrupted:
                self.get_logger().info(
                    f"Resuming job: {job.job_id} ({job.pickup_room} -> {job.dropoff_room}) "
                    f"from {STATE_NAMES.get(job.resume_state, 'UNKNOWN')}"
                    + (f" [ITEM ON ROBOT]" if job.has_item else "")
                )
            else:
                self.get_logger().info(
                    f"Starting job: {job.job_id} ({job.pickup_room} -> {job.dropoff_room})"
                )

            success = self._execute_job(job)

            with self._lock:
                self._active_job = None
                self._cancel_requested = False

            if success:
                job.state = JobStatus.COMPLETE
                job.message = "Delivery complete"
                self.get_logger().info(f"Job complete: {job.job_id}")
            else:
                if job.state != JobStatus.CANCELLED:
                    job.state = JobStatus.FAILED
                self.get_logger().warn(
                    f"Job ended: {job.job_id} ({job.message})")

            self._publish_job_status(job)

            # Save state after job completes
            self._save_state()

            # If queue is empty, return home
            with self._lock:
                queue_empty = len(self._queue) == 0

            if queue_empty:
                self._return_home()
                # Clear state file when queue is empty and we're home
                self._clear_state_file()

    def _execute_job(self, job: Job) -> bool:
        """
        Execute a single delivery job.

        If job.interrupted is True, resumes from job.resume_state
        instead of starting from the beginning.
        """
        # Determine starting point
        if job.interrupted:
            resume_from = job.resume_state
            job.interrupted = False  # Clear flag now that we're handling it
        else:
            resume_from = JobStatus.QUEUED

        # ============================================================
        # Phase 1: Wait for nav stack (if needed)
        # ============================================================
        if resume_from <= JobStatus.WAITING_FOR_NAV:
            job.state = JobStatus.WAITING_FOR_NAV
            job.message = "Waiting for navigation stack"
            self._publish_job_status(job)
            self._save_state()

            if not self._wait_for_nav(timeout=120.0):
                job.message = "Navigation stack not available"
                return False

            if self._cancel_requested:
                job.state = JobStatus.CANCELLED
                job.message = "Cancelled"
                return False

        # ============================================================
        # Phase 2: Pickup (skip if resuming after pickup)
        # ============================================================
        if resume_from <= JobStatus.PICKUP_ARRIVED:

            # Navigate to pickup (if not resuming at PICKUP_ARRIVED)
            if resume_from <= JobStatus.PICKUP_NAV:
                job.state = JobStatus.PICKUP_NAV
                job.message = f"Navigating to {job.pickup_room}"
                self._publish_job_status(job)
                self._save_state()

                if not self._navigate_to(job.pickup_room):
                    job.message = f"Failed to reach {job.pickup_room}"
                    return False

                if self._cancel_requested:
                    job.state = JobStatus.CANCELLED
                    job.message = "Cancelled"
                    return False

            # Wait for pickup confirmation
            job.state = JobStatus.PICKUP_ARRIVED
            job.message = f"At {job.pickup_room}, waiting for loading"
            self._publish_job_status(job)
            self._save_state()

            confirmed = self._wait_for_confirmation(
                location=job.pickup_room,
                timeout=self.PICKUP_TIMEOUT,
                is_pickup=True,
            )

            if not confirmed:
                # Pickup timeout with no confirmation = cancel job
                job.state = JobStatus.CANCELLED
                job.message = f"No pickup confirmation at {job.pickup_room}, job cancelled"
                self.get_logger().warn(f"[{job.job_id}] {job.message}")
                return False

            # CRITICAL: Mark pickup as confirmed — item is now on robot
            job.pickup_confirmed = True
            self._save_state()  # Save immediately!

            self.get_logger().info(
                f"[{job.job_id}] Pickup confirmed — item loaded, must deliver to {job.dropoff_room}"
            )

            if self._cancel_requested:
                # Can't cancel after pickup — item is on robot
                self.get_logger().warn(
                    f"[{job.job_id}] Cancel requested but item is on robot, must complete delivery"
                )
                self._cancel_requested = False

        # ============================================================
        # Phase 3: Dropoff
        # ============================================================

        # Navigate to dropoff (if not resuming at DROPOFF_ARRIVED)
        if resume_from <= JobStatus.DROPOFF_NAV:
            job.state = JobStatus.DROPOFF_NAV
            job.message = f"Navigating to {job.dropoff_room}"
            self._publish_job_status(job)
            self._save_state()

            if not self._navigate_to(job.dropoff_room):
                job.message = f"Failed to reach {job.dropoff_room}"
                # DON'T return False here — we have the item!
                # Try again or manual intervention needed
                self.get_logger().error(
                    f"[{job.job_id}] CRITICAL: Failed to reach dropoff but item is on robot!"
                )
                # For now, still return False but operator needs to intervene
                return False

            # No cancel check here — can't cancel with item on robot

        # Wait for dropoff confirmation
        job.state = JobStatus.DROPOFF_ARRIVED
        job.message = f"At {job.dropoff_room}, waiting for unloading"
        self._publish_job_status(job)
        self._save_state()

        confirmed = self._wait_for_confirmation(
            location=job.dropoff_room,
            timeout=self.DROPOFF_TIMEOUT,
            is_pickup=False,
        )

        if not confirmed:
            # Dropoff timeout = item left there, mark complete anyway
            self.get_logger().warn(
                f"[{job.job_id}] No dropoff confirmation, proceeding anyway")

        # Item delivered
        job.pickup_confirmed = False
        return True

    def _wait_for_confirmation(self, location, timeout, is_pickup):
        """
        Wait for /job/confirm service call or timeout.

        Publishes awaiting_confirmation so UI/terminal can prompt user.
        Returns True if confirmed (proceed=True), False if timeout or abort.
        """
        self._confirm_event.clear()
        self._confirm_proceed = False

        stage = "PICKUP" if is_pickup else "DROPOFF"

        # Publish what we're waiting for
        msg = String()
        msg.data = (f"{stage} at {location} | "
                    f"Call: ros2 service call /job/confirm "
                    f"job_manager/srv/ConfirmJob '{{proceed: true}}' | "
                    f"Timeout: {timeout:.0f}s")
        self._awaiting_pub.publish(msg)

        self.get_logger().info(
            f"Awaiting {stage} confirmation at {location} (timeout: {timeout:.0f}s)"
        )

        # Wait for confirmation or timeout
        got_response = self._confirm_event.wait(timeout=timeout)

        if got_response and self._confirm_proceed:
            self.get_logger().info(f"{stage} confirmed at {location}")
            return True
        elif got_response and not self._confirm_proceed:
            self.get_logger().warn(f"{stage} aborted by user at {location}")
            return False
        else:
            self.get_logger().warn(f"{stage} timeout at {location}")
            return False

    def _return_home(self):
        """Return to home position after all jobs complete."""
        self.get_logger().info("Queue empty, returning home")

        # nav_needed stays true until we get home
        if not self._nav_ready:
            self.get_logger().warn("Nav not ready, skipping return home")
            with self._lock:
                self._nav_needed = False
            return

        if self._navigate_to(self.HOME_ROOM):
            self.get_logger().info("Returned home")
        else:
            self.get_logger().warn("Failed to return home")

        # Now safe to release nav stack
        with self._lock:
            self._nav_needed = False

    # ==================================================================
    # Navigation helper
    # ==================================================================

    def _navigate_to(self, room_name: str) -> bool:
        """Call /navigate_to_room service. Blocks until complete."""
        if not self._nav_client.service_is_ready():
            self.get_logger().warn(
                f"navigate_to_room service not ready, waiting 10s...")
            if not self._nav_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("navigate_to_room not available")
                return False

        request = NavigateToRoom.Request()
        request.room_name = room_name

        self.get_logger().info(f"Navigating to: {room_name}")

        future = self._nav_client.call_async(request)

        # Spin until result, checking for cancellation
        while not future.done():
            if self._cancel_requested and self._active_job and not self._active_job.has_item:
                # Only allow cancel if we don't have an item
                self.get_logger().warn("Cancel during navigation")
                return False
            time.sleep(0.1)

        try:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    f"Reached {room_name} ({result.message})")
                return True
            else:
                self.get_logger().warn(f"Navigation failed: {result.message}")
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

        self.get_logger().info(
            f"Waiting for nav stack (up to {timeout:.0f}s)...")
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

            # Build queue snapshot
            queue_snapshot = [
                {
                    "job_id":           job.job_id,
                    "pickup_room":      job.pickup_room,
                    "dropoff_room":     job.dropoff_room,
                    "priority":         job.priority,
                    "state":            job.state,
                    "state_name":       STATE_NAMES.get(job.state, "UNKNOWN"),
                    "message":          job.message,
                    "has_item":         job.has_item,
                    "interrupted":      job.interrupted,
                }
                for job in self._queue
            ]

            # Include active job in snapshot
            active_snapshot = None
            if self._active_job:
                active_snapshot = {
                    "job_id":           self._active_job.job_id,
                    "pickup_room":      self._active_job.pickup_room,
                    "dropoff_room":     self._active_job.dropoff_room,
                    "priority":         self._active_job.priority,
                    "state":            self._active_job.state,
                    "state_name":       STATE_NAMES.get(self._active_job.state, "UNKNOWN"),
                    "message":          self._active_job.message,
                    "has_item":         self._active_job.has_item,
                    "pickup_confirmed": self._active_job.pickup_confirmed,
                }

        # Nav needed
        nav_msg = Bool()
        nav_msg.data = nav_needed
        self._nav_needed_pub.publish(nav_msg)

        # Active jobs count
        jobs_msg = Int32()
        jobs_msg.data = count
        self._active_jobs_pub.publish(jobs_msg)

        # Full queue list as JSON string
        queue_msg = String()
        queue_msg.data = json.dumps({
            "active_job": active_snapshot,
            "queue": queue_snapshot,
        })
        self._queue_pub.publish(queue_msg)

    def _publish_job_status(self, job: Job):
        """Publish current job status."""
        msg = JobStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.job_id = job.job_id
        msg.pickup_room = job.pickup_room
        msg.dropoff_room = job.dropoff_room
        msg.priority = job.priority
        msg.state = job.state
        msg.message = job.message
        msg.state_name = STATE_NAMES.get(job.state, "UNKNOWN")

        self._status_pub.publish(msg)


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
