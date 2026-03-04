#!/usr/bin/env python3
"""
Job Manager Node

Accepts delivery requests via /request_delivery service, maintains a
priority queue, and executes jobs sequentially by calling the
mission_controller's /navigate_to_room service.

Publishes:
  /system/active_jobs (Int32) — count of queued + active jobs.
    Supervisor watches this: 0 = IDLE, >0 = need nav stack.
  /job_status (JobStatus) — current job state for monitoring.

Services:
  /request_delivery (DeliveryJob) — submit a new delivery job.
  /cancel_job (CancelJob) — cancel a job by ID or cancel current.

Requires:
  /navigate_to_room (mission_controller) — must be available when
    executing jobs. Supervisor brings it up when active_jobs > 0.
  /system/nav_ready (std_msgs/Empty) — supervisor publishes this
    when the full nav stack is up and ready.

Job lifecycle:
  QUEUED -> WAITING_FOR_NAV -> PICKUP_NAV -> PICKUP_ARRIVED ->
  DROPOFF_NAV -> DROPOFF_ARRIVED -> RETURNING -> COMPLETE

  If queue is empty after a job, robot returns home before IDLE.
"""

import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int32, Empty
from job_manager.msg import JobStatus
from job_manager.srv import DeliveryJob, CancelJob
from mission_controller.srv import NavigateToRoom


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


# ============================================================================
# Job Manager
# ============================================================================

class JobManager(Node):

    HOME_ROOM = "home"
    PICKUP_WAIT_SECONDS = 10.0    # Wait at pickup for loading
    DROPOFF_WAIT_SECONDS = 10.0   # Wait at dropoff for unloading

    def __init__(self):
        super().__init__("job_manager")

        # Declare parameters
        self.declare_parameter("home_room", self.HOME_ROOM)
        self.declare_parameter("pickup_wait", self.PICKUP_WAIT_SECONDS)
        self.declare_parameter("dropoff_wait", self.DROPOFF_WAIT_SECONDS)

        self.HOME_ROOM = self.get_parameter("home_room").value
        self.PICKUP_WAIT_SECONDS = self.get_parameter("pickup_wait").value
        self.DROPOFF_WAIT_SECONDS = self.get_parameter("dropoff_wait").value

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

        # Publishers
        self._active_jobs_pub = self.create_publisher(
            Int32, "/system/active_jobs", 10
        )
        self._status_pub = self.create_publisher(
            JobStatus, "/job_status", 10
        )

        # Services
        self.create_service(
            DeliveryJob, "/request_delivery",
            self._handle_delivery_request,
            callback_group=self._cb_group,
        )
        self.create_service(
            CancelJob, "/cancel_job",
            self._handle_cancel,
            callback_group=self._cb_group,
        )

        # Subscription: supervisor signals nav stack is ready
        self.create_subscription(
            Empty, "/system/nav_ready",
            self._nav_ready_cb,
            10,
        )

        # Navigate to room client
        self._nav_client = self.create_client(
            NavigateToRoom, "/navigate_to_room",
            callback_group=self._cb_group,
        )

        # Publish active_jobs count at 1Hz
        self.create_timer(1.0, self._publish_active_jobs)

        # Job executor thread
        self._executor_thread = threading.Thread(
            target=self._job_executor_loop, daemon=True
        )
        self._executor_thread.start()

        self.get_logger().info(
            f"Job Manager started (home: {self.HOME_ROOM})"
        )

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

        self.get_logger().info(
            f"Job accepted: {job_id} ({pickup} -> {dropoff}), "
            f"queue position {position}, total jobs {total}"
        )

        response.job_id = job_id
        response.accepted = True
        response.message = f"Queued at position {position}"
        return response

    def _handle_cancel(self, request, response):
        job_id = request.job_id

        with self._lock:
            # Cancel current active job
            if not job_id or (self._active_job and self._active_job.job_id == job_id):
                if self._active_job:
                    self._cancel_requested = True
                    response.success = True
                    response.message = f"Cancelling active job {self._active_job.job_id}"
                    self.get_logger().warn(
                        f"Cancel requested: {self._active_job.job_id}"
                    )
                else:
                    response.success = False
                    response.message = "No active job to cancel"
                return response

            # Cancel queued job
            for i, job in enumerate(self._queue):
                if job.job_id == job_id:
                    self._queue.remove(job)
                    job.state = JobStatus.CANCELLED
                    job.message = "Cancelled while queued"
                    response.success = True
                    response.message = f"Removed {job_id} from queue"
                    self.get_logger().info(f"Cancelled queued job: {job_id}")
                    self._publish_job_status(job)
                    return response

        response.success = False
        response.message = f"Job {job_id} not found"
        return response

    # ==================================================================
    # Nav ready signal from supervisor
    # ==================================================================

    def _nav_ready_cb(self, msg):
        self._nav_ready = True
        self._nav_ready_event.set()
        self.get_logger().info("Nav stack ready (supervisor signal)")

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

            self.get_logger().info(
                f"Starting job: {job.job_id} "
                f"({job.pickup_room} -> {job.dropoff_room})"
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
                    f"Job ended: {job.job_id} ({job.message})"
                )

            self._publish_job_status(job)

            # If queue is empty, return home
            with self._lock:
                queue_empty = len(self._queue) == 0

            if queue_empty:
                self._return_home()

    def _execute_job(self, job: Job) -> bool:
        """Execute a single delivery job."""

        # Step 1: Wait for nav stack
        job.state = JobStatus.WAITING_FOR_NAV
        job.message = "Waiting for navigation stack"
        self._publish_job_status(job)

        if not self._wait_for_nav(timeout=120.0):
            job.message = "Navigation stack not available"
            return False

        if self._cancel_requested:
            job.state = JobStatus.CANCELLED
            job.message = "Cancelled"
            return False

        # Step 2: Navigate to pickup
        job.state = JobStatus.PICKUP_NAV
        job.message = f"Navigating to {job.pickup_room}"
        self._publish_job_status(job)

        if not self._navigate_to(job.pickup_room):
            job.message = f"Failed to reach {job.pickup_room}"
            return False

        if self._cancel_requested:
            job.state = JobStatus.CANCELLED
            job.message = "Cancelled"
            return False

        # Step 3: Wait at pickup
        job.state = JobStatus.PICKUP_ARRIVED
        job.message = f"At {job.pickup_room}, waiting for loading"
        self._publish_job_status(job)
        self.get_logger().info(
            f"[{job.job_id}] At pickup ({job.pickup_room}), "
            f"waiting {self.PICKUP_WAIT_SECONDS}s"
        )
        time.sleep(self.PICKUP_WAIT_SECONDS)

        if self._cancel_requested:
            job.state = JobStatus.CANCELLED
            job.message = "Cancelled"
            return False

        # Step 4: Navigate to dropoff
        job.state = JobStatus.DROPOFF_NAV
        job.message = f"Navigating to {job.dropoff_room}"
        self._publish_job_status(job)

        if not self._navigate_to(job.dropoff_room):
            job.message = f"Failed to reach {job.dropoff_room}"
            return False

        if self._cancel_requested:
            job.state = JobStatus.CANCELLED
            job.message = "Cancelled"
            return False

        # Step 5: Wait at dropoff
        job.state = JobStatus.DROPOFF_ARRIVED
        job.message = f"At {job.dropoff_room}, waiting for unloading"
        self._publish_job_status(job)
        self.get_logger().info(
            f"[{job.job_id}] At dropoff ({job.dropoff_room}), "
            f"waiting {self.DROPOFF_WAIT_SECONDS}s"
        )
        time.sleep(self.DROPOFF_WAIT_SECONDS)

        return True

    def _return_home(self):
        """Return to home position after all jobs complete."""
        self.get_logger().info("Queue empty, returning home")

        if not self._nav_ready:
            self.get_logger().warn("Nav not ready, skipping return home")
            return

        if self._navigate_to(self.HOME_ROOM):
            self.get_logger().info("Returned home")
        else:
            self.get_logger().warn("Failed to return home")

    # ==================================================================
    # Navigation helper
    # ==================================================================

    def _navigate_to(self, room_name: str) -> bool:
        """Call /navigate_to_room service. Blocks until complete."""
        if not self._nav_client.service_is_ready():
            self.get_logger().warn(
                f"navigate_to_room service not ready, "
                f"waiting 10s..."
            )
            if not self._nav_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("navigate_to_room not available")
                return False

        request = NavigateToRoom.Request()
        request.room_name = room_name

        self.get_logger().info(f"Navigating to: {room_name}")

        future = self._nav_client.call_async(request)

        # Spin until result, checking for cancellation
        while not future.done():
            if self._cancel_requested:
                # Can't cancel a service call, but we can stop waiting
                self.get_logger().warn("Cancel during navigation")
                return False
            time.sleep(0.1)

        try:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    f"Reached {room_name} ({result.message})"
                )
                return True
            else:
                self.get_logger().warn(
                    f"Navigation failed: {result.message}"
                )
                return False
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            return False

    def _wait_for_nav(self, timeout: float) -> bool:
        """Wait for supervisor to signal nav stack is ready."""
        if self._nav_ready:
            return True

        self.get_logger().info(
            f"Waiting for nav stack (up to {timeout:.0f}s)..."
        )
        return self._nav_ready_event.wait(timeout=timeout)

    # ==================================================================
    # Publishers
    # ==================================================================

    def _publish_active_jobs(self):
        """Publish count of active + queued jobs at 1Hz."""
        with self._lock:
            count = len(self._queue)
            if self._active_job is not None:
                count += 1

        msg = Int32()
        msg.data = count
        self._active_jobs_pub.publish(msg)

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

        state_names = {
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
        msg.state_name = state_names.get(job.state, "UNKNOWN")

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
