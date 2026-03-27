# ros_bridge.py
# Bridges FastAPI backend with ROS2 robot stack.
# Built step by step:
#   Step 1 ✅ — Connect to ROS2 + call /request_delivery (priority mapping)
#   Step 2 ✅ — Subscribe /job_status → update task status in tasks.json
#   Step 3 — Subscribe /job/awaiting_confirmation → enable Collect Parcel button
#   Step 4 — Call /cancel_job
#   Step 5 — Call /job/confirm (Collect Parcel clicked)
#   Step 6 — Subscribe /robot_health → alerts
#   Step 7 — Subscribe /system/ready → robot online/offline

import threading
import rclpy
from rclpy.node import Node

# ROS2 message & service types
from job_manager.srv import DeliveryJob
from job_manager.srv import ConfirmJob
from job_manager.msg import JobStatus
from std_msgs.msg import String as StringMsg
from system_supervisor.msg import RobotHealth

import logging
logger = logging.getLogger("ros_bridge")


# ── Job State Mapping ─────────────────────────────────────────────────────
# Maps ROS2 state numbers → human readable labels for the UI
# From API reference (job_status.state field)
STATE_MAP = {
    0: "Queued",
    1: "Preparing Navigation",
    2: "Navigating to Pickup",
    3: "Arrived at Pickup",       # → enable Collect Parcel button
    4: "Navigating to Drop",
    5: "Arrived at Drop",
    6: "Returning Home",
    7: "Complete",
    8: "Failed",
    9: "Cancelled",
}

# Maps ROS2 state numbers → our tasks.json status values
STATUS_MAP = {
    0: "queued",
    1: "queued",
    2: "in_progress",
    3: "in_progress",
    4: "in_progress",
    5: "in_progress",
    6: "in_progress",
    7: "done",
    8: "failed",
    9: "cancelled",
}


# ── Priority Mapping ──────────────────────────────────────────────────────
# Maps UI priority labels → ROS2 priority values
# From API reference: 0 = normal, 1 = high
PRIORITY_MAP = {
    "Low":    0,   # normal
    "Medium": 0,   # normal
    "High":   1,   # high
}


# ── ROS2 Node ─────────────────────────────────────────────────────────────
class RobotBridgeNode(Node):
    """
    Single ROS2 node that handles all communication
    between the FastAPI backend and the robot stack.
    """

    def __init__(self):
        super().__init__("web_dashboard_bridge")
        logger.info("[ros_bridge] Node initialized")

        # ── Step 1: Service clients ────────────────────────────────────
        self._delivery_client = self.create_client(
            DeliveryJob,
            "/request_delivery"
        )

        # Wait for service to be available (non-blocking with timeout)
        logger.info("[ros_bridge] Waiting for /request_delivery service...")
        available = self._delivery_client.wait_for_service(timeout_sec=5.0)
        if available:
            logger.info("[ros_bridge] /request_delivery service ready ✓")
        else:
            logger.warning("[ros_bridge] /request_delivery service not available — robot may be offline")

        # ── More service clients added in later steps ──────────────────
        # self._cancel_client  → Step 4 (skipped)

        # ── Step 5: Confirm job service client ────────────────────────────
        self._confirm_client = self.create_client(
            ConfirmJob,
            "/job/confirm"
        )
        logger.info("[ros_bridge] /job/confirm service client created")

        # ── Step 2: Job status subscription ───────────────────────────
        # Stores latest job status in memory for fast API reads
        self._current_job_status = None

        self._job_status_sub = self.create_subscription(
            JobStatus,
            "/job_status",
            self._on_job_status,
            10                      # queue depth
        )
        logger.info("[ros_bridge] Subscribed to /job_status")

        # ── Step 3: Awaiting confirmation subscription ─────────────────
        self._awaiting_confirmation = {"waiting": False, "data": ""}

        self._awaiting_confirmation_sub = self.create_subscription(
            StringMsg,
            "/job/awaiting_confirmation",
            self._on_awaiting_confirmation,
            10
        )
        logger.info("[ros_bridge] Subscribed to /job/awaiting_confirmation")

        # ── Subscriptions added in later steps ────────────────────────
        # ── Step 6: Robot health subscription ─────────────────────────────
        self._robot_health = None

        self._robot_health_sub = self.create_subscription(
            RobotHealth,
            "/robot_health",
            self._on_robot_health,
            10
        )
        logger.info("[ros_bridge] Subscribed to /robot_health")

        # /system/ready  → Step 7

    # ── Step 1: Submit Delivery Request ───────────────────────────────────
    def send_delivery_request(
        self,
        pickup_room: str,
        dropoff_room: str,
        priority_label: str,   # "Low" / "Medium" / "High" from UI
    ) -> dict:
        """
        Calls /request_delivery service on the robot.
        Maps UI priority label to ROS2 priority value (0 or 1).

        Returns:
            { "accepted": bool, "job_id": str, "message": str }
        """

        # ── Map priority label → ROS2 value ───────────────────────────
        priority_value = PRIORITY_MAP.get(priority_label, 0)
        logger.info(
            f"[ros_bridge] Sending delivery: {pickup_room} → {dropoff_room} "
            f"| priority: {priority_label} → {priority_value}"
        )

        # ── Check service is available ────────────────────────────────
        if not self._delivery_client.service_is_ready():
            logger.error("[ros_bridge] /request_delivery service not available")
            return {
                "accepted": False,
                "job_id": "",
                "message": "Robot is offline. Cannot submit delivery request."
            }

        # ── Build request ─────────────────────────────────────────────
        request = DeliveryJob.Request()
        request.pickup_room  = pickup_room
        request.dropoff_room = dropoff_room
        request.priority     = priority_value   # 0=normal, 1=high

        # ── Call service (synchronous) ────────────────────────────────
        future = self._delivery_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            logger.error("[ros_bridge] /request_delivery call timed out")
            return {
                "accepted": False,
                "job_id": "",
                "message": "Request timed out. Robot may be busy."
            }

        response = future.result()
        logger.info(
            f"[ros_bridge] /request_delivery response: "
            f"accepted={response.accepted} job_id={response.job_id} "
            f"message={response.message}"
        )

        return {
            "accepted": response.accepted,
            "job_id":   response.job_id,
            "message":  response.message,
        }


    # ── Step 2: Job Status Callback ────────────────────────────────────────
    def _on_job_status(self, msg: JobStatus) -> None:
        """
        Called every time robot publishes to /job_status.
        Updates:
          1. In-memory current status (for fast API reads)
          2. tasks.json (keeps local task history accurate)
        """
        state_num  = msg.state
        state_name = STATE_MAP.get(state_num, "Unknown")
        status     = STATUS_MAP.get(state_num, "queued")

        logger.info(
            f"[ros_bridge] /job_status: "
            f"job_id={msg.job_id} state={state_num} ({state_name})"
        )

        # ── Store latest status in memory ─────────────────────────────
        self._current_job_status = {
            "ros_job_id": msg.job_id,
            "pickup":     msg.pickup_room,
            "drop":       msg.dropoff_room,
            "priority":   msg.priority,
            "state":      state_num,
            "state_name": state_name,
            "status":     status,
            "message":    msg.message,
        }

        # ── Sync to tasks.json ─────────────────────────────────────────
        try:
            from task_store import _read, _write
            data = _read()
            for t in data["tasks"]:
                if t.get("ros_job_id") == msg.job_id:
                    t["status"]     = status
                    t["state_name"] = state_name
                    t["message"]    = msg.message
                    _write(data)
                    logger.info(
                        f"[ros_bridge] Synced task {t['task_id']} "
                        f"→ {status} ({state_name})"
                    )
                    break
        except Exception as e:
            logger.error(f"[ros_bridge] Failed to sync tasks.json: {e}")


    def get_current_job_status(self) -> dict:
        """
        Returns latest job status from /job_status topic.
        Called by GET /api/current-task in main.py.
        Returns None if robot is idle.
        """
        return self._current_job_status


    # ── Step 3: Awaiting Confirmation Callback ─────────────────────────────
    def _on_awaiting_confirmation(self, msg: StringMsg) -> None:
        """
        Called when robot publishes to /job/awaiting_confirmation.
        Fired at PICKUP_ARRIVED and DROPOFF_ARRIVED.
        Backup trigger for Collect Parcel button.
        """
        logger.info(f"[ros_bridge] /job/awaiting_confirmation: {msg.data}")
        self._awaiting_confirmation = {
            "waiting": True,
            "data": msg.data
        }

    def get_awaiting_confirmation(self) -> dict:
        """Returns current awaiting confirmation state."""
        return self._awaiting_confirmation

    def clear_awaiting_confirmation(self) -> None:
        """Called after user confirms — resets the flag."""
        self._awaiting_confirmation = {"waiting": False, "data": ""}
    # ── Step 5: Confirm Job ────────────────────────────────────────────────
    def confirm_job(self, proceed: bool = True) -> dict:
        """
        Calls /job/confirm service on robot.
        proceed=True  → robot continues to next step
        proceed=False → robot aborts job
        Called when user clicks Collect Parcel or Parcel Received.
        """
        if not self._confirm_client.service_is_ready():
            logger.error("[ros_bridge] /job/confirm service not available")
            return {"success": False, "message": "Robot is offline."}

        request = ConfirmJob.Request()
        request.proceed = proceed

        future = self._confirm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            logger.error("[ros_bridge] /job/confirm timed out")
            return {"success": False, "message": "Request timed out."}

        response = future.result()
        logger.info(f"[ros_bridge] /job/confirm response: {response.success} {response.message}")
        return {"success": response.success, "message": response.message}
    
    # ── Step 6: Robot Health Callback ─────────────────────────────────────
    def _on_robot_health(self, msg: RobotHealth) -> None:
        """
        Called every 1 Hz from /robot_health topic.
        Stores latest health data in memory for alerts section.
        """
        self._robot_health = {
            "cpu_percent":      msg.cpu_percent,
            "memory_used_mb":   msg.memory_used_mb,
            "memory_total_mb":  msg.memory_total_mb,
            "system_state":     msg.system_state,
            "supervisor_state": msg.supervisor_state,
            "system_message":   msg.system_message,
            "autonomous_enabled": msg.autonomous_enabled,
        }
        logger.debug(f"[ros_bridge] /robot_health: cpu={msg.cpu_percent}% msg={msg.system_message}")

    def get_robot_health(self) -> dict:
        """Returns latest robot health data."""
        return self._robot_health
    

# ── Singleton ─────────────────────────────────────────────────────────────
# One node instance shared across all FastAPI requests
_node: RobotBridgeNode = None
_ros_thread: threading.Thread = None


def init_ros() -> None:
    """
    Initializes rclpy and starts the ROS2 node in a background thread.
    Called once at FastAPI startup.
    """
    global _node, _ros_thread

    rclpy.init()
    _node = RobotBridgeNode()

    # Spin in background thread so it doesn't block FastAPI
    _ros_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    _ros_thread.start()

    logger.info("[ros_bridge] ROS2 bridge started in background thread")


def shutdown_ros() -> None:
    """Called at FastAPI shutdown."""
    global _node
    if _node:
        _node.destroy_node()
    rclpy.shutdown()
    logger.info("[ros_bridge] ROS2 bridge shut down")


def get_node() -> RobotBridgeNode:
    """Returns the singleton node instance."""
    return _node


# ── Public API (called by main.py routes) ─────────────────────────────────

def submit_delivery(pickup_room: str, dropoff_room: str, priority_label: str) -> dict:
    """
    Called by POST /api/delivery route.
    Sends delivery request to robot and returns result.
    """
    node = get_node()
    if node is None:
        return {
            "accepted": False,
            "job_id": "",
            "message": "ROS2 bridge not initialized."
        }
    return node.send_delivery_request(pickup_room, dropoff_room, priority_label)


def get_current_status() -> dict:
    """
    Called by GET /api/current-task route.
    Returns latest job status from /job_status topic.
    Returns None if robot is idle.
    """
    node = get_node()
    if node is None:
        return None
    return node.get_current_job_status()


def get_awaiting_confirmation() -> dict:
    """Called by GET /api/awaiting-confirmation route."""
    node = get_node()
    if node is None:
        return {"waiting": False, "data": ""}
    return node.get_awaiting_confirmation()


def clear_awaiting_confirmation() -> None:
    """Called after /job/confirm is sent."""
    node = get_node()
    if node:
        node.clear_awaiting_confirmation()
def confirm_job(proceed: bool = True) -> dict:
    """Called by POST /api/confirm-collection and /api/confirm-delivery."""
    node = get_node()
    if node is None:
        return {"success": False, "message": "ROS2 bridge not initialized."}
    return node.confirm_job(proceed)
def get_robot_health() -> dict:
    """Called by GET /api/robot-health route."""
    node = get_node()
    if node is None:
        return None
    return node.get_robot_health()