# ros_bridge.py
# Bridges FastAPI backend with ROS2 robot stack.
# Built step by step:
#   Step 1 ✅ — Connect to ROS2 + call /request_delivery (priority mapping)
#   Step 2 — Subscribe /job_status → update task status in tasks.json
#   Step 3 — Subscribe /job/awaiting_confirmation → enable Collect Parcel button
#   Step 4 — Call /cancel_job
#   Step 5 — Call /job/confirm (Collect Parcel clicked)
#   Step 6 — Subscribe /robot_health → alerts
#   Step 7 — Subscribe /system/ready → robot online/offline

import threading
import rclpy
from rclpy.node import Node

# ROS2 service message types
from job_manager.srv import DeliveryJob

import logging
logger = logging.getLogger("ros_bridge")


# ── Priority Mapping ──────────────────────────────────────────────────────
# Maps UI priority labels to ROS2 priority values
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
        # self._cancel_client  → Step 4
        # self._confirm_client → Step 5

        # ── Subscriptions added in later steps ────────────────────────
        # /job_status              → Step 2
        # /job/awaiting_confirmation → Step 3
        # /robot_health            → Step 6
        # /system/ready            → Step 7


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
