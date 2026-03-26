# main.py
# FastAPI server - full backend (no ROS, mock-ready)
# Routes:
#   GET  /                          → serves dashboard HTML
#   GET  /api/rooms                 → all rooms from YAML config
#   GET  /api/rooms/{room_id}       → single room detail
#   POST /api/delivery              → submit delivery request
#   GET  /api/current-task          → robot's active task
#   GET  /api/queue                 → upcoming task queue (max 10)
#   DELETE /api/queue/{task_id}     → cancel a queued task
#   GET  /api/stats                 → queue stats summary
#   GET  /api/health                → health check

import os
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, status, Request
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware

from config import load_config, get_rooms, get_room_by_id
from ros_bridge import init_ros, shutdown_ros, submit_delivery as ros_submit_delivery
from task_store import (
    init as init_store,
    add_task,
    get_current_task,
    get_queue,
    cancel_task,
    get_all_tasks,
    get_queue_stats,
    QUEUE_DEPTH,
)
from models import (
    DeliveryRequest,
    DeliveryResponse,
    CurrentTaskResponse,
    QueueResponse,
    CancelResponse,
)


# ── Lifespan ──────────────────────────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("[server] Starting Robot Delivery System...")
    load_config()       # Parse tiles_config.yaml → room list
    init_store()        # Ensure tasks.json exists and is ready
    init_ros()          # Start ROS2 bridge
    print("[server] Ready ✓")
    yield
    shutdown_ros()      # Shutdown ROS2 bridge
    print("[server] Shutting down...")


# ── App ───────────────────────────────────────────────────────────────────
app = FastAPI(
    title="Robot Delivery System",
    description="Backend API for the campus robot delivery dashboard",
    version="1.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# frontend/ folder inside UI/ in the repo
FRONTEND_DIR = os.getenv(
    "FRONTEND_DIR",
    os.path.join(
        os.path.expanduser("~"),
        "MATERIAL_TRANSFER_ROBOT_2026/CODE/UI/frontend"
    )
)


# ── Serve Dashboard ───────────────────────────────────────────────────────

@app.get("/", include_in_schema=False)
async def serve_dashboard():
    index_path = os.path.join(FRONTEND_DIR, "index.html")
    if not os.path.exists(index_path):
        raise HTTPException(status_code=404, detail="Dashboard HTML not found")
    return FileResponse(index_path)


# ── Rooms ─────────────────────────────────────────────────────────────────

@app.get("/api/rooms", tags=["Rooms"])
async def list_rooms():
    """Returns all delivery rooms from tiles_config.yaml."""
    return get_rooms()


@app.get("/api/rooms/{room_id}", tags=["Rooms"])
async def get_room(room_id: str):
    """Returns details for a single room by ID."""
    room = get_room_by_id(room_id.upper())
    if not room:
        raise HTTPException(status_code=404, detail=f"Room '{room_id}' not found in config")
    return room


# ── Delivery ──────────────────────────────────────────────────────────────

@app.post("/api/delivery", response_model=DeliveryResponse, tags=["Delivery"])
async def submit_delivery(req: DeliveryRequest, request: Request):
    """
    Submit a new delivery request.
    Validates rooms exist in config, pickup != drop, queue not full.
    """
    pickup_room = get_room_by_id(req.pickup)
    if not pickup_room:
        raise HTTPException(status_code=400, detail=f"Pickup room '{req.pickup}' not found")

    drop_room = get_room_by_id(req.drop)
    if not drop_room:
        raise HTTPException(status_code=400, detail=f"Drop room '{req.drop}' not found")

    if req.pickup == req.drop:
        raise HTTPException(status_code=400, detail="Pickup and drop rooms cannot be the same")

    try:
        task = add_task(
            pickup=req.pickup,
            drop=req.drop,
            priority=req.priority,
            requested_by=req.requested_by or "anonymous",
            requester_ip=request.client.host,
        )
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e))

    # ── Send to ROS2 robot ────────────────────────────────────────────
    ros_result = ros_submit_delivery(
        pickup_room=req.pickup,
        dropoff_room=req.drop,
        priority_label=req.priority,
    )

    if not ros_result["accepted"]:
        # ROS rejected — remove from local store
        from task_store import cancel_task
        cancel_task(task.task_id)
        raise HTTPException(
            status_code=503,
            detail=f"Robot rejected request: {ros_result['message']}"
        )

    # Save robot's job_id alongside our REQ-XX id
    from task_store import _read, _write
    data = _read()
    for t in data["tasks"]:
        if t["task_id"] == task.task_id:
            t["ros_job_id"] = ros_result["job_id"]
            break
    _write(data)

    return DeliveryResponse(
        success=True,
        task_id=task.task_id,
        message=f"Delivery request accepted. Task {task.task_id} (Robot Job: {ros_result['job_id']}) added to queue.",
        task=task,
    )


# ── Current Task ──────────────────────────────────────────────────────────

@app.get("/api/current-task")
async def current_task():
    from ros_bridge import get_current_status
    status = get_current_status()
    if status is None:
        return {"status": "idle", "task": None}
    return {"status": "active", "task": status}


# ── Queue ─────────────────────────────────────────────────────────────────

@app.get("/api/queue", response_model=QueueResponse, tags=["Robot Status"])
async def queue():
    """Returns upcoming task queue (max 10 — matches ROS topic queue depth)."""
    tasks = get_queue()
    return QueueResponse(queue_depth=QUEUE_DEPTH, count=len(tasks), tasks=tasks)


@app.delete("/api/queue/{task_id}", response_model=CancelResponse, tags=["Delivery"])
async def cancel(task_id: str, request: Request):
    """
    Cancels a queued task.
    Only the requester (matched by IP) can cancel their own task.
    Cannot cancel in_progress tasks.
    """
    #requester_ip = request.client.host

    try:
        task = cancel_task(task_id.upper())
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e))

    if not task:
        raise HTTPException(status_code=404, detail=f"Task '{task_id}' not found")

    return CancelResponse(success=True, task_id=task.task_id,
                          message=f"Task {task.task_id} has been cancelled.")


# ── Stats / Debug ─────────────────────────────────────────────────────────

@app.get("/api/stats", tags=["Debug"])
async def stats():
    """Summary of all task counts by status."""
    return get_queue_stats()


@app.get("/api/tasks/all", tags=["Debug"])
async def all_tasks():
    """All tasks ever submitted — for debugging."""
    return get_all_tasks()


# ── Health ────────────────────────────────────────────────────────────────


@app.get("/api/my-ip", tags=["Health"])
async def my_ip(request: Request):
    """Returns the caller's IP address — used by frontend for cancel ownership check."""
    return {"ip": request.client.host}


@app.get("/api/health", tags=["Health"])
async def health():
    return {"status": "ok", "service": "robot-delivery-system", "version": "1.0.0"}
