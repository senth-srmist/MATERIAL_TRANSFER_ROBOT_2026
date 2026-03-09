# models.py
# Pydantic models for request validation and API response shapes

from pydantic import BaseModel, field_validator
from typing import Optional, Literal
from datetime import datetime


# ── Request Models ────────────────────────────────────────────────────────

class DeliveryRequest(BaseModel):
    """Payload sent by the frontend form when submitting a delivery."""
    pickup: str
    drop: str
    priority: Literal["Low", "Medium", "High"] = "Medium"
    requested_by: Optional[str] = "anonymous"

    @field_validator("pickup", "drop")
    @classmethod
    def must_not_be_empty(cls, v: str) -> str:
        if not v or not v.strip():
            raise ValueError("Room ID must not be empty")
        return v.strip().upper()


# ── Response Models ───────────────────────────────────────────────────────

class Task(BaseModel):
    """Full task object stored in tasks.json and returned by API."""
    task_id: str
    pickup: str
    drop: str
    priority: str
    status: Literal["queued", "in_progress", "done", "cancelled"]
    requested_by: str
    requester_ip: str = ""
    timestamp: str                  # ISO format string


class DeliveryResponse(BaseModel):
    """Response after successfully submitting a delivery request."""
    success: bool
    task_id: str
    message: str
    task: Task


class CurrentTaskResponse(BaseModel):
    """Response for GET /api/current-task."""
    status: Literal["idle", "in_progress"]
    task: Optional[Task] = None


class QueueResponse(BaseModel):
    """Response for GET /api/queue."""
    queue_depth: int                # Max allowed (10)
    count: int                      # How many currently queued
    tasks: list[Task]


class CancelResponse(BaseModel):
    """Response after cancelling a task."""
    success: bool
    task_id: str
    message: str
