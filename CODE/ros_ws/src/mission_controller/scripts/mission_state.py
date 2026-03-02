"""
Mission State Persistence

Saves mission progress to disk so the mission controller can
detect and report interrupted missions after a restart.

State file: /tmp/mission_state.json
Written before each tile transition step.
Cleared on mission completion or explicit abort.

This is NOT automatic resume — the mission controller reads the
state on startup, logs what was interrupted, and reports it to
the caller. The caller can then decide to re-send the mission.
"""

import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional


MISSION_STATE_FILE = Path("/tmp/mission_state.json")


@dataclass
class MissionState:
    """Persistent mission progress."""
    room_name: str
    tile_sequence: List[int]
    current_step: int            # Index into tile_sequence
    tiles_completed: List[int]
    start_timestamp: float
    last_update_timestamp: float

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2)

    @classmethod
    def from_json(cls, data: str) -> "MissionState":
        d = json.loads(data)
        return cls(**d)


def save_mission_state(state: MissionState) -> bool:
    """Write mission state to disk. Returns True on success."""
    try:
        state.last_update_timestamp = time.time()
        MISSION_STATE_FILE.write_text(state.to_json())
        return True
    except Exception:
        return False


def load_mission_state() -> Optional[MissionState]:
    """Load mission state from disk. Returns None if no state or invalid."""
    try:
        if not MISSION_STATE_FILE.exists():
            return None
        data = MISSION_STATE_FILE.read_text()
        if not data.strip():
            return None
        return MissionState.from_json(data)
    except Exception:
        return None


def clear_mission_state() -> None:
    """Remove mission state file (mission completed or aborted)."""
    try:
        MISSION_STATE_FILE.unlink(missing_ok=True)
    except Exception:
        pass
