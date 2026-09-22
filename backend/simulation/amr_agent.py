"""Independent AMR agent — each robot runs autonomously."""
from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


class RobotStatus(str, Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    WAITING = "WAITING"
    REROUTING = "REROUTING"
    BLOCKED = "BLOCKED"
    CHARGING = "CHARGING"
    FAILED = "FAILED"


@dataclass
class AMRState:
    """Full state snapshot of an AMR."""
    robot_id: str
    row: int
    col: int
    prev_row: int
    prev_col: int
    velocity: float  # cells per tick
    direction: str  # N S E W NONE
    battery: float  # 0-100
    status: RobotStatus
    destination: Optional[Tuple[int, int]]
    current_path: List[Tuple[int, int]]
    path_index: int
    movement_intent: Optional[Tuple[int, int]]  # next intended position
    current_task_id: Optional[str]
    task_queue: List[str]
    waiting_ticks: int
    idle_ticks: int
    distance_travelled: float
    completed_tasks: int
    conflict_count: int
    reroute_count: int
    local_map_version: int
    known_neighbor_ids: List[str]
    trajectory_history: List[Tuple[int, int]]  # last N positions
    tick: int

    def pos(self) -> Tuple[int, int]:
        return (self.row, self.col)

    def get_short_horizon(self, steps: int = 5) -> List[Tuple[int, int]]:
        """Return the next N positions on the current path."""
        start = self.path_index
        end = min(start + steps + 1, len(self.current_path))
        return self.current_path[start:end]

    def to_dict(self) -> dict:
        return {
            "robot_id": self.robot_id,
            "row": self.row,
            "col": self.col,
            "prev_row": self.prev_row,
            "prev_col": self.prev_col,
            "velocity": self.velocity,
            "direction": self.direction,
            "battery": self.battery,
            "status": self.status.value,
            "destination": list(self.destination) if self.destination else None,
            "current_path": [list(p) for p in self.current_path],
            "path_index": self.path_index,
            "movement_intent": list(self.movement_intent) if self.movement_intent else None,
            "current_task_id": self.current_task_id,
            "task_queue": self.task_queue,
            "waiting_ticks": self.waiting_ticks,
            "idle_ticks": self.idle_ticks,
            "distance_travelled": self.distance_travelled,
            "completed_tasks": self.completed_tasks,
            "conflict_count": self.conflict_count,
            "reroute_count": self.reroute_count,
            "known_neighbor_ids": self.known_neighbor_ids,
            "tick": self.tick,
        }


class AMRAgent:
    """An independent autonomous mobile robot agent."""

    HISTORY_LEN = 20
    NEIGHBOR_RADIUS = 8  # cells
    BATTERY_DRAIN_PER_MOVE = 0.05
    BATTERY_DRAIN_IDLE = 0.01
    LOW_BATTERY_THRESHOLD = 20.0

    def __init__(
        self,
        robot_id: str,
        start_row: int,
        start_col: int,
        battery: float = 100.0,
    ):
        self.robot_id = robot_id
        self.state = AMRState(
            robot_id=robot_id,
            row=start_row,
            col=start_col,
            prev_row=start_row,
            prev_col=start_col,
            velocity=0.0,
            direction="NONE",
            battery=battery,
            status=RobotStatus.IDLE,
            destination=None,
            current_path=[],
            path_index=0,
            movement_intent=None,
            current_task_id=None,
            task_queue=[],
            waiting_ticks=0,
            idle_ticks=0,
            distance_travelled=0.0,
            completed_tasks=0,
            conflict_count=0,
            reroute_count=0,
            local_map_version=0,
            known_neighbor_ids=[],
            trajectory_history=[(start_row, start_col)],
            tick=0,
        )
        # Task phase tracking
        self._task_phase: str = "NONE"  # NONE / TO_PICKUP / TO_DELIVERY
        self._pickup_pos: Optional[Tuple[int, int]] = None
        self._delivery_pos: Optional[Tuple[int, int]] = None

    # ------------------------------------------------------------------
    # Path management
    # ------------------------------------------------------------------

    def set_path(self, path: List[Tuple[int, int]]):
        """Assign a new path to follow."""
        self.state.current_path = path
        self.state.path_index = 0
        if len(path) > 1:
            self.state.destination = path[-1]
            self.state.status = RobotStatus.MOVING
            self.state.velocity = 1.0
            self._update_intent()
        elif len(path) == 1:
            self.state.destination = path[0]

    def _update_intent(self):
        idx = self.state.path_index + 1
        if idx < len(self.state.current_path):
            self.state.movement_intent = self.state.current_path[idx]
        else:
            self.state.movement_intent = None

    def compute_direction(self, fr: Tuple[int, int], to: Tuple[int, int]) -> str:
        dr = to[0] - fr[0]
        dc = to[1] - fr[1]
        if dr < 0:
            return "N"
        if dr > 0:
            return "S"
        if dc < 0:
            return "W"
        if dc > 0:
            return "E"
        return "NONE"

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def step_move(self) -> bool:
        """Advance one cell along current path. Returns True if moved."""
        s = self.state
        if s.status == RobotStatus.FAILED:
            return False
        if not s.current_path or s.path_index >= len(s.current_path) - 1:
            s.velocity = 0.0
            s.direction = "NONE"
            s.movement_intent = None
            if s.status == RobotStatus.MOVING:
                s.status = RobotStatus.IDLE
            return False

        next_idx = s.path_index + 1
        next_pos = s.current_path[next_idx]

        # Move
        s.prev_row, s.prev_col = s.row, s.col
        s.direction = self.compute_direction((s.row, s.col), next_pos)
        s.row, s.col = next_pos
        s.path_index = next_idx
        s.distance_travelled += 1.0
        s.battery = max(0.0, s.battery - self.BATTERY_DRAIN_PER_MOVE)
        s.status = RobotStatus.MOVING
        s.velocity = 1.0
        s.waiting_ticks = 0

        # Update history
        s.trajectory_history.append((s.row, s.col))
        if len(s.trajectory_history) > self.HISTORY_LEN:
            s.trajectory_history.pop(0)

        self._update_intent()
        return True

    def mark_waiting(self):
        self.state.status = RobotStatus.WAITING
        self.state.velocity = 0.0
        self.state.waiting_ticks += 1
        self.state.battery = max(0.0, self.state.battery - self.BATTERY_DRAIN_IDLE)

    def mark_rerouting(self):
        self.state.status = RobotStatus.REROUTING
        self.state.reroute_count += 1

    def mark_conflict(self):
        self.state.conflict_count += 1

    def mark_idle(self):
        self.state.status = RobotStatus.IDLE
        self.state.velocity = 0.0
        self.state.idle_ticks += 1
        self.state.movement_intent = None

    def fail(self):
        self.state.status = RobotStatus.FAILED
        self.state.velocity = 0.0
        self.state.movement_intent = None

    def recover(self):
        if self.state.status == RobotStatus.FAILED:
            self.state.status = RobotStatus.IDLE
            self.state.current_path = []
            self.state.path_index = 0
            self.state.destination = None
            self.state.movement_intent = None

    def has_reached_destination(self) -> bool:
        s = self.state
        if not s.destination:
            return False
        return (s.row, s.col) == s.destination

    def update_neighbors(self, all_states: List[AMRState]):
        """Update list of nearby robot IDs."""
        my_r, my_c = self.state.row, self.state.col
        self.state.known_neighbor_ids = [
            st.robot_id for st in all_states
            if st.robot_id != self.robot_id
            and math.hypot(st.row - my_r, st.col - my_c) <= self.NEIGHBOR_RADIUS
        ]

    def snapshot(self) -> AMRState:
        from copy import deepcopy
        return deepcopy(self.state)
