"""Deadlock detection and resolution for NEXFLEET."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from backend.simulation.amr_agent import AMRState, RobotStatus


WAITING_DEADLOCK_THRESHOLD = 8   # ticks waiting before suspected deadlock
CYCLE_HISTORY_LEN = 6


@dataclass
class DeadlockEvent:
    tick: int
    robots: List[str]
    resolution: str

    def to_dict(self) -> dict:
        return {"tick": self.tick, "robots": self.robots, "resolution": self.resolution}


class DeadlockDetector:
    """Detects and resolves robot deadlocks."""

    def __init__(self):
        self._events: List[DeadlockEvent] = []
        self._position_history: Dict[str, List[Tuple[int, int]]] = {}  # robot -> recent positions

    def update_history(self, states: List[AMRState]):
        for s in states:
            if s.robot_id not in self._position_history:
                self._position_history[s.robot_id] = []
            self._position_history[s.robot_id].append((s.row, s.col))
            if len(self._position_history[s.robot_id]) > CYCLE_HISTORY_LEN * 2:
                self._position_history[s.robot_id].pop(0)

    def detect_deadlocks(
        self,
        states: List[AMRState],
        tick: int,
    ) -> List[List[str]]:
        """
        Detect deadlocked robot groups.
        Returns list of groups (each group = list of robot_ids in deadlock).
        """
        deadlocked_groups = []

        # Strategy 1: Long waiting time
        long_waiters = [
            s for s in states
            if s.status == RobotStatus.WAITING
            and s.waiting_ticks >= WAITING_DEADLOCK_THRESHOLD
            and s.current_task_id is not None
        ]

        if len(long_waiters) >= 2:
            deadlocked_groups.append([s.robot_id for s in long_waiters])
        elif len(long_waiters) == 1:
            deadlocked_groups.append([long_waiters[0].robot_id])

        # Strategy 2: Lack of progress (robot has not moved in N ticks)
        for s in states:
            if s.robot_id in self._position_history:
                hist = self._position_history[s.robot_id]
                if (
                    len(hist) >= CYCLE_HISTORY_LEN
                    and s.current_task_id is not None
                    and s.status not in (RobotStatus.IDLE, RobotStatus.CHARGING, RobotStatus.FAILED)
                    and len(set(hist[-CYCLE_HISTORY_LEN:])) == 1  # no movement
                ):
                    # Single robot stuck
                    group = [s.robot_id]
                    if not any(s.robot_id in g for g in deadlocked_groups):
                        deadlocked_groups.append(group)

        return deadlocked_groups

    def resolve(
        self,
        group: List[str],
        states: List[AMRState],
        tick: int,
    ) -> str:
        """
        Resolve a deadlock group.
        Returns the resolution strategy applied.
        """
        state_map = {s.robot_id: s for s in states}

        # Choose the robot with the highest waiting time to yield/reroute
        group_states = [state_map[rid] for rid in group if rid in state_map]
        if not group_states:
            return "NO_ACTION"

        # Sort by waiting ticks descending; that robot yields
        group_states.sort(key=lambda s: -s.waiting_ticks)
        yielder = group_states[0]

        resolution = f"YIELD_REROUTE:{yielder.robot_id}"
        self._events.append(DeadlockEvent(
            tick=tick,
            robots=group,
            resolution=resolution,
        ))
        return resolution

    def get_events(self, limit: int = 20) -> List[dict]:
        return [e.to_dict() for e in self._events[-limit:]]
