"""Contract-Net distributed task auction for NEXFLEET."""
from __future__ import annotations
import asyncio
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from backend.task_allocation.task import Task, TaskStatus
from backend.simulation.amr_agent import AMRState, RobotStatus


@dataclass
class Bid:
    robot_id: str
    task_id: str
    bid_value: float      # lower is better
    distance_to_pickup: float
    battery: float
    workload: int
    submitted_at: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "robot_id": self.robot_id,
            "task_id": self.task_id,
            "bid_value": round(self.bid_value, 3),
            "distance_to_pickup": self.distance_to_pickup,
            "battery": self.battery,
            "workload": self.workload,
        }


class AuctionEvent:
    TASK_ANNOUNCED = "TASK_ANNOUNCED"
    BID_SUBMITTED = "BID_SUBMITTED"
    TASK_AWARDED = "TASK_AWARDED"
    TASK_NO_BIDDERS = "TASK_NO_BIDDERS"


@dataclass
class AuctionRecord:
    task_id: str
    event: str
    robot_id: Optional[str]
    bid: Optional[Bid]
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "event": self.event,
            "robot_id": self.robot_id,
            "bid": self.bid.to_dict() if self.bid else None,
            "timestamp": self.timestamp,
        }


def compute_bid(
    robot_state: AMRState,
    task: Task,
) -> Optional[Bid]:
    """
    Compute a bid for a task from a robot's perspective.
    Lower bid_value = better candidate.

    Formula:
        bid = distance_to_pickup
            + 0.5 * workload_penalty
            + battery_penalty
            + priority_bonus
    """
    if robot_state.status == RobotStatus.FAILED:
        return None
    if robot_state.battery < 15.0:
        return None  # Too low battery

    dist = math.hypot(
        robot_state.row - task.pickup[0],
        robot_state.col - task.pickup[1],
    )
    workload = len(robot_state.task_queue) + (1 if robot_state.current_task_id else 0)
    workload_penalty = workload * 10.0
    battery_penalty = max(0.0, (50.0 - robot_state.battery)) * 0.3
    priority_bonus = (5 - task.priority) * 2.0  # higher priority tasks cost less

    bid_value = dist + workload_penalty + battery_penalty + priority_bonus

    return Bid(
        robot_id=robot_state.robot_id,
        task_id=task.task_id,
        bid_value=bid_value,
        distance_to_pickup=dist,
        battery=robot_state.battery,
        workload=workload,
    )


class AuctionManager:
    """Manages task announcement, bidding, and awarding."""

    def __init__(self):
        self._history: List[AuctionRecord] = []
        self._active_bids: Dict[str, List[Bid]] = {}  # task_id -> bids
        self._latest_bids: List[Bid] = []  # last auction's bids for dashboard
        self._latest_winner: Optional[str] = None
        self._latest_task_id: Optional[str] = None

    def run_auction(
        self,
        task: Task,
        robot_states: List[AMRState],
    ) -> Optional[str]:
        """
        Run a synchronous auction for a task.
        Returns winning robot_id or None if no valid bids.
        """
        # Announce
        self._history.append(AuctionRecord(
            task_id=task.task_id,
            event=AuctionEvent.TASK_ANNOUNCED,
            robot_id=None,
            bid=None,
        ))

        # Collect bids
        bids: List[Bid] = []
        for state in robot_states:
            bid = compute_bid(state, task)
            if bid is not None:
                bids.append(bid)
                self._history.append(AuctionRecord(
                    task_id=task.task_id,
                    event=AuctionEvent.BID_SUBMITTED,
                    robot_id=state.robot_id,
                    bid=bid,
                ))

        self._active_bids[task.task_id] = bids
        self._latest_bids = bids
        self._latest_task_id = task.task_id

        if not bids:
            self._history.append(AuctionRecord(
                task_id=task.task_id,
                event=AuctionEvent.TASK_NO_BIDDERS,
                robot_id=None,
                bid=None,
            ))
            return None

        # Award to lowest bid
        winner = min(bids, key=lambda b: b.bid_value)
        self._latest_winner = winner.robot_id
        self._history.append(AuctionRecord(
            task_id=task.task_id,
            event=AuctionEvent.TASK_AWARDED,
            robot_id=winner.robot_id,
            bid=winner,
        ))
        return winner.robot_id

    def get_history(self, limit: int = 50) -> List[dict]:
        return [r.to_dict() for r in self._history[-limit:]]

    def get_latest_auction(self) -> dict:
        return {
            "task_id": self._latest_task_id,
            "bids": [b.to_dict() for b in self._latest_bids],
            "winner": self._latest_winner,
        }
