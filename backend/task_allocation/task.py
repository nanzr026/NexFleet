"""Task data model for NEXFLEET."""
from __future__ import annotations
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    TO_PICKUP = "TO_PICKUP"
    TO_DELIVERY = "TO_DELIVERY"
    COMPLETED = "COMPLETED"
    REASSIGNING = "REASSIGNING"
    FAILED = "FAILED"


@dataclass
class Task:
    task_id: str
    pickup: Tuple[int, int]       # (row, col)
    delivery: Tuple[int, int]     # (row, col)
    pickup_station: str
    delivery_station: str
    priority: int = 1             # 1 (normal) to 5 (critical)
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot: Optional[str] = None
    created_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    completed_at: Optional[float] = None
    announced_at: Optional[float] = None

    @classmethod
    def create(
        cls,
        pickup: Tuple[int, int],
        delivery: Tuple[int, int],
        pickup_station: str = "",
        delivery_station: str = "",
        priority: int = 1,
    ) -> "Task":
        return cls(
            task_id=f"T-{uuid.uuid4().hex[:6].upper()}",
            pickup=pickup,
            delivery=delivery,
            pickup_station=pickup_station,
            delivery_station=delivery_station,
            priority=priority,
        )

    @property
    def completion_time(self) -> Optional[float]:
        if self.completed_at and self.started_at:
            return self.completed_at - self.started_at
        return None

    @property
    def total_time(self) -> Optional[float]:
        if self.completed_at:
            return self.completed_at - self.created_at
        return None

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "pickup": list(self.pickup),
            "delivery": list(self.delivery),
            "pickup_station": self.pickup_station,
            "delivery_station": self.delivery_station,
            "priority": self.priority,
            "status": self.status.value,
            "assigned_robot": self.assigned_robot,
            "created_at": self.created_at,
            "started_at": self.started_at,
            "completed_at": self.completed_at,
            "completion_time": self.completion_time,
            "total_time": self.total_time,
        }
