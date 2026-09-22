"""Metrics collection for NEXFLEET experiments."""
from __future__ import annotations
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class SimulationMetrics:
    mode: str  # "traditional" or "nexfleet"
    start_time: float = field(default_factory=time.time)
    end_time: Optional[float] = None

    # Task metrics
    total_tasks: int = 0
    completed_tasks: int = 0
    task_completion_times: List[float] = field(default_factory=list)
    task_start_tick: Dict[str, int] = field(default_factory=dict)
    task_end_tick: Dict[str, int] = field(default_factory=dict)

    # Movement metrics
    total_distance: float = 0.0
    per_robot_distance: Dict[str, float] = field(default_factory=dict)

    # Time metrics
    total_waiting_ticks: int = 0
    total_idle_ticks: int = 0
    per_robot_waiting: Dict[str, int] = field(default_factory=dict)

    # Safety metrics
    collisions: int = 0
    conflicts_detected: int = 0
    conflicts_resolved: int = 0
    deadlocks_detected: int = 0
    deadlocks_resolved: int = 0
    reroutes: int = 0
    task_reassignments: int = 0

    # Planning
    planning_times_ms: List[float] = field(default_factory=list)

    # AI metrics
    gnn_inference_times_ms: List[float] = field(default_factory=list)
    marl_inference_times_ms: List[float] = field(default_factory=list)

    # Communication
    messages_sent: int = 0
    messages_dropped: int = 0

    # Ticks
    total_ticks: int = 0
    makespan_ticks: int = 0  # ticks from start to last task completion

    def record_task_complete(self, task_id: str, completion_time_s: float, tick: int):
        self.completed_tasks += 1
        self.task_completion_times.append(completion_time_s)
        self.task_end_tick[task_id] = tick
        self.makespan_ticks = tick

    def record_conflict(self, resolved: bool = True):
        self.conflicts_detected += 1
        if resolved:
            self.conflicts_resolved += 1

    def record_collision(self):
        self.collisions += 1

    def record_deadlock(self, resolved: bool = True):
        self.deadlocks_detected += 1
        if resolved:
            self.deadlocks_resolved += 1

    def record_reroute(self):
        self.reroutes += 1

    def record_reassignment(self):
        self.task_reassignments += 1

    def record_planning_time(self, ms: float):
        self.planning_times_ms.append(ms)

    def record_gnn_inference(self, ms: float):
        self.gnn_inference_times_ms.append(ms)

    def record_marl_inference(self, ms: float):
        self.marl_inference_times_ms.append(ms)

    def finalize(self, tick: int):
        self.end_time = time.time()
        self.total_ticks = tick
        if not self.makespan_ticks:
            self.makespan_ticks = tick

    def throughput(self) -> float:
        """Tasks completed per tick."""
        if self.makespan_ticks == 0:
            return 0.0
        return self.completed_tasks / self.makespan_ticks

    def avg_task_completion_time(self) -> float:
        if not self.task_completion_times:
            return 0.0
        return sum(self.task_completion_times) / len(self.task_completion_times)

    def avg_planning_time_ms(self) -> float:
        if not self.planning_times_ms:
            return 0.0
        return sum(self.planning_times_ms) / len(self.planning_times_ms)

    def to_dict(self) -> dict:
        return {
            "mode": self.mode,
            "makespan_ticks": self.makespan_ticks,
            "total_ticks": self.total_ticks,
            "total_tasks": self.total_tasks,
            "completed_tasks": self.completed_tasks,
            "avg_task_completion_time": round(self.avg_task_completion_time(), 3),
            "throughput_tasks_per_tick": round(self.throughput(), 4),
            "total_distance": round(self.total_distance, 2),
            "total_waiting_ticks": self.total_waiting_ticks,
            "total_idle_ticks": self.total_idle_ticks,
            "collisions": self.collisions,
            "conflicts_detected": self.conflicts_detected,
            "conflicts_resolved": self.conflicts_resolved,
            "deadlocks_detected": self.deadlocks_detected,
            "deadlocks_resolved": self.deadlocks_resolved,
            "reroutes": self.reroutes,
            "task_reassignments": self.task_reassignments,
            "avg_planning_time_ms": round(self.avg_planning_time_ms(), 3),
            "messages_sent": self.messages_sent,
            "messages_dropped": self.messages_dropped,
        }
