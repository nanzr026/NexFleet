"""Task lifecycle manager."""
from __future__ import annotations
import time
from typing import Dict, List, Optional, Tuple

from backend.task_allocation.task import Task, TaskStatus
from backend.task_allocation.auction import AuctionManager
from backend.simulation.amr_agent import AMRState


class TaskManager:
    def __init__(self, auction: Optional[AuctionManager] = None):
        self._tasks: Dict[str, Task] = {}
        self.auction = auction or AuctionManager()

    def add_task(self, task: Task) -> Task:
        self._tasks[task.task_id] = task
        return task

    def get_task(self, task_id: str) -> Optional[Task]:
        return self._tasks.get(task_id)

    def all_tasks(self) -> List[Task]:
        return list(self._tasks.values())

    def pending_tasks(self) -> List[Task]:
        return [t for t in self._tasks.values() if t.status == TaskStatus.PENDING]

    def assign_pending_tasks(
        self, robot_states: List[AMRState]
    ) -> List[Tuple[str, str]]:
        """
        Run auction for all pending tasks.
        Returns list of (task_id, robot_id) assignments.
        """
        assignments = []
        for task in sorted(self.pending_tasks(), key=lambda t: -t.priority):
            winner = self.auction.run_auction(task, robot_states)
            if winner:
                task.status = TaskStatus.ASSIGNED
                task.assigned_robot = winner
                task.started_at = time.time()
                assignments.append((task.task_id, winner))
        return assignments

    def mark_to_pickup(self, task_id: str):
        t = self._tasks.get(task_id)
        if t:
            t.status = TaskStatus.TO_PICKUP

    def mark_to_delivery(self, task_id: str):
        t = self._tasks.get(task_id)
        if t:
            t.status = TaskStatus.TO_DELIVERY

    def mark_completed(self, task_id: str):
        t = self._tasks.get(task_id)
        if t:
            t.status = TaskStatus.COMPLETED
            t.completed_at = time.time()

    def release_task(self, task_id: str):
        """Release a task back to PENDING (e.g., robot failure)."""
        t = self._tasks.get(task_id)
        if t and t.status not in (TaskStatus.COMPLETED, TaskStatus.FAILED):
            t.status = TaskStatus.REASSIGNING
            t.assigned_robot = None
            t.started_at = None

    def reassign_pending_task(
        self, task_id: str, status: TaskStatus = TaskStatus.PENDING
    ):
        t = self._tasks.get(task_id)
        if t:
            t.status = status
            t.assigned_robot = None

    def get_stats(self) -> dict:
        tasks = list(self._tasks.values())
        completed = [t for t in tasks if t.status == TaskStatus.COMPLETED]
        ct_times = [t.completion_time for t in completed if t.completion_time]
        return {
            "total": len(tasks),
            "pending": len([t for t in tasks if t.status == TaskStatus.PENDING]),
            "assigned": len([t for t in tasks if t.status == TaskStatus.ASSIGNED]),
            "in_progress": len([t for t in tasks if t.status in (TaskStatus.TO_PICKUP, TaskStatus.TO_DELIVERY)]),
            "completed": len(completed),
            "avg_completion_time": sum(ct_times) / len(ct_times) if ct_times else 0.0,
        }

    def to_list(self) -> List[dict]:
        return [t.to_dict() for t in self._tasks.values()]
