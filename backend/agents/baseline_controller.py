"""Traditional Stop-and-Wait baseline controller."""
from __future__ import annotations
import time
from typing import Dict, List, Optional, Tuple

from algorithms.astar.astar import astar
from backend.simulation.amr_agent import AMRAgent, AMRState, RobotStatus
from backend.simulation.warehouse import Warehouse
from backend.coordination.conflict_detector import ConflictDetector, ReservationTable
from backend.task_allocation.task import Task, TaskStatus
from backend.metrics.collector import SimulationMetrics


class BaselineController:
    """
    Traditional Stop-and-Wait coordination.
    When robots encounter a conflict: detect -> stop -> wait -> resume.
    No proactive conflict avoidance or rerouting.
    """

    def __init__(self, warehouse: Warehouse, metrics: SimulationMetrics):
        self.warehouse = warehouse
        self.metrics = metrics
        self.detector = ConflictDetector()
        self.reservation = ReservationTable()

    def step(
        self,
        agents: Dict[str, AMRAgent],
        tasks: Dict[str, Task],
        tick: int,
    ) -> List[dict]:
        events = []
        self.reservation.clear()

        # Reserve current positions
        for agent in agents.values():
            if agent.state.status != RobotStatus.FAILED:
                self.reservation.reserve(
                    (agent.state.row, agent.state.col), agent.robot_id
                )

        # Assign tasks if robots are idle
        self._assign_tasks(agents, tasks, tick)

        # Determine intended moves
        intents: Dict[str, Optional[Tuple[int, int]]] = {}
        for rid, agent in agents.items():
            s = agent.state
            if s.status == RobotStatus.FAILED:
                intents[rid] = None
                continue
            intents[rid] = s.movement_intent

        # Detect conflicts
        all_states = [a.state for a in agents.values()]
        conflicts = self.detector.detect(all_states, intents, tick)

        # Traditional: stop conflicting robots and wait
        blocked_robots = set()
        for conflict in conflicts:
            self.metrics.record_conflict(resolved=True)
            events.append({"type": "CONFLICT", "tick": tick, "data": conflict.to_dict()})
            # Stop all but the first robot (simple priority: alphabetical)
            sorted_robots = sorted(conflict.robot_ids)
            for rid in sorted_robots[1:]:
                blocked_robots.add(rid)

        # Execute moves
        for rid, agent in agents.items():
            s = agent.state
            if s.status == RobotStatus.FAILED:
                continue

            if rid in blocked_robots:
                agent.mark_waiting()
                events.append({"type": "STOP_AND_WAIT", "tick": tick, "robot_id": rid})
                continue

            intent = intents.get(rid)
            if intent is None:
                if s.status != RobotStatus.IDLE:
                    agent.mark_idle()
                continue

            # Check if next cell is free
            if self.reservation.is_reserved_by_other(intent, rid):
                agent.mark_waiting()
                events.append({"type": "STOP_AND_WAIT", "tick": tick, "robot_id": rid})
                continue

            # Move
            moved = agent.step_move()
            if moved:
                self.reservation.release(rid)
                self.reservation.reserve((s.row, s.col), rid)
                self.metrics.total_distance += 1.0
                self.metrics.per_robot_distance[rid] = (
                    self.metrics.per_robot_distance.get(rid, 0.0) + 1.0
                )

            # Task phase progression
            self._check_task_progress(agent, tasks, tick, events)

        # Track waiting/idle
        for agent in agents.values():
            s = agent.state
            if s.status == RobotStatus.WAITING:
                self.metrics.total_waiting_ticks += 1
                self.metrics.per_robot_waiting[s.robot_id] = (
                    self.metrics.per_robot_waiting.get(s.robot_id, 0) + 1
                )
            elif s.status == RobotStatus.IDLE:
                self.metrics.total_idle_ticks += 1

        return events

    def _assign_tasks(self, agents, tasks, tick):
        for task_id, task in tasks.items():
            if task.status != TaskStatus.ASSIGNED:
                continue
            rid = task.assigned_robot
            if rid not in agents:
                continue
            agent = agents[rid]
            if agent.state.current_task_id:
                continue  # Already has a task

            # Assign: navigate to pickup first
            agent.state.current_task_id = task_id
            task.status = TaskStatus.TO_PICKUP
            t0 = time.time()
            path = astar(
                (agent.state.row, agent.state.col),
                task.pickup,
                self.warehouse.is_passable,
                self.warehouse.neighbors,
            )
            ms = (time.time() - t0) * 1000
            self.metrics.record_planning_time(ms)
            if path:
                agent.set_path(path)

    def _check_task_progress(self, agent: AMRAgent, tasks, tick, events):
        s = agent.state
        task_id = s.current_task_id
        if not task_id or task_id not in tasks:
            return
        task = tasks[task_id]

        if task.status == TaskStatus.TO_PICKUP and (s.row, s.col) == task.pickup:
            task.status = TaskStatus.TO_DELIVERY
            t0 = time.time()
            path = astar(
                (s.row, s.col),
                task.delivery,
                self.warehouse.is_passable,
                self.warehouse.neighbors,
            )
            ms = (time.time() - t0) * 1000
            self.metrics.record_planning_time(ms)
            if path:
                agent.set_path(path)
            events.append({"type": "PICKED_UP", "tick": tick, "task_id": task_id, "robot_id": s.robot_id})

        elif task.status == TaskStatus.TO_DELIVERY and (s.row, s.col) == task.delivery:
            ct = tick - self.metrics.task_start_tick.get(task_id, tick)
            task.status = TaskStatus.COMPLETED
            task.completed_at = time.time()
            s.current_task_id = None
            s.completed_tasks += 1
            self.metrics.record_task_complete(task_id, ct, tick)
            events.append({"type": "TASK_COMPLETED", "tick": tick, "task_id": task_id, "robot_id": s.robot_id})
