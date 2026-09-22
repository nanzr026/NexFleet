"""Main discrete-tick simulation engine for NEXFLEET."""
from __future__ import annotations
import asyncio
import json
import random
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from backend.simulation.warehouse import Warehouse
from backend.simulation.amr_agent import AMRAgent, RobotStatus
from backend.task_allocation.task import Task, TaskStatus
from backend.task_allocation.auction import AuctionManager
from backend.task_allocation.manager import TaskManager
from backend.metrics.collector import SimulationMetrics
from backend.coordination.conflict_detector import ConflictDetector


class SimulationMode:
    TRADITIONAL = "traditional"
    NEXFLEET = "nexfleet"


class SimulationState:
    STOPPED = "stopped"
    RUNNING = "running"
    PAUSED = "paused"
    FINISHED = "finished"


class SimulationEngine:
    """
    Discrete-tick warehouse simulation engine.
    Orchestrates warehouse, agents, tasks, and controller.
    """

    DEFAULT_TICK_INTERVAL = 0.2  # seconds between ticks in real-time mode
    MAX_TICKS = 2000

    def __init__(
        self,
        mode: str = SimulationMode.NEXFLEET,
        seed: int = 42,
        tick_interval: float = 0.2,
        max_ticks: int = 2000,
    ):
        self.mode = mode
        self.seed = seed
        self.tick_interval = tick_interval
        self.max_ticks = max_ticks

        self.warehouse = Warehouse()
        self.agents: Dict[str, AMRAgent] = {}
        self.task_manager = TaskManager(AuctionManager())
        self.metrics = SimulationMetrics(mode=mode)
        self.controller = None
        self.state = SimulationState.STOPPED
        self.tick = 0
        self.events: List[dict] = []
        self._event_callbacks: List[Callable] = []
        self._rng = random.Random(seed)

        self._setup_agents()
        self._setup_controller()

    def _setup_agents(self):
        """Create 3 AMR agents at fixed starting positions."""
        starts = [
            ("AMR-01", 1, 1),
            ("AMR-02", 1, 15),
            ("AMR-03", 1, 28),
        ]
        for robot_id, r, c in starts:
            agent = AMRAgent(robot_id, r, c)
            self.agents[robot_id] = agent
            self.metrics.per_robot_distance[robot_id] = 0.0
            self.metrics.per_robot_waiting[robot_id] = 0

    def _setup_controller(self):
        if self.mode == SimulationMode.TRADITIONAL:
            from backend.agents.baseline_controller import BaselineController
            self.controller = BaselineController(self.warehouse, self.metrics)
        else:
            from backend.agents.nexfleet_controller import NexFleetController
            self.controller = NexFleetController(self.warehouse, self.metrics)

    def add_event_callback(self, cb: Callable[[dict], None]):
        self._event_callbacks.append(cb)

    def _emit(self, event: dict):
        self.events.append(event)
        for cb in self._event_callbacks:
            try:
                cb(event)
            except Exception:
                pass

    def add_task_from_stations(
        self,
        pickup_station: str,
        delivery_station: str,
        priority: int = 1,
    ) -> Optional[Task]:
        ps = self.warehouse.get_station(pickup_station)
        ds = self.warehouse.get_station(delivery_station)
        if not ps or not ds:
            return None
        task = Task.create(
            pickup=ps.pos(),
            delivery=ds.pos(),
            pickup_station=pickup_station,
            delivery_station=delivery_station,
            priority=priority,
        )
        self.task_manager.add_task(task)
        self.metrics.total_tasks += 1
        self.metrics.task_start_tick[task.task_id] = self.tick
        self._emit({"type": "TASK_CREATED", "tick": self.tick, "task": task.to_dict()})
        return task

    def assign_pending_tasks(self):
        robot_states = [a.state for a in self.agents.values()]
        assignments = self.task_manager.assign_pending_tasks(robot_states)
        for task_id, robot_id in assignments:
            self._emit({
                "type": "TASK_ASSIGNED",
                "tick": self.tick,
                "task_id": task_id,
                "robot_id": robot_id,
            })

    def fail_robot(self, robot_id: str):
        if robot_id not in self.agents:
            return
        agent = self.agents[robot_id]
        old_task_id = agent.state.current_task_id
        agent.fail()
        self._emit({"type": "ROBOT_FAILED", "tick": self.tick, "robot_id": robot_id})

        if old_task_id:
            self.task_manager.release_task(old_task_id)
            task = self.task_manager.get_task(old_task_id)
            if task:
                task.status = TaskStatus.PENDING
                task.assigned_robot = None
            agent.state.current_task_id = None
            self.assign_pending_tasks()
            self.metrics.record_reassignment()
            self._emit({
                "type": "TASK_REASSIGNED",
                "tick": self.tick,
                "task_id": old_task_id,
            })

    def recover_robot(self, robot_id: str):
        if robot_id not in self.agents:
            return
        self.agents[robot_id].recover()
        self._emit({"type": "ROBOT_RECOVERED", "tick": self.tick, "robot_id": robot_id})

    def add_obstacle(self, row: int, col: int) -> bool:
        result = self.warehouse.add_obstacle(row, col)
        if result:
            self._emit({"type": "OBSTACLE_ADDED", "tick": self.tick, "pos": [row, col]})
            # Invalidate paths that pass through this cell
            for agent in self.agents.values():
                path = agent.state.current_path
                if (row, col) in path:
                    task_id = agent.state.current_task_id
                    agent.mark_rerouting()
                    if task_id and task_id in self.task_manager._tasks:
                        task = self.task_manager._tasks[task_id]
                        from algorithms.astar.astar import astar
                        if task.status == TaskStatus.TO_PICKUP:
                            dest = task.pickup
                        elif task.status == TaskStatus.TO_DELIVERY:
                            dest = task.delivery
                        else:
                            continue
                        new_path = astar(
                            (agent.state.row, agent.state.col),
                            dest,
                            self.warehouse.is_passable,
                            self.warehouse.neighbors,
                        )
                        if new_path:
                            agent.set_path(new_path)
                            self._emit({
                                "type": "REROUTING",
                                "tick": self.tick,
                                "robot_id": agent.robot_id,
                                "reason": "OBSTACLE_DETECTED",
                            })
        return result

    def remove_obstacle(self, row: int, col: int) -> bool:
        result = self.warehouse.remove_obstacle(row, col)
        if result:
            self._emit({"type": "OBSTACLE_REMOVED", "tick": self.tick, "pos": [row, col]})
        return result

    def step(self) -> List[dict]:
        """Execute one simulation tick."""
        if self.state != SimulationState.RUNNING:
            return []

        self.tick += 1

        # Assign any pending tasks
        self.assign_pending_tasks()

        # Run controller
        tasks_dict = {t.task_id: t for t in self.task_manager.all_tasks()}
        tick_events = self.controller.step(self.agents, tasks_dict, self.tick)

        for ev in tick_events:
            self._emit(ev)
            
        # Battery drain model
        for agent in self.agents.values():
            st = agent.state
            s = str(st.status)
            if "MOVING" in s:
                st.battery = max(0.0, st.battery - 0.05)
            elif "WAITING" in s or "REROUTING" in s:
                st.battery = max(0.0, st.battery - 0.01)
            elif "IDLE" in s:
                st.battery = max(0.0, st.battery - 0.002)

        # Check completion
        all_complete = (
            self.metrics.completed_tasks >= self.metrics.total_tasks
            and self.metrics.total_tasks > 0
        )
        if all_complete or self.tick >= self.max_ticks:
            self.state = SimulationState.FINISHED
            self.metrics.finalize(self.tick)
            self._emit({"type": "SIMULATION_COMPLETE", "tick": self.tick})

        return tick_events

    def get_full_state(self) -> dict:
        return {
            "tick": self.tick,
            "state": self.state,
            "mode": self.mode,
            "robots": [a.state.to_dict() for a in self.agents.values()],
            "tasks": self.task_manager.to_list(),
            "warehouse": self.warehouse.to_dict(),
            "metrics": self.metrics.to_dict(),
            "auction": self.task_manager.auction.get_latest_auction(),
            "events": self.events[-20:],
        }

    def start(self):
        self.state = SimulationState.RUNNING

    def pause(self):
        self.state = SimulationState.PAUSED

    def resume(self):
        if self.state == SimulationState.PAUSED:
            self.state = SimulationState.RUNNING

    def reset(self, mode: Optional[str] = None, seed: Optional[int] = None):
        if mode:
            self.mode = mode
        if seed is not None:
            self.seed = seed
        self._rng = random.Random(self.seed)
        self.tick = 0
        self.events = []
        self.agents = {}
        self.task_manager = TaskManager(AuctionManager())
        self.metrics = SimulationMetrics(mode=self.mode)
        self.state = SimulationState.STOPPED
        self.warehouse = Warehouse()
        self._setup_agents()
        self._setup_controller()
