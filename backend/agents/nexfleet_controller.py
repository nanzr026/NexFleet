"""NEXFLEET decentralized risk-adaptive controller."""
from __future__ import annotations
import math
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from algorithms.astar.astar import astar, astar_with_reservation
from backend.simulation.amr_agent import AMRAgent, AMRState, RobotStatus
from backend.simulation.warehouse import Warehouse
from backend.coordination.conflict_detector import ConflictDetector, ReservationTable
from backend.coordination.deadlock import DeadlockDetector
from backend.task_allocation.task import Task, TaskStatus
from backend.metrics.collector import SimulationMetrics


class NexFleetController:
    """
    Decentralized risk-adaptive coordination with:
    - Local negotiation / yield
    - Rerouting
    - Deadlock detection & resolution
    - Dynamic obstacle avoidance
    - Risk-adaptive coordination (uses risk_fn if provided)
    """

    RISK_MEDIUM = 0.4
    RISK_HIGH = 0.7

    def __init__(
        self,
        warehouse: Warehouse,
        metrics: SimulationMetrics,
        risk_fn: Optional[Callable[[List[AMRState]], Dict[Tuple[str, str], float]]] = None,
        action_fn: Optional[Callable[[AMRState, List[AMRState], float], str]] = None,
    ):
        self.warehouse = warehouse
        self.metrics = metrics
        self.risk_fn = risk_fn
        self.action_fn = action_fn
        self.detector = ConflictDetector()
        self.reservation = ReservationTable()
        self.deadlock_detector = DeadlockDetector()
        self._events_this_tick: List[dict] = []
        self._risk_engine = "HEURISTIC_FALLBACK"
        self._decision_engine = "RULE_BASED_FALLBACK"

    @property
    def risk_engine(self) -> str:
        return self._risk_engine

    @property
    def decision_engine(self) -> str:
        return self._decision_engine

    def set_risk_fn(self, fn: Callable, engine_name: str = "GNN"):
        self.risk_fn = fn
        self._risk_engine = engine_name

    def set_action_fn(self, fn: Callable, engine_name: str = "MARL"):
        self.action_fn = fn
        self._decision_engine = engine_name

    def step(
        self,
        agents: Dict[str, AMRAgent],
        tasks: Dict[str, Task],
        tick: int,
    ) -> List[dict]:
        self._events_this_tick = []
        self.reservation.clear()
        all_states = [a.state for a in agents.values()]

        # Reserve current positions
        for agent in agents.values():
            if agent.state.status != RobotStatus.FAILED:
                self.reservation.reserve(
                    (agent.state.row, agent.state.col), agent.robot_id
                )

        # Update neighbor awareness
        for agent in agents.values():
            agent.update_neighbors(all_states)

        # Assign tasks
        self._assign_tasks(agents, tasks, tick)

        # Compute pairwise risk
        risk_map: Dict[Tuple[str, str], float] = {}
        if self.risk_fn:
            try:
                t0 = time.time()
                risk_map = self.risk_fn(all_states)
                self.metrics.record_gnn_inference((time.time() - t0) * 1000)
            except Exception:
                risk_map = self._heuristic_risk(all_states)
        else:
            risk_map = self._heuristic_risk(all_states)

        # Update deadlock history
        self.deadlock_detector.update_history(all_states)

        # Detect deadlocks
        deadlock_groups = self.deadlock_detector.detect_deadlocks(all_states, tick)
        for group in deadlock_groups:
            resolution = self.deadlock_detector.resolve(group, all_states, tick)
            self.metrics.record_deadlock(resolved=True)
            self._events_this_tick.append({
                "type": "DEADLOCK_DETECTED",
                "tick": tick,
                "robots": group,
                "resolution": resolution,
            })
            # Force reroute for the designated robot
            if ":" in resolution:
                reroute_rid = resolution.split(":")[1]
                if reroute_rid in agents:
                    self._force_reroute(agents[reroute_rid], tasks, tick)

        # Determine intended moves
        intents: Dict[str, Optional[Tuple[int, int]]] = {}
        for rid, agent in agents.items():
            s = agent.state
            if s.status == RobotStatus.FAILED:
                intents[rid] = None
            else:
                intents[rid] = s.movement_intent

        # Per-robot coordination decisions
        blocked_robots = set()
        yield_robots = set()
        reroute_robots = set()

        for rid, agent in agents.items():
            if agent.state.status == RobotStatus.FAILED:
                continue

            # Find max risk involving this robot
            max_risk = 0.0
            risk_partner = None
            for (r1, r2), risk in risk_map.items():
                if rid in (r1, r2):
                    if risk > max_risk:
                        max_risk = risk
                        risk_partner = r2 if r1 == rid else r1

            # Determine action based on risk
            action = self._decide_action(agent.state, max_risk, all_states, tick)

            if action == "WAIT":
                blocked_robots.add(rid)
            elif action == "YIELD":
                yield_robots.add(rid)
            elif action == "REROUTE":
                reroute_robots.add(rid)

            if max_risk > 0.1:
                self._events_this_tick.append({
                    "type": "RISK_ASSESSMENT",
                    "tick": tick,
                    "robot_id": rid,
                    "partner": risk_partner,
                    "risk": round(max_risk, 3),
                    "action": action,
                    "risk_engine": self._risk_engine,
                    "decision_engine": self._decision_engine,
                })

        # Detect immediate conflicts
        conflicts = self.detector.detect(all_states, intents, tick)
        for conflict in conflicts:
            self.metrics.record_conflict(resolved=True)
            self._events_this_tick.append({"type": "CONFLICT", "tick": tick, "data": conflict.to_dict()})
            sorted_robots = sorted(conflict.robot_ids)
            for rid in sorted_robots[1:]:
                blocked_robots.add(rid)

        # Handle reroutes before moves
        for rid in reroute_robots:
            if rid not in blocked_robots and rid in agents:
                self._force_reroute(agents[rid], tasks, tick)

        # Execute moves
        for rid, agent in agents.items():
            s = agent.state
            if s.status == RobotStatus.FAILED:
                continue

            if rid in blocked_robots:
                agent.mark_waiting()
                continue

            if rid in yield_robots:
                agent.mark_waiting()
                continue

            intent = intents.get(rid)
            if intent is None:
                if s.status not in (RobotStatus.IDLE, RobotStatus.REROUTING):
                    agent.mark_idle()
                continue

            # Safety check reservation
            if self.reservation.is_reserved_by_other(intent, rid):
                # Try to negotiate: if risk is high, reroute instead of wait
                max_risk_val = max(
                    (v for (r1, r2), v in risk_map.items() if rid in (r1, r2)),
                    default=0.0,
                )
                if max_risk_val >= self.RISK_HIGH:
                    self._force_reroute(agent, tasks, tick)
                else:
                    agent.mark_waiting()
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

            self._check_task_progress(agent, tasks, tick)

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

        return self._events_this_tick

    def _decide_action(
        self,
        state: AMRState,
        risk: float,
        all_states: List[AMRState],
        tick: int,
    ) -> str:
        """Risk-adaptive action selection."""
        if self.action_fn:
            try:
                t0 = time.time()
                action = self.action_fn(state, all_states, risk)
                self.metrics.record_marl_inference((time.time() - t0) * 1000)
                return action
            except Exception:
                pass

        # Rule-based fallback
        if risk < self.RISK_MEDIUM:
            return "PROCEED"
        elif risk < self.RISK_HIGH:
            # Yield if robot has lower priority (more waiting time means lower priority)
            return "YIELD" if state.waiting_ticks > 3 else "PROCEED"
        else:
            return "REROUTE"

    def _heuristic_risk(
        self,
        states: List[AMRState],
    ) -> Dict[Tuple[str, str], float]:
        """Analytical risk estimator (fallback when GNN unavailable)."""
        risk_map = {}
        CONFLICT_DIST = 5.0
        active = [s for s in states if s.status != "FAILED"]

        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                si, sj = active[i], active[j]
                dist = math.hypot(si.row - sj.row, si.col - sj.col)
                if dist > CONFLICT_DIST * 2:
                    risk = 0.0
                else:
                    # Distance factor
                    dist_risk = max(0.0, 1.0 - dist / CONFLICT_DIST)

                    # Path intersection factor
                    path_risk = 0.0
                    hi = si.get_short_horizon(4)
                    hj = sj.get_short_horizon(4)
                    if hi and hj:
                        intersection = set(hi) & set(hj)
                        path_risk = min(1.0, len(intersection) * 0.25)

                    # Heading conflict
                    heading_risk = 0.0
                    if si.direction != "NONE" and sj.direction != "NONE":
                        opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}
                        if opposite.get(si.direction) == sj.direction:
                            heading_risk = 0.4

                    risk = min(1.0, dist_risk * 0.5 + path_risk * 0.3 + heading_risk * 0.2)

                key = (si.robot_id, sj.robot_id)
                risk_map[key] = round(risk, 3)

        return risk_map

    def _force_reroute(
        self,
        agent: AMRAgent,
        tasks: Dict[str, Task],
        tick: int,
    ):
        s = agent.state
        task_id = s.current_task_id
        if not task_id or task_id not in tasks:
            return
        task = tasks[task_id]
        agent.mark_rerouting()
        self.metrics.record_reroute()

        # Determine current destination
        if task.status == TaskStatus.TO_PICKUP:
            dest = task.pickup
        elif task.status == TaskStatus.TO_DELIVERY:
            dest = task.delivery
        else:
            return

        # Reroute with reservation avoidance
        reserved = set(self.reservation.get_table().keys()) - {(s.row, s.col)}
        t0 = time.time()
        path = astar_with_reservation(
            (s.row, s.col),
            dest,
            self.warehouse.is_passable,
            self.warehouse.neighbors,
            reserved_cells=reserved,
        )
        ms = (time.time() - t0) * 1000
        self.metrics.record_planning_time(ms)

        if path:
            agent.set_path(path)
            self._events_this_tick.append({
                "type": "REROUTING",
                "tick": tick,
                "robot_id": s.robot_id,
                "new_path_len": len(path),
            })
        else:
            agent.mark_waiting()

    def _assign_tasks(self, agents, tasks, tick):
        for task_id, task in tasks.items():
            if task.status != TaskStatus.ASSIGNED:
                continue
            rid = task.assigned_robot
            if rid not in agents:
                continue
            agent = agents[rid]
            if agent.state.current_task_id:
                continue

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

    def _check_task_progress(self, agent: AMRAgent, tasks, tick):
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
            self._events_this_tick.append({
                "type": "PICKED_UP",
                "tick": tick,
                "task_id": task_id,
                "robot_id": s.robot_id,
            })

        elif task.status == TaskStatus.TO_DELIVERY and (s.row, s.col) == task.delivery:
            ct = tick - self.metrics.task_start_tick.get(task_id, tick)
            task.status = TaskStatus.COMPLETED
            task.completed_at = time.time()
            s.current_task_id = None
            s.completed_tasks += 1
            self.metrics.record_task_complete(task_id, ct, tick)
            self._events_this_tick.append({
                "type": "TASK_COMPLETED",
                "tick": tick,
                "task_id": task_id,
                "robot_id": s.robot_id,
            })
