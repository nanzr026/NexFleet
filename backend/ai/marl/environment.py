"""Multi-agent RL environment for NEXFLEET."""
from __future__ import annotations
import math
import random
from typing import Dict, List, Optional, Tuple

import torch

ACTION_PROCEED = 0
ACTION_WAIT = 1
ACTION_YIELD = 2
ACTION_REROUTE = 3
ACTION_NAMES = ["PROCEED", "WAIT", "YIELD", "REROUTE"]
N_ACTIONS = 4

OBS_DIM = 20  # observation dimension per robot


class MARLEnv:
    """
    Lightweight multi-agent RL environment wrapping the NEXFLEET simulator.

    Observation per robot (20 dims):
        own: row_n, col_n, vel, battery_n, status_enc, waiting_n (6)
        goal: goal_dir_sin, goal_dir_cos, goal_dist_n (3)
        neighbors (up to 2): row_n, col_n, vel, dir_sin, dir_cos (5 each = 10)
        risk: max_gnn_risk (1)

    Actions:
        0: PROCEED
        1: WAIT
        2: YIELD
        3: REROUTE

    Rewards:
        +0.5: moving toward goal
        +5.0: task completed
        -0.1: waiting
        -0.05: unnecessary distance (rerouting)
        -risk: conflict risk penalty
        -10.0: collision
        -2.0: deadlock
    """

    MAX_TICKS = 300
    N_TASKS_PER_EPISODE = 6
    MAX_NEIGHBORS = 2

    def __init__(self, seed: int = 42):
        self.seed = seed
        self._rng = random.Random(seed)
        self.engine = None
        self._risk_engine = None
        self.tick = 0
        self.done = False
        self._prev_dist: Dict[str, float] = {}

    def set_risk_engine(self, engine):
        self._risk_engine = engine

    def reset(self) -> Dict[str, torch.Tensor]:
        from backend.simulation.engine import SimulationEngine, SimulationMode
        self.engine = SimulationEngine(
            mode=SimulationMode.NEXFLEET,
            seed=self._rng.randint(0, 9999),
            tick_interval=0.0,
            max_ticks=self.MAX_TICKS,
        )
        pickups = ["P1", "P2", "P3", "P4"]
        deliveries = ["D1", "D2", "D3", "D4"]
        for _ in range(self.N_TASKS_PER_EPISODE):
            p = self._rng.choice(pickups)
            d = self._rng.choice(deliveries)
            self.engine.add_task_from_stations(p, d)

        self.engine.assign_pending_tasks()
        self.engine.start()
        self.tick = 0
        self.done = False
        self._prev_dist = self._compute_goal_distances()
        return self._get_observations()

    def step(
        self,
        actions: Dict[str, int],
    ) -> Tuple[Dict[str, torch.Tensor], Dict[str, float], bool, dict]:
        """
        Apply actions and advance one tick.
        Returns: (observations, rewards, done, info)
        """
        if self.done:
            obs = self._get_observations()
            return obs, {rid: 0.0 for rid in self.engine.agents}, True, {}

        # Apply MARL actions to controller
        for rid, action in actions.items():
            if rid in self.engine.agents:
                agent = self.engine.agents[rid]
                self._apply_action(agent, action)

        prev_completed = self.engine.metrics.completed_tasks
        prev_dist = dict(self._prev_dist)

        # Step simulation
        self.engine.step()
        self.tick += 1

        new_completed = self.engine.metrics.completed_tasks
        new_dist = self._compute_goal_distances()
        self._prev_dist = new_dist

        # Compute rewards
        rewards = {}
        for rid, agent in self.engine.agents.items():
            r = 0.0

            # Task completion
            if new_completed > prev_completed:
                if agent.state.completed_tasks > 0:
                    r += 5.0

            # Progress toward goal
            if rid in prev_dist and rid in new_dist:
                progress = prev_dist[rid] - new_dist[rid]
                r += progress * 0.5

            # Waiting penalty
            if str(agent.state.status).split('.')[-1] == "WAITING":
                r -= 0.1

            # Risk penalty
            if self._risk_engine:
                all_states = [a.state for a in self.engine.agents.values()]
                risk_map = self._risk_engine.compute_risk(all_states, self.tick)
                max_risk = max(
                    (v for (r1, r2), v in risk_map.items() if rid in (r1, r2)),
                    default=0.0,
                )
                r -= max_risk * 0.3

            # Collision penalty (vertex conflict = collision)
            if self.engine.metrics.collisions > 0:
                r -= 10.0

            rewards[rid] = round(r, 4)

        from backend.simulation.engine import SimulationState
        self.done = (
            self.engine.state == SimulationState.FINISHED
            or self.tick >= self.MAX_TICKS
        )

        obs = self._get_observations()
        info = {
            "tick": self.tick,
            "completed_tasks": new_completed,
            "collisions": self.engine.metrics.collisions,
        }
        return obs, rewards, self.done, info

    def _apply_action(self, agent, action: int):
        if action == ACTION_WAIT:
            if hasattr(agent, 'mark_waiting'):
                agent.mark_waiting()
        elif action == ACTION_YIELD:
            if hasattr(agent, 'mark_waiting'):
                agent.mark_waiting()
        # PROCEED and REROUTE are handled by the simulation controller

    def _compute_goal_distances(self) -> Dict[str, float]:
        dists = {}
        for rid, agent in self.engine.agents.items():
            s = agent.state
            if s.destination:
                dists[rid] = math.hypot(s.row - s.destination[0], s.col - s.destination[1])
            else:
                dists[rid] = 0.0
        return dists

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        obs = {}
        all_states = [a.state for a in self.engine.agents.values()]

        risk_map = {}
        if self._risk_engine:
            try:
                risk_map = self._risk_engine.compute_risk(all_states, self.tick)
            except Exception:
                pass

        for rid, agent in self.engine.agents.items():
            s = agent.state
            rows = self.engine.warehouse.rows
            cols = self.engine.warehouse.cols

            # Own state (6)
            status_list = ["IDLE", "MOVING", "WAITING", "REROUTING", "BLOCKED", "CHARGING", "FAILED"]
            status_str = str(s.status).split('.')[-1]
            status_enc = status_list.index(status_str) / 6.0 if status_str in status_list else 0.0
            own = [
                s.row / rows,
                s.col / cols,
                getattr(s, 'velocity', 0.0),
                getattr(s, 'battery', 100.0) / 100.0,
                status_enc,
                min(1.0, getattr(s, 'waiting_ticks', 0) / 20.0),
            ]

            # Goal direction (3)
            if s.destination:
                dr = s.destination[0] - s.row
                dc = s.destination[1] - s.col
                dist = math.hypot(dr, dc)
                goal_sin = dr / (dist + 1e-6)
                goal_cos = dc / (dist + 1e-6)
                goal_dist = min(1.0, dist / max(rows, cols))
            else:
                goal_sin, goal_cos, goal_dist = 0.0, 0.0, 0.0
            goal_feat = [goal_sin, goal_cos, goal_dist]

            # Neighbors (up to 2, 5 each = 10 dims)
            neighbor_feat = []
            for ns in [st for st in all_states if st.robot_id != rid][:self.MAX_NEIGHBORS]:
                dir_map = {"NONE": 0, "N": 1, "S": 2, "E": 3, "W": 4}
                dir_str = str(getattr(ns, 'direction', 'NONE')).split('.')[-1]
                dir_rad = dir_map.get(dir_str, 0) * (2 * math.pi / 5)
                neighbor_feat.extend([
                    ns.row / rows,
                    ns.col / cols,
                    getattr(ns, 'velocity', 0.0),
                    math.sin(dir_rad),
                    math.cos(dir_rad),
                ])
            # Pad to MAX_NEIGHBORS * 5
            while len(neighbor_feat) < self.MAX_NEIGHBORS * 5:
                neighbor_feat.append(0.0)

            # Risk (1)
            max_risk = max(
                (v for (r1, r2), v in risk_map.items() if rid in (r1, r2)),
                default=0.0,
            )
            risk_feat = [max_risk]

            feat = own + goal_feat + neighbor_feat + risk_feat
            assert len(feat) == OBS_DIM, f"OBS_DIM mismatch: {len(feat)} != {OBS_DIM}"
            obs[rid] = torch.tensor(feat, dtype=torch.float32)

        return obs

    def get_robot_ids(self) -> List[str]:
        if self.engine:
            return list(self.engine.agents.keys())
        return []
