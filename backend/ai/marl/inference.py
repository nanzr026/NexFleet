"""MARL inference engine."""
from __future__ import annotations
from pathlib import Path
from typing import Dict, List, Optional

import torch

from backend.ai.marl.environment import OBS_DIM, N_ACTIONS, ACTION_NAMES
from backend.ai.marl.policy import MARLPolicy

MODEL_PATH = Path("models/marl_policy.pt")


class MARLInferenceEngine:
    """
    Runtime MARL action selection.
    Falls back to RULE_BASED_FALLBACK if model unavailable.
    """

    def __init__(self):
        self.policy: Optional[MARLPolicy] = None
        self._decision_engine = "RULE_BASED_FALLBACK"
        self._load_policy()

    def _load_policy(self, path: Optional[Path] = None):
        p = path or MODEL_PATH
        if not p.exists():
            print("[MARL Inference] No trained policy found. Using RULE_BASED_FALLBACK.")
            return

        try:
            checkpoint = torch.load(str(p), map_location="cpu", weights_only=False)
            config = checkpoint.get("config", {})
            self.policy = MARLPolicy(
                obs_dim=config.get("obs_dim", OBS_DIM),
                n_actions=config.get("n_actions", N_ACTIONS),
                hidden=config.get("hidden", 128),
            )
            self.policy.load_state_dict(checkpoint["model_state_dict"])
            self.policy.eval()
            self._decision_engine = "MARL"
            print(f"[MARL Inference] Policy loaded. Engine: MARL")
        except Exception as e:
            print(f"[MARL Inference] Failed to load policy: {e}. Using RULE_BASED_FALLBACK.")

    def reload_policy(self, path: Optional[Path] = None):
        self._load_policy(path)

    @property
    def decision_engine(self) -> str:
        return self._decision_engine

    def get_action(
        self,
        robot_state,
        all_states: list,
        risk: float = 0.0,
    ) -> str:
        """
        Get recommended action for a robot.
        Returns action name string.
        """
        if self._decision_engine == "MARL" and self.policy:
            try:
                obs = self._build_obs(robot_state, all_states, risk)
                action_idx = self.policy.greedy_action(obs)
                return ACTION_NAMES[action_idx]
            except Exception:
                pass

        return self._rule_based(robot_state, risk)

    def _rule_based(self, state, risk: float) -> str:
        if risk < 0.4:
            return "PROCEED"
        elif risk < 0.7:
            return "YIELD" if getattr(state, 'waiting_ticks', 0) > 3 else "PROCEED"
        else:
            return "REROUTE"

    def _build_obs(
        self,
        state,
        all_states: list,
        risk: float,
    ) -> torch.Tensor:
        import math
        rows, cols = 30, 30
        status_list = ["IDLE", "MOVING", "WAITING", "REROUTING", "BLOCKED", "CHARGING", "FAILED"]
        status_str = str(getattr(state, 'status', 'IDLE')).split('.')[-1]
        status_val = status_list.index(status_str) / 6.0 if status_str in status_list else 0.0
        own = [
            state.row / rows,
            state.col / cols,
            getattr(state, 'velocity', 0.0),
            getattr(state, 'battery', 100.0) / 100.0,
            status_val,
            min(1.0, getattr(state, 'waiting_ticks', 0) / 20.0),
        ]
        if state.destination:
            dr = state.destination[0] - state.row
            dc = state.destination[1] - state.col
            dist = math.hypot(dr, dc)
            goal_sin = dr / (dist + 1e-6)
            goal_cos = dc / (dist + 1e-6)
            goal_dist = min(1.0, dist / max(rows, cols))
        else:
            goal_sin, goal_cos, goal_dist = 0.0, 0.0, 0.0
        goal_feat = [goal_sin, goal_cos, goal_dist]

        dir_map = {"NONE": 0, "N": 1, "S": 2, "E": 3, "W": 4}
        neighbor_feat = []
        neighbors = [s for s in all_states if s.robot_id != state.robot_id][:2]
        for ns in neighbors:
            dir_str = str(getattr(ns, 'direction', 'NONE')).split('.')[-1]
            dir_rad = dir_map.get(dir_str, 0) * (2 * math.pi / 5)
            neighbor_feat.extend([
                ns.row / rows,
                ns.col / cols,
                getattr(ns, 'velocity', 0.0),
                math.sin(dir_rad),
                math.cos(dir_rad),
            ])
        while len(neighbor_feat) < 10:
            neighbor_feat.append(0.0)

        feat = own + goal_feat + neighbor_feat + [risk]
        return torch.tensor(feat, dtype=torch.float32)

    def get_status(self) -> dict:
        return {
            "decision_engine": self._decision_engine,
            "policy_loaded": self.policy is not None,
        }
