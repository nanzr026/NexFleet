"""GNN inference engine for runtime conflict risk."""
from __future__ import annotations
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import torch

from backend.ai.gnn.graph_builder import GraphData, InteractionGraphBuilder
from backend.ai.gnn.model import GNNRiskModel

MODEL_PATH = Path("models/gnn_risk.pt")
FED_MODEL_PATH = Path("models/federated_gnn.pt")


class GNNInferenceEngine:
    """
    Runtime GNN inference.
    Falls back to HEURISTIC_FALLBACK if model not available.
    """

    def __init__(self):
        self.model: Optional[GNNRiskModel] = None
        self.builder = InteractionGraphBuilder()
        self._risk_engine = "HEURISTIC_FALLBACK"
        self._load_model()

    def _load_model(self, path: Optional[Path] = None):
        """Attempt to load GNN model from disk."""
        p = path or FED_MODEL_PATH
        if not p.exists():
            p = MODEL_PATH
        if not p.exists():
            print("[GNN Inference] No trained model found. Using HEURISTIC_FALLBACK.")
            self._risk_engine = "HEURISTIC_FALLBACK"
            return

        try:
            checkpoint = torch.load(str(p), map_location="cpu", weights_only=False)
            config = checkpoint.get("config", {})
            self.model = GNNRiskModel(
                node_dim=config.get("node_dim", 8),
                edge_dim=config.get("edge_dim", 5),
                hidden=config.get("hidden", 64),
                mp_rounds=config.get("mp_rounds", 2),
            )
            self.model.load_state_dict(checkpoint["model_state_dict"])
            self.model.eval()
            self._risk_engine = "GNN"
            print(f"[GNN Inference] Model loaded from {p}. Engine: GNN")
        except Exception as e:
            print(f"[GNN Inference] Failed to load model: {e}. Using HEURISTIC_FALLBACK.")
            self._risk_engine = "HEURISTIC_FALLBACK"

    def reload_model(self, path: Optional[Path] = None):
        self._load_model(path)

    @property
    def risk_engine(self) -> str:
        return self._risk_engine

    def compute_risk(
        self,
        robot_states: list,
        tick: int = 0,
    ) -> Dict[Tuple[str, str], float]:
        """
        Compute pairwise conflict risk.
        Returns dict: {(robot_id_i, robot_id_j): risk_float}
        """
        if self._risk_engine == "HEURISTIC_FALLBACK" or self.model is None:
            return self._heuristic_risk(robot_states)

        graph = self.builder.build(robot_states, tick)
        if graph is None:
            return {}

        try:
            self.model.eval()
            with torch.no_grad():
                risks = self.model(
                    graph.node_features,
                    graph.edge_index,
                    graph.edge_features,
                )
            # Map back to robot_id pairs
            result = {}
            robot_ids = graph.robot_ids
            for i in range(graph.edge_index.size(1)):
                src_idx = graph.edge_index[0, i].item()
                dst_idx = graph.edge_index[1, i].item()
                if src_idx < len(robot_ids) and dst_idx < len(robot_ids):
                    key = (robot_ids[src_idx], robot_ids[dst_idx])
                    result[key] = round(risks[i].item(), 4)
            return result
        except Exception as e:
            print(f"[GNN Inference] Error: {e}. Falling back to heuristic.")
            return self._heuristic_risk(robot_states)

    def _heuristic_risk(
        self,
        states: list,
    ) -> Dict[Tuple[str, str], float]:
        """Analytical risk estimator — deterministic, no randomness."""
        risk_map = {}
        CONFLICT_DIST = 6.0
        active = [s for s in states if getattr(s, 'status', 'FAILED') != 'FAILED']

        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                si, sj = active[i], active[j]
                dist = math.hypot(
                    getattr(si, 'row', 0) - getattr(sj, 'row', 0),
                    getattr(si, 'col', 0) - getattr(sj, 'col', 0),
                )

                if dist > CONFLICT_DIST * 2:
                    risk = 0.0
                else:
                    dist_risk = max(0.0, 1.0 - dist / CONFLICT_DIST)

                    pi = getattr(si, 'current_path', [])
                    pj = getattr(sj, 'current_path', [])
                    if pi and pj:
                        set_i = set(tuple(p) for p in pi[:6])
                        set_j = set(tuple(p) for p in pj[:6])
                        path_risk = min(1.0, len(set_i & set_j) * 0.3)
                    else:
                        path_risk = 0.0

                    dir_i = getattr(si, 'direction', 'NONE')
                    dir_j = getattr(sj, 'direction', 'NONE')
                    opp = {"N": "S", "S": "N", "E": "W", "W": "E"}
                    heading_risk = 0.35 if (dir_i != 'NONE' and opp.get(dir_i) == dir_j) else 0.0

                    risk = min(1.0, dist_risk * 0.5 + path_risk * 0.3 + heading_risk * 0.2)

                key = (si.robot_id, sj.robot_id)
                risk_map[key] = round(risk, 4)

        return risk_map

    def get_status(self) -> dict:
        return {
            "risk_engine": self._risk_engine,
            "model_loaded": self.model is not None,
            "model_path": str(MODEL_PATH),
        }
