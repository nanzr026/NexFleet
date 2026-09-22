"""Interaction graph builder for NEXFLEET GNN.

Builds a robot interaction graph from simulator state.
Nodes = AMRs, Edges = proximate robot pairs.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import torch

# Status encoding
STATUS_MAP = {
    "IDLE": 0, "MOVING": 1, "WAITING": 2, "REROUTING": 3,
    "BLOCKED": 4, "CHARGING": 5, "FAILED": 6,
}
DIRECTION_MAP = {"NONE": 0, "N": 1, "S": 2, "E": 3, "W": 4}
EDGE_RADIUS = 12.0  # cells


@dataclass
class GraphData:
    """Robot interaction graph for one timestep."""
    node_features: torch.Tensor   # [N, F_node]
    edge_index: torch.Tensor      # [2, E] long
    edge_features: torch.Tensor   # [E, F_edge]
    robot_ids: List[str]
    tick: int
    label: Optional[torch.Tensor] = None  # [E] binary conflict label

    @property
    def num_nodes(self) -> int:
        return self.node_features.size(0)

    @property
    def num_edges(self) -> int:
        return self.edge_index.size(1)


class InteractionGraphBuilder:
    """
    Builds robot interaction graphs for GNN input.

    Node features (8 dims, normalized):
        row/rows, col/cols, velocity, dir_sin, dir_cos,
        battery/100, task_state_enc, waiting_norm

    Edge features (5 dims, normalized):
        rel_dist/max_dist, rel_vel, path_intersect, ttc_norm, route_overlap
    """

    NODE_DIM = 8
    EDGE_DIM = 5

    def __init__(self, rows: int = 30, cols: int = 30):
        self.rows = rows
        self.cols = cols

    def build(self, robot_states: list, tick: int = 0) -> Optional[GraphData]:
        """
        Build a GraphData from a list of AMRState-like objects.
        Returns None if fewer than 2 robots.
        """
        active = [s for s in robot_states if getattr(s, 'status', 'FAILED') != 'FAILED']
        if len(active) < 2:
            return None

        N = len(active)
        node_feats = []
        for s in active:
            node_feats.append(self._node_features(s))

        node_tensor = torch.tensor(node_feats, dtype=torch.float32)  # [N, NODE_DIM]

        # Build edges between robots within EDGE_RADIUS
        src_list, dst_list, edge_feats = [], [], []
        for i in range(N):
            for j in range(N):
                if i == j:
                    continue
                si, sj = active[i], active[j]
                dist = math.hypot(si.row - sj.row, si.col - sj.col)
                if dist <= EDGE_RADIUS:
                    src_list.append(i)
                    dst_list.append(j)
                    edge_feats.append(self._edge_features(si, sj, dist))

        if not src_list:
            # No edges - create self-loops for all nodes so GNN can still process
            for i in range(N):
                src_list.append(i)
                dst_list.append(i)
                edge_feats.append([0.0] * self.EDGE_DIM)

        edge_index = torch.tensor(
            [src_list, dst_list], dtype=torch.long
        )  # [2, E]
        edge_tensor = torch.tensor(edge_feats, dtype=torch.float32)  # [E, EDGE_DIM]

        robot_ids = [s.robot_id for s in active]
        return GraphData(
            node_features=node_tensor,
            edge_index=edge_index,
            edge_features=edge_tensor,
            robot_ids=robot_ids,
            tick=tick,
        )

    def _node_features(self, s) -> List[float]:
        row_n = s.row / self.rows
        col_n = s.col / self.cols
        vel_n = getattr(s, 'velocity', 0.0)
        direction = getattr(s, 'direction', 'NONE')
        dir_idx = DIRECTION_MAP.get(direction, 0)
        dir_rad = dir_idx * (2 * math.pi / 5)
        dir_sin = math.sin(dir_rad)
        dir_cos = math.cos(dir_rad)
        battery_n = getattr(s, 'battery', 100.0) / 100.0
        status = getattr(s, 'status', 'IDLE')
        status_val = STATUS_MAP.get(str(status), 0) / 6.0
        waiting_n = min(1.0, getattr(s, 'waiting_ticks', 0) / 20.0)
        return [row_n, col_n, vel_n, dir_sin, dir_cos, battery_n, status_val, waiting_n]

    def _edge_features(self, si, sj, dist: float) -> List[float]:
        max_dist = EDGE_RADIUS
        rel_dist = dist / max_dist

        vi = getattr(si, 'velocity', 0.0)
        vj = getattr(sj, 'velocity', 0.0)
        rel_vel = abs(vi - vj)

        # Path intersection
        pi = getattr(si, 'current_path', [])
        pj = getattr(sj, 'current_path', [])
        if pi and pj:
            set_i = set(tuple(p) for p in pi[:8])
            set_j = set(tuple(p) for p in pj[:8])
            path_intersect = min(1.0, len(set_i & set_j) / max(1, min(len(set_i), len(set_j))))
        else:
            path_intersect = 0.0

        # Time-to-conflict estimate
        if dist < 1e-3:
            ttc = 0.0
        elif vi + vj > 0:
            ttc = min(1.0, dist / (vi + vj + 1e-6) / 10.0)
        else:
            ttc = 1.0

        # Route overlap (shared intent horizon)
        intent_i = getattr(si, 'movement_intent', None)
        intent_j = getattr(sj, 'movement_intent', None)
        if intent_i and intent_j and intent_i == intent_j:
            route_overlap = 1.0
        else:
            route_overlap = path_intersect * 0.5

        return [rel_dist, rel_vel, path_intersect, ttc, route_overlap]

    def build_from_dict(self, robot_dicts: list, tick: int = 0) -> Optional[GraphData]:
        """Build from list of robot state dicts (API-compatible)."""
        class DictState:
            pass

        states = []
        for d in robot_dicts:
            s = DictState()
            s.robot_id = d.get('robot_id', 'unknown')
            s.row = d.get('row', 0)
            s.col = d.get('col', 0)
            s.velocity = d.get('velocity', 0.0)
            s.direction = d.get('direction', 'NONE')
            s.battery = d.get('battery', 100.0)
            s.status = d.get('status', 'IDLE')
            s.waiting_ticks = d.get('waiting_ticks', 0)
            s.current_path = [tuple(p) for p in d.get('current_path', [])]
            s.movement_intent = tuple(d['movement_intent']) if d.get('movement_intent') else None
            states.append(s)
        return self.build(states, tick)
