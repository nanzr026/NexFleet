"""Federated learning client (per-robot local trainer)."""
from __future__ import annotations
import random
from collections import OrderedDict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.optim as optim

from backend.ai.gnn.model import GNNRiskModel
from backend.ai.gnn.graph_builder import InteractionGraphBuilder, GraphData


DATA_DIR = Path("data/gnn")


class FederatedClient:
    """
    Federated learning client for one AMR robot.

    Each client:
    1. Receives global GNN model
    2. Trains locally on its own trajectory samples
    3. Returns model parameters (NOT raw data)
    """

    def __init__(
        self,
        robot_id: str,
        local_epochs: int = 3,
        lr: float = 1e-3,
        seed: int = 42,
    ):
        self.robot_id = robot_id
        self.local_epochs = local_epochs
        self.lr = lr
        self.seed = seed
        self._local_samples: List[Tuple[GraphData, torch.Tensor]] = []
        self._local_loss_history: List[float] = []
        self.builder = InteractionGraphBuilder()

    def receive_global_model(self, global_state_dict: OrderedDict) -> GNNRiskModel:
        """Load global model weights into a local copy."""
        model = GNNRiskModel()
        model.load_state_dict(global_state_dict)
        return model

    def add_local_sample(self, graph: GraphData, label: torch.Tensor):
        """Add one labeled graph sample to local dataset."""
        self._local_samples.append((graph, label))

    def generate_local_samples_from_trajectory(
        self,
        trajectory: List[dict],
        tick_offset: int = 0,
    ):
        """
        Generate local samples from robot trajectory data.
        trajectory: list of robot state snapshots (dicts).
        """
        builder = self.builder
        for i, snapshot in enumerate(trajectory[:-1]):
            # Build a minimal multi-robot graph from snapshot
            graph = builder.build_from_dict([snapshot], tick=tick_offset + i)
            if graph is None:
                continue
            # Heuristic label: conflict if velocity=0 and status is WAITING/BLOCKED
            is_conflict = (
                snapshot.get('status') in ('WAITING', 'BLOCKED')
                and snapshot.get('waiting_ticks', 0) > 3
            )
            label = torch.full(
                (graph.num_edges,), float(is_conflict), dtype=torch.float32
            )
            self._local_samples.append((graph, label))

    def train_local(
        self,
        global_state_dict: OrderedDict,
    ) -> Tuple[OrderedDict, int, float]:
        """
        Perform local training.
        Returns: (updated_state_dict, n_samples, avg_loss)
        """
        torch.manual_seed(self.seed)
        model = self.receive_global_model(global_state_dict)
        model.train()
        optimizer = optim.Adam(model.parameters(), lr=self.lr)
        criterion = nn.BCELoss()

        if not self._local_samples:
            # No local data: return global model unchanged with n=0
            return OrderedDict(model.state_dict()), 0, 0.0

        total_loss = 0.0
        n_batches = 0

        for epoch in range(self.local_epochs):
            for graph, label in self._local_samples:
                if graph.num_edges == 0:
                    continue
                optimizer.zero_grad()
                pred = model(
                    graph.node_features,
                    graph.edge_index,
                    graph.edge_features,
                )
                # Handle size mismatch gracefully
                min_len = min(len(pred), len(label))
                loss = criterion(pred[:min_len], label[:min_len])
                loss.backward()
                optimizer.step()
                total_loss += loss.item()
                n_batches += 1

        avg_loss = total_loss / max(n_batches, 1)
        self._local_loss_history.append(avg_loss)

        return OrderedDict(model.state_dict()), len(self._local_samples), avg_loss

    def get_sample_count(self) -> int:
        return len(self._local_samples)

    def clear_samples(self):
        self._local_samples.clear()
