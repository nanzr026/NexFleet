"""Federated learning server / aggregator."""
from __future__ import annotations
import time
from collections import OrderedDict
from pathlib import Path
from typing import Dict, List, Optional

import torch

from backend.ai.gnn.model import GNNRiskModel
from backend.ai.federated.client import FederatedClient
from backend.ai.federated.fedavg import fedavg

MODEL_DIR = Path("models")


class FederatedRound:
    def __init__(self, round_num: int):
        self.round_num = round_num
        self.participants: List[str] = []
        self.sample_counts: Dict[str, int] = {}
        self.local_losses: Dict[str, float] = {}
        self.global_val_loss: Optional[float] = None
        self.global_val_acc: Optional[float] = None
        self.global_f1: Optional[float] = None
        self.aggregation_status: str = "PENDING"
        self.timestamp: float = time.time()

    def to_dict(self) -> dict:
        return {
            "round": self.round_num,
            "participants": self.participants,
            "sample_counts": self.sample_counts,
            "local_losses": self.local_losses,
            "global_val_loss": self.global_val_loss,
            "global_val_acc": self.global_val_acc,
            "global_f1": self.global_f1,
            "aggregation_status": self.aggregation_status,
            "timestamp": self.timestamp,
        }


class FederatedServer:
    """
    Federated learning server.
    Coordinates global model distribution, local training, and FedAvg aggregation.
    """

    def __init__(
        self,
        clients: List[FederatedClient],
        global_model: Optional[GNNRiskModel] = None,
        validation_dataset=None,
    ):
        self.clients = clients
        self.global_model = global_model or GNNRiskModel()
        self.validation_dataset = validation_dataset
        self.model_version = 0
        self.rounds: List[FederatedRound] = []
        self._history = {"global_val_loss": [], "global_val_acc": [], "global_f1": []}

    def run_round(
        self,
        round_num: int,
        participating_robot_ids: Optional[List[str]] = None,
    ) -> FederatedRound:
        """
        Execute one federated learning round.
        Returns FederatedRound result.
        """
        fl_round = FederatedRound(round_num)
        print(f"[FL Server] Starting Round {round_num}")

        # Determine participants
        active_clients = [
            c for c in self.clients
            if participating_robot_ids is None or c.robot_id in participating_robot_ids
        ]

        global_state = OrderedDict(self.global_model.state_dict())

        # Local training on each client
        local_updates = []
        for client in active_clients:
            fl_round.participants.append(client.robot_id)
            updated_dict, n_samples, local_loss = client.train_local(global_state)
            fl_round.sample_counts[client.robot_id] = n_samples
            fl_round.local_losses[client.robot_id] = round(local_loss, 4)
            if n_samples > 0:
                local_updates.append((updated_dict, n_samples))
            print(
                f"  {client.robot_id}: samples={n_samples} local_loss={local_loss:.4f}"
            )

        # FedAvg aggregation
        if local_updates:
            aggregated = fedavg(local_updates)
            self.global_model.load_state_dict(aggregated)
            self.model_version += 1
            fl_round.aggregation_status = "COMPLETED"
        else:
            fl_round.aggregation_status = "SKIPPED (no client data)"
            print(f"[FL Server] Round {round_num}: No client data. Skipping aggregation.")

        # Validate on held-out data
        if self.validation_dataset:
            val_metrics = self._validate()
            fl_round.global_val_loss = round(val_metrics.get("loss", 0.0), 4)
            fl_round.global_val_acc = round(val_metrics.get("accuracy", 0.0), 4)
            fl_round.global_f1 = round(val_metrics.get("f1", 0.0), 4)
            self._history["global_val_loss"].append(fl_round.global_val_loss)
            self._history["global_val_acc"].append(fl_round.global_val_acc)
            self._history["global_f1"].append(fl_round.global_f1)

        self.rounds.append(fl_round)
        print(
            f"[FL Server] Round {round_num} complete. "
            f"Model v{self.model_version}. "
            f"Status: {fl_round.aggregation_status}"
        )
        return fl_round

    def run_federation(
        self,
        n_rounds: int = 5,
        save: bool = True,
    ) -> List[FederatedRound]:
        """Run N federated rounds."""
        results = []
        for r in range(1, n_rounds + 1):
            result = self.run_round(r)
            results.append(result)

        if save:
            MODEL_DIR.mkdir(parents=True, exist_ok=True)
            torch.save({
                "model_state_dict": self.global_model.state_dict(),
                "config": {"node_dim": 8, "edge_dim": 5, "hidden": 64, "mp_rounds": 2},
                "model_version": self.model_version,
                "rounds": [r.to_dict() for r in self.rounds],
            }, MODEL_DIR / "federated_gnn.pt")
            print(f"[FL Server] Saved federated model to models/federated_gnn.pt")

        return results

    def _validate(self) -> dict:
        """Validate global model on held-out dataset."""
        import torch.nn as nn
        from backend.ai.gnn.trainer import evaluate_model
        criterion = nn.BCELoss()
        self.global_model.eval()
        return evaluate_model(self.global_model, self.validation_dataset, criterion)

    def get_status(self) -> dict:
        return {
            "model_version": self.model_version,
            "n_rounds_completed": len(self.rounds),
            "clients": [c.robot_id for c in self.clients],
            "history": self._history,
            "latest_round": self.rounds[-1].to_dict() if self.rounds else None,
        }
