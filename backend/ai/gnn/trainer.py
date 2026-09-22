"""GNN trainer for conflict risk prediction."""
from __future__ import annotations
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.optim as optim

from backend.ai.gnn.model import GNNRiskModel
from backend.ai.gnn.dataset import GNNDataset, GNNSample, split_dataset

MODEL_DIR = Path("models")


def train_gnn(
    dataset: GNNDataset,
    epochs: int = 30,
    lr: float = 1e-3,
    hidden: int = 64,
    mp_rounds: int = 2,
    seed: int = 42,
    save_path: Optional[Path] = None,
    verbose: bool = True,
) -> Tuple[GNNRiskModel, Dict]:
    """
    Train the GNN conflict-risk model.
    Returns (model, training_history).
    """
    torch.manual_seed(seed)
    train_ds, val_ds, test_ds = split_dataset(dataset, seed=seed)

    if verbose:
        print(f"[GNN Trainer] Train: {len(train_ds)}, Val: {len(val_ds)}, Test: {len(test_ds)}")

    model = GNNRiskModel(node_dim=8, edge_dim=5, hidden=hidden, mp_rounds=mp_rounds)
    optimizer = optim.Adam(model.parameters(), lr=lr)
    criterion = nn.BCELoss()

    history = {"train_loss": [], "val_loss": [], "val_acc": [], "val_f1": []}

    best_val_loss = float("inf")
    best_state = None

    for epoch in range(epochs):
        model.train()
        total_loss = 0.0
        n_batches = 0

        for sample in train_ds.samples:
            g = sample.graph
            if g.num_edges == 0:
                continue

            optimizer.zero_grad()
            pred = model(g.node_features, g.edge_index, g.edge_features)
            loss = criterion(pred, sample.label)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
            n_batches += 1

        train_loss = total_loss / max(n_batches, 1)

        # Validation
        val_metrics = evaluate_model(model, val_ds, criterion)
        history["train_loss"].append(round(train_loss, 4))
        history["val_loss"].append(round(val_metrics["loss"], 4))
        history["val_acc"].append(round(val_metrics["accuracy"], 4))
        history["val_f1"].append(round(val_metrics["f1"], 4))

        if val_metrics["loss"] < best_val_loss:
            best_val_loss = val_metrics["loss"]
            best_state = {k: v.clone() for k, v in model.state_dict().items()}

        if verbose and (epoch + 1) % 5 == 0:
            print(
                f"  Epoch {epoch+1}/{epochs}: "
                f"train_loss={train_loss:.4f} "
                f"val_loss={val_metrics['loss']:.4f} "
                f"val_acc={val_metrics['accuracy']:.4f} "
                f"val_f1={val_metrics['f1']:.4f}"
            )

    # Restore best model
    if best_state:
        model.load_state_dict(best_state)

    # Test evaluation
    test_metrics = evaluate_model(model, test_ds, criterion)
    history["test_metrics"] = test_metrics
    if verbose:
        print(
            f"[GNN Trainer] Test: loss={test_metrics['loss']:.4f} "
            f"acc={test_metrics['accuracy']:.4f} f1={test_metrics['f1']:.4f}"
        )

    # Save model
    if save_path is None:
        MODEL_DIR.mkdir(parents=True, exist_ok=True)
        save_path = MODEL_DIR / "gnn_risk.pt"

    torch.save({
        "model_state_dict": model.state_dict(),
        "config": {"node_dim": 8, "edge_dim": 5, "hidden": hidden, "mp_rounds": mp_rounds},
        "history": history,
        "test_metrics": test_metrics,
    }, save_path)
    if verbose:
        print(f"[GNN Trainer] Model saved to {save_path}")

    return model, history


def evaluate_model(
    model: GNNRiskModel,
    dataset: GNNDataset,
    criterion: nn.Module,
) -> Dict:
    model.eval()
    total_loss = 0.0
    all_preds = []
    all_labels = []

    with torch.no_grad():
        for sample in dataset.samples:
            g = sample.graph
            if g.num_edges == 0:
                continue
            pred = model(g.node_features, g.edge_index, g.edge_features)
            loss = criterion(pred, sample.label)
            total_loss += loss.item()
            all_preds.extend((pred > 0.5).float().tolist())
            all_labels.extend(sample.label.tolist())

    n = len(all_preds)
    if n == 0:
        return {"loss": 0.0, "accuracy": 0.0, "precision": 0.0, "recall": 0.0, "f1": 0.0}

    tp = sum(p == 1 and l == 1 for p, l in zip(all_preds, all_labels))
    fp = sum(p == 1 and l == 0 for p, l in zip(all_preds, all_labels))
    fn = sum(p == 0 and l == 1 for p, l in zip(all_preds, all_labels))
    tn = sum(p == 0 and l == 0 for p, l in zip(all_preds, all_labels))

    accuracy = (tp + tn) / n
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1 = (2 * precision * recall / (precision + recall)) if (precision + recall) > 0 else 0.0

    return {
        "loss": total_loss / max(len(dataset.samples), 1),
        "accuracy": accuracy,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
    }
