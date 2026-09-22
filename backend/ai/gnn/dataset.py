"""GNN training dataset generator for NEXFLEET.

Runs simulation episodes and collects (graph, label) pairs.
Label = 1 if a conflict occurs within H steps ahead.
"""
from __future__ import annotations
import json
import os
import random
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import torch
from torch.utils.data import Dataset

from backend.ai.gnn.graph_builder import GraphData, InteractionGraphBuilder


DATA_DIR = Path("data/gnn")
CONFLICT_HORIZON = 8  # look-ahead ticks


@dataclass
class GNNSample:
    graph: GraphData
    label: torch.Tensor  # [E] float, 1.0 = conflict within horizon


class GNNDataset(Dataset):
    def __init__(self, samples: List[GNNSample]):
        self.samples = samples

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> GNNSample:
        return self.samples[idx]


def generate_gnn_training_data(
    num_episodes: int = 30,
    ticks_per_episode: int = 200,
    seed: int = 42,
    horizon: int = CONFLICT_HORIZON,
    save: bool = True,
) -> GNNDataset:
    """
    Generate GNN training data by running simulation episodes.
    Returns a GNNDataset with labeled graph snapshots.
    """
    from backend.simulation.engine import SimulationEngine, SimulationMode, SimulationState
    from backend.coordination.conflict_detector import ConflictDetector

    builder = InteractionGraphBuilder()
    detector = ConflictDetector()
    rng = random.Random(seed)

    all_samples: List[GNNSample] = []

    print(f"[GNN Dataset] Generating {num_episodes} episodes x {ticks_per_episode} ticks...")

    for ep in range(num_episodes):
        ep_seed = seed + ep * 7
        engine = SimulationEngine(
            mode=SimulationMode.TRADITIONAL,
            seed=ep_seed,
            tick_interval=0.0,
            max_ticks=ticks_per_episode,
        )

        # Add some tasks
        pickups = ["P1", "P2", "P3", "P4"]
        deliveries = ["D1", "D2", "D3", "D4"]
        n_tasks = rng.randint(4, 8)
        for _ in range(n_tasks):
            p = rng.choice(pickups)
            d = rng.choice(deliveries)
            engine.add_task_from_stations(p, d)

        engine.start()

        # Collect snapshots
        snapshots = []  # list of (tick, states, intents)
        for tick in range(ticks_per_episode):
            states = [a.state for a in engine.agents.values()]
            intents = {rid: a.state.movement_intent for rid, a in engine.agents.items()}
            snapshots.append((tick, [s.__dict__.copy() for s in states], dict(intents)))
            engine.step()
            if engine.state == SimulationState.FINISHED:
                break

        # Build conflict timeline
        conflict_ticks: set = set()
        for i, (tick, states_raw, intents) in enumerate(snapshots):
            # Re-hydrate state objects for conflict detection
            class _S:
                pass

            state_objs = []
            for sd in states_raw:
                s = _S()
                for k, v in sd.items():
                    setattr(s, k, v)
                state_objs.append(s)

            try:
                conflicts = detector.detect(state_objs, intents, tick)
                if conflicts:
                    conflict_ticks.add(tick)
            except Exception:
                pass

        # Create labeled graph samples
        for i, (tick, states_raw, intents) in enumerate(snapshots):
            class _S:
                pass

            state_objs = []
            for sd in states_raw:
                s = _S()
                for k, v in sd.items():
                    setattr(s, k, v)
                state_objs.append(s)

            graph = builder.build(state_objs, tick)
            if graph is None:
                continue

            # Check if any conflict in [tick, tick+horizon]
            future_conflict = any(
                t in conflict_ticks
                for t in range(tick, tick + horizon + 1)
            )

            # Label all edges with the conflict signal
            label = torch.full(
                (graph.num_edges,), float(future_conflict), dtype=torch.float32
            )
            graph.label = label

            all_samples.append(GNNSample(graph=graph, label=label))

        if (ep + 1) % 5 == 0:
            print(f"  Episode {ep+1}/{num_episodes}: {len(all_samples)} samples so far")

    print(f"[GNN Dataset] Total samples: {len(all_samples)}")

    dataset = GNNDataset(all_samples)

    if save:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        # Save as json-serialisable lists
        saved_data = []
        for sample in all_samples:
            saved_data.append({
                "node_features": sample.graph.node_features.numpy().tolist(),
                "edge_index": sample.graph.edge_index.numpy().tolist(),
                "edge_features": sample.graph.edge_features.numpy().tolist(),
                "label": sample.label.numpy().tolist(),
                "robot_ids": sample.graph.robot_ids,
                "tick": sample.graph.tick,
            })
        out_file = DATA_DIR / "gnn_dataset.json"
        with open(out_file, "w") as f:
            json.dump(saved_data, f)
        print(f"[GNN Dataset] Saved to {out_file}")

    return dataset


def load_gnn_dataset(path: Optional[Path] = None) -> Optional[GNNDataset]:
    p = path or (DATA_DIR / "gnn_dataset.json")
    if not p.exists():
        return None
    with open(p) as f:
        data = json.load(f)
    samples = []
    for d in data:
        node_features = torch.tensor(d["node_features"], dtype=torch.float32)
        edge_index = torch.tensor(d["edge_index"], dtype=torch.long)
        edge_features = torch.tensor(d["edge_features"], dtype=torch.float32)
        label = torch.tensor(d["label"], dtype=torch.float32)
        graph = GraphData(
            node_features=node_features,
            edge_index=edge_index,
            edge_features=edge_features,
            robot_ids=d["robot_ids"],
            tick=d["tick"],
            label=label,
        )
        samples.append(GNNSample(graph=graph, label=label))
    return GNNDataset(samples)


def split_dataset(
    dataset: GNNDataset,
    train_ratio: float = 0.7,
    val_ratio: float = 0.15,
    seed: int = 42,
) -> Tuple[GNNDataset, GNNDataset, GNNDataset]:
    rng = random.Random(seed)
    indices = list(range(len(dataset)))
    rng.shuffle(indices)
    n = len(indices)
    n_train = int(n * train_ratio)
    n_val = int(n * val_ratio)
    train_idx = indices[:n_train]
    val_idx = indices[n_train:n_train + n_val]
    test_idx = indices[n_train + n_val:]
    train_ds = GNNDataset([dataset.samples[i] for i in train_idx])
    val_ds = GNNDataset([dataset.samples[i] for i in val_idx])
    test_ds = GNNDataset([dataset.samples[i] for i in test_idx])
    return train_ds, val_ds, test_ds
