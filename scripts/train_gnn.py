#!/usr/bin/env python
"""Quick GNN training script for NEXFLEET."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend.ai.gnn.dataset import generate_gnn_training_data, load_gnn_dataset
from backend.ai.gnn.trainer import train_gnn

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Train NEXFLEET GNN conflict-risk model")
    parser.add_argument("--episodes", type=int, default=20, help="Number of simulation episodes")
    parser.add_argument("--epochs", type=int, default=20, help="Training epochs")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--load-existing", action="store_true", help="Load saved dataset if available")
    args = parser.parse_args()

    print("=" * 50)
    print("NEXFLEET GNN Quick Training")
    print("=" * 50)

    if args.load_existing:
        ds = load_gnn_dataset()
        if ds is None:
            print("No existing dataset found. Generating...")
            ds = generate_gnn_training_data(num_episodes=args.episodes, seed=args.seed)
    else:
        ds = generate_gnn_training_data(num_episodes=args.episodes, seed=args.seed)

    print(f"Dataset size: {len(ds)} samples")
    model, history = train_gnn(ds, epochs=args.epochs, seed=args.seed)
    print("\nTraining complete!")
    print(f"Final val F1: {history['val_f1'][-1] if history['val_f1'] else 'N/A'}")
    if "test_metrics" in history:
        tm = history["test_metrics"]
        print(f"Test  acc={tm['accuracy']:.4f}  f1={tm['f1']:.4f}  precision={tm['precision']:.4f}  recall={tm['recall']:.4f}")
