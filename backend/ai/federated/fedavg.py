"""Weighted Federated Averaging (FedAvg) implementation."""
from __future__ import annotations
from collections import OrderedDict
from typing import List, Tuple

import torch


def fedavg(
    local_updates: List[Tuple[OrderedDict, int]],
) -> OrderedDict:
    """
    Weighted FedAvg aggregation.

    Formula:
        global_weights = sum(n_k * w_k) / sum(n_k)

    Args:
        local_updates: List of (state_dict, n_samples) tuples

    Returns:
        Aggregated global state dict
    """
    if not local_updates:
        raise ValueError("No local updates provided to FedAvg")

    total_samples = sum(n for _, n in local_updates)
    if total_samples == 0:
        raise ValueError("Total sample count is zero")

    # Initialize with zeros (copy structure from first dict)
    global_weights = OrderedDict()
    first_dict, _ = local_updates[0]
    for key in first_dict.keys():
        global_weights[key] = torch.zeros_like(first_dict[key], dtype=torch.float32)

    # Weighted sum
    for state_dict, n_samples in local_updates:
        weight = n_samples / total_samples
        for key in global_weights.keys():
            global_weights[key] += state_dict[key].float() * weight

    return global_weights
