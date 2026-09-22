"""Lightweight GNN conflict-risk prediction model.

Uses manual message passing (no PyTorch Geometric required).
Predicts per-edge conflict probability.
"""
from __future__ import annotations
from typing import Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F


class EdgeMLP(nn.Module):
    def __init__(self, in_dim: int, hidden: int, out_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.LayerNorm(hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, out_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class GNNRiskModel(nn.Module):
    """
    Lightweight Graph Neural Network for conflict risk prediction.

    Architecture:
        1. Node encoder: MLP on node features
        2. Message passing: 2 rounds of edge-neighbor aggregation
        3. Edge risk predictor: MLP on (node_i, node_j, edge_feat) -> risk

    Input:
        node_features: [N, NODE_DIM]
        edge_index:    [2, E]
        edge_features: [E, EDGE_DIM]

    Output:
        edge_risks: [E] in (0, 1)
    """

    def __init__(
        self,
        node_dim: int = 8,
        edge_dim: int = 5,
        hidden: int = 64,
        mp_rounds: int = 2,
    ):
        super().__init__()
        self.node_dim = node_dim
        self.edge_dim = edge_dim
        self.hidden = hidden
        self.mp_rounds = mp_rounds

        # Node encoder
        self.node_encoder = nn.Sequential(
            nn.Linear(node_dim, hidden),
            nn.LayerNorm(hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
        )

        # Message passing layers
        self.mp_layers = nn.ModuleList([
            nn.Sequential(
                nn.Linear(hidden + hidden + edge_dim, hidden),
                nn.LayerNorm(hidden),
                nn.ReLU(),
                nn.Linear(hidden, hidden),
            )
            for _ in range(mp_rounds)
        ])

        # Edge risk predictor
        self.edge_predictor = nn.Sequential(
            nn.Linear(hidden + hidden + edge_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden // 2),
            nn.ReLU(),
            nn.Linear(hidden // 2, 1),
        )

    def forward(
        self,
        node_features: torch.Tensor,
        edge_index: torch.Tensor,
        edge_features: torch.Tensor,
    ) -> torch.Tensor:
        """
        Returns:
            risk: [E] tensor with values in (0, 1)
        """
        N = node_features.size(0)
        E = edge_index.size(1)

        # Encode nodes
        h = self.node_encoder(node_features)  # [N, hidden]

        src = edge_index[0]  # [E]
        dst = edge_index[1]  # [E]

        # Message passing rounds
        for mp_layer in self.mp_layers:
            # Aggregate messages: for each dst node, gather from src
            h_src = h[src]  # [E, hidden]
            h_dst = h[dst]  # [E, hidden]

            # Compute messages
            msgs = mp_layer(
                torch.cat([h_src, h_dst, edge_features], dim=-1)
            )  # [E, hidden]

            # Aggregate (sum) messages per destination node
            h_new = torch.zeros(N, self.hidden, device=h.device)
            h_new.scatter_add_(0, dst.unsqueeze(1).expand_as(msgs), msgs)

            # Residual
            h = h + h_new

        # Predict edge risk
        h_src = h[src]  # [E, hidden]
        h_dst = h[dst]  # [E, hidden]
        edge_input = torch.cat([h_src, h_dst, edge_features], dim=-1)  # [E, 2*hidden+edge_dim]
        logits = self.edge_predictor(edge_input).squeeze(-1)  # [E]
        risk = torch.sigmoid(logits)
        return risk

    def predict_pairwise_risk(
        self,
        node_features: torch.Tensor,
        edge_index: torch.Tensor,
        edge_features: torch.Tensor,
    ) -> dict:
        """Returns {(src_idx, dst_idx): risk_float} dict."""
        self.eval()
        with torch.no_grad():
            risks = self.forward(node_features, edge_index, edge_features)
        result = {}
        for i in range(edge_index.size(1)):
            s = edge_index[0, i].item()
            d = edge_index[1, i].item()
            result[(s, d)] = risks[i].item()
        return result
