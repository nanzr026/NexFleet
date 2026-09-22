"""Lightweight MARL policy network."""
from __future__ import annotations
import torch
import torch.nn as nn
import torch.nn.functional as F

from backend.ai.marl.environment import OBS_DIM, N_ACTIONS


class MARLPolicy(nn.Module):
    """
    Lightweight policy network for each robot.
    Input: observation [OBS_DIM]
    Output: action logits [N_ACTIONS]
    """

    def __init__(
        self,
        obs_dim: int = OBS_DIM,
        n_actions: int = N_ACTIONS,
        hidden: int = 128,
    ):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.LayerNorm(hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, n_actions),
        )
        self.value_head = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, 1),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """Returns action logits."""
        return self.net(obs)

    def get_action_probs(self, obs: torch.Tensor) -> torch.Tensor:
        logits = self.forward(obs)
        return F.softmax(logits, dim=-1)

    def get_value(self, obs: torch.Tensor) -> torch.Tensor:
        return self.value_head(obs)

    def sample_action(self, obs: torch.Tensor) -> int:
        probs = self.get_action_probs(obs)
        return torch.multinomial(probs, 1).item()

    def greedy_action(self, obs: torch.Tensor) -> int:
        with torch.no_grad():
            logits = self.forward(obs)
        return logits.argmax().item()
