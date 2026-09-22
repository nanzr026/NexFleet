"""MARL trainer using REINFORCE with baseline."""
from __future__ import annotations
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.optim as optim

from backend.ai.marl.environment import MARLEnv, OBS_DIM, N_ACTIONS, ACTION_NAMES
from backend.ai.marl.policy import MARLPolicy

MODEL_DIR = Path("models")


def train_marl(
    n_episodes: int = 50,
    max_steps: int = 200,
    lr: float = 3e-4,
    gamma: float = 0.95,
    hidden: int = 128,
    seed: int = 42,
    save_path: Optional[Path] = None,
    verbose: bool = True,
) -> Tuple[MARLPolicy, Dict]:
    """
    Train MARL policy using REINFORCE with shared policy across all robots.
    Returns (policy, history).
    """
    torch.manual_seed(seed)
    env = MARLEnv(seed=seed)
    policy = MARLPolicy(obs_dim=OBS_DIM, n_actions=N_ACTIONS, hidden=hidden)
    optimizer = optim.Adam(policy.parameters(), lr=lr)

    history = {"episode_rewards": [], "episode_lengths": [], "episode_completions": []}

    for episode in range(n_episodes):
        obs_dict = env.reset()
        robot_ids = env.get_robot_ids()

        ep_log_probs: List[torch.Tensor] = []
        ep_rewards: List[float] = []
        ep_length = 0
        total_reward = 0.0

        for step in range(max_steps):
            actions = {}
            log_probs_step = []

            for rid in robot_ids:
                obs = obs_dict.get(rid, torch.zeros(OBS_DIM))
                logits = policy(obs)
                probs = torch.softmax(logits, dim=-1)
                dist = torch.distributions.Categorical(probs)
                action = dist.sample()
                actions[rid] = action.item()
                log_probs_step.append(dist.log_prob(action))

            obs_dict, rewards, done, info = env.step(actions)

            step_reward = sum(rewards.values()) / max(len(rewards), 1)
            ep_rewards.append(step_reward)
            if log_probs_step:
                ep_log_probs.append(torch.stack(log_probs_step).mean())
            total_reward += step_reward
            ep_length += 1

            if done:
                break

        # Compute discounted returns
        returns = []
        G = 0.0
        for r in reversed(ep_rewards):
            G = r + gamma * G
            returns.insert(0, G)

        returns_t = torch.tensor(returns, dtype=torch.float32)
        # Normalize returns
        if returns_t.std() > 1e-6:
            returns_t = (returns_t - returns_t.mean()) / (returns_t.std() + 1e-8)

        # Policy gradient loss
        if ep_log_probs:
            log_probs_t = torch.stack(ep_log_probs)
            n = min(len(log_probs_t), len(returns_t))
            loss = -(log_probs_t[:n] * returns_t[:n]).mean()

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 0.5)
            optimizer.step()

        history["episode_rewards"].append(round(total_reward, 3))
        history["episode_lengths"].append(ep_length)
        history["episode_completions"].append(info.get("completed_tasks", 0))

        if verbose and (episode + 1) % 10 == 0:
            avg_r = sum(history["episode_rewards"][-10:]) / 10
            avg_c = sum(history["episode_completions"][-10:]) / 10
            print(
                f"  Episode {episode+1}/{n_episodes}: "
                f"avg_reward={avg_r:.3f} avg_completions={avg_c:.1f}"
            )

    # Save
    if save_path is None:
        MODEL_DIR.mkdir(parents=True, exist_ok=True)
        save_path = MODEL_DIR / "marl_policy.pt"

    torch.save({
        "model_state_dict": policy.state_dict(),
        "config": {"obs_dim": OBS_DIM, "n_actions": N_ACTIONS, "hidden": hidden},
        "history": history,
    }, save_path)
    if verbose:
        print(f"[MARL Trainer] Policy saved to {save_path}")

    return policy, history
