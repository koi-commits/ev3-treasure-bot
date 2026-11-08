"""Small recurrent actor-critic used for training and EV3 export."""

from __future__ import annotations

import torch
from torch import nn


class RecurrentPolicy(nn.Module):
    def __init__(self, obs_size, action_count, hidden_size=24):
        super().__init__()
        self.obs_size = obs_size
        self.action_count = action_count
        self.hidden_size = hidden_size
        self.gru = nn.GRUCell(obs_size, hidden_size)
        self.actor = nn.Linear(hidden_size, action_count)
        self.critic = nn.Linear(hidden_size, 1)
        self.reset_parameters()

    def reset_parameters(self):
        for name, value in self.named_parameters():
            if value.ndim >= 2:
                nn.init.orthogonal_(value, gain=0.8)
            else:
                nn.init.zeros_(value)
        # Premature release is destructive; demonstrations/RL can overcome this.
        with torch.no_grad():
            self.actor.bias[-1] = -3.0

    def step(self, obs, hidden, mask=None):
        if mask is not None:
            hidden = hidden * mask.unsqueeze(-1)
        hidden = self.gru(obs, hidden)
        return self.actor(hidden), self.critic(hidden).squeeze(-1), hidden

    def sequence(self, obs, hidden, masks):
        logits, values = [], []
        for t in range(obs.shape[0]):
            logit, value, hidden = self.step(obs[t], hidden, masks[t])
            logits.append(logit)
            values.append(value)
        return torch.stack(logits), torch.stack(values), hidden


def save_checkpoint(path, model, metadata=None):
    torch.save({
        "state_dict": model.state_dict(),
        "obs_size": model.obs_size,
        "action_count": model.action_count,
        "hidden_size": model.hidden_size,
        "metadata": metadata or {},
    }, path)


def load_checkpoint(path, device="cpu"):
    ckpt = torch.load(path, map_location=device, weights_only=False)
    model = RecurrentPolicy(ckpt["obs_size"], ckpt["action_count"], ckpt["hidden_size"])
    model.load_state_dict(ckpt["state_dict"])
    model.to(device)
    return model, ckpt.get("metadata", {})

