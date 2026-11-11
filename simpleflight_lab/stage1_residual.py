"""Stage 1-B contact residual over the frozen RM27 precision Track actor."""

import torch
from torch import nn

from .ppo import Actor
from .stage1_task import POLICY_OBS_DIM
from .task import track_hold_observation


# Raw corrections in actor order [p, q, r, T]. The outer tanh and RM27
# decoder bound the physical change to [0.70, 1.70, 0.70] rad/s and 0.294 N.
RAW_RESIDUAL_BOUND = (0.35, 0.85, 0.35, 0.40)


class Stage1ResidualPolicy(nn.Module):
    """Provide separate flight and contact paths for the same frozen prior."""

    def __init__(self, track: Actor, initial_std: float = 1.0):
        super().__init__()
        self.track = track.requires_grad_(False).eval()
        self.residual = Actor(POLICY_OBS_DIM, initial_std)
        nn.init.zeros_(self.residual.mean[-1].weight)
        self.register_buffer("raw_bound", torch.tensor(RAW_RESIDUAL_BOUND), persistent=False)

    @torch.no_grad()
    def flight(self, obs45: torch.Tensor) -> torch.Tensor:
        """Return the original Track mean directly on the original input."""
        return self.track(obs45)

    def contact_base(self, obs21: torch.Tensor) -> torch.Tensor:
        """Lift the unchanged stationary bumper displacement for the Track prior."""
        if obs21.ndim != 2 or obs21.shape[1] != POLICY_OBS_DIM:
            raise ValueError(f"Stage 1-B contact requires batched {POLICY_OBS_DIM}-dimensional state")
        return self.flight(track_hold_observation(obs21[:, :18]))

    def compose(self, base_raw: torch.Tensor, latent: torch.Tensor) -> torch.Tensor:
        """Bound each PPO sample before applying the unchanged action decoder."""
        return base_raw + self.raw_bound * latent.tanh()

    def contact(self, obs21: torch.Tensor, latent: torch.Tensor) -> torch.Tensor:
        return self.compose(self.contact_base(obs21), latent)

    def contact_mean(self, obs21: torch.Tensor) -> torch.Tensor:
        return self.contact(obs21, self.residual(obs21))
