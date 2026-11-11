"""Single-agent tensor port of upstream MAPPO (MIT; see upstream/SimpleFlight/LICENSE).

Uses the upstream ValueNorm1 and time-major GAE, which have no simulator or
TorchRL dependencies. The Gaussian is over raw actions; tanh belongs to the env.
"""

import math
import runpy
from pathlib import Path

import torch
from torch import nn
from torch.nn import functional as F

UPSTREAM_REVISION = "f5ae8fc3689edc7c982e07cbfd587eeeab72f279"
_UTILS = Path(__file__).resolve().parents[1] / "upstream/SimpleFlight/omni_drones/learning/utils"
ValueNorm1 = runpy.run_path(str(_UTILS / "valuenorm.py"))["ValueNorm1"]
compute_gae = runpy.run_path(str(_UTILS / "gae.py"))["compute_gae_"]

ROLLOUT_STEPS = 64
MINIBATCHES = 16
EPOCHS = 4
CLIP = 0.1
GAMMA = 0.995
GAE_LAMBDA = 0.95
LR = 0.0005
MAX_GRAD_NORM = 10.0
HUBER_DELTA = 10.0
ACTION_DIM = 4


def network(in_dim: int, out_dim: int, actor: bool) -> nn.Sequential:
    layers = [nn.LayerNorm(in_dim)]
    for i in range(3):
        layers.append(nn.Linear(in_dim, 256))
        if i < 2:
            layers.append(nn.ELU())
        layers.append(nn.LayerNorm(256))
        in_dim = 256
    head = nn.Linear(in_dim, out_dim)
    if actor:
        nn.init.xavier_uniform_(head.weight, gain=0.01)
        nn.init.zeros_(head.bias)
    else:
        nn.init.orthogonal_(head.weight, gain=0.01)
    return nn.Sequential(*layers, head)


def log_prob(action: torch.Tensor, mean: torch.Tensor, log_std: torch.Tensor) -> torch.Tensor:
    return (-0.5 * ((action - mean) * torch.exp(-log_std)).square()
            - log_std - 0.5 * math.log(2.0 * math.pi)).sum(dim=-1)


class Actor(nn.Module):
    def __init__(self, obs_dim: int, initial_std: float = 1.0):
        super().__init__()
        self.mean = network(obs_dim, ACTION_DIM, actor=True)
        self.log_std = nn.Parameter(torch.full((ACTION_DIM,), math.log(initial_std)))

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.mean(obs)

    def sample(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        mean = self(obs)
        action = mean + self.log_std.exp() * torch.randn_like(mean)
        return action, log_prob(action, mean, self.log_std)


class PPO:
    def __init__(self, obs: dict[str, torch.Tensor], initial_std: float, *, learning_rate=LR,
                 gamma=GAMMA, gae_lambda=GAE_LAMBDA, actor: Actor | None = None):
        self.actor = (Actor(obs["policy"].shape[-1], initial_std) if actor is None else actor).to(obs["policy"].device)
        self.critic = network(obs["critic"].shape[-1], 1, actor=False).to(obs["critic"].device)
        self.normalizer = ValueNorm1((1,), beta=0.995).to(obs["critic"].device)
        fused = obs["policy"].is_cuda
        self.actor_opt = torch.optim.Adam(self.actor.parameters(), lr=learning_rate, fused=fused)
        self.critic_opt = torch.optim.Adam(self.critic.parameters(), lr=learning_rate, fused=fused)
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        worlds = obs["policy"].shape[0]
        shape = (ROLLOUT_STEPS, worlds)
        device = obs["policy"].device
        self.rollout = {k: torch.empty((*shape, v.shape[-1]), device=device) for k, v in obs.items()}
        self.rollout["action"] = torch.empty((*shape, ACTION_DIM), device=device)
        self.rollout["value"] = torch.empty((*shape, 1), device=device)
        self.rollout["log_prob"] = torch.empty(shape, device=device)
        self.rollout["reward"] = torch.empty((*shape, 1), device=device)
        self.rollout["done"] = torch.empty((*shape, 1), device=device, dtype=torch.bool)
        self.last_episode_return = torch.zeros(worlds, device=device)
        self.episode_return = torch.zeros(worlds, device=device)
        self.episode_ticks = torch.zeros(worlds, device=device)
        self.last_episode_ticks = torch.zeros(worlds, device=device)

    @torch.no_grad()
    def collect(self, env, obs):
        batch = self.rollout
        errors = torch.empty_like(batch["reward"])
        for t in range(ROLLOUT_STEPS):
            for key in ("policy", "critic"):
                batch[key][t].copy_(obs[key])
            action, old_log_prob = self.actor.sample(obs["policy"])
            batch["action"][t].copy_(action)
            batch["log_prob"][t].copy_(old_log_prob)
            batch["value"][t].copy_(self.critic(obs["critic"]))
            obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated | truncated
            batch["reward"][t, :, 0].copy_(reward)
            batch["done"][t, :, 0].copy_(done)
            self.episode_return.add_(reward)
            self.episode_ticks.add_(1)
            self.last_episode_return.copy_(torch.where(done, self.episode_return, self.last_episode_return))
            self.last_episode_ticks.copy_(torch.where(done, self.episode_ticks, self.last_episode_ticks))
            self.episode_return.masked_fill_(done, 0)
            self.episode_ticks.masked_fill_(done, 0)
            errors[t, :, 0] = env.step_error_m
        values = self.normalizer.denormalize(batch["value"])
        next_value = self.normalizer.denormalize(self.critic(obs["critic"]))
        advantages, returns = compute_gae(batch["reward"], batch["done"], values, next_value,
                                          gamma=self.gamma, lmbda=self.gae_lambda)
        batch["advantage"] = (advantages.squeeze(-1) - advantages.mean()) / (advantages.std() + 1e-8)
        self.normalizer.update(returns)
        batch["return"] = self.normalizer.normalize(returns)
        completed = self.last_episode_ticks > 0
        n_completed = completed.sum().clamp(min=1)
        stats = torch.stack((batch["reward"].mean(), errors.square().mean().sqrt(),
                             batch["done"].sum(), self.last_episode_return.sum() / n_completed,
                             self.last_episode_ticks.sum() / n_completed))
        return obs, stats

    def update(self, before_minibatch=None):
        flat = {k: v.flatten(0, 1) for k, v in self.rollout.items()}
        n = flat["policy"].shape[0]
        metrics = []
        for _ in range(EPOCHS):
            for idx in torch.randperm(n, device=flat["policy"].device).reshape(MINIBATCHES, -1):
                if before_minibatch is not None:
                    before_minibatch()
                b = {k: v[idx] for k, v in flat.items()}
                mean = self.actor(b["policy"])
                log_ratio = log_prob(b["action"], mean, self.actor.log_std) - b["log_prob"]
                ratio = log_ratio.exp()
                surrogate = torch.minimum(ratio * b["advantage"], ratio.clamp(1-CLIP, 1+CLIP) * b["advantage"])
                actor_loss = -ACTION_DIM * surrogate.mean()
                self.actor_opt.zero_grad(set_to_none=True)
                actor_loss.backward()
                actor_grad = nn.utils.clip_grad_norm_(self.actor.parameters(), MAX_GRAD_NORM)
                self.actor_opt.step()

                value = self.critic(b["critic"])
                clipped = b["value"] + (value - b["value"]).clamp(-CLIP, CLIP)
                value_loss = torch.maximum(F.huber_loss(value, b["return"], delta=HUBER_DELTA),
                                           F.huber_loss(clipped, b["return"], delta=HUBER_DELTA))
                self.critic_opt.zero_grad(set_to_none=True)
                value_loss.backward()
                critic_grad = nn.utils.clip_grad_norm_(self.critic.parameters(), MAX_GRAD_NORM)
                self.critic_opt.step()
                with torch.no_grad():
                    metrics.append(torch.stack((actor_loss, value_loss, actor_grad, critic_grad,
                                                (ratio - 1 - log_ratio).mean())))
        return torch.stack(metrics).mean(dim=0)
