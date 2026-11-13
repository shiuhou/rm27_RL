"""Isolated regional loss weighting for the frozen Direct-CTBR PPO learner.

Only update reductions change. Collection, GAE, global advantage normalization,
ValueNorm, minibatch permutation, optimizers and simulator remain original.
"""
from contextlib import contextmanager
import json
from unittest.mock import patch

import numpy as np

from .rotation_bridge_metrics import _RegionRecorder
from .terminal_curriculum import BANDS


def regional_weights(regions, target):
    """Mean-one weights; absent regions have no samples and cannot be fabricated."""
    regions = np.asarray(regions)
    mass = np.asarray([target.get(b, 0.) for b in BANDS], dtype=np.float64)
    if (regions.size == 0 or not np.issubdtype(regions.dtype, np.integer)
            or np.any((regions < 0) | (regions >= len(BANDS)))
            or set(target) - set(BANDS) or not np.isfinite(mass).all()
            or np.any(mass < 0) or not np.isclose(mass.sum(), 1., rtol=0, atol=1e-12)):
        raise ValueError('Invalid regions or target mass')
    counts = np.bincount(regions.ravel(), minlength=len(BANDS))
    present = counts > 0
    if np.any(present & (mass == 0)):
        raise ValueError('Collected region outside frozen mixture')
    normalized = mass * present / mass[present].sum()
    per_region = np.divide(regions.size * normalized, counts,
                           out=np.zeros_like(mass), where=present)
    return per_region[regions]


def weighted_update(learner, before_minibatch=None):
    """Original PPO with weighted scalar reductions (including scalar value max)."""
    import torch
    from torch import nn
    from torch.nn import functional as F
    from simpleflight_lab import ppo

    flat = {k: v.flatten(0, 1) for k, v in learner.rollout.items()}
    n = flat['policy'].shape[0]
    metrics = []
    for _ in range(ppo.EPOCHS):
        for idx in torch.randperm(n, device=flat['policy'].device).reshape(ppo.MINIBATCHES, -1):
            if before_minibatch is not None:
                before_minibatch()
            b = {k: v[idx] for k, v in flat.items()}
            weight = b['update_weight']
            mean = learner.actor(b['policy'])
            log_ratio = ppo.log_prob(b['action'], mean, learner.actor.log_std) - b['log_prob']
            ratio = log_ratio.exp()
            surrogate = torch.minimum(ratio * b['advantage'],
                                      ratio.clamp(1-ppo.CLIP, 1+ppo.CLIP) * b['advantage'])
            actor_loss = -ppo.ACTION_DIM * (weight * surrogate).mean()
            learner.actor_opt.zero_grad(set_to_none=True)
            actor_loss.backward()
            actor_grad = nn.utils.clip_grad_norm_(learner.actor.parameters(), ppo.MAX_GRAD_NORM)
            learner.actor_opt.step()

            value = learner.critic(b['critic'])
            clipped = b['value'] + (value - b['value']).clamp(-ppo.CLIP, ppo.CLIP)
            # Preserve the pinned learner's maximum of two scalar losses;
            # do not accidentally introduce elementwise value clipping here.
            value_loss = torch.maximum(
                (weight[:, None] * F.huber_loss(value, b['return'], delta=ppo.HUBER_DELTA, reduction='none')).mean(),
                (weight[:, None] * F.huber_loss(clipped, b['return'], delta=ppo.HUBER_DELTA, reduction='none')).mean())
            learner.critic_opt.zero_grad(set_to_none=True)
            value_loss.backward()
            critic_grad = nn.utils.clip_grad_norm_(learner.critic.parameters(), ppo.MAX_GRAD_NORM)
            learner.critic_opt.step()
            with torch.no_grad():
                metrics.append(torch.stack((actor_loss, value_loss, actor_grad, critic_grad,
                                             (weight * (ratio - 1 - log_ratio)).mean())))
    return torch.stack(metrics).mean(dim=0)


@contextmanager
def install_regional_update(output, target, *, balanced, completed_transitions=0, updates=0):
    """Both arms record weights; control executes the exact original update."""
    import torch
    from simpleflight_lab import ppo

    original = ppo.PPO

    class RegionalUpdatePPO(original):
        def collect(self, env, obs):
            recorder = _RegionRecorder(env, self.rollout['reward'].shape[0])
            obs, stats = super().collect(recorder, obs)
            self.update_regions = recorder.regions.cpu().numpy()
            self.update_weights = (regional_weights(self.update_regions, target) if balanced
                                   else np.ones_like(self.update_regions, dtype=np.float64))
            if balanced:
                self.rollout['update_weight'] = torch.as_tensor(
                    self.update_weights, device=self.rollout['reward'].device, dtype=self.rollout['reward'].dtype)
            return obs, stats

        def update(self, before_minibatch=None):
            nonlocal completed_transitions, updates
            result = (weighted_update(self, before_minibatch) if balanced
                      else super().update(before_minibatch))
            if not torch.isfinite(result).all():
                raise RuntimeError('Nonfinite regional PPO update')
            completed_transitions += self.update_regions.size
            updates += 1
            weights = self.update_weights
            row = dict(completed_transitions=completed_transitions, updates=updates, balanced=balanced,
                       weight_min=float(weights.min()), weight_max=float(weights.max()),
                       effective_sample_size=float(weights.sum()**2 / np.square(weights).sum()),
                       regions={b: dict(steps=int((self.update_regions == i).sum()),
                                        loss_mass=float(weights[self.update_regions == i].sum()/weights.sum()))
                                for i, b in enumerate(BANDS)})
            with (output / 'update-weights.jsonl').open('a') as f:
                f.write(json.dumps(row, allow_nan=False) + '\n')
            return result

    with patch.object(ppo, 'PPO', RegionalUpdatePPO):
        yield
