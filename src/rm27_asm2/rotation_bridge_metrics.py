"""Observe regional PPO rollouts through the pinned workflow's existing learner."""

from contextlib import contextmanager
import json
import math
from pathlib import Path
from unittest.mock import patch

import numpy as np

from .terminal_curriculum import BANDS


SATURATION_TANH = 0.95
SUMMARY_FIELDS = (
    'reward', 'critic_stored', 'critic_denormalized', 'gae_raw', 'gae_normalized',
    'log_prob', 'exploration_z_l2',
)


class _RegionRecorder:
    """Capture the region of the episode being stepped before automatic reset."""

    def __init__(self, env, steps):
        self.env = env
        self.regions = env.active_band.new_empty((steps, env.active_band.numel()))
        self.tick = 0

    def step(self, action):
        self.regions[self.tick].copy_(self.env.active_band)
        self.tick += 1
        return self.env.step(action)

    def __getattr__(self, name):
        return getattr(self.env, name)


def _summarize(samples):
    regions = samples['region']
    if np.any((regions < 0) | (regions >= len(BANDS))):
        raise ValueError('Unknown reset region in PPO rollout')
    if any(not np.isfinite(samples[key]).all() for key in (*SUMMARY_FIELDS, 'action_raw', 'log_std')):
        raise ValueError('Nonfinite sample in completed PPO rollout')

    # These rates describe policy-command clipping pressure, separate from motor saturation.
    saturated = np.abs(np.tanh(samples['action_raw'])) >= SATURATION_TANH
    summary = {}
    for index, band in enumerate(BANDS):
        selected = regions == index
        count = int(selected.sum())
        row = {'steps': count, 'ends': int(samples['done'][selected].sum())}
        if count:
            row['command_saturation_rate'] = saturated[selected].mean(axis=0).tolist()
            for key in SUMMARY_FIELDS:
                values = samples[key][selected]
                p10, p50, p90 = np.quantile(values, [.10, .50, .90])
                row[key] = dict(mean=float(values.mean(dtype=np.float64)),
                                p10=float(p10), p50=float(p50), p90=float(p90))
        summary[band] = row
    return summary


def _write_rollout(output, pending, row):
    completed, update, tensors = pending
    samples = {key: value.detach().cpu().numpy() for key, value in tensors.items()}
    log_std = samples['log_std'].astype(np.float64)
    # Exact Gaussian identity for the norm of (sample - actor mean) / actor std,
    # up to the precision of the log-probability recorded by the actual learner.
    z_squared = -2. * (samples['log_prob'] + log_std.sum()
                       + 0.5 * log_std.size * math.log(2. * math.pi))
    samples['exploration_z_l2'] = np.sqrt(np.maximum(z_squared, 0.))
    summary = {
        'format': 1,
        'completed_transitions': completed,
        'charged_transitions': row['transitions'],
        'updates': update,
        'regions': _summarize(samples),
        'policy_std_raw': np.exp(log_std).tolist(),
        'command_saturation_abs_tanh': SATURATION_TANH,
    }
    directory = output / 'regional-rollouts'
    directory.mkdir(exist_ok=True)
    path = directory / f'rollout-{completed:06d}.npz'
    if path.exists():
        raise FileExistsError(path)
    temporary = path.with_suffix('.tmp')
    with temporary.open('wb') as stream:
        np.savez_compressed(stream, **samples, band_names=BANDS,
                            summary_json=json.dumps(summary, allow_nan=False))
    temporary.replace(path)


@contextmanager
def install_metrics(output, *, completed_transitions=0, updates=0):
    """Instrument one workflow arm in its isolated training process.

    The initial counters come from the same resume checkpoint as the workflow.
    A rollout is written after its matching learning-log row appears. The
    original learner owns collection, GAE, optimization and update completion.
    """
    from simpleflight_lab import ppo

    output = Path(output)
    original_ppo = ppo.PPO
    original_gae = ppo.compute_gae
    capture = pending = None

    def record_gae(reward, done, value, next_value, **kwargs):
        advantage, returns = original_gae(reward, done, value, next_value, **kwargs)
        if capture is not None:
            capture['critic_denormalized'] = value[..., 0]
            capture['gae_raw'] = advantage[..., 0]
        return advantage, returns

    def publish_completed():
        nonlocal pending
        log = output / 'learning.jsonl'
        if pending is None or not log.exists():
            return
        rows = log.read_text().splitlines()
        if not rows:
            return
        row = json.loads(rows[-1])
        completed, update, _ = pending
        if row['updates'] < update:
            return
        if (row['completed_transitions'], row['updates']) != (completed, update):
            raise ValueError('Regional rollout counters differ from completed workflow update')
        _write_rollout(output, pending, row)
        pending = None

    class RegionalPPO(original_ppo):
        def collect(self, env, obs):
            nonlocal capture, pending, completed_transitions, updates
            publish_completed()
            if pending is not None:
                raise RuntimeError('Previous PPO rollout has no completed workflow log row')
            batch = self.rollout
            recorder = _RegionRecorder(env, batch['reward'].shape[0])
            capture = {}
            try:
                obs, stats = super().collect(recorder, obs)
                tensors = capture
            finally:
                capture = None
            if recorder.tick != recorder.regions.shape[0] or 'gae_raw' not in tensors:
                raise RuntimeError('Incomplete PPO region or GAE capture')
            tensors.update(
                region=recorder.regions,
                reward=batch['reward'][..., 0],
                critic_stored=batch['value'][..., 0],
                gae_normalized=batch['advantage'],
                action_raw=batch['action'],
                log_prob=batch['log_prob'],
                log_std=self.actor.log_std.detach().clone(),
                done=batch['done'][..., 0],
            )
            completed_transitions += recorder.regions.numel()
            updates += 1
            pending = completed_transitions, updates, tensors
            return obs, stats

    with patch.object(ppo, 'PPO', RegionalPPO), patch.object(ppo, 'compute_gae', record_gae):
        try:
            yield
        finally:
            publish_completed()
