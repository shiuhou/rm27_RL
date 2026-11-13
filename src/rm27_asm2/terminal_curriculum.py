"""Reset sampling and crash-conservative accounting for the existing PPO trainer."""
from __future__ import annotations

import fcntl
import json
import time
from pathlib import Path

import numpy as np

from .asset import read_json
from simpleflight_lab.stage1_training import LearningLimit, write_json

BANDS = ('terminal', 'braking', 'rotation_late', 'rotation_middle', 'rotation_early', 'handover',
         'rotation_bridge')
MIX = ((.10, .45, .45, 0., 0., 0., 0.), (.10, .35, .35, .20, 0., 0., 0.),
       (.10, .30, .25, .20, .15, 0., 0.))


class ResetSampler:
    def __init__(self, bands, mode, seed, settings):
        self.bands = np.asarray(bands)
        self.mode, self.settings = mode, settings
        self.fixed_mix = None
        if 'fixed_mix' in settings:
            mix = settings['fixed_mix']
            if mode != 'fixed' or not isinstance(mix, dict) or set(mix) - set(BANDS):
                raise ValueError('Static reset mixture requires fixed sampling and known bands')
            masses = np.asarray([mix.get(band, 0.) for band in BANDS], dtype=float)
            if (masses.shape != (len(BANDS),) or not np.isfinite(masses).all() or (masses < 0.).any()
                    or not np.isclose(masses.sum(), 1., rtol=0., atol=1e-12)):
                raise ValueError('Static reset mixture requires finite nonnegative mass summing to one')
            self.fixed_mix = tuple(masses.tolist())
        self.rng = np.random.default_rng(seed)
        self.stage = 0
        self.probability = np.full(len(bands), .5)
        self.refreshes = 0
        self.draws = np.zeros(len(bands), dtype=np.int64)
        if self.fixed_mix is not None:
            self.weights()

    def weights(self):
        n = len(self.bands)
        weights = np.zeros(n)
        if self.mode == 'fixed':
            masses = MIX[self.stage] if self.fixed_mix is None else self.fixed_mix
            for band, mass in zip(BANDS, masses):
                mask = self.bands == band
                if mass and not mask.any():
                    raise ValueError(f'Empty curriculum band {band}')
                if mask.any():
                    weights[mask] = mass / mask.sum()
        elif self.mode == 'adaptive':
            p = self.probability
            boundary = self.settings['boundary_floor'] + 4*p*(1-p)
            weights = self.settings['boundary_mix']*boundary/boundary.sum()
            weights += self.settings['discovery_mix']/n
            for band, mass in self.settings['retention'].items():
                mask = self.bands == band
                if not mask.any():
                    raise ValueError(f'Empty retention band {band}')
                weights[mask] += mass/mask.sum()
        else:
            raise ValueError(self.mode)
        if not np.isclose(weights.sum(), 1.):
            raise ValueError('Reset mixture must sum to one')
        return weights

    def sample(self, count):
        rows = self.rng.choice(len(self.bands), size=count, p=self.weights())
        np.add.at(self.draws, rows, 1)
        return rows

    def refresh(self, trials):
        # Each row is the SAME frozen actor, nominal and declared perturbations.
        trials = np.asarray(trials)
        if trials.shape != (len(self.bands), 3) or not np.isin(trials, [0, 1]).all():
            raise ValueError('Expected three complete binary physical outcomes per reset')
        self.probability = trials.mean(axis=1)
        self.refreshes += 1

    def state_dict(self):
        state = dict(mode=self.mode, bands=self.bands.tolist(), stage=self.stage,
                     probability=self.probability.tolist(), refreshes=self.refreshes,
                     draws=self.draws.tolist(), rng=self.rng.bit_generator.state)
        if self.fixed_mix is not None:
            state['fixed_mix'] = list(self.fixed_mix)
        return state

    def load_state_dict(self, state):
        mix = None if self.fixed_mix is None else list(self.fixed_mix)
        if (state['mode'] != self.mode or state['bands'] != self.bands.tolist()
                or state.get('fixed_mix') != mix):
            raise ValueError('Sampler configuration changed on resume')
        self.stage, self.refreshes = state['stage'], state['refreshes']
        self.probability = np.asarray(state['probability'])
        self.draws = np.asarray(state['draws'], dtype=np.int64)
        self.rng.bit_generator.state = state['rng']
        self.weights()


class CampaignLedger:
    """Reserve an entire update before executing it; interrupted reservations stay charged.

    The exclusive campaign lock spans process lifetime. A crash can over-count one
    update/time slice, never silently recover spent resources as a new allowance.
    """
    def __init__(self, directory, config_sha, caps):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.lock = (directory/'campaign.lock').open('a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.path, self.caps = directory/'ledger.json', caps
        self.data = read_json(self.path) if self.path.exists() else dict(
            config_sha256=config_sha, arms={a: dict(transitions=0, learning_s=0., updates=0,
                                                  checkpoint=None, pending=None, abandoned=[]) for a in ('A', 'B')})
        if self.data['config_sha256'] != config_sha:
            self.lock.close()
            raise ValueError('Cannot change configuration or budgets on resume')
        for arm in self.data['arms'].values():
            if arm['pending']:
                arm['abandoned'].append(arm['pending'])
                arm['pending'] = None
        self.flush()

    def flush(self):
        write_json(self.path, self.data)

    def remaining(self, arm):
        row = self.data['arms'][arm]
        rows = self.data['arms'].values()
        return (min(self.caps['per_arm_transitions']-row['transitions'],
                    self.caps['total_transitions']-sum(r['transitions'] for r in rows)),
                min(self.caps['per_arm_learning_s']-row['learning_s'],
                    self.caps['total_learning_s']-sum(r['learning_s'] for r in self.data['arms'].values())))

    def reserve(self, arm, transitions):
        frames, seconds = self.remaining(arm)
        if transitions > frames:
            raise LearningLimit('TRANSITION_LIMIT')
        if seconds <= .5:
            raise LearningLimit('TIME_LIMIT')
        seconds = min(seconds, self.caps['update_reservation_s'])
        row = self.data['arms'][arm]
        row['transitions'] += transitions
        row['learning_s'] += seconds
        row['pending'] = dict(transitions=transitions, learning_s=seconds,
                              started_unix_s=time.time())
        self.flush()
        return seconds

    def settle(self, arm, elapsed, checkpoint, complete):
        row = self.data['arms'][arm]
        reserved = row['pending']
        if reserved is None or elapsed < 0:
            raise ValueError('No pending learning reservation')
        row['learning_s'] += elapsed-reserved['learning_s']
        row['pending'] = None
        if complete:
            row['updates'] += 1
            row['checkpoint'] = checkpoint
        else:
            row['abandoned'].append(dict(reserved, actual_learning_s=elapsed))
        self.flush()

    def close(self):
        self.lock.close()
