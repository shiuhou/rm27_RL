"""Independent learnable flight baseline; unchanged physical plant and flight scorer.

Upstream-inspired body observations and bounded proximity/stability rewards.
No controller labels, residual controller, privileged motor observations or trajectory.
"""
from __future__ import annotations
import json
from pathlib import Path
import gymnasium as gym
import numpy as np
from scipy.spatial.transform import Rotation
from .flight_probe import FlightProbeEnv
from .paths import ROOT

CONFIG = ROOT/'configs/flight_working/attitude_baseline.json'

class WorkingFlightEnv(FlightProbeEnv):
    def __init__(self, config=CONFIG, *, plant=None, training=True):
        self.working_config = json.loads(Path(config).read_text())
        super().__init__(ROOT/self.working_config['base_config'], plant=plant)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (12,), dtype=np.float32)
        self.curriculum = 'hover'
        self.training = training
        self.rl_config['reward'] = dict(self.rl_config['reward'],
            event_success=self.working_config['reward']['event_success'],
            event_failure=self.working_config['reward']['event_failure'])

    def set_curriculum(self, stage):
        if stage not in ('hover', 'tracking'):
            raise ValueError(stage)
        self.curriculum = stage

    def reset(self, *, seed=None, options=None):
        if options is None and self.curriculum == 'hover':
            # Seed before sampling; parent reset preserves this RNG when seed=None.
            gym.Env.reset(self, seed=seed)
            options = dict(reference_index=0, initial_perturbation=
                self.np_random.uniform(self.initial_ranges[:, 0], self.initial_ranges[:, 1]))
            seed = None
        return super().reset(seed=seed, options=options)

    def _observation(self):
        s = self.plant.state
        # R maps body to world. All vector errors below use the body frame.
        # Subtracting nominal up removes the constant gravity feature, not tilt.
        obs = np.r_[s.rotation.T@(self.reference_position-s.position)/.2,
                    s.rotation.T@s.velocity/.5,
                    (s.rotation.T@np.array([0.,0.,1.])-np.array([0.,0.,1.]))/.2,
                    s.omega_body/2.].astype(np.float32)
        self._observation_valid = bool(np.isfinite(obs).all())
        if not self._observation_valid:
            if not self._active and hasattr(self, '_last_valid_observation'):
                return self._last_valid_observation.copy()
            raise FloatingPointError('Nonfinite flight observation')
        self._last_valid_observation = obs.copy()
        return obs

    def _running_reward(self, before, after, command, previous, dt, **kwargs):
        r = self.working_config['reward']
        e, v = after['position_error_m'], after['speed_m_s']
        tilt = np.deg2rad(after['tilt_deg'])
        rate = after['body_rate_rad_s']
        # Nonnegative, bounded rewards make early failure lose future rewards;
        # the identical negative terminal cost is also discounted less at failure.
        position = .5/(1+(e/r['position_scale_m'])**2)+.5*np.exp(-(e/r['fine_position_scale_m'])**2)
        stability = (np.exp(-(v/r['speed_scale_m_s'])**2)
            + np.exp(-(tilt/r['tilt_scale_rad'])**2)
            + np.exp(-(rate/r['rate_scale_rad_s'])**2))/3.
        gain = dt*r['gain_per_s']
        return dict(guide=gain*.6*position, alignment=gain*.3*stability,
                    action=0., constraint=0., time=gain*.1, control=0., event=0.)

    def step(self, physical_ctbr):
        obs, reward, terminated, truncated, info = super().step(physical_ctbr)
        if self.working_config.get('horizon_bootstrap', False) and terminated:
            result = info['result']
            if result['episode_elapsed_s'] >= 4.-1e-12 and not (
                    set(result['failure_modes'])-{'NOT_STABLE_AT_HORIZON'}):
                # Continuing flight objective, artificial 4s training time limit.
                # Independent final-halfsecond scoring remains exactly unchanged.
                event = info['reward_components']['event']
                reward -= event
                info['reward_components']['event'] = 0.
                self.reward_component_totals['event'] -= event
                result['reward_component_totals'] = self.reward_component_totals.copy()
                result['total_reward'] = sum(self.reward_component_totals.values())
                terminated, truncated = False, True
                info['working_horizon_bootstrap'] = True
        return obs, reward, terminated, truncated, info

class FeasibleCTBR(gym.ActionWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.action_space = gym.spaces.Box(-1., 1., (4,), dtype=np.float32)

    def action(self, action):
        a = np.clip(np.asarray(action, float), -1., 1.)
        c = self.env.working_config
        collective = self.env.plant.hover_force*(1+c['collective_fraction']*a[0])
        if c.get('action_mode', 'body_rates') == 'attitude_setpoint':
            rotation = self.env.plant.state.rotation
            yaw = np.arctan2(rotation[1,0], rotation[0,0])
            desired = Rotation.from_euler('ZYX', [yaw, c['tilt_range_rad']*a[2],
                                                 c['tilt_range_rad']*a[1]]).as_matrix()
            rates = c['attitude_gain_per_s']*Rotation.from_matrix(rotation.T@desired).as_rotvec()
            rates[2] += c['yaw_rate_gain_rad_s']*a[3]
            rates = np.clip(rates, -c['rate_gain_rad_s'], c['rate_gain_rad_s'])
        else:
            rates = c['rate_gain_rad_s']*a[1:]
        return np.r_[collective, rates]

    def reverse_action(self, physical):
        c = self.env.working_config
        if c.get('action_mode', 'body_rates') != 'body_rates':
            raise ValueError('Attitude setpoints are not a state-independent inverse of CTBR')
        p = np.asarray(physical)
        return np.r_[(p[0]/self.env.plant.hover_force-1)/c['collective_fraction'], p[1:]/c['rate_gain_rad_s']]
