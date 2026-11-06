"""Independent noncontact state-flight diagnostic; original contact task is unchanged."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from rm27_contact_physics.model import frame
from .env import contact_observation
from .paths import ROOT
from .stage1_env import Stage1ContactEnv, fixed_evaluation_initials
from .stage1_evaluation import Stage1Evaluator

TASK_ID = 'state_flight_control_probe_v1'
OBSERVATION_SCHEMA = 'flight_reference_state_v1'
DEFAULT_CONFIG = ROOT / 'configs/flight_probe/task_a.json'


def failure_tail_reward(start_intervals, end_intervals, *, rate_per_s=12., gamma=.999):
    """Fold an absorbing failure tail into the current 5ms PPO transition.

    Counts are completed 1ms intervals. The unfinished current slot has exponent
    zero; subsequent slots have exponents 1..N. No physical steps are performed.
    """
    if (start_intervals % 5 or not 0 <= start_intervals < 4000
            or not start_intervals <= end_intervals <= start_intervals + 5):
        raise ValueError('Failure tail requires a valid control slot on the 1ms grid')
    current_end = start_intervals + 5
    remaining_slots = (4000-current_end)//5
    discounted_future = gamma * (-np.expm1(remaining_slots*np.log(gamma))) / (1-gamma)
    return -float(rate_per_s*((current_end-end_intervals)*.001 + .005*discounted_future))


def load_config(path=DEFAULT_CONFIG):
    c = json.loads(Path(path).read_text())
    if (c.get('task_id') != TASK_ID or c.get('observation_schema') != OBSERVATION_SCHEMA
            or c.get('schema_version') != 1 or c.get('horizon_s') != 4.0
            or c.get('lambda_control') not in (0, 1)
            or c.get('base_task_config') != 'configs/stage1/task_stage1a_seed0.json'):
        raise ValueError('Invalid flight-probe contract')
    if c['stability'] != dict(window_s=.5, position_m=.05, speed_m_s=.1, tilt_deg=5., body_rate_rad_s=.2):
        raise ValueError('Flight-probe acceptance differs from the approved pilot')
    if c['control_cost'] != dict(position_scale_m=.2, speed_scale_m_s=.5, tilt_scale_deg=20., cap=4.):
        raise ValueError('Dense coordination cost differs from preregistration')
    if c['references_world_m'] != [[0., 0., 0.], [.2, 0., 0.], [-.2, 0., 0.], [0., .2, 0.], [0., -.2, 0.]]:
        raise ValueError('Reference set differs from the declared probe')
    if c['reference_probabilities'] != [.5, .125, .125, .125, .125]:
        raise ValueError('Reference distribution differs from the declared probe')
    if 'failure_tail_cost' in c and (c['lambda_control'] != 1 or
            c['failure_tail_cost'] != dict(rate_per_s=12., gamma_per_control=.999)):
        raise ValueError('Failure-tail pilot must retain the fixed 12/s candidate and PPO discount')
    return c


def fixed_cases(env):
    names = ['hover_nominal'] + [f'hover_{group}_{axis}_{sign:+d}'
        for group in ('position', 'velocity', 'attitude') for axis in range(3) for sign in (-1, 1)]
    cases = [dict(name=name, reference_index=0, initial_perturbation=p.tolist())
             for name, p in zip(names, fixed_evaluation_initials(env.rl_config))]
    cases += [dict(name=name, reference_index=i, initial_perturbation=np.zeros(12).tolist())
              for i, name in enumerate(['shift_x_plus', 'shift_x_minus', 'shift_y_plus', 'shift_y_minus'], 1)]
    return cases


class FlightEvaluator(Stage1Evaluator):
    """Use unchanged safety/contact extraction; score a sustained terminal flight state."""
    protocol = 'noncontact_terminal_halfsecond_completed_intervals_v1'

    def __init__(self, config, plant, reference, hover):
        self.reference = np.asarray(reference).copy()
        self.hover = np.asarray(hover).copy()
        self.gates = config['stability']
        self.stable_since = None
        self.position_error_integral = self.speed_squared_integral = 0.
        self.overshoot = self.peak_tilt = self.peak_speed = self.peak_rate = 0.
        self.force_intervals = []
        super().__init__(config, plant)

    def metrics(self, state):
        m = super().metrics(state)
        error = state.position - self.reference
        tilt = float(np.degrees(np.arccos(np.clip(state.rotation[2, 2], -1., 1.))))
        m.update(position_error_m=float(np.linalg.norm(error)), reference_error_world_m=error.tolist(),
                 contact_alignment_cosine=float(state.rotation[2, 2]), tilt_deg=tilt)
        return m

    def stable(self, m):
        g = self.gates
        return (m['position_error_m'] <= g['position_m'] and m['speed_m_s'] <= g['speed_m_s']
                and m['tilt_deg'] <= g['tilt_deg'] and m['body_rate_rad_s'] <= g['body_rate_rad_s'])

    def initialize(self):
        super().initialize()
        if self.stable(self.last_metrics):
            self.stable_since = 0.

    def advance(self, before, saturated=False):
        if self.done:
            raise RuntimeError('Finished flight episode')
        self._safety_state(before)
        if self.reasons:
            self.done = True
            return before
        old = self.last_metrics.copy()
        start = float(self.plant.data.time)
        if not np.isclose(start, self.interval_end_time, atol=1e-12, rtol=0):
            raise RuntimeError('Flight interval clock mismatch')
        mujoco.mj_step(self.plant.model, self.plant.data)
        data = self.plant.data
        dt = float(data.time) - start
        if not np.isclose(dt, .001, atol=1e-12, rtol=0):
            raise RuntimeError('Flight probe requires actual 1ms completed intervals')
        self.force_intervals.append(data.actuator_force[self.plant.motor_ids].copy())
        contacts = self._contacts()
        self.completed_intervals += 1
        self.elapsed_s = self.completed_intervals * .001
        self.interval_end_time = float(data.time)
        self.saturated_s += .001 if saturated else 0.
        self.last_interval = dict(start_time_s=start, end_time_s=float(data.time), contacts=contacts,
                                  proxy_saturated=bool(saturated))
        if contacts:
            self.reasons.add('UNEXPECTED_CONTACT')
            self.first_contact = dict(time_s=float(data.time), contacts=contacts, accepted=False)
            self.events.append(dict(event='FIRST_CONTACT', **self.first_contact))
        mujoco.mj_forward(self.plant.model, data)
        state = self.plant.state
        self._safety_state(state)
        m = self.last_metrics
        self.position_error_integral += .0005 * (old['position_error_m'] + m['position_error_m'])
        self.speed_squared_integral += .0005 * (old['speed_m_s']**2 + m['speed_m_s']**2)
        self.peak_tilt = max(self.peak_tilt, m['tilt_deg'])
        self.peak_speed = max(self.peak_speed, m['speed_m_s'])
        self.peak_rate = max(self.peak_rate, m['body_rate_rad_s'])
        shift = self.reference-self.hover
        if np.linalg.norm(shift) > 0:
            self.overshoot = max(self.overshoot, float((state.position-self.reference) @ (shift/np.linalg.norm(shift))))
        if self.stable(old) and self.stable(m):
            if self.stable_since is None:
                self.stable_since = self.elapsed_s-.001
        else:
            self.stable_since = None
        if self.elapsed_s >= self.config['horizon_s']-1e-12:
            if self.stable_since is None or self.elapsed_s-self.stable_since < self.gates['window_s']-1e-12:
                self.reasons.add('NOT_STABLE_AT_HORIZON')
            self.done = True
        self.done = self.done or bool(self.reasons)
        return state

    def summary(self):
        passed = self.done and not self.reasons and self.elapsed_s >= self.config['horizon_s']-1e-12
        return dict(task_id=TASK_ID, observation_schema=OBSERVATION_SCHEMA, passed=bool(passed),
            failure_modes=sorted(self.reasons), episode_elapsed_s=self.elapsed_s,
            events=copy.deepcopy(self.events), first_contact=copy.deepcopy(self.first_contact),
            metrics=copy.deepcopy(self.last_metrics), completed_physics_intervals=self.completed_intervals,
            saturated_time_s=self.saturated_s, evaluation_protocol=self.protocol,
            position_error_integral_m_s=self.position_error_integral,
            speed_rms_m_s=float(np.sqrt(self.speed_squared_integral/max(self.elapsed_s, 1e-12))),
            settling_time_s=self.stable_since if passed else None,
            stable_tail_s=0. if self.stable_since is None else self.elapsed_s-self.stable_since,
            overshoot_m=self.overshoot, peak_tilt_deg=self.peak_tilt,
            peak_speed_m_s=self.peak_speed, peak_body_rate_rad_s=self.peak_rate,
            reference_world_m=self.reference.tolist(), contact_task_success='NOT_EVALUATED')


class FlightProbeEnv(Stage1ContactEnv):
    task_id, observation_schema = TASK_ID, OBSERVATION_SCHEMA

    def __init__(self, config=DEFAULT_CONFIG, *, plant=None):
        spec = load_config(config)
        base = ROOT / spec['base_task_config']
        super().__init__(base, plant=plant)
        self.rl_config.update(copy.deepcopy(spec))
        self.sources.update(base_task_config=base, rl_config=Path(config).resolve())
        self.hover_position = self.geometry['center'] + self.rl_config['initial_state']['normal_standoff_m']*self.geometry['normal']
        self.reference_position = self.hover_position.copy()
        self.reference_index = 0

    def _observation(self):
        observation = contact_observation(self.plant.state, self.reference_position, np.array([0., 0., 1.]),
            self.previous_action_ctbr, max(0., self.horizon_s-(self.plant.state.timestamp-self.episode_start)))
        self._observation_valid = bool(np.isfinite(observation).all())
        if not self._observation_valid:
            if not self._active and hasattr(self, '_last_valid_observation'):
                return self._last_valid_observation.copy()
            raise FloatingPointError('Nonfinite flight observation')
        self._last_valid_observation = observation.copy()
        return observation

    def reset(self, *, seed=None, options=None):
        if self._closed:
            raise RuntimeError('Closed flight environment')
        gym.Env.reset(self, seed=seed)
        if options is None:
            index = int(self.np_random.choice(5, p=self.rl_config['reference_probabilities']))
            initial = self.np_random.uniform(self.initial_ranges[:, 0], self.initial_ranges[:, 1])
        else:
            if set(options) != {'reference_index', 'initial_perturbation'}:
                raise ValueError('Flight reset requires reference index and initial perturbation')
            index, initial = options['reference_index'], np.asarray(options['initial_perturbation'], float)
        if (not isinstance(index, (int, np.integer)) or isinstance(index, bool) or not 0 <= index < 5
                or initial.shape != (12,) or not np.isfinite(initial).all()
                or np.any(initial < self.initial_ranges[:, 0]-1e-12) or np.any(initial > self.initial_ranges[:, 1]+1e-12)):
            raise ValueError('Flight reset outside the declared initial distribution')
        self.reference_index, self.initial_perturbation = index, initial.copy()
        self.reference_position = self.hover_position + self.rl_config['references_world_m'][index]
        state = self.plant.reset(position=self.hover_position+self.geometry['target_rotation']@initial[:3],
            velocity=initial[3:6], rotation=Rotation.from_rotvec(initial[6:9]).as_matrix(), omega_body=initial[9:])
        self.episode_start = float(state.timestamp)
        self.previous_action_ctbr = np.r_[self.plant.hover_force, np.zeros(3)]
        self.reward_component_totals = dict(guide=0., alignment=0., action=0., constraint=0., time=0., control=0., event=0.)
        self.evaluator = FlightEvaluator(self.rl_config, self.plant, self.reference_position, self.hover_position)
        self.evaluator.initialize()
        self._terminal_result = self.evaluator.summary() if self.evaluator.done else None
        self._active = not self.evaluator.done
        return self._observation(), self._info(0, self.episode_start, False, False, dict.fromkeys(self.reward_component_totals, 0.))

    def _running_reward(self, before, after, command, previous, dt, *, saturated=False, clipped=False):
        result = super()._running_reward(before, after, command, previous, dt, saturated=saturated, clipped=clipped)
        c = self.rl_config['control_cost']
        cost = sum(min((after[key]/c[scale])**2, c['cap']) for key, scale in
            [('position_error_m', 'position_scale_m'), ('speed_m_s', 'speed_scale_m_s'), ('tilt_deg', 'tilt_scale_deg')])
        result['control'] = -self.rl_config['lambda_control']*dt*cost
        return result

    def _info(self, *args):
        result = super()._info(*args)
        result.update(reference_index=self.reference_index, reference_world_m=self.reference_position.copy())
        if self._terminal_result is not None:
            result['termination_reason'] = 'STABLE_FLIGHT' if self._terminal_result['passed'] else self._terminal_result['failure_modes']
        return result

    def step(self, action):
        self.evaluator.force_intervals = []
        start_intervals = self.evaluator.completed_intervals
        transition = super().step(action)
        forces = np.asarray(self.evaluator.force_intervals).reshape(-1, 4)
        transition[-1]['motor_force_intervals_N'] = forces
        if 'failure_tail_cost' in self.rl_config and transition[2]:
            obs, reward, terminated, truncated, info = transition
            safety_failure = bool(set(self._terminal_result['failure_modes']) - {'NOT_STABLE_AT_HORIZON'})
            compensation = failure_tail_reward(start_intervals, self.evaluator.completed_intervals) if safety_failure else 0.
            # Keep the historical seven-component schema; explicitly identify the
            # tail within event, in addition to the unchanged original event cost.
            info['reward_components']['event'] += compensation
            self.reward_component_totals['event'] += compensation
            detail = dict(terminal_reward=compensation, start_intervals=start_intervals,
                          end_intervals=self.evaluator.completed_intervals, safety_failure=safety_failure,
                          rate_per_s=12., gamma_per_control=.999, simulated_tail_intervals=0)
            self._terminal_result.update(failure_tail=detail,
                reward_component_totals=self.reward_component_totals.copy(),
                total_reward=sum(self.reward_component_totals.values()))
            info['result'] = copy.deepcopy(self._terminal_result)
            transition = obs, reward+compensation, terminated, truncated, info
        return transition


class FlightPDBaseline:
    """Independent state feedback PD reference, no trajectory or training labels."""
    def action(self, env):
        s = env.plant.state
        acceleration = np.array([4., 4., 6.])*(env.reference_position-s.position)-np.array([3.2, 3.2, 4.])*s.velocity
        desired_force = env.plant.info['mass_kg']*(acceleration-np.array(env.plant.model.opt.gravity))
        desired_rotation = frame(desired_force)
        rotation_error = Rotation.from_matrix(s.rotation.T@desired_rotation).as_rotvec()
        rates = np.clip(5.*rotation_error, -2., 2.)
        collective = float(np.dot(desired_force, s.rotation[:, 2]))
        return np.r_[np.clip(collective, env._action_low[0], env._action_high[0]), rates]
