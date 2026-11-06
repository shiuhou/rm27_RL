"""Bounded paired PPO flight diagnostic using the existing SB3/checkpoint path."""
from __future__ import annotations

import argparse
import copy
import json
import hashlib
import pickle
from pathlib import Path
import shutil
import time
import traceback

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure

from .flight_probe import FlightProbeEnv, FlightPDBaseline, TASK_ID, fixed_cases, load_config
from .paths import ROOT
from .ppo import (NormalizedCTBR, _BudgetCallback, _WallLimit, _algorithm_kwargs,
                  _learn_rollout, _load_checkpoint, _model_state, _normalization_state,
                  _save_checkpoint, _state_hash, _write_json, make_vec_env)
from .provenance import provenance
from .stage1_runner import SmokeBudget

REWARD_KEYS = ('guide', 'alignment', 'action', 'constraint', 'time', 'control', 'event')


def validate_train_config(c):
    reference = json.loads((ROOT/'configs/stage1/ppo_stage1a_beta01_seed0.json').read_text())
    if (c.get('protocol') not in ('flight_probe_ab', 'flight_probe_failure_tail') or c.get('task_id') != TASK_ID
            or c.get('algorithm') != 'PPO' or c.get('seed') != 0 or c.get('n_envs') != 1
            or c.get('torch_threads') != 1 or c.get('rate_scale_beta') != .1
            or c.get('action_parameterization') != 'hover_centered_v1' or c.get('ppo') != reference['ppo']
            or c.get('budget') != dict(transitions=65536, learning_wall_s=900)
            or c.get('evaluation_budget') != dict(controls=55200, physics_substeps=276000, wall_s=900)):
        raise ValueError('Flight A/B differs from the approved pilot')
    task = load_config(ROOT/c['env_config'])
    if c['protocol'] == 'flight_probe_failure_tail':
        if task.get('failure_tail_cost') != dict(rate_per_s=12., gamma_per_control=c['ppo']['gamma']):
            raise ValueError('Failure-tail and PPO discounts must agree')
        if c.get('initial_checkpoint') != 'artifacts/agile_flight/state_flight_probe_ab_seed0_01/run/b/checkpoints/initial':
            raise ValueError('C must start from the original B initial checkpoint')
    elif 'failure_tail_cost' in task:
        raise ValueError('Historical A/B does not include failure-tail compensation')
    return c


def capture(env, info, reward=0.):
    plant, state = env.plant, env.plant.state
    forces = np.zeros((5, 4))
    f = info.get('motor_force_intervals_N', np.empty((0, 4)))
    forces[:len(f)] = f
    return dict(time_s=float(state.timestamp-env.episode_start), qpos=plant.data.qpos.copy(), qvel=plant.data.qvel.copy(),
        position=state.position, velocity=state.velocity, rotation=state.rotation, omega_body=state.omega_body,
        motor_state_N=plant.data.act.copy(), motor_commands_N=plant.data.ctrl.copy(),
        motor_force_substeps_N=forces, physics_substeps=int(info['physics_substeps']),
        observation=env._observation(), action_ctbr=env.previous_action_ctbr.copy(),
        reward=float(reward), reward_components=np.array([info['reward_components'][k] for k in REWARD_KEYS]),
        action_clipped=bool(info['action_clipped']), proxy_saturated=bool(info['proxy_saturated']),
        reference_index=env.reference_index, reference_world_m=env.reference_position.copy(),
        transition_id=0, has_policy_action=False, gaussian_sample=np.zeros(4), actor_mean=np.zeros(4),
        actor_std=np.zeros(4), executed_action_normalized=np.zeros(4), action_requested_ctbr=np.zeros(4),
        normalized_observation_before=np.zeros(26, np.float32), behavior_value=0., behavior_log_prob=0.)


def save_episode(path, env, rows, summary, initial_state):
    path.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(path/'trajectory.npz', initial_integration_state=initial_state,
                        **{k:np.asarray([r[k] for r in rows]) for k in rows[0]})
    result = dict(summary, total_reward=float(sum(r['reward'] for r in rows)),
                  reference_index=env.reference_index, initial_perturbation=env.initial_perturbation.tolist(),
                  reward_component_names=list(REWARD_KEYS), zero_thrust_time_s=float(sum(
                      r['physics_substeps']*.001 for r in rows[1:] if r['action_ctbr'][0] == 0)),
                  physical_ctbr_clip_controls=sum(r['action_clipped'] for r in rows[1:]),
                  mixer_clip_controls=sum(r['proxy_saturated'] for r in rows[1:]),
                  actor_clip_controls=sum(bool(np.any(np.abs(r['gaussian_sample'])>1)) for r in rows[1:]),
                  controls=len(rows)-1, trajectory_fields='Gaussian sample and mean/std are distinct; 1ms actual solver motor forces padded to five rows, valid count=physics_substeps')
    _write_json(path/'summary.json', result)
    _write_json(path/'config.json', dict(rl=env.rl_config, private_model_path=str(env.plant.sources['private_scene_binary'])))
    return result


class Recorder(gym.Wrapper):
    def __init__(self, env, output, budget):
        super().__init__(env)
        self.output, self.budget = output, budget
        self.index = self.completed = 0
        self.counts = {}
        self.rows = []
        self.pending_done = None

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        raw = self.unwrapped
        self.snapshot = raw.plant.snapshot()
        self.rows = [capture(raw, info)]
        self.episode_identity = dict(reference_index=raw.reference_index,
                                     initial_perturbation=raw.initial_perturbation.copy())
        return obs, info

    def step(self, action):
        try:
            self.budget.before_step()
        except TimeoutError as exc:
            raise _WallLimit(str(exc)) from exc
        result = self.env.step(action)
        self.budget.record(result[4])
        row = capture(self.unwrapped, result[4], result[1])
        row['transition_id'] = self.budget.controls
        row['executed_action_normalized'] = np.asarray(action).copy()
        mapping = self.env.env  # Monitor -> NormalizedCTBR
        row['action_requested_ctbr'] = mapping.action(action)
        self.rows.append(row)
        # DummyVecEnv resets before callback. Detach the just-ended episode now.
        if result[2] or result[3]:
            self.pending_done = (self.rows, self.snapshot, dict(result[4]['result'], execution_status='COMPLETE'), self.episode_identity)
        if any(x in result[4].get('result', {}).get('failure_modes', []) for x in ('NUMERICAL_FAILURE','MUJOCO_WARNING')):
            raise FloatingPointError('Flight numerical/simulator failure')
        return result

    def annotate(self, **values):
        rows = self.pending_done[0] if self.pending_done else self.rows
        rows[-1].update(values, has_policy_action=True)
        if self.pending_done:
            rows, snapshot, result, identity = self.pending_done
            # Preserve ended episode identity after DummyVecEnv has reset the plant.
            raw = self.unwrapped
            current = (raw.reference_index, raw.initial_perturbation)
            raw.reference_index, raw.initial_perturbation = identity['reference_index'], identity['initial_perturbation']
            saved = save_episode(self.output/f'episode_{self.index:05d}', raw, rows, result, snapshot)
            raw.reference_index, raw.initial_perturbation = current
            with (self.output/'episodes.jsonl').open('a') as f:
                f.write(json.dumps(saved, allow_nan=False)+'\n')
            for key in result['failure_modes'] or ['STABLE_FLIGHT']:
                self.counts[key] = self.counts.get(key, 0)+1
            self.index += 1
            self.completed += 1
            self.pending_done = None

    def flush(self):
        if len(self.rows)>1:
            save_episode(self.output/f'episode_{self.index:05d}_partial', self.unwrapped, self.rows,
                         dict(self.unwrapped.evaluator.summary(), execution_status='INCOMPLETE_BUDGET_END'), self.snapshot)


class ProbeCallback(_BudgetCallback):
    def __init__(self, deadline, recorder, output, update):
        super().__init__(deadline)
        self.recorder, self.output, self.update = recorder, output, update
        self.means, self.stds = [], []

    def _on_step(self):
        ok = super()._on_step()
        with torch.no_grad():
            distribution = self.model.policy.get_distribution(self.locals['obs_tensor']).distribution
            mean, std = distribution.mean[0].cpu().numpy(), distribution.stddev[0].cpu().numpy()
        self.means.append(mean.copy()); self.stds.append(std.copy())
        self.recorder.annotate(gaussian_sample=self.locals['actions'][0].copy(), actor_mean=mean.copy(), actor_std=std.copy(),
            normalized_observation_before=self.locals['obs_tensor'][0].cpu().numpy().copy(),
            behavior_value=float(self.locals['values'][0].item()), behavior_log_prob=float(self.locals['log_probs'][0].item()))
        return ok

    def _on_rollout_end(self):
        super()._on_rollout_end()
        b = self.model.rollout_buffer
        self.output.mkdir(parents=True, exist_ok=True)
        end = self.model.num_timesteps
        np.savez_compressed(self.output/f'update_{self.update:04d}.npz',
            transition_id=np.arange(end-b.buffer_size+1,end+1), normalized_observation=b.observations[:,0],
            gaussian_sample=b.actions[:,0], actor_mean=np.asarray(self.means), actor_std=np.asarray(self.stds),
            reward=b.rewards[:,0], value=b.values[:,0], log_prob=b.log_probs[:,0],
            advantage=b.advantages[:,0], value_target=b.returns[:,0], episode_start=b.episode_starts[:,0],
            last_done=self.locals['dones'], last_bootstrap_value=self.locals['values'].cpu().numpy(),
            note='GAE before PPO minibatch advantage normalization; sampled actions are unclipped')


def record_case(env, case, output, budget, *, model=None, vec=None, baseline=None):
    opts={k:case[k] for k in ('reference_index','initial_perturbation')}
    obs, info = env.reset(seed=0, options=opts)
    snapshot = env.plant.snapshot();rows=[capture(env,info)]
    mapping = NormalizedCTBR(env, 'hover_centered_v1', rate_scale_beta=.1)
    status='COMPLETE'
    try:
        while env._active:
            budget.before_step()
            if baseline is not None:
                desired=baseline.action(env)
                action=(desired-mapping.center)/np.r_[mapping.half_range[0],mapping.half_range[1:]*.1]
                mean=action.copy();std=np.zeros(4);value=logprob=0.;normobs=obs.copy()
            else:
                normobs=vec.normalize_obs(obs)
                with torch.no_grad():
                    tensor=torch.as_tensor(normobs[None],device=model.device)
                    d=model.policy.get_distribution(tensor).distribution
                    mean=d.mean[0].cpu().numpy();std=d.stddev[0].cpu().numpy()
                    value=float(model.policy.predict_values(tensor).item())
                action=mean.copy();logprob=0.
            executed=np.clip(action,-1,1);requested=mapping.action(executed)
            obs,reward,done,truncated,info=env.step(requested);budget.record(info)
            row=capture(env,info,reward);row.update(has_policy_action=True,gaussian_sample=action.copy(),actor_mean=mean.copy(),actor_std=std.copy(),
                executed_action_normalized=executed.copy(),action_requested_ctbr=requested.copy(),normalized_observation_before=normobs.copy(),
                behavior_value=value,behavior_log_prob=logprob,transition_id=len(rows))
            rows.append(row)
            if truncated:raise RuntimeError('Flight task must terminate, not truncate')
    except TimeoutError:
        status='INCOMPLETE_EVALUATION_BUDGET'
    summary=dict(env._terminal_result or env.evaluator.summary(), case=case['name'], execution_status=status,
                 policy='TRADITIONAL_STATE_PD' if baseline else 'DETERMINISTIC_CHECKPOINT',
                 gaussian_sample_semantics='DETERMINISTIC_MEAN_NO_RANDOM_SAMPLE')
    return save_episode(output,env,rows,summary,snapshot)


def evaluate_probe(checkpoint, output, *, budget=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    own=budget is None
    if own:budget=SmokeBudget(controls=18400,substeps=92000,wall_s=900)
    model,vec,meta=_load_checkpoint(checkpoint,training=False,n_envs=1,expected_task=TASK_ID)
    env=vec.venv.envs[0].unwrapped;stats=_state_hash(_normalization_state(vec));results=[]
    try:
        for i,case in enumerate(fixed_cases(env)):
            if time.monotonic()>=budget.deadline or budget.controls>=budget.max_controls:
                results.append(dict(case=case['name'],execution_status='NOT_RUN_BUDGET',passed=False));continue
            results.append(record_case(env,case,output/f'episode_{i:02d}',budget,model=model,vec=vec))
        if stats!=_state_hash(_normalization_state(vec)):raise RuntimeError('Evaluation modified normalizer')
        summary=dict(task_id=TASK_ID,success_count=sum(r['passed'] for r in results),episode_count=23,
            complete=all(r['execution_status']=='COMPLETE' for r in results),episodes=results,normalization_frozen=True)
        _write_json(output/'summary.json',summary)
        return summary
    finally:vec.close()


def run_engineering(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    budget=SmokeBudget(controls=6000,substeps=30000,wall_s=180)
    started=time.monotonic();env=FlightProbeEnv(ROOT/'configs/flight_probe/task_a.json')
    initialized=time.monotonic()-started;cases=fixed_cases(env);clearance=[];results=[]
    try:
        for case in cases:
            obs,info=env.reset(seed=0,options={k:case[k] for k in ('reference_index','initial_perturbation')})
            np.testing.assert_allclose(obs[:3],env.reference_position-env.plant.state.position,atol=1e-7)
            if not env._active:raise RuntimeError(f'Infeasible initial state: {case}')
            clearance.append(dict(case=case['name'],initial_overlap=False))
        for index in [0,19,20,21,22]:
            results.append(record_case(env,cases[index],output/cases[index]['name'],budget,baseline=FlightPDBaseline()))
        summary=dict(passed=all(x['passed'] for x in results),baseline_cases=results,reset_checks=clearance,
                     cost=budget.summary(),initialization_wall_s=initialized,compile_wall_s=0)
        _write_json(output/'summary.json',summary)
        return summary
    finally:env.close()


def run_pair(output, *, failure_tail=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    eval_budget=SmokeBudget(controls=55200,substeps=276000,wall_s=900);eval_budget.pause()
    ca=validate_train_config(json.loads((ROOT/'configs/flight_probe/ppo_a.json').read_text()))
    cb=validate_train_config(json.loads((ROOT/'configs/flight_probe/ppo_b.json').read_text()))
    ta=load_config(ROOT/ca.pop('env_config'));tb=load_config(ROOT/cb.pop('env_config'))
    if ca!=cb or ta.pop('lambda_control')!=0 or tb.pop('lambda_control')!=1 or ta!=tb:
        raise ValueError('A/B must differ only in lambda_control')
    if failure_tail:
        cc=validate_train_config(json.loads((ROOT/'configs/flight_probe/ppo_c.json').read_text()))
        original=json.loads((ROOT/'configs/flight_probe/ppo_b.json').read_text())
        compared={k:v for k,v in cc.items() if k not in ('protocol','env_config','initial_checkpoint')}
        assert compared=={k:v for k,v in original.items() if k not in ('protocol','env_config')}
        tc=load_config(ROOT/cc['env_config']); tc.pop('failure_tail_cost')
        assert tc==load_config(ROOT/original['env_config'])
    common=None;results={};total_learn=0.;total_controls=0;initial=None
    for arm in (('c',) if failure_tail else ('a','b')):
        start_init=time.monotonic();cfg=validate_train_config(json.loads((ROOT/f'configs/flight_probe/ppo_{arm}.json').read_text()))
        dest=output/arm;dest.mkdir();vec=make_vec_env(cfg)
        torch.set_num_threads(1);model=PPO('MlpPolicy',vec,seed=0,device='cpu',verbose=0,**_algorithm_kwargs(cfg));model.set_logger(configure(format_strings=[]))
        vec.reset();env=vec.venv.envs[0].unwrapped
        if failure_tail:
            checkpoint=ROOT/cfg['initial_checkpoint']
            meta=json.loads((checkpoint/'checkpoint.json').read_text())
            assert meta['completed_rollout_updates']==0 and meta['state']['num_timesteps']==0
            for name,digest in meta['files'].items():
                assert hashlib.sha256((checkpoint/name).read_bytes()).hexdigest()==digest
            # set_parameters restores both policy and optimizer without drawing RNG.
            model.set_parameters(checkpoint/'model.zip',exact_match=True,device='cpu')
            with (checkpoint/'normalize.pkl').open('rb') as stream: saved_normalizer=pickle.load(stream)
            assert not vec.norm_reward and not saved_normalizer.norm_reward
            vec.obs_rms=copy.deepcopy(saved_normalizer.obs_rms);vec.ret_rms=copy.deepcopy(saved_normalizer.ret_rms)
            assert _model_state(model,vec)==meta['state']
            identity=json.loads((checkpoint.parent.parent/'initial_identity.json').read_text())
            assert _state_hash(env.plant.snapshot())==identity['physical_snapshot_hash']
            assert _state_hash(env.np_random.bit_generator.state)==identity['rng_hash']
        source=provenance(env,algorithm='PPO_CPU',extra_sources={'training_config':ROOT/f'configs/flight_probe/ppo_{arm}.json'})
        _write_json(dest/'config.json',cfg);_write_json(dest/'provenance.json',source)
        if common is None:
            common=dict(parameters=copy.deepcopy(model.get_parameters()),obs_rms=copy.deepcopy(vec.obs_rms),ret_rms=copy.deepcopy(vec.ret_rms),state=_model_state(model,vec),
                        raw_observation=env._observation().copy(),physical_snapshot=env.plant.snapshot().copy(),rng=copy.deepcopy(env.np_random.bit_generator.state),
                        torch_rng=torch.get_rng_state().clone(),numpy_rng=np.random.get_state())
        else:
            model.set_parameters(common['parameters'],exact_match=True);vec.obs_rms=copy.deepcopy(common['obs_rms']);vec.ret_rms=copy.deepcopy(common['ret_rms'])
            assert _model_state(model,vec)==common['state']
            np.testing.assert_array_equal(env._observation(),common['raw_observation'])
            np.testing.assert_array_equal(env.plant.snapshot(),common['physical_snapshot'])
            assert _state_hash(env.np_random.bit_generator.state)==_state_hash(common['rng'])
        _write_json(dest/'initial_identity.json',dict(state=_model_state(model,vec),matched_common=True,physical_snapshot_hash=_state_hash(env.plant.snapshot()),rng_hash=_state_hash(env.np_random.bit_generator.state)))
        _save_checkpoint(dest/'checkpoints/initial',model,vec,cfg,source,0)
        initialization_s=time.monotonic()-start_init
        if arm=='a':
            with eval_budget.activity():initial=evaluate_probe(dest/'checkpoints/initial',output/'initial_evaluation',budget=eval_budget)
            if not initial['complete']:raise RuntimeError('Initial fixed-set evaluation incomplete; no training')
        torch.set_rng_state(common['torch_rng']);np.random.set_state(common['numpy_rng'])
        # Each arm starts from the exact same RNG, network, optimizer, RMS and seeded environment.
        budget=SmokeBudget(controls=65536,substeps=327680,wall_s=min(900.,1800.-total_learn))
        recorder=Recorder(vec.venv.envs[0],dest/'training_episodes',budget);vec.venv.envs[0]=recorder
        completed=0;stop='TRANSITION_BUDGET';error=None;recent=[]
        try:
            for update in range(1,129):
                # Leave bounded time for evidence sealing; avoid starting a likely incomplete rollout.
                reserve=max(2.,max(recent[-8:],default=0.)*1.2+1.)
                if budget.deadline-time.monotonic()<reserve:
                    stop='WALL_BUDGET_RESERVE';break
                tick=time.monotonic()
                evidence=_learn_rollout(model,vec,budget.deadline,512,callback_factory=lambda deadline:ProbeCallback(deadline,recorder,dest/'rollouts',update))
                if failure_tail and update == 1:
                    original_rollout=ROOT/cfg['initial_checkpoint']
                    original_rollout=original_rollout.parent.parent/'rollouts/update_0001.npz'
                    with np.load(original_rollout) as old, np.load(dest/'rollouts/update_0001.npz') as new:
                        keys=('gaussian_sample','actor_mean','actor_std','normalized_observation','value','log_prob','episode_start')
                        for key in keys: np.testing.assert_array_equal(old[key],new[key])
                    _write_json(dest/'first_rollout_identity.json',dict(passed=True,controls=512,exact_fields=keys,
                        reward_and_advantage='Intentionally different at safety failure; all actions before first update exactly match B'))
                completed=update;evidence.update(update=update,training_cost=budget.summary())
                with (dest/'updates.jsonl').open('a') as f:f.write(json.dumps(evidence,allow_nan=False)+'\n')
                _save_checkpoint(dest/f'checkpoints/update_{update:04d}',model,vec,cfg,source,update)
                for label in (['early'] if update==1 else [])+(['midpoint'] if update==64 else []):
                    shutil.copytree(dest/f'checkpoints/update_{update:04d}',dest/f'checkpoints/{label}')
                recent.append(time.monotonic()-tick)
                if update%8==0:print(json.dumps(dict(arm=arm,update=update,controls=budget.controls,episodes=recorder.completed,wall_s=budget.summary()['elapsed_wall_s'])),flush=True)
        except _WallLimit as exc:
            stop='WALL_BUDGET';error=str(exc)
        except Exception:
            stop='ENGINEERING_ERROR';error=traceback.format_exc()
        finally:
            recorder.flush()
            source_checkpoint=dest/f'checkpoints/update_{completed:04d}' if completed else dest/'checkpoints/initial'
            shutil.copytree(source_checkpoint,dest/'checkpoints/final')
            budget.pause();cost=budget.summary();total_learn+=cost['elapsed_wall_s'];total_controls+=cost['reference_controls']
            summary=dict(arm=arm,lambda_control=int(arm!='a'),stop_reason=stop,error=error,training_cost=cost,
                complete_updates=completed,complete_episodes=recorder.completed,episode_outcomes=recorder.counts,
                initialization_wall_s=initialization_s,compile_wall_s=0,
                incomplete_update_controls=budget.controls-completed*512,final_is_latest_complete_update=True)
            _write_json(dest/'summary.json',summary);vec.close()
        if stop=='ENGINEERING_ERROR':raise RuntimeError(error)
        with eval_budget.activity():summary['evaluation']=evaluate_probe(dest/'checkpoints/final',dest/'evaluation',budget=eval_budget)
        _write_json(dest/'summary.json',summary);results[arm]=summary
        print(json.dumps(dict(arm=arm,finished=True,successes=summary['evaluation']['success_count'],controls=budget.controls,episodes=recorder.completed)),flush=True)
        if total_learn>=1800 or total_controls>=131072:break
    result=dict(task_id=TASK_ID,arms=results,learning_total_wall_s=total_learn,training_total_controls=total_controls,
                evaluation_cost=eval_budget.summary(),initial_evaluation=initial,paired_initial_state_exact=True)
    _write_json(output/'summary.json',result);return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--failure-tail',action='store_true',help='Run only C from original B initial checkpoint');p.add_argument('--engineering-only',action='store_true');p.add_argument('--engineering-evidence',type=Path)
    args=p.parse_args()
    if args.engineering_only:result=run_engineering(args.output)
    else:
        if args.engineering_evidence is None or not json.loads((args.engineering_evidence/'summary.json').read_text())['passed']:
            raise ValueError('Passing independent control engineering evidence is required before training')
        result=run_pair(args.output,failure_tail=args.failure_tail)
    print(json.dumps(result if args.engineering_only else {k:v for k,v in result.items() if k not in ('arms','initial_evaluation')},indent=2))
    return 0 if result.get('passed',True) else 2

if __name__=='__main__':raise SystemExit(main())
