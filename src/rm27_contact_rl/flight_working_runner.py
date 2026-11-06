"""Bounded CPU PPO run, shared learning/evaluation ledger and recorded MuJoCo episodes."""
from __future__ import annotations
import os
for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name]='1'
import argparse, copy, hashlib, json, multiprocessing as mp, signal, time, traceback
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import SubprocVecEnv
from .flight_working import WorkingFlightEnv, FeasibleCTBR, CONFIG
from .flight_probe import fixed_cases, FlightPDBaseline
from .flight_probe_runner import capture
from .paths import ROOT

def write(path, data):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(data,indent=2,allow_nan=False,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else str(v)))
    tmp.replace(path)

def make_env(config):
    return FeasibleCTBR(WorkingFlightEnv(config))

class BudgetCallback(BaseCallback):
    def __init__(self, deadline, remaining, output):
        super().__init__(); self.deadline=deadline; self.remaining=remaining; self.output=output
        self.steps=0; self.episodes=0; self.substeps=0; self.clips=0; self.mix=0; self.reasons={}
        self.sample_stats=[]; self.stop_reason=None
    def _on_step(self):
        self.steps+=self.training_env.num_envs
        infos=self.locals['infos']
        self.substeps+=sum(i['physics_substeps'] for i in infos)
        self.mix+=sum(i['proxy_saturated'] for i in infos)
        self.clips+=int(np.any(np.abs(self.locals['actions'])>1,axis=1).sum())
        if self.n_calls%64==0:
            with torch.no_grad():
                dist=self.model.policy.get_distribution(self.locals['obs_tensor']).distribution
                self.sample_stats.append(dict(transitions=self.model.num_timesteps,
                  sample=self.locals['actions'].tolist(), mean=dist.mean.cpu().numpy().tolist(),
                  std=dist.stddev.cpu().numpy().tolist()))
        for i in infos:
            if 'result' in i:
                self.episodes+=1
                reason=','.join(i['result']['failure_modes']) or 'STABLE_FLIGHT'
                self.reasons[reason]=self.reasons.get(reason,0)+1
                with (self.output/'training_episodes.jsonl').open('a') as f:
                    f.write(json.dumps(dict(transitions=self.model.num_timesteps,**i['result']))+'\n')
        if self.steps>self.remaining: self.stop_reason='TRANSITIONS'
        if time.monotonic()>=self.deadline-1.: self.stop_reason='LEARNING_WALL'
        return self.stop_reason is None
    def _on_rollout_end(self):
        # Compact genuine PPO rollout evidence, without per-step disk writes.
        b=self.model.rollout_buffer
        stats=dict(transitions=self.model.num_timesteps,reward_mean=float(b.rewards.mean()),
          advantage_mean=float(b.advantages.mean()),advantage_std=float(b.advantages.std()),
          value_mean=float(b.values.mean()),return_mean=float(b.returns.mean()),
          action_mean=b.actions.mean(axis=(0,1)).tolist(),action_std=b.actions.std(axis=(0,1)).tolist())
        with (self.output/'rollouts.jsonl').open('a') as f: f.write(json.dumps(stats)+'\n')
        if self.sample_stats:
            with (self.output/'policy_samples.jsonl').open('a') as f:
                for row in self.sample_stats: f.write(json.dumps(row)+'\n')
            self.sample_stats=[]

_EVAL_ENV=None
_EVAL_MODEL=None
_EVAL_PD=False

def eval_init(config, checkpoint, pd):
    global _EVAL_ENV,_EVAL_MODEL,_EVAL_PD
    torch.set_num_threads(1)
    _EVAL_ENV=FeasibleCTBR(WorkingFlightEnv(config,training=False))
    _EVAL_MODEL=None if pd else PPO.load(checkpoint,device='cpu')
    _EVAL_PD=pd

def evaluate_case(payload):
    case,directory,deadline=payload
    env=_EVAL_ENV; raw=env.unwrapped
    obs,info=env.reset(options={k:case[k] for k in ('reference_index','initial_perturbation')})
    initial=raw.plant.snapshot(); rows=[capture(raw,info)]; done=False
    while not done:
        if time.monotonic()>=deadline: raise TimeoutError('Shared evaluation budget exhausted')
        if _EVAL_PD:
            physical=FlightPDBaseline().action(raw)
            action=np.zeros(4); mean=action.copy(); std=np.zeros(4)
        else:
            with torch.no_grad():
                dist=_EVAL_MODEL.policy.get_distribution(torch.as_tensor(obs[None],device='cpu')).distribution
                mean=dist.mean[0].numpy(); std=dist.stddev[0].numpy()
            action=np.clip(mean,-1,1)
        before=obs.copy()
        obs,r,terminated,truncated,info=raw.step(physical) if _EVAL_PD else env.step(action)
        done=terminated or truncated
        row=capture(raw,info,r)
        row.update(actor_mean=mean.copy(),actor_std=std.copy(),gaussian_sample=np.zeros(4),
          executed_action_normalized=action.copy(),has_policy_action=not _EVAL_PD,
          normalized_observation_before=before,action_requested_ctbr=raw.previous_action_ctbr.copy(),
          deterministic_mean_clipped=bool(np.any(np.abs(mean)>1)))
        rows.append(row)
    summary=raw.evaluator.summary()
    summary.update(case=case['name'],controls=len(rows)-1,
      actor_clip_fraction=float(np.mean([x.get('deterministic_mean_clipped',False) for x in rows[1:]])),
      mixer_clip_fraction=float(np.mean([x['proxy_saturated'] for x in rows[1:]])),
      zero_thrust_fraction=float(np.mean([x['action_ctbr'][0]==0 for x in rows[1:]])),
      total_reward=float(sum(x['reward'] for x in rows)),
      controller='PD' if _EVAL_PD else 'PPO deterministic mean')
    if directory:
        path=Path(directory)/case['name'];path.mkdir(parents=True,exist_ok=False)
        keys=rows[-1].keys() & rows[0].keys()
        # Capture has an old 26-vector placeholder only on reset; use actual new schema.
        rows[0]['normalized_observation_before']=rows[0]['observation'].copy()
        np.savez_compressed(path/'trajectory.npz',initial_integration_state=initial,
            **{k:np.asarray([x[k] for x in rows]) for k in keys})
        write(path/'summary.json',summary)
        # Replay loads the unchanged base plant. Learning task is explicit and separate.
        base=json.loads((ROOT/raw.rl_config['base_task_config']).read_text())
        write(path/'config.json',dict(rl=base,policy_task='state_flight_working_v1',
          working_config=raw.working_config,private_model_path=str(raw.plant.sources['private_scene_binary'])))
    return summary

def evaluate(config,checkpoint,cases,output,seconds,pd=False):
    start=time.monotonic(); deadline=start+seconds
    # Four independent original MuJoCo plants; evaluations include worker startup.
    with ProcessPoolExecutor(max_workers=4,mp_context=mp.get_context('fork'),
            initializer=eval_init,initargs=(str(config),str(checkpoint),pd)) as pool:
        results=list(pool.map(evaluate_case,[(c,str(output) if output else None,deadline) for c in cases]))
    elapsed=time.monotonic()-start
    write(Path(output)/'results.json',dict(cases=results,wall_s=elapsed,
       passed=sum(r['passed'] for r in results),total=len(results)))
    return results,elapsed

def run(args):
    torch.set_num_threads(1)
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    config=Path(args.config).resolve(); cfg=json.loads(config.read_text())
    write(output/'config.json',cfg)
    limits=cfg['budget']
    ledger=dict(learning_wall_s=0.,evaluation_wall_s=0.,transitions=0,physical_substeps=0,
        complete_training_episodes=0,initialization_wall_s=0.,compilation_wall_s=0.,state='INITIALIZING')
    start=time.monotonic()
    env=WorkingFlightEnv(config); cases=fixed_cases(env); env.close()
    ledger['initialization_wall_s']+=time.monotonic()-start
    # PD checks same reset/scorer/physics, including action representability.
    pd_cases=[cases[0]]+cases[19:]
    pd,used=([],0.) if args.skip_pd else evaluate(config,None,pd_cases,output/'pd',limits['evaluation_wall_s'],pd=True)
    ledger['evaluation_wall_s']+=used
    if not args.skip_pd and not all(x['passed'] for x in pd):
        ledger['state']='BLOCKED_PD';write(output/'ledger.json',ledger);return
    start=time.monotonic()
    v=SubprocVecEnv([lambda:make_env(config)]*args.envs,start_method='fork');v.seed(0)
    model=PPO('MlpPolicy',v,seed=0,device='cpu',verbose=0,**cfg['ppo'],
       policy_kwargs=dict(net_arch=dict(pi=cfg['policy']['net_arch'],vf=cfg['policy']['net_arch']),
          activation_fn=getattr(torch.nn,cfg['policy']['activation']),log_std_init=cfg['policy']['log_std_init']))
    if cfg['policy']['zero_initial_action_head']:
        torch.nn.init.zeros_(model.policy.action_net.weight);torch.nn.init.zeros_(model.policy.action_net.bias)
    if cfg['policy'].get('actor_zero_equilibrium',False):
        for layer in [*model.policy.mlp_extractor.policy_net.modules(),model.policy.action_net]:
            if isinstance(layer,torch.nn.Linear) and layer.bias is not None:
                torch.nn.init.zeros_(layer.bias);layer.bias.requires_grad_(False)
    ledger['initialization_wall_s']+=time.monotonic()-start
    model.set_logger(configure(str(output/'logs'),['csv']))
    (output/'checkpoints').mkdir();model.save(output/'checkpoints/initial')
    initial,used=evaluate(config,output/'checkpoints/initial.zip',[cases[i] for i in (0,7,10,13)],output/'evaluations/initial',limits['evaluation_wall_s']-ledger['evaluation_wall_s'])
    ledger['evaluation_wall_s']+=used
    ledger['state']='TRAINING';write(output/'ledger.json',ledger)
    stage='hover'; best=-1.; no_improvement=0; iteration=0; last_results=[]
    try:
      while ledger['transitions']<limits['transitions'] and ledger['learning_wall_s']<limits['learning_wall_s']:
        iteration+=1
        remaining=limits['transitions']-ledger['transitions']
        # Exact multiples prevent SB3 silently exceeding total transition budget.
        rollout=args.envs*cfg['ppo']['n_steps']
        block=min(args.block,(remaining//rollout)*rollout)
        if block<=0:break
        started=time.monotonic()
        callback=BudgetCallback(started+limits['learning_wall_s']-ledger['learning_wall_s'],block,output)
        model.learn(total_timesteps=block,callback=callback,reset_num_timesteps=False,progress_bar=False)
        elapsed=time.monotonic()-started
        ledger['learning_wall_s']+=elapsed;ledger['transitions']+=callback.steps
        ledger['physical_substeps']+=callback.substeps;ledger['complete_training_episodes']+=callback.episodes
        tag=f'{stage}_{ledger["transitions"]:07d}'
        model.save(output/'checkpoints'/tag);model.save(output/'checkpoints/latest')
        ledger['optimizer_epochs']=model._n_updates
        record=dict(iteration=iteration,stage=stage,**ledger,block_wall_s=elapsed,
          controls_per_s=callback.steps/elapsed,complete_episodes=callback.episodes,
          failures=callback.reasons,actor_clip_fraction=callback.clips/max(callback.steps,1),
          mixer_clip_fraction=callback.mix/max(callback.steps,1),checkpoint=tag)
        with (output/'progress.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
        write(output/'ledger.json',ledger);print(json.dumps(record),flush=True)
        if callback.stop_reason=='LEARNING_WALL':break
        # Retain >=90s for formal23; periodic nominal plus perturbed velocity/attitude.
        budget=limits['evaluation_wall_s']-ledger['evaluation_wall_s']
        if budget<110:break
        small=[cases[i] for i in (0,7,10,13)] if stage=='hover' else [cases[i] for i in (0,19,20,21,22)]
        results,used=evaluate(config,output/'checkpoints/latest.zip',small,output/'evaluations'/tag,budget-80)
        ledger['evaluation_wall_s']+=used;last_results=results
        score=sum(r['passed'] for r in results)*10-sum(r['position_error_integral_m_s'] for r in results)/len(results)
        improved=score>best+.02
        if improved:best=score;no_improvement=0;model.save(output/'checkpoints/best')
        else:no_improvement+=1
        write(output/'ledger.json',ledger)
        print(json.dumps(dict(evaluation=tag,passed=sum(r['passed'] for r in results),score=score,
          position_error=[r['metrics']['position_error_m'] for r in results],evaluation_wall_s=ledger['evaluation_wall_s'])),flush=True)
        # Reset between evaluation blocks; count interrupted episodes explicitly.
        ledger['curriculum_or_evaluation_resets']=ledger.get('curriculum_or_evaluation_resets',0)+args.envs
        if stage=='hover' and all(r['passed'] for r in results) and limits['evaluation_wall_s']-ledger['evaluation_wall_s']>150:
            full,used=evaluate(config,output/'checkpoints/latest.zip',cases[:19],output/'evaluations'/f'{tag}_hover_gate',limits['evaluation_wall_s']-ledger['evaluation_wall_s']-65)
            ledger['evaluation_wall_s']+=used
            if sum(r['passed'] for r in full)>=18:
                model.save(output/'checkpoints/hover_pass');stage='tracking';best=-1.;no_improvement=0
                v.env_method('set_curriculum','tracking');print('HOVER_GATE_PASSED tracking enabled',flush=True)
        if stage=='tracking' and all(r['passed'] for r in results) and len(results)==5:
            break
        model._last_obs=None
        # No blind budget escalation: preserve one candidate for a supported redesign.
        if ledger['transitions']>=args.candidate_cap or (no_improvement>=3 and ledger['transitions']>=131072):
            ledger['candidate_stop']='CAP_OR_STALLED';break
      model.save(output/'checkpoints/final')
      ledger['state']='EVALUATING';write(output/'ledger.json',ledger)
      results,used=evaluate(config,output/'checkpoints/final.zip',cases,output/'formal_final',limits['evaluation_wall_s']-ledger['evaluation_wall_s'])
      ledger['evaluation_wall_s']+=used
      ledger.update(state='COMPLETE',hover_passed=sum(r['passed'] for r in results[:19]),
        shift_passed=sum(r['passed'] for r in results[19:]),curriculum=stage)
    except BaseException as exc:
      ledger.update(state='INTERRUPTED',exception=repr(exc),traceback=traceback.format_exc())
      model.save(output/'checkpoints/interrupted')
      raise
    finally:
      write(output/'ledger.json',ledger);v.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--config',default=str(CONFIG))
    p.add_argument('--envs',type=int,default=8);p.add_argument('--block',type=int,default=32768)
    p.add_argument('--candidate-cap',type=int,default=327680);p.add_argument('--skip-pd',action='store_true')
    run(p.parse_args())
