"""R6 isolated reset intervention on the unchanged resumable PPO workflow."""
from __future__ import annotations
import argparse
from contextlib import contextmanager, ExitStack
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from .asset import ROOT,digest,read_json,write_result
from . import regional_update_experiment as previous
from . import rotation_bridge_experiment as instrumented
from . import terminal_workflow as workflow
from .r6_curriculum import FrontierSampler
CONFIG=ROOT/'configs/rm27_asm2_r6.json'

def configuration(path=CONFIG):
    exp=read_json(path)
    if exp['schema']!='asm2_r6_competence_reset_v1' or exp['per_arm_transitions']!=98304:raise ValueError('R6 initial cap changed')
    base,_,_,source=previous.configuration(ROOT/exp['source_config'])
    cfg=copy.deepcopy(base)
    cfg.update(config_path=str(Path(path).relative_to(ROOT)),output=exp['output'],r6=exp,
        evaluate_every=98304,evaluation_wait_s=1800,budget=dict(per_arm_transitions=98304,total_transitions=196608))
    return cfg,base,source,ROOT/exp['output']

def freeze(cfg,base,source,out):
    from .regional_update_runtime import verify_alias
    from .r3_experiment import fresh_cases
    if (out/'freeze.json').exists():return verify_freeze(cfg,out)
    old=verify_alias(base,source);workflow.legacy_inputs(cfg)
    dataset=read_json(out/'dataset.json')
    for p,h in dataset['pins'].items():
        if digest(ROOT/p)!=h:raise ValueError(p)
    for name in old['archives']:
        if (out/name).exists():raise FileExistsError(name)
        shutil.copyfile(source/name,out/name)
    heldout=fresh_cases(dict(cfg,r3=dict(heldout_seed=cfg['r6']['heldout_seed'])),source,out)
    paths=['src/rm27_asm2/r6_curriculum.py','src/rm27_asm2/r6_experiment.py','src/rm27_asm2/r6_dataset.py','tests/test_asm2_r6.py']
    pins={**old['historical_source_pins'],**old['source_pins'],**cfg['pins'],**dataset['pins']}
    pins.update({p:digest(ROOT/p) for p in paths})
    pins={p:digest(ROOT/p) for p in pins}
    archives={n:digest(out/n) for n in [*old['archives'],'heldout.npz','dataset.json','intervention-training.npz','curriculum.json']}
    write_result(out/'freeze.json',dict(config_sha256=digest(ROOT/cfg['config_path']),archives=archives,
        source_pins=pins,historical_source_pins={},runtime=old['runtime'],heldout=heldout,budget=cfg['budget'],
        parent=cfg['parent'],parent_sha256=digest(ROOT/cfg['parent']),
        method=cfg['r6']['method'],continuation_gate=cfg['r6']['extension_gate'],
        selection='Last common endpoint only; seal actor hashes before fresh evaluation.',
        constraints='Same rewards, horizons, actor/critic, CTBR, optimizers, seed and native benchmark. Full physical state in resets only; no teacher action or phase input.',
        information='25 observed autonomous successful paths from five sources supply recurrent positive outcomes; failure labels retain nonmonotonicity. New predecessor proposals require online success, not descriptor similarity. No formal reachability claim.',
        frozen_unix_s=time.time()))

def verify_freeze(cfg,out):return previous.verify_freeze(cfg,out)

@contextmanager
def intervention(out,arm):
    from . import terminal_training
    original=terminal_training.CurriculumEnvironment
    class RecordedEnvironment(original):
        def __init__(self,*args,**kwargs):
            self.r6_records=[];self.r6_ticks=0;self.r6_terminal=None
            super().__init__(*args,**kwargs)
        def reset(self,ids,rows=None):
            if self.r6_terminal is not None:
                state,_=self.sim.observe()
                self.r6_terminal[0][ids]=state[ids]
                self.r6_terminal[1][ids]=self.sim.thrust[ids]
                self.r6_terminal[2][ids]=self.sim.targets[ids]
            super().reset(ids,rows)
        def step(self,action):
            old_rows=self.active_row.clone()
            before=[v.detach().cpu().numpy().copy() for v in (self.state,self.sim.thrust,self.sim.targets,old_rows,action)]
            self.r6_terminal=[v.clone() for v in (self.state,self.sim.thrust,self.sim.targets)]
            result=super().step(action)
            obs,reward,terminated,truncated,extras=result
            done=terminated|truncated
            after=[v.clone() for v in (self.state,self.sim.thrust,self.sim.targets)]
            for a,t in zip(after,self.r6_terminal):a[done]=t[done]
            self.r6_terminal=None
            rows=old_rows[done].cpu().numpy()
            worlds=self.torch.nonzero(done).flatten().cpu().numpy()
            success=[self._episode_events.get(int(w))=='DESIGNATED_LOADED' for w in worlds]
            if arm=='B':self.sampler.record(rows,success)
            self.r6_records.append(before+[v.detach().cpu().numpy().copy() for v in (*after,reward,done)])
            self.r6_ticks+=1
            if len(self.r6_records)==64:
                folder=out/arm/'physical-rollouts';folder.mkdir(exist_ok=True)
                # Resume count comes from existing completed physical files (no overwrite).
                name=len(list(folder.glob('*.npz')))+1
                keys=('states','motor_thrust_N','motor_target_N','reset_row','action_raw','next_states','next_motor_thrust_N','next_motor_target_N','reward','done')
                with (folder/f'rollout-{name:04d}.npz').open('xb') as f:
                    np.savez_compressed(f,**{k:np.stack([r[j] for r in self.r6_records]) for j,k in enumerate(keys)})
                self.r6_records=[]
            return result
    with ExitStack() as stack:
        stack.enter_context(patch.object(terminal_training,'CurriculumEnvironment',RecordedEnvironment))
        if arm=='B':
            metadata=read_json(out/'curriculum.json')
            stack.enter_context(patch.object(workflow,'load_bank',lambda _:dict(np.load(out/'intervention-training.npz',allow_pickle=False))))
            stack.enter_context(patch.object(workflow,'ResetSampler',lambda *a,**kw:FrontierSampler(*a,**kw,metadata=metadata)))
        yield

def docker_command(cfg,out,command):
    args=previous.docker_command(cfg,out,command)
    args[args.index('rm27-asm2-regional-update')]='rm27-asm2-r6'
    args[args.index('rm27_asm2.regional_update_experiment')]='rm27_asm2.r6_experiment'
    return args

def run(cfg,out):
    def launch(command,**kwargs):return subprocess.Popen(docker_command(cfg,out,command),**kwargs)
    with patch.object(workflow,'subprocess',SimpleNamespace(Popen=launch,STDOUT=subprocess.STDOUT)):
        instrumented.run(cfg,out)

def train(cfg,out,arm,resume,stop_updates):
    from isaaclab.app import AppLauncher
    launcher=AppLauncher(headless=True);code=0
    try:
        import torch
        from .rotation_bridge_metrics import install_metrics
        cp=torch.load(out/arm/'resume.pt',map_location='cpu',weights_only=True) if resume else {}
        with instrumented.training_budget(),intervention(out,arm),install_metrics(out/arm,
             completed_transitions=cp.get('completed_transitions',0),updates=cp.get('workflow_updates',0)):
            workflow.train(cfg,out,arm,resume,stop_updates)
    except Exception:traceback.print_exc();code=2
    sys.stdout.flush();sys.stderr.flush();os._exit(code)

def main():
    p=argparse.ArgumentParser();p.add_argument('--workflow-config',type=Path,default=CONFIG)
    p.add_argument('command',choices=['freeze','run','train']);p.add_argument('--arm',choices=['A','B'])
    p.add_argument('--resume',action='store_true');p.add_argument('--stop-updates',type=int,default=0)
    a=p.parse_args();cfg,base,source,out=configuration(a.workflow_config.resolve())
    if a.command=='freeze':freeze(cfg,base,source,out);return
    verify_freeze(cfg,out)
    if a.command=='run':run(cfg,out)
    else:train(cfg,out,a.arm,a.resume,a.stop_updates)
if __name__=='__main__':main()
