"""R7 continuation-success auxiliary intervention on the unchanged resumable PPO workflow."""
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
CONFIG=ROOT/'configs/rm27_asm2_r7.json'

def configuration(path=CONFIG):
    exp=read_json(path)
    if exp['schema']!='asm2_r7_auxiliary_v1' or exp['per_arm_transitions']!=98304:raise ValueError('R6 initial cap changed')
    base,_,_,source=previous.configuration(ROOT/exp['source_config'])
    cfg=copy.deepcopy(base)
    cfg.update(config_path=str(Path(path).relative_to(ROOT)),output=exp['output'],r7=exp,
        evaluate_every=98304,evaluation_wait_s=1800,budget=dict(per_arm_transitions=98304,total_transitions=196608))
    return cfg,base,source,ROOT/exp['output']

def freeze(cfg,base,source,out):
    from .regional_update_runtime import verify_alias
    from .r3_experiment import fresh_cases
    if (out/'freeze.json').exists():return verify_freeze(cfg,out)
    old=verify_alias(base,source);workflow.legacy_inputs(cfg)
    dataset=read_json(out/'supervision.json')
    for p,h in dataset['pins'].items():
        if digest(ROOT/p)!=h:raise ValueError(p)
    for name in old['archives']:
        if (out/name).exists():raise FileExistsError(name)
        shutil.copyfile(source/name,out/name)
    heldout=fresh_cases(dict(cfg,r3=dict(heldout_seed=cfg['r7']['heldout_seed'])),source,out)
    paths=['src/rm27_asm2/continuation_auxiliary.py','src/rm27_asm2/r7_experiment.py','src/rm27_asm2/continuation_supervision.py','tests/test_asm2_continuation_auxiliary.py']
    pins={**old['historical_source_pins'],**old['source_pins'],**cfg['pins'],**dataset['pins']}
    pins.update({p:digest(ROOT/p) for p in paths})
    pins={p:digest(ROOT/p) for p in pins}
    archives={n:digest(out/n) for n in [*old['archives'],'heldout.npz','supervision.json','supervision.npz']}
    write_result(out/'freeze.json',dict(config_sha256=digest(ROOT/cfg['config_path']),archives=archives,
        source_pins=pins,historical_source_pins={},runtime=old['runtime'],heldout=heldout,budget=cfg['budget'],
        parent=cfg['parent'],parent_sha256=digest(ROOT/cfg['parent']),
        method=cfg['r7']['method'],continuation_gate=cfg['r7']['extension_gate'],
        selection='Last common endpoint only; seal actor hashes before fresh evaluation.',
        constraints='Same rewards, horizons, actor/critic, CTBR, optimizers, seed and native benchmark. Full physical state in resets only; no teacher action or phase input.',
        information='150 actual Fixed A initial continuations, 83 positive and 67 negative, five source groups. Binary loss coefficient .01 into Actor penultimate features. Original resets and PPO objective.',
        frozen_unix_s=time.time()))

def verify_freeze(cfg,out):return previous.verify_freeze(cfg,out)

@contextmanager
def intervention(out,arm):
    if arm == 'B':
        from .continuation_auxiliary import install_auxiliary
        with install_auxiliary(out/'supervision.npz', coefficient=.01):
            yield
    else:
        yield

def docker_command(cfg,out,command):
    args=previous.docker_command(cfg,out,command)
    args[args.index('rm27-asm2-regional-update')]='rm27-asm2-r7'
    args[args.index('rm27_asm2.regional_update_experiment')]='rm27_asm2.r7_experiment'
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
