"""R9 single-arm wiring; archived R8 control, shared native reservation ledger."""
from contextlib import contextmanager
from unittest.mock import patch
from pathlib import Path
import argparse,copy,fcntl,inspect,json,os,shutil,subprocess,sys,time,traceback
import numpy as np
from .asset import ROOT,digest,read_json,write_result
from . import r8_experiment as r8
from . import terminal_workflow as workflow
from . import rotation_bridge_experiment as instrumented
from .demonstration_regularizer import install_regularizer
CONFIG=ROOT/'configs/rm27_asm2_r9.json'
PREFLIGHT=ROOT/'.agents/runs/asm2-terminal/r9-preflight/freeze.json'


def configuration():
    exp=read_json(CONFIG);cfg,_,_,old=r8.configuration()
    cfg=copy.deepcopy(cfg)
    cfg.update(config_path=str(CONFIG.relative_to(ROOT)),output=exp['output'],r9=exp,
        budget=dict(per_arm_transitions=393216,total_transitions=393216))
    return cfg,old,ROOT/exp['output']


def check_preflight():
    f=read_json(PREFLIGHT)
    for p,h in {**f['r9_pins'],**f['protected_pins']}.items():
        if digest(ROOT/p)!=h:raise ValueError('Changed preflight pin: '+p)
    return f


def freeze():
    check_preflight();cfg,old,out=configuration()
    if (out/'freeze.json').exists():return verify(cfg,out)
    out.mkdir(parents=True,exist_ok=False)
    f=read_json(old/'freeze.json')
    for name,h in f['archives'].items():
        if digest(old/name)!=h:raise ValueError(name)
        shutil.copyfile(old/name,out/name)
    pins={**f['source_pins'],str(CONFIG.relative_to(ROOT)):digest(CONFIG),
          'src/rm27_asm2/r9_experiment.py':digest(Path(__file__)),
          'src/rm27_asm2/r9_evaluation.py':digest(ROOT/'src/rm27_asm2/r9_evaluation.py'),
          'src/rm27_asm2/demonstration_regularizer.py':digest(ROOT/'src/rm27_asm2/demonstration_regularizer.py')}
    write_result(out/'authorization.json',dict(user_instruction='APPROVE_R9_393216; wire, verify pins and execute frozen R9',
        max_transitions=393216,max_native=831,r8_rerun=False,automatic_extension=False,commits_pushes=False,preflight_sha256=digest(PREFLIGHT)))
    write_result(out/'freeze.json',dict(config_sha256=digest(CONFIG),archives=f['archives'],source_pins=pins,historical_source_pins={},
        runtime=f['runtime'],budget=cfg['budget'],parent=cfg['parent'],parent_sha256=f['parent_sha256'],
        preflight_sha256=digest(PREFLIGHT),authorization_sha256=digest(out/'authorization.json'),
        integration='Single B arm; same R8 initialization; reuse zero-dose evaluation. Only preregistered retention stop replaces old inherited stop.',
        frozen_unix_s=time.time()))
    return verify(cfg,out)


def verify(cfg,out):
    check_preflight()
    f=workflow.verify_freeze(cfg,out)
    amendment=read_json(out/'integration-amendment.json') if (out/'integration-amendment.json').exists() else None
    for path,expected in {**f['historical_source_pins'],**f['source_pins']}.items():
        actual=digest(ROOT/path)
        if actual==expected:continue
        allowed={'src/rm27_asm2/r9_experiment.py','src/rm27_asm2/r9_evaluation.py'}
        if (amendment is None or path not in allowed or amendment['original_freeze_sha256']!=digest(out/'freeze.json')
                or amendment['changes'].get(path)!={'before_sha256':expected,'after_sha256':actual}):
            raise ValueError('Changed frozen input: '+path)
    if digest(PREFLIGHT)!=f['preflight_sha256']:raise ValueError('Preflight receipt changed')
    if read_json(out/'authorization.json')['max_native']!=831:raise ValueError('Authorization changed')
    return f


def native_count(out):
    p=out/'native-ledger.json'
    return read_json(p)['charged'] if p.exists() else 0


@contextmanager
def native_budget(out,label,maximum):
    """Reserve before each real native rollout, including forked workers/replays."""
    from . import terminal_native_eval as native
    from .backward_frontier import Capture
    original=native.rollout
    (out/'native-records').mkdir(exist_ok=True)
    def observe(plant,state,motors,targets,command,horizon,limits):
        with (out/'native.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            p=out/'native-ledger.json'
            ledger=read_json(p) if p.exists() else dict(charged=0,cap=831,groups={})
            if ledger['charged']>=831 or ledger['groups'].get(label,0)>=maximum:raise RuntimeError('Native budget exhausted')
            ledger['charged']+=1;ledger['groups'][label]=ledger['groups'].get(label,0)+1
            index=ledger['charged'];workflow.write_json(p,ledger)
        folder=out/'native-records'/f'{index:04d}';folder.mkdir(exist_ok=False)
        write_result(folder/'started.json',dict(label=label,horizon_s=horizon,initial_state=np.asarray(state).tolist(),
            motor_thrust_N=np.asarray(motors).tolist(),motor_target_N=np.asarray(targets).tolist()))
        sample=dict(state=state,motor_thrust_N=motors,motor_target_N=targets)
        with Capture(plant,sample) as capture:
            result,trace=original(plant,state,motors,targets,command,horizon,limits)
        with (folder/'physics.npz').open('xb') as f:
            np.savez_compressed(f,states=np.array(capture.states),motor_thrust_N=np.array(capture.motors),
                motor_target_N=np.array(capture.targets),sample_times_s=np.array(capture.times),
                ctbr_N_rad_s=np.array(capture.commands),control_times_s=np.array(capture.command_times),
                allocated_motor_target_N=np.array(capture.allocated),dt=plant.dt)
        # Saving a replay from this rollout consumes no extra rollout.
        _,_,_,_,_,goal=native.load_evaluation_inputs(ROOT/configuration()[0]['task_config'])
        native.save_replay(folder/'replay.npz',label+'-'+str(index),trace,result,goal,
                           native.target_mesh(plant,plant.spec.legal_targets),plant.profile.policy_dt_s)
        write_result(folder/'contacts.json',capture.contacts)
        result['r9_record']=str(folder.relative_to(ROOT))
        write_result(folder/'outcome.json',result)
        return result,trace
    with patch.object(native,'rollout',observe):yield


def evaluate_request(cfg,out,request):
    destination=ROOT/request['output']
    if destination.exists():return read_json(destination)
    n=int(destination.stem.split('-')[-1]);cp=ROOT/request['checkpoint']
    if n==0:
        old=ROOT/'.agents/runs/asm2-terminal/r8'
        with np.load(cp.with_suffix('.npz')) as a,np.load(old/'bc.npz') as b:
            for k in b.files:
                if k!='checkpoint_sha256':np.testing.assert_array_equal(a[k],b[k])
        report=copy.deepcopy(read_json(old/'B/eval-000000.json'))
        report.update(checkpoint_sha256=digest(cp),weights_sha256=digest(cp.with_suffix('.npz')),
                      archived_zero_dose_reuse=str((old/'B/eval-000000.json').relative_to(ROOT)))
        report['development']['actor'].update(checkpoint_sha256=report['checkpoint_sha256'],weights_sha256=report['weights_sha256'])
        write_result(destination,report);return report
    if n not in (98304,196608,294912,393216):raise ValueError('Unregistered evaluation checkpoint')
    from .rotation_bridge_diagnostics import native_transition_diagnostics
    protocol=cfg['diagnostics']
    with native_budget(out,f'development-{n}',138),native_transition_diagnostics(ROOT/protocol['transition_protocol'],protocol['transition_protocol_sha256']):
        return workflow.native_evaluate(cfg,out,request)


def wired_train():
    """Keep frozen trainer file unchanged; replace only its legacy collapse predicate."""
    source=inspect.getsource(workflow.train)
    old="recent = history[-3:]\n                    if state['transitions'] >= 100000 and len(recent) == 3 and all(\n                        h['cohorts']['braking']['qualified'] < 15 and h['cohorts']['terminal']['qualified'] < 15 for h in recent):"
    new="recent = history[-2:]\n                    if len(recent) == 2 and all(\n                        h['cohorts']['braking']['qualified'] == 0 and h['cohorts']['terminal']['qualified'] == 0 for h in recent):"
    if source.count(old)!=1:raise ValueError('Frozen trainer integration anchor changed')
    namespace={};exec(compile(source.replace(old,new),'<R9 isolated collapse predicate>','exec'),workflow.__dict__,namespace)
    return namespace['train']


def train(cfg,out,resume,stop_updates):
    from isaaclab.app import AppLauncher
    launcher=AppLauncher(headless=True);code=0
    try:
        import torch
        from .rotation_bridge_metrics import install_metrics
        folder=out/'B'
        cp=torch.load(folder/'resume.pt',map_location='cpu',weights_only=True) if resume else {}
        def record(row):
            with (folder/'regularizer.jsonl').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
        with instrumented.training_budget(),r8.intervention(cfg,out,'B',resume),install_regularizer(
            out/'demonstrations.npz',completed_transitions=cp.get('completed_transitions',0),
            initial=.01,decay_transitions=393216,resume_state=cp.get('r9_regularizer'),diagnostics_callback=record),install_metrics(
            folder,completed_transitions=cp.get('completed_transitions',0),updates=cp.get('workflow_updates',0)):
            wired_train()(cfg,out,'B',resume,stop_updates)
    except Exception:traceback.print_exc();code=2
    sys.stdout.flush();sys.stderr.flush();os._exit(code)


def run(cfg,out):
    verify(cfg,out);folder=out/'B';folder.mkdir(exist_ok=True)
    for integration in (True,False):
        result=read_json(folder/'result.json') if (folder/'result.json').exists() else None
        if result and result['stop_reason']=='TRANSITION_LIMIT':break
        if result and result['stop_reason'] not in ('INTEGRATION_PAUSE',):raise RuntimeError('Stopped campaign cannot silently restart')
        if integration and (folder/'resume.pt').exists():continue
        cmd=[str(ROOT/'run_asm2_terminal_pilot.sh'),'workflow','--workflow-config',str(CONFIG.relative_to(ROOT)),'train','--arm','B']
        if (folder/'resume.pt').exists():cmd+=['--resume']
        if integration:cmd+=['--stop-updates','1']
        args=r8.previous.docker_command(cfg,out,cmd)
        args[args.index('rm27-asm2-regional-update')]='rm27-asm2-r9'
        args[args.index('rm27_asm2.regional_update_experiment')]='rm27_asm2.r9_experiment'
        with (folder/'launch.log').open('a') as stream:
            process=subprocess.Popen(args,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
            while process.poll() is None:
                req=folder/'evaluation-request.json'
                if req.exists():
                    r=read_json(req)
                    if not (ROOT/r['output']).exists():evaluate_request(cfg,out,r)
                time.sleep(1)
            if process.returncode:raise RuntimeError('R9 trainer exited; inspect launch.log. Do not discard charged transitions.')
        if integration and read_json(folder/'result.json')['stop_reason']!='INTEGRATION_PAUSE':raise RuntimeError('Invalid charged restart boundary')
    from .r9_evaluation import finish
    finish()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['freeze','run','train','finish'])
    parser.add_argument('--workflow-config');parser.add_argument('--arm',choices=['B']);parser.add_argument('--resume',action='store_true');parser.add_argument('--stop-updates',type=int,default=0)
    a=parser.parse_args()
    if a.command=='freeze':freeze();return
    cfg,_,out=configuration();verify(cfg,out)
    if a.command=='train':train(cfg,out,a.resume,a.stop_updates)
    elif a.command=='run':run(cfg,out)
    else:
        from .r9_evaluation import finish
        finish()
if __name__=='__main__':main()
