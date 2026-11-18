"""R9 native evaluation and deterministic replay allocation, never reruns R8."""
import copy,json
from pathlib import Path
import numpy as np
from .asset import ROOT,digest,read_json,write_result
from .r9_experiment import configuration,verify,native_budget,native_count
from .r8_report import summary,ENDPOINTS
from . import terminal_native_eval as native
from .rotation_bridge_diagnostics import native_transition_diagnostics,_trial_counts


def inputs(cfg):
    task,profile,spec,_,manifest,goal=native.load_evaluation_inputs(ROOT/cfg['task_config'])
    return task,profile,spec,manifest,goal,read_json(native.HANDOVER_CONFIG)['flight_envelope']


def demonstration_starts(cfg,out,n):
    dest=out/f'demonstrated-{n:06d}.json'
    if dest.exists():return read_json(dest)
    task,profile,spec,_,goal,limits=inputs(cfg)
    plant=native.NativeContact(profile,spec);cp=out/'B'/f'checkpoint-{n:06d}.pt'
    command,_=native.load_controller(cp.with_suffix('.npz'),digest(cp),profile,goal)
    starts={}
    for row in read_json(out/'demonstrations.json')['records']:
        if not row['demonstration']:continue
        path=ROOT/row['path'];seed=read_json(path/'outcome.json')['seed']
        with np.load(path/'physics.npz') as d:
            current=(d['states'][0],d['motor_thrust_N'][0],d['motor_target_N'][0])
        if seed in starts:
            for a,b in zip(starts[seed],current):np.testing.assert_array_equal(a,b)
        else:starts[seed]=current
    rows=[]
    with native_budget(out,f'demonstrated-{n}',3):
        for seed,sample in sorted(starts.items()):
            result,_=native.rollout(plant,*sample,command,2.3,limits)
            rows.append(dict(seed=seed,**result))
    write_result(dest,dict(checkpoint_sha256=digest(cp),rows=rows,qualified=sum(r['qualified'] for r in rows)))
    return read_json(dest)


def identity(row):
    return str(row.get('case',row.get('seed',''))),int(row.get('source_frame',0)),str(row.get('variant','')),str(row.get('population',''))


def representatives(rows,limit,old_rows=(),*,earlier_only=False):
    ordered=sorted(rows,key=identity);selected=[];seen=set()
    old={identity(r):r['qualified'] for r in old_rows}
    # Newly qualified representatives, distinct anchors first.
    for row in ordered:
        if earlier_only and row.get('population')!='earlier_fresh':continue
        anchor=identity(row)[:2]
        if row['qualified'] and not old.get(identity(row),False) and anchor not in seen:
            selected.append(row);seen.add(anchor)
            if len(selected)==limit:return selected
    reasons={r['reason'] for r in selected}
    for row in ordered:
        if row not in selected and row['reason'] not in reasons:
            selected.append(row);reasons.add(row['reason'])
            if len(selected)==limit:return selected
    for row in ordered:
        if row not in selected:selected.append(row)
        if len(selected)==limit:break
    return selected


def reproduce(cfg,out,n,rows,label,cap):
    dest=out/(label+'-reproductions.json')
    if dest.exists():return read_json(dest)
    if len(rows)>cap:raise ValueError('Replay allocation changed')
    _,profile,spec,_,goal,limits=inputs(cfg);plant=native.NativeContact(profile,spec)
    cp=out/'B'/f'checkpoint-{n:06d}.pt';command,_=native.load_controller(cp.with_suffix('.npz'),digest(cp),profile,goal)
    results=[]
    with native_budget(out,label,cap):
        for row in rows:
            folder=ROOT/row['r9_record'];start=read_json(folder/'started.json')
            result,_=native.rollout(plant,np.array(start['initial_state']),np.array(start['motor_thrust_N']),
                np.array(start['motor_target_N']),command,start['horizon_s'],limits)
            for key in ('qualified','reason','end_s','contact'):
                if result[key]!=row[key]:raise ValueError(f'Native replay mismatch {key} {folder}')
            with np.load(folder/'physics.npz') as old,np.load(ROOT/result['r9_record']/'physics.npz') as new:
                for key in old.files:np.testing.assert_array_equal(old[key],new[key])
            results.append(dict(identity=identity(row),original=row['r9_record'],replay=result['r9_record'],qualified=result['qualified']))
    write_result(dest,dict(rows=results,exact_physics_reproduction=True))
    return read_json(dest)


def seal_selection(cfg,out):
    dest=out/'final-selection.json'
    if dest.exists():return read_json(dest)
    ledger=read_json(out/'ledger.json');row=ledger['arms']['B']
    if row['pending'] or ledger['arms']['A']['transitions'] or row['transitions']>393216:raise ValueError('Invalid training ledger')
    finished=read_json(out/'B/result.json')
    if finished['stop_reason'] not in ('TRANSITION_LIMIT','PERSISTENT_RETENTION_COLLAPSE'):raise ValueError('Incomplete training')
    points=[n for n in ENDPOINTS if (out/'B'/f'eval-{n:06d}.json').exists()]
    rows={n:summary(read_json(out/'B'/f'eval-{n:06d}.json')) for n in points}
    eligible=[n for n,r in rows.items() if r['eligible']]
    selected=max(eligible,key=lambda n:(rows[n]['distinct'],rows[n]['robust'],rows[n]['earlier'],rows[n]['braking']['qualified'],n)) if eligible else points[-1]
    selections={k:dict(transitions=n,checkpoint_sha256=digest(out/'B'/f'checkpoint-{n:06d}.pt')) for k,n in [('selected',selected),('final',points[-1])]}
    write_result(dest,dict(selections=selections,development=rows,eligible=eligible,rule=cfg['r8']['selection'],
        ledger_sha256=digest(out/'ledger.json'),heldout_sha256=digest(out/'heldout.npz')))
    return read_json(dest)


def fresh(cfg,out,selections):
    from .terminal_workflow_final import score_suite
    dest=out/'heldout-results.json'
    if dest.exists():return read_json(dest)
    task,profile,spec,manifest,goal,limits=inputs(cfg);plant=native.NativeContact(profile,spec)
    bank=dict(np.load(out/'heldout.npz'));populations=[(str(k),bank,np.flatnonzero(bank['region']==k)) for k in dict.fromkeys(bank['region'])]
    if sum(len(i) for _,_,i in populations)!=94:raise ValueError('Fresh suite size changed')
    horizons=dict(terminal=task['braking_horizon_s'],braking=task['braking_horizon_s'],rotation=task['handover_horizon_s'],handover=task['handover_horizon_s'])
    scored={};aliases={};protocol=cfg['diagnostics']
    # Final first so a duplicated selected/final actor consumes only final's12 replay allowance.
    for name in ('final','selected'):
        pin=selections[name];n=pin['transitions']
        same=next((k for k in scored if selections[k]['checkpoint_sha256']==pin['checkpoint_sha256']),None)
        if same:aliases[name]=same;continue
        single=out/f'fresh-{name}.json'
        if single.exists():scored[name]=read_json(single);continue
        cp=out/'B'/f'checkpoint-{n:06d}.pt'
        controllers={name:native.load_controller(cp.with_suffix('.npz'),pin['checkpoint_sha256'],profile,goal)}
        with native_budget(out,'fresh-'+name,94),native_transition_diagnostics(ROOT/protocol['transition_protocol'],protocol['transition_protocol_sha256']):
            result=score_suite(plant,controllers,populations,manifest,horizons,limits,cfg['workers'],lambda rows:{})[name]
        result['cohorts']={k:_trial_counts([r for r in result['episodes'] if r['population']==k]) for k,_,_ in populations}
        write_result(single,result);scored[name]=result
    write_result(dest,dict(results=scored,aliases=aliases,selection_sha256=digest(out/'final-selection.json')))
    return read_json(dest)


def finish():
    cfg,old,out=configuration();verify(cfg,out)
    selection=seal_selection(cfg,out)
    for n in map(int,selection['development']):
        report=read_json(out/'B'/f'eval-{n:06d}.json');control=read_json(old/'B'/f'eval-{n:06d}.json')
        demo=demonstration_starts(cfg,out,n)
        earlier=representatives(report['bridge']['rows'],6,control['bridge']['rows'])
        braking=representatives([r for r in report['development']['episodes'] if r['phase']=='braking'],5,
                               [r for r in control['development']['episodes'] if r['phase']=='braking'])
        reproduce(cfg,out,n,[*demo['rows'],*earlier,*braking],f'replays-development-{n}',14)
    held=fresh(cfg,out,selection['selections'])
    for name,record in held['results'].items():
        cap=12 if name=='final' else 11
        reproduce(cfg,out,selection['selections'][name]['transitions'],representatives(record['episodes'],cap,earlier_only=True),f'replays-fresh-{name}',cap)
    write_result(out/'evaluation-complete.json',dict(native_charged=native_count(out),native_cap=831,
        training_transitions=read_json(out/'ledger.json')['arms']['B']['transitions'],r8_new_rollouts=0,
        selections=selection['selections']))
    print('R9_EVALUATION_COMPLETE',read_json(out/'evaluation-complete.json'),flush=True)
