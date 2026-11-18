"""R8 development-only selection and sealed autonomous native evaluation."""
from collections import Counter
import json
import numpy as np
from .asset import ROOT,read_json,digest,write_result
from .r8_experiment import configuration,verify_freeze
from .r3_report import write_once
from . import terminal_native_eval as native
from .rotation_bridge_diagnostics import native_transition_diagnostics,_trial_counts

ENDPOINTS=(98304,196608,294912,393216)


def summary(report):
    bridge=report['bridge'];cohorts=report['development']['cohorts']
    anchors={}
    for row in bridge['rows']:
        anchors.setdefault(f"{row['case']}:{row['source_frame']}",[]).append(bool(row['qualified']))
    eligible=(cohorts['braking']['qualified']>=32 and cohorts['terminal']['qualified']>=27
        and not cohorts['braking']['safety_failures'] and not cohorts['terminal']['safety_failures'])
    return dict(earlier=bridge['qualified'],robust=bridge['robust'],anchors={k:sum(v) for k,v in anchors.items()},
        distinct=sum(any(v) for v in anchors.values()),earlier_exits=bridge['exits'],earlier_reasons=bridge['reasons'],
        rotation=cohorts['rotation'],braking=cohorts['braking'],terminal=cohorts['terminal'],eligible=eligible)


def select():
    cfg,_,_,out=configuration();frozen=verify_freeze(cfg,out)
    ledger=read_json(out/'ledger.json')
    if any(v['transitions']!=393216 or v['pending'] is not None for v in ledger['arms'].values()):
        raise ValueError('Both complete matched arms required before selection')
    selections={};development={}
    for arm in ('A','B'):
        rows={n:summary(read_json(out/arm/f'eval-{n:06d}.json')) for n in ENDPOINTS}
        eligible=[n for n,r in rows.items() if r['eligible']]
        n=max(eligible,key=lambda n:(rows[n]['distinct'],rows[n]['robust'],rows[n]['earlier'],rows[n]['braking']['qualified'],n)) if eligible else ENDPOINTS[-1]
        development[arm]=rows
        selections[arm]=out/arm/f'checkpoint-{n:06d}.pt'
        selections[arm+'_final']=out/arm/'checkpoint-393216.pt'
    selections.update(BC=out/'bc.pt',fixed_a=ROOT/cfg['parent'])
    pins={name:dict(checkpoint=str(p.relative_to(ROOT)),checkpoint_sha256=digest(p),weights_sha256=digest(p.with_suffix('.npz'))) for name,p in selections.items()}
    write_once(out/'final-selection.json',dict(selections=pins,development=development,
        rule=cfg['r8']['selection'],freeze_sha256=digest(out/'freeze.json'),heldout_sha256=frozen['archives']['heldout.npz']))
    return cfg,out,pins


def fresh():
    from .terminal_workflow_final import score_suite
    cfg,out,pins=select()
    if (out/'heldout-results.json').exists():return
    bank=dict(np.load(out/'heldout.npz',allow_pickle=False))
    task,profile,spec,_,manifest,goal=native.load_evaluation_inputs(ROOT/cfg['task_config'])
    controllers={};aliases={}
    for name,p in pins.items():
        same=next((k for k in controllers if pins[k]['checkpoint_sha256']==p['checkpoint_sha256']),None)
        if same:aliases[name]=same
        else:controllers[name]=native.load_controller((ROOT/p['checkpoint']).with_suffix('.npz'),p['checkpoint_sha256'],profile,goal)
    populations=[(str(name),bank,np.flatnonzero(bank['region']==name)) for name in dict.fromkeys(bank['region'])]
    plant=native.NativeContact(profile,spec);limits=read_json(native.HANDOVER_CONFIG)['flight_envelope']
    horizons=dict(terminal=task['braking_horizon_s'],braking=task['braking_horizon_s'],rotation=task['handover_horizon_s'],handover=task['handover_horizon_s'])
    protocol=cfg['diagnostics']
    with native_transition_diagnostics(ROOT/protocol['transition_protocol'],protocol['transition_protocol_sha256']):
        scored=score_suite(plant,controllers,populations,manifest,horizons,limits,cfg['workers'],lambda rows:{},replay_dir=out/'heldout-replays',goal=goal)
    for result in scored.values():
        result['cohorts']={name:_trial_counts([r for r in result['episodes'] if r['population']==name]) for name,_,_ in populations}
    write_result(out/'heldout-results.json',dict(selection_sha256=digest(out/'final-selection.json'),results=scored,aliases=aliases,
        limitation='earlier_fresh: fresh perturbations of development anchors; rotation/braking/terminal/reached handover: separate original sources.'))


def demonstration_starts():
    """Evaluate three unique saved seeds; no prefix execution or actor switching."""
    cfg,_,_,out=configuration();verify_freeze(cfg,out)
    task,profile,spec,_,_,goal=native.load_evaluation_inputs(ROOT/cfg['task_config'])
    limits=read_json(native.HANDOVER_CONFIG)['flight_envelope'];plant=native.NativeContact(profile,spec)
    metadata=read_json(out/'demonstrations.json');starts={}
    for r in metadata['records']:
        if not r['demonstration']:continue
        p=ROOT/r['path'];o=read_json(p/'outcome.json')
        a=dict(np.load(p/'physics.npz'))
        seed=o['seed']
        current=(a['states'][0],a['motor_thrust_N'][0],a['motor_target_N'][0])
        if seed in starts:
            for old,new in zip(starts[seed],current):np.testing.assert_array_equal(old,new)
        else:starts[seed]=current
    actors={'BC':out/'bc.pt','fixed_a':ROOT/cfg['parent']}
    for arm in ('A','B'):
        for n in ENDPOINTS:
            p=out/arm/f'checkpoint-{n:06d}.pt'
            if p.exists():actors[f'{arm}-{n}']=p
    folder=out/'demonstrated-starts';folder.mkdir(exist_ok=True)
    mesh=native.target_mesh(plant,spec.legal_targets)
    for name,cp in actors.items():
        dest=folder/f'{name}.json'
        if dest.exists():
            if read_json(dest)['checkpoint_sha256']!=digest(cp):raise ValueError('Demo evaluation actor changed')
            continue
        command,_=native.load_controller(cp.with_suffix('.npz'),digest(cp),profile,goal)
        rows=[]
        for seed,(state,motors,targets) in starts.items():
            result,trace=native.rollout(plant,state,motors,targets,command,2.3,limits)
            path=folder/f'{name}-{seed}.npz'
            native.save_replay(path,f'{name}-{seed}',trace,result,goal,mesh,profile.policy_dt_s)
            rows.append(dict(seed=seed,**result,replay=str(path.relative_to(ROOT))))
        write_result(dest,dict(checkpoint_sha256=digest(cp),rows=rows,qualified=sum(r['qualified'] for r in rows),
            meaning='Three correlated demonstrated starts, two original families. Every action generated autonomously. Not new-anchor generalization.'))
        print('DEMO_START',name,sum(r['qualified'] for r in rows),[r['reason'] for r in rows],flush=True)


def plots():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cfg,_,_,out=configuration()
    fig,axes=plt.subplots(2,3,figsize=(13,7),layout='constrained')
    for arm,color in [('A','tab:blue'),('B','tab:orange')]:
        pairs=[(n,summary(read_json(out/arm/f'eval-{n:06d}.json'))) for n in (0,*ENDPOINTS) if (out/arm/f'eval-{n:06d}.json').exists()]
        for ax,key in zip(axes[0],('earlier','braking','terminal')):
            ax.plot([n/1024 for n,_ in pairs],[r[key] if key=='earlier' else r[key]['qualified'] for _,r in pairs],'o-',label=arm,color=color)
            ax.set_title(key+' native qualification');ax.set_xlabel('PPO transitions /1024')
        logs=[json.loads(l) for l in (out/arm/'learning.jsonl').read_text().splitlines()]
        axes[1,0].plot([r['transitions']/1024 for r in logs],[r['region_counts']['rotation_bridge']['qualified'] for r in logs],label=arm,color=color)
        demos=[(n,read_json(out/'demonstrated-starts'/f'{arm}-{n}.json')['qualified']) for n in ENDPOINTS if (out/'demonstrated-starts'/f'{arm}-{n}.json').exists()]
        axes[1,1].plot([n/1024 for n,_ in demos],[v for _,v in demos],'o-',label=arm,color=color)
    bc=read_json(out/'bc-results.json')
    for fold in bc['folds']:
        axes[1,2].plot([r['epoch'] for r in fold['history']],[r['validation'] for r in fold['history']],label=fold['source'])
    axes[1,0].set_title('Earlier training successes (cumulative)');axes[1,1].set_title('Demonstrated starts /3');axes[1,2].set_title('BC source-excluded action error')
    for ax in axes.flat:ax.grid(alpha=.2);ax.legend()
    fig.savefig(out/'learning-curves.png',dpi=170);plt.close(fig)

if __name__=='__main__':
    import sys
    {'fresh':fresh,'demo':demonstration_starts,'plots':plots}[sys.argv[1]]()
