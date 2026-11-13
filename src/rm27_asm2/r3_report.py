"""Endpoint decisions and sealed fresh native evaluation for the bounded R3 run."""
from __future__ import annotations

import argparse
from collections import Counter
import json

import numpy as np

from .asset import ROOT, digest, read_json, write_result
from .r3_experiment import configuration, verify_freeze
from .rotation_bridge_diagnostics import _trial_counts, paired_bridge_readout, native_transition_diagnostics


def write_once(path, result):
    result=json.loads(json.dumps(result,allow_nan=False))
    if path.exists():
        if read_json(path) != result:
            raise ValueError(f'Existing analysis changed: {path}')
    else:
        write_result(path, result)


def learning_summary(output):
    results={}; credits={}
    for arm in ('A','B'):
        paths=sorted((output/arm/'regional-rollouts').glob('*.npz'))
        arrays=[dict(np.load(p,allow_pickle=False)) for p in paths]
        if not arrays:continue
        keys=('region','reward','gae_raw','gae_normalized','critic_denormalized','done','action_raw')
        data={k:np.concatenate([a[k] for a in arrays],axis=0) for k in keys}
        mc=np.full_like(data['reward'],np.nan)
        # Each process begins with valid fresh resets: update 1, 2..96, 97..192.
        bounds=[0,64,min(96*64,len(mc)),len(mc)]
        for start,end in zip(bounds[:-1],bounds[1:]):
            running=np.full(mc.shape[1],np.nan)
            for t in range(end-1,start-1,-1):
                running=np.where(data['done'][t],data['reward'][t],data['reward'][t]+.995*running)
                mc[t]=running
        regions={}
        for index,name in enumerate(arrays[0]['band_names']):
            mask=data['region']==index; valid=mask & np.isfinite(mc)
            if not mask.any():continue
            regions[str(name)]=dict(steps=int(mask.sum()),done=int(data['done'][mask].sum()),
                raw_advantage_std=float(data['gae_raw'][mask].std()),
                normalized_advantage_mean=float(data['gae_normalized'][mask].mean()),
                reward_mean=float(data['reward'][mask].mean()),
                tanh_saturation_fraction=np.mean(np.abs(np.tanh(data['action_raw'][mask]))>=.95,axis=0).tolist(),
                uncensored_mc_samples=int(valid.sum()),
                realized_return_value_bias=float((data['critic_denormalized'][valid]-mc[valid]).mean()) if valid.any() else None)
        results[arm]=dict(rollouts=len(arrays),regions=regions,
                         first_std=np.exp(arrays[0]['log_std']).tolist(),last_std=np.exp(arrays[-1]['log_std']).tolist())
        credits[arm]=[]
        # In the unchanged early reward, only a qualified terminal event is positive.
        bridge=int(np.flatnonzero(arrays[0]['band_names']=='rotation_bridge')[0])
        for t,w in np.argwhere((data['region']==bridge)&data['done']&(data['reward']>.5)):
            boundary=max(b for b in (0,64,6144) if b<=t)
            ends=np.flatnonzero(data['done'][boundary:t,w])
            start=boundary+(int(ends[-1])+1 if len(ends) else 0)
            if not np.all(data['region'][start:t+1,w]==bridge):
                raise ValueError('Successful episode crosses a reset boundary')
            values={k:data[k][start:t+1,w] for k in ('reward','gae_raw','gae_normalized','critic_denormalized')}
            n=t+1-start; half=max(1,n//2)
            credits[arm].append(dict(end_update=int(t//64+1),end_rollout_tick=int(t%64),world=int(w),
                steps=int(n),duration_s=float(n*.005),
                first_half_raw_mean=float(values['gae_raw'][:half].mean()),
                last_20_raw_mean=float(values['gae_raw'][-20:].mean()),
                first_half_normalized_mean=float(values['gae_normalized'][:half].mean()),
                last_20_normalized_mean=float(values['gae_normalized'][-20:].mean()),
                **{k:v.tolist() for k,v in values.items()}))
    counts={v['rollouts'] for v in results.values()}
    if len(counts)!=1 or len(results)!=2:
        raise ValueError('Learning comparison requires a completed matched endpoint')
    endpoint=counts.pop()*1024
    write_once(output/f'learning-summary-{endpoint:06d}.json',results)
    write_once(output/f'successful-episode-credit-{endpoint:06d}.json',dict(episodes=credits,
        scope='Saved stochastic training only; no native rescoring or complete state traces. Realized GAE under changing policies, not a counterfactual causal estimate.'))
    return results


def assessment(output, endpoint):
    reports = {a:read_json(output/a/f'eval-{endpoint:06d}.json') for a in ('A','B')}
    comparison = paired_bridge_readout(reports['A'], reports['B'])
    logs = {a:[json.loads(s) for s in (output/a/'learning.jsonl').read_text().splitlines()] for a in ('A','B')}
    rows = {a:next(r for r in logs[a] if r['completed_transitions']==endpoint) for a in logs}
    initial = read_json(output/'A/eval-000000.json')
    baseline_anchors = {(r['case'],r['source_frame']) for r in initial['bridge']['rows'] if r['qualified']}
    new = sorted({(r['case'],r['source_frame']) for r in reports['B']['bridge']['rows'] if r['qualified']}-baseline_anchors)
    checks = dict(zero_earlier_training_qualifications=rows['B']['region_counts']['rotation_bridge']['qualified']==0,
                  no_native_entry_gain=comparison['intervention']['newly_entered']<=comparison['control']['newly_entered'],
                  no_native_qualification_gain=comparison['net_qualification_delta']<=0,
                  no_new_qualified_anchor=not new)
    robust_sets = {}
    for arm, report in reports.items():
        by_anchor = {}
        for row in report['bridge']['rows']:
            by_anchor.setdefault((row['case'], row['source_frame']), []).append(row['qualified'])
        robust_sets[arm] = {k for k,v in by_anchor.items() if len(v)==3 and all(v)}
    robust_gain = bool(robust_sets['B']-robust_sets['A'])
    result = dict(endpoint=endpoint, comparison=comparison, new_B_anchors=new,
                  robust_counts={a:reports[a]['bridge']['robust'] for a in reports},
                  r3_development_scientific_go=comparison['scientific_learning_signal'] and robust_gain,
                  training={a:{k:rows[a][k] for k in ('transitions','updates','learning_s','region_counts','exploration','episodes','loss')} for a in rows},
                  futility_checks=checks, stop_for_futility=all(checks.values()),
                  native_reports={a:digest(output/a/f'eval-{endpoint:06d}.json') for a in reports})
    path=output/('intermediate-assessment.json' if endpoint==98304 else 'endpoint-assessment.json')
    write_once(path,result)
    return result


def final(cfg, output, endpoint):
    from . import terminal_native_eval as native
    from .terminal_workflow_final import score_suite
    frozen=verify_freeze(cfg,output)
    ledger=read_json(output/'ledger.json')
    if any(r['pending'] is not None or r['transitions']!=endpoint for r in ledger['arms'].values()):
        raise ValueError('Final evaluation requires both stopped arms at the common endpoint')
    decision=assessment(output,endpoint)
    if endpoint==98304 and not decision['stop_for_futility']:
        raise ValueError('Intermediate endpoint has not met the frozen futility rule')
    selections={}
    for arm in ('A','B'):
        checkpoint=output/arm/f'checkpoint-{endpoint:06d}.pt'
        selections[arm]=dict(checkpoint=str(checkpoint.relative_to(ROOT)),checkpoint_sha256=digest(checkpoint),
                             weights_sha256=digest(checkpoint.with_suffix('.npz')))
    parent=ROOT/cfg['parent']
    selections['fixed_a']=dict(checkpoint=cfg['parent'],checkpoint_sha256=digest(parent),
                               weights_sha256=digest(parent.with_suffix('.npz')))
    receipt=dict(endpoint=endpoint,selection='Last common endpoint; no best-checkpoint selection',
                 selections=selections,freeze_sha256=digest(output/'freeze.json'),
                 heldout_sha256=frozen['archives']['heldout.npz'],ledger_sha256=digest(output/'ledger.json'),
                 analysis_sha256=digest(__file__))
    path=output/'final-selection.json'
    if path.exists() and read_json(path)!=receipt:
        raise ValueError('Selection changed after heldout opening')
    write_once(path,receipt)
    final_path=output/'heldout-results.json'
    if final_path.exists():
        result=read_json(final_path)
        if result['selection_sha256']!=digest(path):
            raise ValueError('Heldout result belongs to a different selection')
        return result
    bank=dict(np.load(output/'heldout.npz',allow_pickle=False))
    _,profile,spec,mp,manifest,goal=native.load_evaluation_inputs(ROOT/cfg['task_config'])
    controllers={a:native.load_controller(ROOT/s['checkpoint'].replace('.pt','.npz'),s['checkpoint_sha256'],profile,goal)
                 for a,s in selections.items()}
    populations=[(str(name),bank,np.flatnonzero(bank['region']==name)) for name in dict.fromkeys(bank['region'])]
    plant=native.NativeContact(profile,spec)
    limits=read_json(native.HANDOVER_CONFIG)['flight_envelope']
    task=read_json(ROOT/cfg['task_config'])
    horizons=dict(terminal=task['braking_horizon_s'],braking=task['braking_horizon_s'],
                  rotation=task['handover_horizon_s'],handover=task['handover_horizon_s'])
    protocol=cfg['diagnostics']
    with native_transition_diagnostics(ROOT/protocol['transition_protocol'],protocol['transition_protocol_sha256']):
        scored=score_suite(plant,controllers,populations,manifest,horizons,limits,cfg['workers'],lambda rows:{},
                           replay_dir=output/'heldout-replays',goal=goal)
    for actor,result in scored.items():
        # Keep earlier-anchor perturbations separate from disjoint-source rotations.
        result['cohorts']={name:_trial_counts([r for r in result['episodes'] if r['population']==name])
                           for name,_,_ in populations}
        anchors={}
        for row in result['episodes']:
            if row['population']=='earlier_fresh':
                anchors.setdefault(f"{row['case']}:{row['source_frame']}",[]).append(row['qualified'])
        result['earlier_anchors']={k:sum(v) for k,v in anchors.items()}
        result['earlier_robust_3of3']=sum(all(v) and len(v)==3 for v in anchors.values())
    result=dict(selection_sha256=digest(path),model=plant.metadata,results=scored,
                 counts=frozen['heldout']['counts'],limits=frozen['heldout']['limitation'])
    write_result(final_path,result)
    return result


def plots(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(13,7),layout='constrained')
    for arm,color in [('A','#2166ac'),('B','#b2182b')]:
        rows=[json.loads(s) for s in (output/arm/'learning.jsonl').read_text().splitlines()]
        x=np.array([r['transitions'] for r in rows])/1024
        axes[0,0].plot(x,[np.mean(r['exploration']) for r in rows],color=color,label=arm)
        axes[0,1].plot(x,[r['region_counts']['rotation_bridge']['qualified'] for r in rows],color=color)
        axes[0,2].plot(x,[r['region_counts']['rotation_bridge']['episodes'] for r in rows],color=color)
        reports=[read_json(p) for p in sorted((output/arm).glob('eval-*.json')) if '-development' not in p.name]
        xx=[int(p.stem.split('-')[-1])/1024 for p in sorted((output/arm).glob('eval-*.json')) if '-development' not in p.name]
        axes[1,0].plot(xx,[r['bridge']['qualified'] for r in reports],'o-',color=color)
        axes[1,1].plot(xx,[_trial_counts(r['bridge']['rows'])['newly_entered'] for r in reports],'o-',color=color)
        axes[1,2].plot(xx,[r['development']['cohorts']['braking']['qualified'] for r in reports],'o-',color=color)
    titles=['Mean Gaussian std (raw action)','Earlier training qualifications (cumulative)',
            'Earlier completed training episodes','Native earlier qualified / 36',
            'Native new descriptor entries / 36','Native braking qualified / 36']
    for ax,title in zip(axes.flat,titles):
        ax.set_title(title,fontsize=10);ax.set_xlabel('Additional transitions / 1024');ax.grid(alpha=.2)
    axes[0,0].legend(title='A control / B exploration')
    for ext in ('png','pdf'):fig.savefig(output/f'learning-evidence.{ext}',dpi=170)
    plt.close(fig)
    if not (output/'heldout-results.json').exists():return
    results=read_json(output/'heldout-results.json')['results']
    names=list(results['A']['cohorts'])
    fig,ax=plt.subplots(figsize=(10,4),layout='constrained')
    x=np.arange(len(names));width=.25
    for i,(arm,color) in enumerate([('fixed_a','#777777'),('A','#2166ac'),('B','#b2182b')]):
        counts=[results[arm]['cohorts'][n]['qualified'] for n in names]
        bars=ax.bar(x+(i-1)*width,counts,width,label=arm,color=color)
        ax.bar_label(bars,labels=[f"{c}/{results[arm]['cohorts'][n]['trials']}" for n,c in zip(names,counts)],fontsize=8,padding=3)
    ax.set_xticks(x,names);ax.set_ylabel('Native qualified contacts');ax.legend();ax.set_ylim(0,max(5,ax.get_ylim()[1]*1.2))
    for ext in ('png','pdf'):fig.savefig(output/f'heldout-qualification.{ext}',dpi=170)
    plt.close(fig)
    from .contact_task import load_contact_spec
    from .flight_suite import rotation_matrix
    spec=load_contact_spec()
    fig,axes=plt.subplots(2,2,figsize=(10,6),layout='constrained')
    for arm,color in [('A','#2166ac'),('B','#b2182b')]:
        candidates=sorted((output/'heldout-replays').glob(f'{arm}-rotation-*.npz'))
        if not candidates:continue
        data=np.load(candidates[0],allow_pickle=False)
        states=data['states'][:,0]; t=data['sample_times_s']
        local=(states[:,:3]-spec.target_center)@spec.target_rotation
        vel=states[:,7:10]@spec.target_rotation
        alignment=np.degrees(np.arccos(np.clip([rotation_matrix(s[3:7])[:,2]@spec.target_normal for s in states],-1,1)))
        label=f"{arm}: {data['names'][0]} ({data['contact_outcome'].item()})"
        for ax,y in zip(axes.flat,[local[:,0],vel[:,0],-vel[:,2],alignment]):
            ax.plot(t,y,color=color,label=label)
    for ax,title in zip(axes.flat,['Target tangent position (m)','Target tangent velocity (m/s)',
                                   'Inward COM normal speed (m/s)','Underside alignment (deg)']):
        ax.set_title(title,fontsize=10);ax.set_xlabel('Time (s)');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=6)
    fig.suptitle('Representative saved native autonomous trajectories; case IDs identify comparability')
    for ext in ('png','pdf'):fig.savefig(output/f'representative-trajectories.{ext}',dpi=170)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('assess','final','plots'))
    parser.add_argument('--endpoint',type=int,choices=(98304,196608),default=98304)
    args=parser.parse_args()
    cfg,_,_,output=configuration();verify_freeze(cfg,output)
    learning_summary(output)
    if args.command=='assess':
        r=assessment(output,args.endpoint)
        print(json.dumps({k:r[k] for k in ('endpoint','futility_checks','stop_for_futility','r3_development_scientific_go')},indent=2))
    elif args.command=='final':final(cfg,output,args.endpoint)
    else:plots(output)


if __name__=='__main__':main()
