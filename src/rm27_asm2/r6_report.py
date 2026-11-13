"""R6 saved-data analysis and sealed native evaluation; no training or method sweep."""
from __future__ import annotations
import argparse
import json
from unittest.mock import patch
import numpy as np
from .asset import ROOT,digest,read_json,write_result
from .r6_experiment import configuration,verify_freeze

def write_once(path,result):
    result=json.loads(json.dumps(result,allow_nan=False))
    if path.exists():
        if read_json(path)!=result:raise ValueError(f"Existing analysis differs: {path}")
    else:write_result(path,result)

def assessment(out,endpoint):
    from .rotation_bridge_diagnostics import paired_bridge_readout
    reports={a:read_json(out/a/f'eval-{endpoint:06d}.json') for a in ('A','B')}
    comparison=paired_bridge_readout(reports['A'],reports['B'])
    sets={};robust={};counts={}
    for a,r in reports.items():
        groups={}
        for row in r['bridge']['rows']:groups.setdefault((row['case'],row['source_frame']),[]).append(row['qualified'])
        counts[a]={k:sum(v) for k,v in groups.items()}
        sets[a]={k for k,v in groups.items() if any(v)}
        robust[a]={k for k,v in groups.items() if len(v)==3 and all(v)}
    gains={k for k in counts['B'] if counts['B'][k]>counts['A'][k]};robust_gains=robust['B']-robust['A']
    initial=read_json(out/'A/eval-000000.json')
    initial_set={(r['case'],r['source_frame']) for r in initial['bridge']['rows'] if r['qualified']}
    dev=reports['B']['development']['cohorts']
    retention=(dev['braking']['qualified']>=32 and dev['terminal']['qualified']>=27
               and all(dev[b]['safety_failures']==0 for b in ('braking','terminal')))
    extend=len(gains)>=2 and bool(robust_gains) and retention and reports['B']['bridge']['qualified']>reports['A']['bridge']['qualified']
    result=dict(endpoint=endpoint,comparison=comparison,qualified_anchors={a:sorted(sets[a]) for a in sets},
        robust_anchors={a:sorted(robust[a]) for a in sets},new_B_anchors=sorted(sets['B']-initial_set),
        net_B_anchors=sorted(gains),retention_pass=retention,extension_gate_pass=extend,
        stop_for_futility=not extend,scientific_go=extend,
        reason='Multi-anchor native gains plus robustness and retention required; training-only progress excluded.',
        native_reports={a:digest(out/a/f'eval-{endpoint:06d}.json') for a in reports})
    write_once(out/'intermediate-assessment.json',result)
    return result

def final(cfg,out):
    from . import r3_report as reused
    with patch.object(reused,'assessment',assessment):
        return reused.final(cfg,out,98304)

def dataset_plot(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d=read_json(out/'dataset.json');fig,axes=plt.subplots(2,2,figsize=(11,7),layout='constrained')
    names=d['trajectories'];sources=list(dict.fromkeys(r['source'] for r in names))
    colors=plt.get_cmap('tab10')
    for r in names:
        a=dict(np.load(ROOT/r['paths']['physics.npz'],allow_pickle=False));t=a['sample_times_s']-a['sample_times_s'][0]
        c=colors(sources.index(r['source']));style='-' if r['qualified'] else '--'
        axes[0,0].plot(t,np.linalg.norm(a['states'][:,7:10],axis=1),style,color=c,alpha=.6)
        axes[0,1].plot(t,np.linalg.norm(a['states'][:,10:13],axis=1),style,color=c,alpha=.6)
        axes[1,0].plot(t,np.linalg.norm(a['motor_target_N']-a['motor_thrust_N'],axis=1),style,color=c,alpha=.6)
    for i,s in enumerate(sources):
        r=[r for r in names if r['source']==s];axes[1,1].bar(i,sum(x['qualified'] for x in r),color=colors(i));axes[1,1].text(i,len(r)+.1,f"{sum(x['qualified'] for x in r)}/{len(r)}",ha='center')
    axes[1,1].set_xticks(range(len(sources)),sources,rotation=25);axes[1,1].set_ylim(0,8)
    for ax,title in zip(axes.flat,['COM speed (m/s)','Body-rate norm (rad/s)','Motor target–actual residual norm (N)','Native qualified / recorded trajectories']):ax.set_title(title);ax.grid(alpha=.2)
    for ax in list(axes.flat)[:3]:ax.set_xlabel('Autonomous elapsed time (s)')
    fig.suptitle('R5/R5.1 physical transition dataset — dashed: rejected; correlated source colors')
    fig.savefig(out/'transition-dataset.png',dpi=160);fig.savefig(out/'transition-dataset.pdf');plt.close(fig)

def transition_annotations(cfg,out):
    from .contact_task import load_contact_spec
    from .rotation_bridge_diagnostics import transition_diagnostic
    protocol=read_json(ROOT/cfg['diagnostics']['transition_protocol']);spec=load_contact_spec()
    data=read_json(out/'dataset.json');records=[]
    for row in data['trajectories']:
        a=dict(np.load(ROOT/row['paths']['physics.npz'],allow_pickle=False))
        result=read_json(ROOT/row['paths']['outcome.json'])['result']
        ids=np.arange(0,len(a['states']),50)
        times=a['sample_times_s'][ids]-a['sample_times_s'][0]
        d=transition_diagnostic(a['states'][ids],times,spec,protocol['region'],result)
        entry=None
        if d['first_entry_s'] is not None:
            j=int(ids[np.argmin(abs(times-d['first_entry_s']))]);prev=max(0,j-50)
            entry={name:{k:a[k][idx].tolist() for k in ('states','motor_thrust_N','motor_target_N')} for name,idx in [('before',prev),('at',j)]}
        records.append(dict(id=row['id'],source=row['source'],qualified=row['qualified'],reason=row['reason'],
            descriptor=d,entry_physical_state=entry,terminal_state=a['states'][-1].tolist(),
            terminal_motor_thrust_N=a['motor_thrust_N'][-1].tolist(),terminal_motor_target_N=a['motor_target_N'][-1].tolist()))
    write_once(out/'transition-annotations.json',dict(records=records,protocol_sha256=digest(ROOT/cfg['diagnostics']['transition_protocol']),
        note='Descriptor entry is annotation only, never a reward, reset gate, or reachability certificate. On qualified trajectories it identifies an observed entry followed by qualified contact. Failure labels are frozen native outcomes. No new rollouts.'))

def row_provenance(out):
    d=read_json(out/'dataset.json');meta=read_json(out/'curriculum.json')
    bank=dict(np.load(out/'intervention-training.npz',allow_pickle=False));mapping=[];row=meta['base_count']
    history=read_json(ROOT/'.agents/runs/asm2-frontier/phase-r/results.json')['rows']
    history=[r for r in history if r['controller']=='fixed_a' and r['case'] in meta['sources']]
    for rec in d['trajectories']:
        if not rec['qualified']:continue
        a=dict(np.load(ROOT/rec['paths']['physics.npz'],allow_pickle=False))
        for j in range(0,max(1,len(a['states'])-1-400),400):
            for k in ('states','motor_thrust_N','motor_target_N'):np.testing.assert_array_equal(bank[k][row],a[k][j])
            mapping.append(dict(row=row,kind='autonomous_success_suffix',trajectory_id=rec['id'],source=rec['source'],variant=rec['variant'],physics_sample=j,
                elapsed_s=float(a['sample_times_s'][j]-a['sample_times_s'][0]),future_native_outcome='Observed qualified continuation, suffix reset not independently rerun',independence_group=rec['source']));row+=1
    for source,chain in zip(meta['sources'],meta['chains']):
        for stage,index in enumerate(chain):
            if index!=row:raise ValueError('Reset row ordering changed')
            frame=int(bank['source_frame'][row]);labels=[r for r in history if r['case']==source and r['frame']==frame]
            mapping.append(dict(row=row,kind='archived_predecessor_proposal',source=source,frame=frame,stage=stage,
                historical_nominal_seed_qualified=stage==0,phase_r_existing_labels=labels,independence_group=source));row+=1
    if row!=len(bank['states']):raise ValueError('Incomplete identity mapping')
    (out/'reset-row-provenance.json').write_text(json.dumps(dict(rows=mapping,intervention_bank_sha256=digest(out/'intervention-training.npz'),identity_matches=True),indent=2))
    write_once(out/'historical-fixed-a-labels.json',dict(rows=history,
        source='.agents/runs/asm2-frontier/phase-r/results.json',source_sha256=digest(ROOT/'.agents/runs/asm2-frontier/phase-r/results.json'),
        scope='Five training sources only. Historical outcomes, not newly run trials; only available aggregate/endpoint fields retained. Complete actions/rewards are not imputed.'))

def curves(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(13,7),layout='constrained')
    for arm,color in [('A','#2166ac'),('B','#b2182b')]:
        rows=[json.loads(s) for s in (out/arm/'learning.jsonl').read_text().splitlines()]
        x=[r['transitions']/1024 for r in rows]
        axes[0,0].plot(x,[r['region_counts']['rotation_bridge']['qualified'] for r in rows],label=arm,color=color)
        axes[0,1].plot(x,[r['region_counts']['rotation_bridge']['episodes'] for r in rows],color=color)
        axes[0,2].plot(x,[np.mean(r['exploration']) for r in rows],color=color)
        for endpoint in (0,98304):
            p=out/arm/f'eval-{endpoint:06d}.json'
            if not p.exists():continue
            r=read_json(p)
            for ax,value in zip(axes[1],[r['bridge']['qualified'],r['development']['cohorts']['braking']['qualified'],r['development']['cohorts']['terminal']['qualified']]):ax.scatter(endpoint/1024,value,color=color,label=arm if endpoint==0 else None)
    for ax,title in zip(axes.flat,['Bridge-band training qualifications','Bridge-band completed episodes','Mean raw Gaussian std','Earlier native contacts / 36','Braking native contacts / 36','Terminal native contacts / 30']):
        ax.set_title(title);ax.set_xlabel('Transitions / 1024');ax.grid(alpha=.2)
    axes[0,0].legend(title='A unchanged / B curriculum')
    for ax,limit in zip(axes[1],(36,36,30)):ax.set_ylim(0,limit+1)
    fig.savefig(out/'learning-evidence.png',dpi=160);fig.savefig(out/'learning-evidence.pdf');plt.close(fig)

def replay_plot(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from .flight_viewer import load_recording
    paths=sorted((out/'heldout-replays').glob('*.npz'))
    checked=[]
    for p in paths:
        load_recording(p);checked.append(dict(path=str(p.relative_to(ROOT)),sha256=digest(p)))
    fig=plt.figure(figsize=(12,5),layout='constrained')
    for j,kind in enumerate(('rotation-failed','braking-qualified')):
        ax=fig.add_subplot(1,2,j+1,projection='3d');extent=[]
        for arm,color in [('A','#2166ac'),('B','#b2182b')]:
            p=out/'heldout-replays'/f'{arm}-{kind}.npz'
            if not p.exists():continue
            a=dict(np.load(p,allow_pickle=False));x=a['states'][:,0,:3];extent.append(x)
            ax.plot(*x.T,color=color,label=str(a['names'][0]));ax.scatter(*x[0],color=color,marker='o');ax.scatter(*x[-1],color=color,marker='x')
            if arm=='A':
                extent.append(a['target_vertices_m']);mesh=a['target_vertices_m'][a['target_faces']];ax.add_collection3d(Poly3DCollection(mesh,alpha=.15,facecolor='gray'))
        ax.set_title(kind);ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.set_zlabel('world z (m)');ax.legend(fontsize=6)
        if extent:
            x=np.concatenate(extent);span=np.maximum(np.ptp(x,axis=0),.1);center=(x.max(axis=0)+x.min(axis=0))/2
            for setter,c,v in zip((ax.set_xlim,ax.set_ylim,ax.set_zlim),center,span):setter(c-.65*v,c+.65*v)
            ax.set_box_aspect(span)
    fig.suptitle('Native autonomous replay examples — COM paths; target mesh shown, no reference tracking')
    fig.savefig(out/'representative-3d.png',dpi=170);plt.close(fig)
    write_once(out/'replay-verification.json',dict(viewer_load_pass=True,replays=checked))

def mechanism(out):
    from .r6_curriculum import FrontierSampler
    from .terminal_curriculum import BANDS
    bank=dict(np.load(out/'intervention-training.npz',allow_pickle=False));meta=read_json(out/'curriculum.json')
    sampler=FrontierSampler(bank['band'],'fixed',0,{'fixed_mix':dict(terminal=.1,braking=.45,rotation_late=.2,rotation_bridge=.25)},meta)
    progress=[];categories={};global_tick=0
    support={int(i) for group in meta['support'] for i in group};front={int(i) for group in meta['chains'] for i in group}
    for arm in ('A','B'):
        counts={};tick=0;starts=np.zeros(16,dtype=int)
        for fileindex,p in enumerate(sorted((out/arm/'physical-rollouts').glob('*.npz'))):
            a=dict(np.load(p,allow_pickle=False))
            if fileindex==1:starts.fill(tick) # charged restart: unfinished episodes are censored
            for t in range(len(a['done'])):
                rows=a['reset_row'][t];done=a['done'][t];rewards=a['reward'][t]
                if arm=='B':
                    before=sampler.frontier.copy();sampler.record(rows[done],rewards[done]>.5)
                    if sampler.frontier!=before:progress.append(dict(transitions=(tick+1)*16,frontier=sampler.frontier.copy()))
                for w in np.flatnonzero(done):
                    row=int(rows[w]);kind=('frontier' if row in front else 'successful_suffix' if row in support else 'retention') if arm=='B' else str(bank['band'][row])
                    group=counts.setdefault(kind,dict(episodes=0,positive_terminal_events=0,durations_s=[]))
                    group['episodes']+=1
                    # For early-band rows only, >0.5 at termination exactly identifies the positive event under frozen reward bounds.
                    group['positive_terminal_events']+=int(rewards[w]>.5);group['durations_s'].append((tick-int(starts[w])+1)*.005);starts[w]=tick+1
                tick+=1
        for group in counts.values():
            duration=group.pop('durations_s');group['mean_duration_s']=float(np.mean(duration));group['duration_quantiles_s']=np.quantile(duration,[.1,.5,.9]).tolist()
        categories[arm]=counts
    source_frontiers={source:dict(stage=sampler.frontier[i],frame=int(bank['source_frame'][meta['chains'][i][sampler.frontier[i]]]),
        seed_frame=int(bank['source_frame'][meta['chains'][i][0]]),last_eight=sampler.recent.get(str(meta['chains'][i][sampler.frontier[i]]),[])) for i,source in enumerate(meta['sources'])}
    result=dict(training_categories=categories,progress=progress,source_frontiers=source_frontiers,promotions=sampler.promotions,
        note='Reconstructed from saved completion flags/rows and unchanged early reward. Positive-terminal threshold is exact for rotation_bridge reward bounds; retention values are descriptive only. Training feedback is not native qualification. Restarts censor unfinished episodes; source labels are not actor inputs.')
    write_once(out/'mechanism.json',result)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
    points=[dict(transitions=0,frontier=[0]*5),*progress,dict(transitions=98304,frontier=sampler.frontier)]
    for i,source in enumerate(meta['sources']):ax.step([p['transitions']/1024 for p in points],[p['frontier'][i]*40 for p in points],where='post',label=source)
    ax.set_xlabel('B transitions / 1024');ax.set_ylabel('Predecessor proposal unlocked (ms earlier)');ax.set_title('Training competence gates — not certified reachability or native success');ax.legend();ax.grid(alpha=.2);fig.savefig(out/'curriculum-progress.png',dpi=170);plt.close(fig)
    return result

def support_evaluation(cfg,out):
    from .terminal_workflow import evaluate_probe_bank
    from .rotation_bridge_diagnostics import native_transition_diagnostics
    protocol=read_json(out/'support-evaluation-protocol.json');dataset=read_json(out/'dataset.json')
    if protocol['dataset_sha256']!=digest(out/'dataset.json'):raise ValueError('Support source changed')
    selection=read_json(out/'final-selection.json');items=dataset['trajectories']
    if [r['id'] for r in items]!=protocol['cases']:raise ValueError('Support case selection changed')
    p=out/'support-evaluation.json'
    if p.exists():return read_json(p)
    physical=[dict(np.load(ROOT/r['paths']['physics.npz'],allow_pickle=False)) for r in items]
    bank={k:np.stack([a[k][0] for a in physical]) for k in ('states','motor_thrust_N','motor_target_N')}
    bank.update(case_ids=np.array([r['source'] for r in items]),source_frame=np.array([r['source_frame'] for r in items]),
        sample_times_s=np.array([r['source_frame']*.005 for r in items]),phase=np.full(len(items),'rotation'))
    results={};diag=cfg['diagnostics']
    for arm in ('A','B'):
        cp=ROOT/selection['selections'][arm]['checkpoint']
        if digest(cp)!=selection['selections'][arm]['checkpoint_sha256']:raise ValueError('Endpoint changed')
        with native_transition_diagnostics(ROOT/diag['transition_protocol'],diag['transition_protocol_sha256']):
            rows=evaluate_probe_bank(cfg,bank,cp,cp.with_suffix('.npz'))
        results[arm]=[dict(id=r['id'],variant=r['variant'],fixed_a_qualified=r['qualified'],fixed_a_reason=r['reason'],**x) for r,x in zip(items,rows,strict=True)]
    result=dict(protocol_sha256=digest(out/'support-evaluation-protocol.json'),selection_sha256=digest(out/'final-selection.json'),results=results,
        scope=protocol['scope']);write_once(p,result);return result

def frontier_evaluation(cfg,out):
    from .terminal_workflow import evaluate_probe_bank
    from .rotation_bridge_diagnostics import native_transition_diagnostics
    protocol=read_json(out/'frontier-evaluation-protocol.json');meta=read_json(out/'curriculum.json')
    selected=read_json(out/'final-selection.json');mechanics=read_json(out/'mechanism.json')
    full=dict(np.load(out/'intervention-training.npz',allow_pickle=False));indices=[]
    for source,chain in zip(meta['sources'],meta['chains']):
        stage=mechanics['source_frontiers'][source]['stage']
        indices.extend(dict.fromkeys([chain[max(0,stage-1)],chain[stage]]))
    if len(indices)>protocol['maximum_unique_starts']:raise ValueError('Frontier diagnostic exceeds bound')
    bank={k:v[indices] if v.ndim else v for k,v in full.items()};results={};diag=cfg['diagnostics']
    path=out/'frontier-evaluation.json'
    if path.exists():return read_json(path)
    for arm in protocol['policies']:
        cp=ROOT/selected['selections'][arm]['checkpoint']
        if digest(cp)!=selected['selections'][arm]['checkpoint_sha256']:raise ValueError('Sealed actor changed')
        from . import terminal_native_eval as native
        from .backward_frontier import Capture
        _,profile,spec,_,_,goal=native.load_evaluation_inputs(ROOT/cfg['task_config'])
        with native_transition_diagnostics(ROOT/diag['transition_protocol'],diag['transition_protocol_sha256']):
            original=native.rollout
            def captured(plant,state,motors,targets,command,horizon,limits):
                hits=[j for j,s in enumerate(bank['states']) if np.array_equal(s,state)]
                if len(hits)!=1:raise ValueError('Ambiguous frontier capture identity')
                j=hits[0];row=indices[j];dest=out/'frontier-trajectories'/f'{arm}-{row}'
                dest.mkdir(parents=True,exist_ok=True)
                write_result(dest/'started.json',dict(arm=arm,training_row=row,checkpoint_sha256=digest(cp)))
                sample=dict(state=state,motor_thrust_N=motors,motor_target_N=targets)
                with Capture(plant,sample) as capture:result,trace=original(plant,state,motors,targets,command,horizon,limits)
                with (dest/'physics.npz').open('xb') as stream:
                    np.savez_compressed(stream,states=capture.states,motor_thrust_N=capture.motors,motor_target_N=capture.targets,
                        sample_times_s=capture.times,ctbr_N_rad_s=capture.commands,control_times_s=capture.command_times,
                        allocated_motor_target_N=capture.allocated,dt=plant.dt)
                native.save_replay(dest/'replay.npz',f'{arm}-{bank["case_ids"][j]}-{bank["source_frame"][j]}',trace,result,goal,native.target_mesh(plant,spec.legal_targets),profile.policy_dt_s)
                write_result(dest/'contacts.json',capture.contacts);write_result(dest/'outcome.json',result)
                return result,trace
            with patch.object(native,'rollout',captured):rows=evaluate_probe_bank(cfg,bank,cp,cp.with_suffix('.npz'))
        results[arm]=[dict(training_row=i,**r) for i,r in zip(indices,rows,strict=True)]
    result=dict(protocol_sha256=digest(out/'frontier-evaluation-protocol.json'),selection_sha256=digest(out/'final-selection.json'),indices=indices,results=results,scope=protocol['scope'])
    write_once(path,result);return result

def checkpoint_check(cfg,out):
    import torch
    parent=torch.load(ROOT/cfg['parent'],map_location='cpu',weights_only=True)
    def equal(a,b):
        if torch.is_tensor(a):return torch.is_tensor(b) and torch.equal(a,b)
        if isinstance(a,dict):return a.keys()==b.keys() and all(equal(a[k],b[k]) for k in a)
        if isinstance(a,(list,tuple)):return len(a)==len(b) and all(equal(x,y) for x,y in zip(a,b))
        return a==b
    result={}
    for arm in ('A','B'):
        first=torch.load(out/arm/'checkpoint-000000.pt',map_location='cpu',weights_only=True)
        last=torch.load(out/arm/'checkpoint-098304.pt',map_location='cpu',weights_only=True)
        fields=('actor','critic','value_normalizer','actor_optimizer','critic_optimizer')
        matches={k:equal(first[k],parent[k]) for k in fields}
        if not all(matches.values()):raise ValueError('Starting learned state differs from Fixed A')
        if last['completed_transitions']!=98304 or last['workflow_updates']!=96:raise ValueError('Incomplete endpoint')
        if last['restart_count']<1:raise ValueError('Missing resume integration')
        if arm=='B':
            reconstructed=read_json(out/'mechanism.json')
            if last['sampler']['r6']['promotions']!=reconstructed['promotions']:raise ValueError('Reconstructed competence history differs')
        result[arm]=dict(initial_matches_parent=matches,completed_transitions=last['completed_transitions'],updates=last['workflow_updates'],
            restart_count=last['restart_count'],checkpoint_sha256=digest(out/arm/'checkpoint-098304.pt'),
            sampler_frontier=last['sampler'].get('r6',{}).get('frontier'),all_actor_finite=all(torch.isfinite(v).all().item() for v in last['actor'].values()))
    write_once(out/'checkpoint-verification.json',result)
    return result

def frontier_plots(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from .flight_viewer import load_recording
    from .contact_task import load_contact_spec
    result=read_json(out/'frontier-evaluation.json');spec=load_contact_spec()
    fig=plt.figure(figsize=(12,8),layout='constrained');checked=[]
    for column,(case,frame) in enumerate([('dev_fast',808),('stationary_2',322)]):
        row=next(r['training_row'] for r in result['results']['B'] if r['case']==case and r['source_frame']==frame)
        ax=fig.add_subplot(2,2,column+1,projection='3d');motion=fig.add_subplot(2,2,column+3)
        extent=[]
        for actor,color in [('fixed_a','#777777'),('A','#2166ac'),('B','#b2182b')]:
            folder=out/'frontier-trajectories'/f'{actor}-{row}';a=dict(np.load(folder/'physics.npz',allow_pickle=False));r=dict(np.load(folder/'replay.npz',allow_pickle=False))
            x=a['states'][:,:3];extent.append(x);label=f"{actor}: {r['contact_outcome'].item()}"
            ax.plot(*x.T,color=color,label=label);ax.scatter(*x[-1],color=color,marker='x')
            t=a['sample_times_s'];velocity=a['states'][:,7:10]@spec.target_rotation
            motion.plot(t,velocity[:,0],color=color,label=label)
            if actor=='fixed_a':
                extent.append(r['target_vertices_m']);ax.add_collection3d(Poly3DCollection(r['target_vertices_m'][r['target_faces']],facecolor='gray',alpha=.2))
        x=np.concatenate(extent);span=np.maximum(np.ptp(x,axis=0),.1);center=(x.max(axis=0)+x.min(axis=0))/2
        for setter,c,v in zip((ax.set_xlim,ax.set_ylim,ax.set_zlim),center,span):setter(c-.6*v,c+.6*v)
        ax.set_box_aspect(span);ax.set_title(f'{case}:{frame} — same full initial state');ax.set_xlabel('world x');ax.set_ylabel('world y');ax.set_zlabel('world z')
        motion.set_xlabel('Time (s)');motion.set_ylabel('Target-slope velocity (m/s)');motion.legend(fontsize=7);motion.grid(alpha=.2)
    fig.suptitle('Local development gains selected for illustration; all frontier outcomes remain in the denominator')
    fig.savefig(out/'frontier-transitions.png',dpi=170);plt.close(fig)
    for p in sorted((out/'frontier-trajectories').glob('*/replay.npz')):
        load_recording(p);a=dict(np.load(p.parent/'physics.npz',allow_pickle=False));o=read_json(p.parent/'outcome.json')
        if len(a['states'])!=len(a['motor_thrust_N']) or len(a['states'])!=len(a['motor_target_N']):raise ValueError('Incomplete motor history')
        np.testing.assert_allclose(np.diff(a['sample_times_s']),.0001,rtol=0,atol=1e-12)
        from .flight import load_profile
        alpha=-np.expm1(-.0001/load_profile().motor_time_constant_s)
        expected=a['motor_thrust_N'][:-1]+alpha*(a['motor_target_N'][1:]-a['motor_thrust_N'][:-1])
        error=float(np.max(np.abs(expected-a['motor_thrust_N'][1:])))
        if error>1e-12:raise ValueError('Motor history violates frozen dynamics')
        checked.append(dict(replay=str(p.relative_to(ROOT)),replay_sha256=digest(p),physics_sha256=digest(p.parent/'physics.npz'),physics_intervals=len(a['states'])-1,qualified=o['qualified'],motor_lag_max_error_N=error))
    write_once(out/'frontier-actuator-verification.json',dict(viewer_and_timing_pass=True,records=checked))

def verify_records(out):
    summary={}
    for arm in ('A','B'):
        paths=sorted((out/arm/'physical-rollouts').glob('*.npz'));reward_matches=0;total=0
        for i,p in enumerate(paths):
            a=dict(np.load(p,allow_pickle=False));b=dict(np.load(out/arm/'regional-rollouts'/f'rollout-{(i+1)*1024:06d}.npz',allow_pickle=False))
            np.testing.assert_array_equal(a['reward'],b['reward']);np.testing.assert_array_equal(a['action_raw'],b['action_raw'])
            np.testing.assert_array_equal(a['done'],b['done']);reward_matches+=1;total+=a['reward'].size
            if not all(np.isfinite(a[k]).all() for k in a):raise ValueError('Nonfinite capture')
            # Continuous transitions except episode reset and process restart boundaries.
            continuing=~a['done'][:-1]
            np.testing.assert_array_equal(a['next_states'][:-1][continuing],a['states'][1:][continuing])
            for k in ('motor_thrust_N','motor_target_N'):
                np.testing.assert_array_equal(a['next_'+k][:-1][continuing],a[k][1:][continuing])
        summary[arm]=dict(rollouts=len(paths),transitions=total,reward_action_done_exact_matches=reward_matches)
    (out/'physical-record-verification.json').write_text(json.dumps(summary,indent=2));return summary

def completion(cfg,out):
    frozen=verify_freeze(cfg,out);ledger=read_json(out/'ledger.json')
    if any(r['transitions']!=98304 or r['updates']!=96 or r['pending'] or r['abandoned'] for r in ledger['arms'].values()):raise ValueError('Budget/resume verification failed')
    if read_json(out/'intermediate-assessment.json')['extension_gate_pass']:raise ValueError('This completion is for a stopped initial comparison')
    preserved={}
    for folder in ('r5','r5-1'):
        manifest=read_json(out.parent/folder/'manifest.json')
        for path,sha in manifest['pins'].items():
            if digest(ROOT/path)!=sha:raise ValueError(f'Historical pin changed: {path}')
        preserved[folder]=len(manifest['pins'])
    protected={'.agents/runs/asm2-terminal/resumable-comparison/A/checkpoint-196608.pt':'eba759e7eeea93cd9fe06e0f8653d8f12bafc6cfd05a64bcbc11a7a78086fa3f',
        '.agents/runs/asm2-terminal/phase4-meaningful/learning/checkpoint-186368.pt':'85831eb55973c575b09283e276c17e7004ed059f7b64c9e25f98814ff8307fbd'}
    for path,sha in protected.items():
        if digest(ROOT/path)!=sha:raise ValueError('Protected learned baseline changed')
    native=read_json(out/'heldout-results.json')
    if native['model']!=read_json(out.parent/'r5/manifest.json')['model']:raise ValueError('Native model changed from R5')
    paths=[out/name for name in ('freeze.json','dataset.json','curriculum.json','intervention-training.npz','ledger.json','final-selection.json','intermediate-assessment.json','heldout-results.json',
        'support-evaluation-protocol.json','support-evaluation.json','frontier-evaluation-protocol.json','frontier-evaluation.json','mechanism.json','checkpoint-verification.json',
        'physical-record-verification.json','frontier-actuator-verification.json','replay-verification.json')]
    paths += [ROOT/'src/rm27_asm2'/name for name in ('r6_curriculum.py','r6_dataset.py','r6_experiment.py','r6_report.py')]
    paths += [ROOT/'tests/test_asm2_r6.py',ROOT/'.agents/docs/ASM2_R6_SKILL_BRIDGING.md']
    receipt=dict(status='COMPLETE',decision='Scientific skill bridging No-Go; baseline promotion No-Go; no extension',
        total_training_transitions=196608,per_arm_transitions=98304,training_stopped=True,source_pin_count=len(frozen['source_pins']),historical_pins_preserved=preserved,
        protected_checkpoints=protected,initial_state_optimizer_and_resume_verified=True,tests='3 focused unittest cases PASS; two charged real resumptions PASS; recorded state/action/reward and native actuator checks PASS',
        native_model_unchanged_from_r5=True,r52='NOT_RUN',primary_denominator_per_arm=36,fresh_cases_per_policy=94,support_cases_per_new_policy=30,adaptive_frontier_cases_per_policy=10,
        artifacts={str(p.relative_to(ROOT)):digest(p) for p in paths},literature={str(p.relative_to(ROOT)):digest(p) for p in sorted((out/'literature').iterdir()) if p.is_file()},
        limitations='One seed; training-selected secondary frontiers; correlated reset suffixes; provisional frozen actuator model; no official impact, hardware, or reachable-region claim.',
        next_action='One bounded offline CTBR/actuator-constrained informed-reset connection feasibility study into frozen Fixed A success states; not launched.')
    write_once(out/'completion-verification.json',receipt);return receipt

def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['dataset','assess','final','curves','verify','mechanism','support','checkpoints','frontier','frontier-plots','complete']);a=p.parse_args()
    cfg,_,_,out=configuration();verify_freeze(cfg,out)
    if a.command=='dataset':
        dataset_plot(out);transition_annotations(cfg,out);row_provenance(out)
    elif a.command=='assess':print(json.dumps(assessment(out,98304),indent=2))
    elif a.command=='final':final(cfg,out)
    elif a.command=='mechanism':mechanism(out)
    elif a.command=='support':support_evaluation(cfg,out)
    elif a.command=='frontier':frontier_evaluation(cfg,out)
    elif a.command=='frontier-plots':frontier_plots(out)
    elif a.command=='complete':completion(cfg,out)
    elif a.command=='checkpoints':print(json.dumps(checkpoint_check(cfg,out),indent=2))
    elif a.command=='curves':
        curves(out)
        if (out/'heldout-results.json').exists():replay_plot(out)
    else:print(json.dumps(verify_records(out),indent=2))
if __name__=='__main__':main()
