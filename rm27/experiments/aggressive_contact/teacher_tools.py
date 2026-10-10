"""Offline teacher dataset audit and interpretable operating-point figures."""
import json
from pathlib import Path

import numpy as np

from .evidence import write_json,snapshot
from .teacher_data import export,mechanism_metrics,read_trial,SCHEMA


def export_campaign(source,output):
    source,output=Path(source),Path(output);output.mkdir(parents=True,exist_ok=False)
    from .model import load_config
    snapshot(output,load_config())
    rows=[];groups={};errors=[]
    for split in ('design','validation','holdout'):
        for path in sorted((source/split).glob('**/summary.json')):
            if not (path.parent/'flight/trajectory.npz').exists():continue
            relative=path.parent.relative_to(source)
            teacher=path.parent.parent.name
            result=export(path.parent,output/relative,split,teacher)
            rows.append(result)
            group=result['scenario_group']
            if group in groups and groups[group]!=split:errors.append(f'split leakage: {group}')
            groups[group]=split
            data=np.load(Path(result['path'])/'demo.npz')
            if not np.all(np.diff(data['time_s'])>0):errors.append(f'clock: {relative}')
            for key in SCHEMA['actor_allowlist']:
                if SCHEMA['groups'][key]['privileged_only']:errors.append(f'actor leakage: {key}')
                if not np.all(np.isfinite(data[key])):errors.append(f'nonfinite: {relative}/{key}')
    write_json(output/'index.json',rows);write_json(output/'schema.json',SCHEMA)
    result=dict(passed=bool(rows) and not errors,demonstrations=len(rows),samples=sum(r['samples'] for r in rows),
                failures_retained=sum(not r['passed'] for r in rows),split_counts={k:sum(r['split']==k for r in rows) for k in groups.values()},
                actor_allowlist=SCHEMA['actor_allowlist'],errors=errors)
    write_json(output/'summary.json',result);return result


def plot_campaign(source,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from scipy.spatial.transform import Rotation
    from .teacher import stats
    source,output=Path(source),Path(output);output.mkdir(parents=True,exist_ok=False)
    study=json.loads((source/'study.json').read_text());margin=study['qualification_margin_m']
    selection=json.loads((source/'selection.json').read_text());summary=json.loads((source/'summary.json').read_text())
    methods=list(selection['methods']);colors=dict(zip(methods,plt.get_cmap('tab10').colors))
    results={};points=[]
    for split in ('small','stress'):
        results[split]={}
        for name in methods:
            rows=[]
            for p in sorted((source/'holdout'/split/name).glob('seed-*')):
                r=json.loads((p/'summary.json').read_text());r.update(mechanism_metrics(p));rows.append(r)
            results[split][name]=stats(rows,margin)
            for r in rows:points.append(dict(split=split,method=name,seed=r['seed'],passed=r['passed'],**mechanism_metrics(source/'holdout'/split/name/f'seed-{r["seed"]}')))
    write_json(output/'holdout_metrics.json',results);write_json(output/'mechanisms.json',points)
    fig,axs=plt.subplots(1,3,figsize=(15,4.5))
    for i,name in enumerate(methods):
        for split,mark in [('small','o'),('stress','^')]:
            r=results[split][name];x=r['time_to_confirmed_recovery_s']['median']
            axs[0].scatter(x,1000*r['contact_margin_m']['p05'],color=colors[name],marker=mark,label=f'{name} {split}')
            axs[1].scatter(x,r['qualified_successes']/r['trials'],color=colors[name],marker=mark)
            axs[2].scatter(x,r['effort_proxy_N32_s']['median'],color=colors[name],marker=mark)
    for ax in axs:ax.set_xlabel('Departure to held recovery (median s)');ax.grid(alpha=.3)
    axs[0].set_ylabel('Contact margin P05 (mm)');axs[0].axhline(8,color='gray',ls='--');axs[0].legend(fontsize=7)
    axs[1].set_ylabel('PASS + margin >= 8 mm (fraction)');axs[1].set_ylim(0,1.05)
    axs[2].set_ylabel('Motor effort proxy through recovery (N^1.5 s)')
    fig.tight_layout();fig.savefig(output/'operating_points.png',dpi=160);plt.close(fig)
    # Representative paired scenario. No cherry picking: first predeclared small seed.
    fig,axes=plt.subplots(4,2,figsize=(14,13))
    fig3=plt.figure(figsize=(10,6));a3=fig3.add_subplot(111,projection='3d')
    figphase,ap=plt.subplots(figsize=(12,3))
    for j,name in enumerate(methods):
        p=source/'holdout/small'/name/f'seed-{study["holdout_small_start"]}'
        _,s,c,ph,a=read_trial(p);x=a['trace'];t=x[:,0];end=s.get('time_to_confirmed_recovery_s') or t[-1];use=t<=end+1e-9;x=x[use];t=t[use]
        R=Rotation.from_quat(x[:,[5,6,7,4]]).as_matrix();v=x[:,8:11]
        tc=s['first_contact']['time_s'];kw=dict(color=colors[name],label=name,alpha=.85)
        axes[0,0].plot(x[:,1],x[:,3],**kw)
        axes[0,1].plot(t,np.linalg.norm(v,axis=1),**kw)
        axes[1,0].plot(t,np.linalg.norm(np.gradient(v,t,axis=0),axis=1),**kw)
        axes[1,1].plot(t,np.rad2deg(np.arccos(np.clip(R[:,2,2],-1,1))),**kw)
        axes[2,0].plot(t,np.linalg.norm(x[:,11:14],axis=1),**kw)
        axes[2,1].plot(t,x[:,14:18].sum(axis=1)*ph['draw']['thrust_scale'],**kw)
        local=(t>=tc-.7)&(t<=tc+.3)
        axes[3,0].plot(t[local]-tc,x[local,14:18].max(axis=1),**kw)
        n=np.array(ph['target']['normal']);n=n/np.linalg.norm(n)
        axes[3,1].plot(t[local]-tc,np.rad2deg(np.arccos(np.clip(R[local,:,2]@n,-1,1))),**kw)
        a3.plot(x[:,1],x[:,2],x[:,3],**kw)
        ap.barh(j,tc,color=colors[name],alpha=.8);ap.barh(j,end-tc,left=tc,color=colors[name],alpha=.3)
        ap.text(tc,j,f' contact {tc:.2f}',fontsize=7);ap.text(end,j,f' held {end:.2f}',fontsize=7)
    labels=[('World X (m)','Height (m)'),('Time (s)','Speed (m/s)'),('Time (s)','Acceleration including impact (m/s²)'),('Time (s)','Tilt (deg)'),('Time (s)','Body rate (rad/s)'),('Time (s)','Collective physical force (N)'),('Relative to contact (s)','Max motor state (N)'),('Relative to contact (s)','Thrust/target normal angle (deg)')]
    for ax,(xl,yl) in zip(axes.flat,labels):ax.set_xlabel(xl);ax.set_ylabel(yl);ax.grid(alpha=.3)
    axes[0,0].legend(fontsize=8);fig.tight_layout();fig.savefig(output/'mechanisms.png',dpi=150);plt.close(fig)
    a3.set(xlabel='World X (m)',ylabel='World Y (m)',zlabel='Height (m)');a3.legend();fig3.tight_layout();fig3.savefig(output/'trajectory3d.png',dpi=150);plt.close(fig3)
    ap.set_yticks(range(len(methods)),methods);ap.set_xlabel('Time (s); solid: approach, pale: contact to held recovery');figphase.tight_layout();figphase.savefig(output/'phase_timeline.png',dpi=160);plt.close(figphase)
    # Full design failure migration, including rejected candidates.
    candidates=json.loads((source/'candidates.json').read_text());design=[]
    for i,p in enumerate(candidates):
        rows=[json.loads(f.read_text()) for f in sorted((source/'design'/f'candidate-{i:02d}').glob('seed-*/summary.json'))]
        design.append(dict(candidate=i,planned_approach_s=sum(p['segment_times_s']),summary=stats(rows,margin)))
    write_json(output/'design_frontier.json',design)
    result=dict(passed=True,source=str(source),corrected_effort_scope='ends at confirmed recovery; original campaign metric included observation hold',figures=[p.name for p in output.glob('*.png')])
    write_json(output/'summary.json',result);return result
