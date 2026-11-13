"""Saved-data verification and figures for the bounded R6.1 connection study."""
from __future__ import annotations
import json
from collections import Counter
import numpy as np
from .asset import ROOT,digest,read_json,write_result
from .informed_connection import verified,match_report
from .backward_frontier import common
from .contact_task import ContactScorer
from .flight import allocate_ctbr
from .flight_viewer import load_recording

def analyze():
    c,out,m=verified();data=read_json(out/'results.json');rows=data['rows']
    if len(rows)!=20 or len(list((out/'cases').glob('*/started.json')))!=20:raise ValueError('Validation count mismatch')
    if {r['id'] for r in rows}!={r['id'] for r in m['candidates']}:raise ValueError('Candidate set changed')
    _,_,profile,spec,_,_,goal,command,limits=common();byid={r['id']:r for r in m['candidates']};families={f['source']:f for f in m['families']}
    checked=[]
    for row in rows:
        folder=out/'cases'/row['id'];a=dict(np.load(folder/'physics.npz',allow_pickle=False));sample=byid[row['id']];f=families[row['source']]
        for k,expected in [('states',sample['state']),('motor_thrust_N',sample['motor_thrust_N']),('motor_target_N',sample['motor_target_N'])]:np.testing.assert_array_equal(a[k][0],expected)
        base=np.asarray(f['start']['state']);axis,sign=c['variants'][sample['variant']]
        np.testing.assert_allclose(a['states'][0,:3]-base[:3],sign*.0005*spec.target_rotation[:,axis],atol=1e-16,rtol=0)
        np.testing.assert_allclose(a['states'][0,7:10]-base[7:10],sign*.005*spec.target_rotation[:,axis],atol=1e-15,rtol=0)
        np.testing.assert_array_equal(a['states'][0,3:7],base[3:7]);np.testing.assert_array_equal(a['states'][0,10:],base[10:])
        np.testing.assert_allclose(np.diff(a['sample_times_s']),spec.dt,atol=1e-12,rtol=0)
        alpha=-np.expm1(-spec.dt/profile.motor_time_constant_s)
        pred=a['motor_thrust_N'][:-1]+alpha*(a['motor_target_N'][1:]-a['motor_thrust_N'][:-1])
        error=float(np.max(abs(pred-a['motor_thrust_N'][1:])))
        if error>1e-12:raise ValueError('Motor dynamics changed')
        ticks=np.rint(a['control_times_s']/spec.dt).astype(int);cmd=a['ctbr_N_rad_s'];switch=row['switch_tick']
        if len(cmd):
            if np.max(abs(cmd[:,1:]))>2+1e-12 or np.any((cmd[:,0]<.5*profile.hover_thrust_N-1e-12)|(cmd[:,0]>1.5*profile.hover_thrust_N+1e-12)):raise ValueError('Actor CTBR bounds changed')
            allocated,sat=allocate_ctbr(profile,cmd,a['states'][ticks,10:13]);np.testing.assert_allclose(allocated,a['allocated_motor_target_N'],atol=1e-12,rtol=0)
        for k,v in [('states','transfer_state'),('motor_thrust_N','transfer_motors'),('motor_target_N','transfer_targets')]:np.testing.assert_array_equal(a[k][switch],row[v])
        suffix=np.flatnonzero(ticks>=switch) if row['fixed_a_run'] else []
        for j in suffix:np.testing.assert_array_equal(command(a['states'][ticks[j]]),cmd[j])
        if len(suffix) and ticks[suffix[0]]!=switch:raise ValueError('Handover not at preserved control boundary')
        match=match_report(a['states'][switch],a['motor_thrust_N'][switch],a['motor_target_N'][switch],f['target'],c['terminal_tolerances'])
        if match!=row['match']:raise ValueError('Full terminal match changed')
        opt=np.load(out/'optimization'/row['id']/'best-prefix.npz',allow_pickle=False)
        indices=np.rint(opt['sample_times_s']/spec.dt).astype(int)
        np.testing.assert_array_equal(a['states'][indices],opt['states'])
        contacts={r['tick']:r['records'] for r in read_json(folder/'contacts.json')};scorer=ContactScorer(spec)
        scorer.observe([],a['states'][0],0)
        for tick in range(1,len(a['states'])):
            result=scorer.observe(contacts.get(tick,[]),a['states'][tick-1],tick)
            if result is not None:break
        if scorer.finish()!=row['contact']:raise ValueError('Frozen native scorer replay differs')
        load_recording(folder/'replay.npz')
        checked.append(dict(id=row['id'],motor_error_N=error,physics_steps=len(a['states'])-1,suffix_policy_commands_verified=len(suffix),optimizer_prefix_exact=True,
            source_sha256=digest(folder/'physics.npz'),replay_sha256=digest(folder/'replay.npz')))
    aggregate={source:dict(categories=dict(Counter(r['category'] for r in rows if r['source']==source)),
        feasible_prefixes=sum(r['prefix_feasible'] for r in rows if r['source']==source),matched=sum(r['match']['passed'] for r in rows if r['source']==source),
        autonomous_qualified=sum(r['fixed_a_qualified'] for r in rows if r['source']==source)) for source in c['sources']}
    counts=Counter(r['category'] for r in rows);decision='A' if counts['A'] else 'B' if counts['B'] else 'C'
    optimization=[read_json(p) for p in sorted((out/'optimization').glob('*/result.json'))]
    report=dict(decision=decision,counts=dict(counts),sources=aggregate,validation_count=20,training_transitions=0,
        optimization_forward_evaluations=sum(r['forward_evaluations'] for r in optimization),optimization_total_worker_s=sum(r['wall_s'] for r in optimization),
        optimization_runs=len(optimization),prefix_feasible=sum(r['prefix_feasible'] for r in rows),terminal_matched=sum(r['match']['passed'] for r in rows),
        fixed_a_continuations=sum(r['fixed_a_run'] for r in rows),fixed_a_qualified=sum(r['fixed_a_qualified'] for r in rows),
        off_target_qualified=sum(r['fixed_a_qualified'] and not r['match']['passed'] for r in rows),checks=checked,
        scope='Five correlated source families, four numerical sensitivity starts each; per-case optimized controls. No certified reachable set, RL success, or common-control robustness claim.')
    if (out/'analysis.json').exists():
        if read_json(out/'analysis.json')!=report:raise ValueError('Analysis changed')
    else:write_result(out/'analysis.json',report)
    plots(out,m,rows,optimization)
    print(json.dumps({k:report[k] for k in ('decision','counts','sources','prefix_feasible','terminal_matched','fixed_a_continuations','optimization_forward_evaluations')},indent=2))

def plots(out,m,rows,optimization):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from matplotlib.ticker import MaxNLocator
    fig,axes=plt.subplots(1,2,figsize=(12,4),layout='constrained')
    for source in m['config']['sources']:
        r=next(x for x in optimization if x['source']==source and x.get('variant') is None)
        axes[0].semilogy([x['evaluation'] for x in r['history']],np.minimum.accumulate([x['cost'] for x in r['history']]),label=source)
    axes[0].set_xlabel('Detailed-plant forward evaluations');axes[0].set_ylabel('Best squared residual');axes[0].set_title('Nominal warm-start solves');axes[0].legend(fontsize=7)
    matrix=np.array([[r['match']['tolerance_ratios'][k] for k in ['position','velocity','attitude','body_rate','rotor','held_target']] for r in rows])
    im=axes[1].imshow(np.log10(np.maximum(matrix,1e-3)),aspect='auto',cmap='magma',vmin=-1,vmax=max(1,np.log10(matrix.max())))
    axes[1].set_xticks(range(6),['position','velocity','attitude','rate','rotors','held'],rotation=30);axes[1].set_yticks(range(len(rows)),[r['id'] for r in rows],fontsize=5)
    axes[1].set_title('log10(error / frozen tolerance); ≤0 passes');fig.colorbar(im,ax=axes[1]);fig.savefig(out/'matching-and-optimization.png',dpi=180);plt.close(fig)
    fig=plt.figure(figsize=(15,10),layout='constrained')
    for j,source in enumerate(m['config']['sources']):
        ax=fig.add_subplot(2,3,j+1,projection='3d');extent=[]
        for row in [r for r in rows if r['source']==source]:
            a=np.load(out/'cases'/row['id']/'replay.npz');x=a['states'][:,0,:3];extent.append(x)
            switch=np.searchsorted(a['sample_times_s'],row['switch_time_s'])
            line,=ax.plot(*x[:switch+1].T,label=f"{row['variant']}: {row['category']}")
            ax.scatter(*x[switch],marker='x',s=15,color=line.get_color())
            if row['fixed_a_run']:ax.plot(*x[switch:].T,ls='--',color=line.get_color())
        f=next(f for f in m['families'] if f['source']==source);target=np.array(f['target']['state'])[:3];extent.append(target[None]);ax.scatter(*target,marker='*',s=60,color='black')
        ax.add_collection3d(Poly3DCollection(a['target_vertices_m'][a['target_faces']],alpha=.15,facecolor='gray'));extent.append(a['target_vertices_m'])
        x=np.concatenate(extent);span=np.maximum(np.ptp(x,axis=0),.1);center=(x.max(axis=0)+x.min(axis=0))/2
        for setter,v,d in zip((ax.set_xlim,ax.set_ylim,ax.set_zlim),center,span):setter(v-.55*d,v+.55*d)
        ax.set_box_aspect(span,zoom=.78);ax.set_title(source);ax.legend(fontsize=6)
        for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.set_major_locator(MaxNLocator(3));axis.set_tick_params(labelsize=6)
        ax.set_xlabel('world x (m)',fontsize=7);ax.set_ylabel('world y (m)',fontsize=7);ax.set_zlabel('world z (m)',fontsize=7)
    fig.suptitle('Native trajectories: solid optimized prefix; dashed Fixed A; × prefix endpoint; ★ goal position')
    fig.savefig(out/'connections-3d.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(5,2,figsize=(12,12),layout='constrained')
    for j,source in enumerate(m['config']['sources']):
        row=next(r for r in rows if r['source']==source and r['variant']=='slope_minus');a=np.load(out/'cases'/row['id']/'physics.npz')
        for k in range(4):axes[j,0].plot(a['sample_times_s'],a['motor_thrust_N'][:,k]);axes[j,0].plot(a['sample_times_s'],a['motor_target_N'][:,k],':',alpha=.6)
        axes[j,0].set_title(source+' actual / dotted held motor thrust');axes[j,0].set_ylabel('N')
        axes[j,1].plot(a['sample_times_s'],np.linalg.norm(a['states'][:,7:10],axis=1),label='COM speed (m/s)')
        axes[j,1].plot(a['sample_times_s'],np.linalg.norm(a['states'][:,10:13],axis=1),label='body-rate norm (rad/s)')
        for ax in axes[j]:ax.axvline(row['switch_time_s'],color='black',ls='--');ax.set_xlabel('Time (s)');ax.grid(alpha=.2)
    axes[0,1].legend();fig.savefig(out/'actuator-and-state-timelines.png',dpi=150);plt.close(fig)

if __name__=='__main__':analyze()
