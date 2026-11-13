"""Saved-data verification and state-space analysis for the 20-case R5 study."""
from __future__ import annotations

import argparse
from collections import Counter
import json

import numpy as np

from .asset import ROOT,digest,read_json,write_result
from .backward_frontier import OUT,CASES,common
from .contact_task import ContactScorer
from .flight import allocate_ctbr
from .flight_suite import rotation_matrix
from .handover import flight_limits


def save(path,value):
    value=json.loads(json.dumps(value,allow_nan=False))
    if path.exists():
        if read_json(path)!=value:raise ValueError(f'Existing analysis differs: {path}')
    else:write_result(path,value)


def analyze():
    freeze=read_json(OUT/'manifest.json');results=read_json(OUT/'results.json')
    for p,h in freeze['pins'].items():
        if digest(ROOT/p)!=h:raise ValueError(f'Frozen input changed: {p}')
    manifest,old,profile,spec,path,source,goal,actor,limits=common()
    assert results['manifest_sha256']==digest(OUT/'manifest.json')
    assert results['completed_rollouts']==results['started_rollouts']==len(freeze['samples'])==20
    assert len(list((OUT/'cases').glob('*/started.json')))==20
    samples={s['id']:s for s in freeze['samples']};metrics=[];checks=[]
    for row in results['rows']:
        s=samples[row['id']];directory=OUT/'cases'/s['id'];outcome=row['result']
        assert row==read_json(directory/'outcome.json')
        for p,h in row['files'].items():assert digest(directory/p)==h
        with np.load(directory/'physics.npz') as archive:a=dict(archive)
        x,f,u,t=a['states'],a['motor_thrust_N'],a['motor_target_N'],a['sample_times_s']
        n=len(x)-1; assert n==row['physics_steps'] and len(t)==len(f)==len(u)==len(x)
        np.testing.assert_array_equal(x[0],s['state']);np.testing.assert_array_equal(f[0],s['motor_thrust_N'])
        np.testing.assert_array_equal(u[0],s['motor_target_N'])
        np.testing.assert_array_equal(t,np.arange(n+1)*spec.dt)
        assert t[-1]==outcome['end_s'] and np.isfinite(x).all() and np.isfinite(f).all()
        assert np.all((f>=0)&(f<=profile.max_rotor_thrust_N)) and np.all((u>=0)&(u<=profile.max_rotor_thrust_N))
        expected=u[1:]+(f[:-1]-u[1:])*np.exp(-spec.dt/profile.motor_time_constant_s)
        lag_error=float(np.max(abs(f[1:]-expected)))
        assert lag_error<1e-12
        control_ticks=np.rint(a['control_times_s']/spec.dt).astype(int)
        np.testing.assert_array_equal(control_ticks,np.arange(len(control_ticks))*50)
        replayed_commands=np.array([actor(x[k]) for k in control_ticks])
        np.testing.assert_array_equal(replayed_commands,a['ctbr_N_rad_s'])
        allocated,saturated=allocate_ctbr(profile,replayed_commands,x[control_ticks,10:13])
        np.testing.assert_allclose(allocated,a['allocated_motor_target_N'],atol=2e-15,rtol=0)
        assert int(saturated.sum())==outcome['saturation_controls']
        for j,k in enumerate(control_ticks):
            end=control_ticks[j+1] if j+1<len(control_ticks) else n
            np.testing.assert_array_equal(u[k+1:end+1],np.broadcast_to(a['allocated_motor_target_N'][j],(end-k,4)))
        assert np.max(abs(replayed_commands[:,1:]))<=2
        assert np.all((replayed_commands[:,0]>=.5*profile.hover_thrust_N)&(replayed_commands[:,0]<=1.5*profile.hover_thrust_N))
        records={c['tick']:c['records'] for c in read_json(directory/'contacts.json')}
        scorer=ContactScorer(spec);scorer.observe([],x[0],0)
        for k in range(1,n+1):scorer.observe(records.get(k,[]),x[k-1],k)
        assert scorer.finish()==outcome['contact']
        safety=flight_limits(x[-1],spec,limits)
        assert safety==outcome['safety_failure']
        assert outcome['qualified']==bool(scorer.result['qualified'] and safety is None)
        with np.load(directory/'replay.npz') as replay:
            ticks=np.rint(replay['sample_times_s']/spec.dt).astype(int)
            np.testing.assert_array_equal(replay['states'][:,0],x[ticks])
            np.testing.assert_array_equal(replay['ctbr_N_rad_s'][:,0],a['ctbr_N_rad_s'])
        p=x[:,:3]@spec.target_rotation;v=x[:,7:10]@spec.target_rotation
        r=rotation_matrix(x[:,3:7]);angle=np.rad2deg(np.arccos(np.clip(r[:,:,2]@spec.target_normal,-1,1)))
        speed=np.linalg.norm(v,axis=1);peak=int(np.argmax(speed));rates=np.linalg.norm(x[:,10:13],axis=1)
        last_touch=outcome['contact']['first_touch']
        base=dict(id=s['id'],case=s['case'],frame=s['frame'],variant=s['variant'],
            qualified=outcome['qualified'],reason=outcome['reason'],backward_ms=s['backward_s']*1000,
            source_braking_lead_ms=s['source_braking_lead_s']*1000,source_contact_lead_ms=s['source_contact_lead_s']*1000,
            autonomous_contact_decision_ms=outcome['end_s']*1000,
            native_first_touch_ms=None if not last_touch else last_touch['sample_time_s']*1000,
            initial_position_target_m=p[0].tolist(),initial_velocity_target_m_s=v[0].tolist(),
            initial_alignment_deg=float(angle[0]),initial_body_rates_rad_s=x[0,10:13].tolist(),
            initial_motor_thrust_N=f[0].tolist(),initial_motor_target_N=u[0].tolist(),
            motor_target_minus_actual_N=(u[0]-f[0]).tolist(),initial_clearance_mm=s['finite_registered_gap_m']*1000,
            initial_speed_m_s=float(speed[0]),peak_speed_m_s=float(speed[peak]),peak_speed_time_ms=float(t[peak]*1000),
            final_speed_m_s=float(speed[-1]),initial_rate_norm_rad_s=float(rates[0]),
            final_alignment_deg=float(angle[-1]),final_body_rate_norm_rad_s=float(rates[-1]),
            final_position_target_m=p[-1].tolist(),final_velocity_target_m_s=v[-1].tolist(),
            initially_recentering=bool(p[0,0]*v[0,0]<0),
            first_touch=last_touch)
        metrics.append(base);checks.append(dict(id=s['id'],physics_steps=n,motor_filter_max_error_N=lag_error,
            actor_commands_exact=True,allocation_verified=True,scorer_recomputed_exact=True,replay_exact=True))
    summary=[]
    for case in CASES:
        current=[r for r in metrics if r['case']==case];baseline=[r for r in old['rows'] if r['controller']=='fixed_a' and r['case']==case]
        old_nominal=min(r['frame'] for r in baseline if r['qualified'] and r['variant']=='nominal')
        new_qualified=[r['frame'] for r in current if r['qualified']]
        triplet=[r for r in current if r['frame']==min(c['frame'] for c in current)]
        original=current[0]['frame']+round(current[0]['backward_ms']/5)
        new_robust=min(r['frame'] for r in triplet) if all(r['qualified'] for r in triplet) else original
        nominal=[r for r in current if r['variant']=='nominal']
        ordered=sorted(nominal,key=lambda r:r['frame'])
        summary.append(dict(case=case,qualified=sum(r['qualified'] for r in current),total=4,
            old_robust_frame=original,earliest_new_qualified_frame=min(new_qualified) if new_qualified else None,
            old_earliest_nominal_qualified_frame=old_nominal,
            earliest_observed_nominal_qualified_frame=min([old_nominal]+[r['frame'] for r in nominal if r['qualified']]),
            new_triplet_qualifications=sum(r['qualified'] for r in triplet),new_triplet_frame=triplet[0]['frame'],
            observed_robust_frame=new_robust,robust_extension_ms=(original-new_robust)*5,
            sampled_nominal_pattern=[dict(frame=r['frame'],qualified=r['qualified']) for r in ordered],
            earliest_tested_nominal_succeeded=ordered[0]['qualified'],
            prior_nearest_earlier_nominal=[dict(frame=r['frame'],qualified=r['qualified'],reason=r['reason'])
                for r in baseline if r['variant']=='nominal' and r['frame']==max(
                    [q['frame'] for q in baseline if q['variant']=='nominal' and q['frame']<ordered[0]['frame']],default=-1)]))
    save(OUT/'analysis.json',dict(manifest_sha256=digest(OUT/'manifest.json'),results_sha256=digest(OUT/'results.json'),
        counts=dict(qualified=sum(r['qualified'] for r in metrics),total=20,failures=dict(Counter(r['reason'] for r in metrics if not r['qualified']))),
        sources=summary,states=metrics,new_rollouts=20,training_transitions=0,
        analysis_source_sha256=digest(ROOT/'src/rm27_asm2/backward_frontier_report.py')))
    save(OUT/'verification.json',dict(status='PASS',checks=checks,total_physics_steps=sum(c['physics_steps'] for c in checks),
        frozen_pin_count=len(freeze['pins']),new_rollouts=20,repeated_rollouts=0,offline_scoring_only=True,
        manifest_sha256=digest(OUT/'manifest.json'),results_sha256=digest(OUT/'results.json')))
    print(json.dumps(summary,indent=2))


def plots():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d=read_json(OUT/'analysis.json');rows=d['states']
    fig,ax=plt.subplots(1,3,figsize=(15,4.5),constrained_layout=True)
    for j,case in enumerate(CASES):
        selected=[r for r in rows if r['case']==case]
        for row in selected:
            shift={'nominal':0,'minus':-.14,'plus':.14}[row['variant']]
            ax[0].scatter(-row['backward_ms'],j+shift,c='#168264' if row['qualified'] else '#c3483b',marker='o' if row['variant']=='nominal' else '^',s=65)
        ax[0].scatter(0,j,c='#168264',marker='s',s=65)
    ax[0].set_yticks(range(5),CASES);ax[0].set_xlabel('Start time relative to old robust entry (ms)')
    ax[0].set_title('Squares: archived 3/3; circles: nominal\nTriangles: ± sensitivity; green pass/red fail')
    for case in CASES:
        rs=[r for r in rows if r['case']==case and r['variant']=='nominal'];rs.sort(key=lambda r:r['frame'])
        ax[1].plot([r['initial_position_target_m'][0] for r in rs],[r['initial_velocity_target_m_s'][0] for r in rs],alpha=.45)
        for r in rs:
            color='#168264' if r['qualified'] else '#c3483b'
            ax[1].scatter(r['initial_position_target_m'][0],r['initial_velocity_target_m_s'][0],c=color)
            ax[1].annotate(f"{case}:{r['frame']}",(r['initial_position_target_m'][0],r['initial_velocity_target_m_s'][0]),fontsize=6,xytext=(3,4),textcoords='offset points')
            ax[2].scatter(r['initial_alignment_deg'],r['initial_rate_norm_rad_s'],c=color)
    ax[1].set_xlabel('Target-slope COM position (m)');ax[1].set_ylabel('Target-slope velocity (m/s)')
    ax[2].set_xlabel('Initial alignment error (deg)');ax[2].set_ylabel('Initial body-rate norm (rad/s)')
    fig.suptitle('R5 frozen Fixed A: sampled local frontier, not a certified region')
    for ext in ['png','pdf']:fig.savefig(OUT/f'frontier-state-space.{ext}',dpi=180)
    plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(13,7),constrained_layout=True)
    from .contact_task import load_contact_spec
    spec=load_contact_spec()
    for case in CASES:
        row=min((r for r in rows if r['case']==case and r['variant']=='nominal'),key=lambda r:r['frame'])
        with np.load(OUT/'cases'/row['id']/'physics.npz') as a:
            x=a['states'];t=a['sample_times_s'];f=a['motor_thrust_N'];cmd=a['ctbr_N_rad_s'];ct=a['control_times_s']
        p=x[:,:3]@spec.target_rotation;v=x[:,7:10]@spec.target_rotation
        angle=np.rad2deg(np.arccos(np.clip(rotation_matrix(x[:,3:7])[:,:,2]@spec.target_normal,-1,1)))
        axes[0,0].plot(t,p[:,0],label=case);axes[0,1].plot(t,v[:,0]);axes[0,2].plot(t,angle)
        axes[1,0].plot(t,np.linalg.norm(x[:,10:13],axis=1));axes[1,1].plot(t,f.sum(1));axes[1,2].plot(ct,cmd[:,2])
    labels=['Slope position (m)','Slope velocity (m/s)','Alignment (deg)','Body-rate norm (rad/s)','Actual total rotor thrust (N)','Autonomous pitch-rate command (rad/s)']
    for a,label in zip(axes.flat,labels):a.set_ylabel(label);a.set_xlabel('Native continuation time (s)')
    axes[0,0].legend(fontsize=8);fig.suptitle('Earliest new nominal per source — all actor-generated trajectories')
    for ext in ['png','pdf']:fig.savefig(OUT/f'autonomous-transitions.{ext}',dpi=180)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('command',choices=['analyze','plots']);a=p.parse_args()
    {'analyze':analyze,'plots':plots}[a.command]()
