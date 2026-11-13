"""R5.1 matched analysis and saved-data validation; never advances physics."""
from __future__ import annotations
import argparse
from collections import Counter
import json
import numpy as np
from .asset import ROOT,read_json,digest
from .cross_track_validation import OUT,PARENT,ANCHORS,perturb
from .backward_frontier import common
from .backward_frontier_report import save
from .contact_task import ContactScorer
from .flight import allocate_ctbr
from .flight_viewer import load_recording
from .handover import flight_limits


def analyze():
    frozen=read_json(OUT/'manifest.json');data=read_json(OUT/'results.json')
    for p,h in frozen['pins'].items():assert digest(ROOT/p)==h,p
    assert data['manifest_sha256']==digest(OUT/'manifest.json')
    assert data['completed_rollouts']==data['started_rollouts']==10
    assert len(list((OUT/'cases').glob('*/started.json')))==10
    _,_,profile,spec,_,_,_,actor,limits=common()
    parent=read_json(PARENT/'manifest.json');states={s['id']:s for s in frozen['samples']}
    checks=[];rows=[]
    for trial in data['rows']:
        s=states[trial['id']];folder=OUT/'cases'/s['id'];result=trial['result']
        nominal=next(p for p in parent['samples'] if p['id']==s['nominal_id'])
        expected,_=perturb(nominal['state'],spec.target_rotation,-1 if s['variant']=='across_minus' else 1)
        np.testing.assert_array_equal(expected,s['state'])
        for key in ['motor_thrust_N','motor_target_N']:np.testing.assert_array_equal(nominal[key],s[key])
        for name,sha in trial['files'].items():assert digest(folder/name)==sha
        with np.load(folder/'physics.npz') as z:a=dict(z)
        x,f,u,t=a['states'],a['motor_thrust_N'],a['motor_target_N'],a['sample_times_s']
        n=len(x)-1;assert n==trial['physics_steps'];assert t[-1]==result['end_s']
        np.testing.assert_array_equal(x[0],s['state']);np.testing.assert_array_equal(f[0],s['motor_thrust_N'])
        np.testing.assert_array_equal(u[0],s['motor_target_N']);np.testing.assert_array_equal(t,np.arange(n+1)*spec.dt)
        assert np.isfinite(x).all() and np.isfinite(f).all() and np.isfinite(u).all()
        assert np.all((f>=0)&(f<=profile.max_rotor_thrust_N)) and np.all((u>=0)&(u<=profile.max_rotor_thrust_N))
        predicted=u[1:]+(f[:-1]-u[1:])*np.exp(-spec.dt/profile.motor_time_constant_s)
        residual=float(np.max(abs(predicted-f[1:])));assert residual<1e-12
        ticks=np.rint(a['control_times_s']/spec.dt).astype(int)
        np.testing.assert_array_equal(ticks,np.arange(len(ticks))*50)
        commands=np.array([actor(x[k]) for k in ticks])
        np.testing.assert_array_equal(commands,a['ctbr_N_rad_s'])
        allocations,sat=allocate_ctbr(profile,commands,x[ticks,10:13])
        np.testing.assert_allclose(allocations,a['allocated_motor_target_N'],atol=2e-15,rtol=0)
        assert int(sat.sum())==result['saturation_controls']
        for j,k in enumerate(ticks):
            end=ticks[j+1] if j+1<len(ticks) else n
            np.testing.assert_array_equal(u[k+1:end+1],np.broadcast_to(a['allocated_motor_target_N'][j],(end-k,4)))
        assert np.max(abs(commands[:,1:]))<=2
        assert np.all((commands[:,0]>=profile.hover_thrust_N*.5)&(commands[:,0]<=profile.hover_thrust_N*1.5))
        contacts={r['tick']:r['records'] for r in read_json(folder/'contacts.json')}
        scorer=ContactScorer(spec);scorer.observe([],x[0],0)
        for k in range(1,n+1):scorer.observe(contacts.get(k,[]),x[k-1],k)
        assert scorer.finish()==result['contact']
        assert flight_limits(x[-1],spec,limits)==result['safety_failure']
        assert result['qualified']==bool(scorer.result['qualified'] and result['safety_failure'] is None)
        with np.load(folder/'replay.npz') as z:
            sample=np.rint(z['sample_times_s']/spec.dt).astype(int)
            np.testing.assert_array_equal(z['states'][:,0],x[sample])
            np.testing.assert_array_equal(z['ctbr_N_rad_s'][:,0],commands)
        load_recording(folder/'replay.npz')
        # Native envelope checks are at controller boundaries; additionally
        # report any between-boundary violation observed in the saved samples.
        envelope_counts=Counter(filter(None,(flight_limits(v,spec,limits) for v in x)))
        checks.append(dict(id=s['id'],steps=n,motor_filter_max_error_N=residual,actor_commands_exact=True,
            scorer_exact=True,allocation_verified=True,replay_exact=True,initial_invariants_exact=True,
            saved_physics_envelope_violations=dict(envelope_counts)))
    for case,frame in ANCHORS.items():
        nominal=next(p for p in parent['samples'] if p['case']==case and p['frame']==frame and p['variant']=='nominal')
        entries=[(PARENT,nominal,'nominal_reused')]+[(OUT,s,s['variant']) for s in frozen['samples'] if s['case']==case]
        for root,s,label in entries:
            outcome=read_json(root/'cases'/s['id']/'outcome.json')['result'];touch=outcome['contact']['first_touch']
            points=[] if touch is None else touch['records']
            with np.load(root/'cases'/s['id']/'physics.npz') as z:
                trajectory=z['states'][:,:3]@spec.target_rotation
            rows.append(dict(case=case,frame=frame,variant=label,id=s['id'],qualified=outcome['qualified'],
                reason=outcome['reason'],flight_envelope_failure=outcome['safety_failure'],saturated_controls=outcome['saturation_controls'],
                initial_clearance_mm=s['finite_registered_gap_m']*1000,
                initial_position_target_m=(np.array(s['state'][:3])@spec.target_rotation).tolist(),
                initial_velocity_target_m_s=(np.array(s['state'][7:10])@spec.target_rotation).tolist(),
                decision_ms=outcome['end_s']*1000,first_contact_ms=None if touch is None else touch['sample_time_s']*1000,
                groups=[] if touch is None else touch['groups'],
                first_contact_pairs=sorted({(p['geom'],p['other_geom']) for p in points}),
                contact_position_target_min_m=None if not points else np.min([p['target_position_m'] for p in points],axis=0).tolist(),
                contact_position_target_max_m=None if not points else np.max([p['target_position_m'] for p in points],axis=0).tolist(),
                min_normal_approach_speed_m_s=None if not points else min(p['normal_approach_speed_m_s'] for p in points),
                max_tangential_speed_m_s=None if not points else max(p['tangential_speed_m_s'] for p in points),
                final_com_target_m=trajectory[-1].tolist(),new_rollout=root==OUT))
    groups=[]
    for case,frame in ANCHORS.items():
        matched=[r for r in rows if r['case']==case]
        prior_slope=[read_json(PARENT/'cases'/s['id']/'outcome.json')['result']['qualified']
                     for s in parent['samples'] if s['case']==case and s['frame']==frame and s['variant'] in ('minus','plus')]
        groups.append(dict(case=case,frame=frame,nominal=matched[0]['qualified'],
            across_negative=matched[1]['qualified'],across_positive=matched[2]['qualified'],
            both_across_pass=all(r['qualified'] for r in matched),
            same_anchor_slope_variants=prior_slope,
            both_tangent_directions_sampled_successfully=len(prior_slope)==2 and all(prior_slope) and all(r['qualified'] for r in matched)))
    new=[r for r in rows if r['new_rollout']]
    all_across=all(r['qualified'] for r in new)
    conclusion=('LOCALLY_TWO_DIRECTIONAL_SAMPLED_SUCCESS_SUPPORT' if all_across
                else 'MIXED_OR_UNRESOLVED_SENSITIVITY' if any(r['qualified'] for r in new)
                else 'NARROW_DIRECTION_DEPENDENT_SUCCESS_AT_TESTED_OFFSETS')
    save(OUT/'analysis.json',dict(manifest_sha256=digest(OUT/'manifest.json'),results_sha256=digest(OUT/'results.json'),
        implementation_sha256=digest(ROOT/'src/rm27_asm2/cross_track_report.py'),
        conclusion=conclusion,new_qualified=sum(r['qualified'] for r in new),new_rollouts=10,reused_nominals=5,
        two_sided_across_anchors=sum(r['both_across_pass'] for r in groups),
        both_tangent_directions_supported_anchors=sum(r['both_tangent_directions_sampled_successfully'] for r in groups),
        exact_failures=dict(Counter(r['reason'] for r in new if not r['qualified'])),anchors=groups,rows=rows,
        limitations='Two coupled position/velocity directions, not independent coordinate robustness; endpoints do not certify interiors, diagonal interactions, or a full-dimensional reachable set. Stationary_0:330 lacks slope variants at the same anchor.'))
    save(OUT/'verification.json',dict(status='PASS',frozen_pin_count=len(frozen['pins']),checks=checks,
        total_physics_steps=sum(c['steps'] for c in checks),new_rollouts=10,repeated_nominals=0,
        verification_physics_rollouts=0,training_transitions=0,manifest_sha256=digest(OUT/'manifest.json')))
    print(json.dumps(dict(conclusion=conclusion,anchors=groups,qualified=sum(r['qualified'] for r in new)),indent=2))


def plots():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d=read_json(OUT/'analysis.json')
    labels=['across_minus','nominal_reused','across_plus']
    matrix=np.array([[next(r['qualified'] for r in d['rows'] if r['case']==c and r['variant']==v)
                     for v in labels] for c in ANCHORS],int)
    from matplotlib.colors import ListedColormap
    fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
    axes[0].imshow(matrix,vmin=0,vmax=1,cmap=ListedColormap(['#c85a4e','#67b494']),aspect='auto')
    axes[0].set_xticks(range(3),['Across −','Nominal (R5)','Across +'])
    axes[0].set_yticks(range(5),[f'{c}:{f}' for c,f in ANCHORS.items()])
    for i,c in enumerate(ANCHORS):
        for j,v in enumerate(labels):
            row=next(r for r in d['rows'] if r['case']==c and r['variant']==v)
            text='QUALIFIED' if row['qualified'] else row['reason'].replace('FORBIDDEN_AIRFRAME_FIRST_TOUCH','BODY CONTACT').replace('OUTSIDE_REGISTERED_FOOTPRINT','FOOTPRINT MISS')
            axes[0].text(j,i,f"{text}\n{row['decision_ms']:.1f} ms",ha='center',va='center',fontsize=8)
    for i,(c,f) in enumerate(ANCHORS.items()):
        for v,marker in zip(labels,['v','o','^']):
            row=next(r for r in d['rows'] if r['case']==c and r['variant']==v)
            low=row['contact_position_target_min_m'];high=row['contact_position_target_max_m']
            if low is None:continue
            axes[1].plot([low[0]*1000,high[0]*1000],[low[1]*1000,high[1]*1000],c=f'C{i}',alpha=.5)
            axes[1].scatter((low[0]+high[0])*500,(low[1]+high[1])*500,marker=marker,
                facecolors=f'C{i}' if row['qualified'] else 'none',edgecolors=f'C{i}',label=f'{c}' if v==labels[1] else None)
    from .contact_task import load_contact_spec
    spec=load_contact_spec();hs=spec.region_halfsize_m*1000
    axes[1].plot([-hs[0],hs[0],hs[0],-hs[0],-hs[0]],[-hs[1],-hs[1],hs[1],hs[1],-hs[1]],'k--',lw=1)
    axes[1].set_xlabel('First manifold slope position (mm)');axes[1].set_ylabel('Across position (mm)')
    axes[1].set_title('Manifold bounding-range centers; hollow = rejected')
    axes[1].legend(fontsize=7);fig.suptitle('R5.1: ten new transverse variants, five reused nominal controls')
    for ext in ['png','pdf']:
        path=OUT/f'cross-track-outcomes.{ext}'
        if not path.exists():fig.savefig(path,dpi=180)
    plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(13,7),constrained_layout=True)
    for ax,(c,f) in zip(axes.flat,ANCHORS.items()):
        for label,color in zip(['nominal_reused','across_minus','across_plus'],['black','#377eb8','#d95f02']):
            row=next(r for r in d['rows'] if r['case']==c and r['variant']==label)
            folder=(PARENT if label=='nominal_reused' else OUT)/'cases'/row['id']
            with np.load(folder/'physics.npz') as a:
                across=a['states'][:,:3]@spec.target_rotation[:,1];t=a['sample_times_s']
            ax.plot(t*1000,across*1000,color=color,label=label)
        ax.set_title(f'{c}:{f}');ax.set_xlabel('Time (ms)');ax.set_ylabel('COM across target (mm)')
    axes.flat[-1].axis('off');axes.flat[0].legend(fontsize=7)
    fig.suptitle('Matched autonomous transverse trajectories')
    for ext in ['png','pdf']:
        path=OUT/f'cross-track-trajectories.{ext}'
        if not path.exists():fig.savefig(path,dpi=180)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('command',choices=['analyze','plots']);a=p.parse_args()
    {'analyze':analyze,'plots':plots}[a.command]()
