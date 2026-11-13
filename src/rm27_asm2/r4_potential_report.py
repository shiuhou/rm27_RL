"""Explain the frozen R4 rejection; no new candidate, rollout, or training."""
from __future__ import annotations

import argparse
import json

import mujoco
import numpy as np

from .asset import ROOT, digest, read_json
from .braking_potential import BrakingPotential
from .contact_native import NativeContact
from .flight_suite import rotation_matrix
from .r4_potential_audit import CONFIG, auc, inputs, save_once
from .terminal_native_eval import restore


def finite_registered_gap(model, data, aircraft, targets):
    """Numerical native convex-mesh separation with sphere lower-bound pruning.

    This is a static query at an archived contact-free state, not CCD or a
    reachability proof. The containing spheres only prune pairs whose possible
    minimum is above the best exact query. All 11,874 by 25 pairs are included.
    """
    aircraft, targets = np.asarray(aircraft), np.asarray(targets)
    delta = data.geom_xpos[aircraft, None] - data.geom_xpos[targets][None]
    lower = (np.linalg.norm(delta, axis=-1) - model.geom_rbound[aircraft, None]
             - model.geom_rbound[targets][None])
    order = np.argsort(lower, axis=None)
    # A realizable surface distance supplies an upper bound; start with a
    # cutoff greater than every containing-sphere farthest-point distance.
    best = float(np.max(np.linalg.norm(delta, axis=-1)
                        + model.geom_rbound[aircraft, None]+model.geom_rbound[targets][None])+1.)
    pair = None; queries = 0
    for flat in order:
        i, j = np.unravel_index(flat, lower.shape)
        if lower[i, j] > best + 1e-10:
            break
        value = mujoco.mj_geomDistance(model, data, int(aircraft[i]), int(targets[j]), best, None)
        queries += 1
        if value < best:
            best, pair = float(value), (int(aircraft[i]), int(targets[j]))
    if pair is None:
        raise RuntimeError('Finite geometry query did not establish an upper bound')
    return best, pair, queries


def mechanisms():
    cfg, manifest, rows, task, profile, spec, source_path, source = inputs()
    out = ROOT/cfg['output']; result = read_json(out/'potential-audit.json')
    support = read_json(out/'empirical-support.json'); frozen = read_json(out/'freeze.json')
    effective_pins=dict(frozen['pins'])
    correction=out/'freeze-correction.json'
    if correction.exists():
        amendment=read_json(correction)
        if amendment['freeze_sha256']!=digest(out/'freeze.json'):
            raise ValueError('Incorrect amendment parent')
        effective_pins.update(amendment['corrected_pins'])
    for path,sha in effective_pins.items():
        if digest(ROOT/path)!=sha:
            raise ValueError(f'Frozen R4 source changed: {path}')
    limits=read_json(ROOT/task['pins']['handover_limits']['path'])['flight_envelope']
    states=np.array([r['state'] for r in rows]); motors=np.array([r['motor_thrust_N'] for r in rows])
    labels=np.array([r['outcome']['qualified'] for r in rows]); cases=np.array([r['case'] for r in rows])
    nominal=np.array([r['variant']=='nominal' for r in rows]); end=np.array([r['outcome']['end_s'] for r in rows])
    vertices=np.array(support['body_envelope_vertices_m'])
    ungated=np.zeros(len(rows))
    # Diagnostic attribution only: do not change frozen gates or feed this
    # counterfactual score into a learner. Removing a guard isn't validation.
    for case in cfg['training_sources']:
        held=cases==case; use=labels & nominal & ~held
        p=BrakingPotential(states[use],motors[use],end[use],profile,spec,vertices,
            support['target_support_m'],limits,cfg['gamma'],cfg['potential_bound'])
        f=p.features(states[held],motors[held])
        d=np.sqrt(np.stack([np.sum((a[:,None]-b[None])**2,axis=-1) for a,b in zip(f,p.nodes)],axis=-1).mean(-1))
        ungated[held]=(p.bound*p.weights[None]*np.exp(-d)).max(1)

    valleys=[]
    for c in result['curves']:
        v=np.array(c['potential']); t=np.array(c['time_s']); delta=np.diff(v)
        start=peak=0; longest=(0,0); drop=(0.,0,0)
        for i in range(1,len(v)):
            if v[i-1]>=v[peak]:peak=i-1
            if v[peak]-v[i]>drop[0]:drop=(float(v[peak]-v[i]),peak,i)
            if delta[i-1]>=0:start=i
            if i-start>longest[1]-longest[0]:longest=(start,i)
        case_record=source['cases'][c['case']]
        trace=source_path.parent/case_record['trace']['path']
        with np.load(trace) as a:
            x=a['states'][:,0]; z=a['ctbr_N_rad_s'] if 'ctbr_N_rad_s' in a else None
            lo=int(a['handover_frame']); hi=lo+len(v)
            positive_commands=None
            if z is not None:
                command=z[lo:hi-1].reshape(-1,4)
                positive_commands=dict(max_abs_rate=float(np.max(abs(command[:,1:]))),
                    min_collective_hover=float(np.min(command[:,0])/profile.hover_thrust_N),
                    max_collective_hover=float(np.max(command[:,0])/profile.hover_thrust_N))
        selected=[lo+drop[1],lo+drop[2]]
        target_p=x[selected,:3] @ spec.target_rotation
        target_v=x[selected,7:10] @ spec.target_rotation
        alignment=np.rad2deg(np.arccos(np.clip(rotation_matrix(x[selected,3:7])[:,:,2] @ spec.target_normal,-1,1)))
        valleys.append(dict(case=c['case'],max_drawdown=drop[0],
            drawdown_start_s=float(t[drop[1]]),drawdown_end_s=float(t[drop[2]]),
            longest_decreasing_seconds=float(t[longest[1]]-t[longest[0]]),
            positive_discounted_shaping_fraction=float(np.mean(cfg['gamma']*v[1:]-v[:-1]>0)),
            all_potential_samples_positive=bool(np.all(v>0)),
            position_at_drawdown_endpoints_target_m=target_p.tolist(),
            velocity_at_drawdown_endpoints_target_m_s=target_v.tolist(),
            alignment_at_drawdown_endpoints_deg=alignment.tolist(),
            archived_witness_commands=positive_commands))

    # Only query the 15 known-positive states rejected by the plane bound.
    # They are not new continuations. The frozen candidate remains rejected.
    selection=[i for i,r in enumerate(result['scores']) if r['qualified'] and r['potential']==0]
    plant=NativeContact(profile,spec)
    if plant.metadata!=manifest['model']:
        raise ValueError('Native model changed')
    aircraft=[plant.model.geom(n).id for n in spec.entries]
    targets=[plant.model.geom(n).id for n in sorted(spec.legal_targets)]
    gaps=[]
    for i in selection:
        r=rows[i]
        contacts=restore(plant,states[i],motors[i],np.array(r['motor_target_N']))
        gap,pair,queries=finite_registered_gap(plant.model,plant.data,aircraft,targets)
        gaps.append(dict(case=r['case'],frame=r['frame'],variant=r['variant'],
            numerical_finite_registered_gap_m=gap,
            closest_pair=[plant.model.geom(g).name for g in pair],
            narrowphase_queries=queries,
            signed_contacts_all_field=sum(c['distance_m']<=0 for c in contacts),
            separating_plane_bound_m=result['geometry'][i]['target_separation_lower_bound_m']))
    save_once(out/'mechanism-analysis.json',dict(
        audit_sha256=digest(out/'potential-audit.json'),
        implementation_sha256=digest(ROOT/'src/rm27_asm2/r4_potential_report.py'),
        ungated_source_excluded_auc=auc(labels,ungated),
        ungated_scores=ungated.tolist(),valleys=valleys,finite_geometry_queries=gaps,
        new_native_continuations=0,new_physics_steps=0,
        interpretation='Ungated scores are attribution only, not a replacement candidate. The early potential valleys have positive clearance throughout and persist independently of the near-contact clearance-mask failure.'))
    print(json.dumps(dict(ungated_auc=auc(labels,ungated),valleys=valleys,finite_gap_min=min(r['numerical_finite_registered_gap_m'] for r in gaps),
                         finite_gap_max=max(r['numerical_finite_registered_gap_m'] for r in gaps)),indent=2))


def figures():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cfg=read_json(CONFIG);out=ROOT/cfg['output'];d=read_json(out/'potential-audit.json');m=read_json(out/'mechanism-analysis.json')
    fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
    for c in d['curves']:
        v=np.array(c['potential']);t=np.array(c['time_s'])
        # 50 ms bin averages make the sign trend legible, not a reward change.
        r=cfg['gamma']*v[1:]-v[:-1]; n=len(r)//10
        axes[0].plot(t[1:n*10+1].reshape(n,10).mean(1),r[:n*10].reshape(n,10).mean(1),label=c['case'])
    axes[0].axhline(0,color='black',linewidth=.7);axes[0].set_xlabel('Seconds from saved handover')
    axes[0].set_ylabel('Shaping / 5 ms step (50 ms bin means)');axes[0].legend(fontsize=8)
    rows=m['finite_geometry_queries']; ids=np.arange(len(rows))
    axes[1].scatter(ids,[r['separating_plane_bound_m']*1000 for r in rows],label='Conservative plane bound',marker='x')
    axes[1].scatter(ids,[r['numerical_finite_registered_gap_m']*1000 for r in rows],label='Finite detailed-mesh distance',s=20)
    axes[1].axhline(0,color='black',linewidth=.7);axes[1].set_xlabel('15 archived qualified near-contact starts')
    axes[1].set_ylabel('Separation (mm)');axes[1].legend(fontsize=8)
    fig.suptitle('R4: early directional failure is separate from clearance false exclusions')
    for ext in ['png','pdf']:fig.savefig(out/f'failure-mechanisms.{ext}',dpi=170)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('command',choices=['mechanisms','figures']);a=p.parse_args()
    {'mechanisms':mechanisms,'figures':figures}[a.command]()
