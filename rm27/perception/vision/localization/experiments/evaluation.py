"""Conditional trajectory/behavior metrics; missing evidence stays unavailable."""
import math
import numpy as np

from .benchmark import timing_stats
from .common import require, number
from .trajectory import pose_matrix


def behavior_metrics(frames, records, benchmark, events=None):
    result=dict(input_frame_count=len(frames), processed_frame_count=benchmark.get('processed_frame_count'),
                emitted_pose_count=len(records), emitted_pose_fraction=len(records)/len(frames),
                tracking_coverage=None, initialization_frame=None, initialization_time_s=None,
                lost_intervals=None,relocalization_count=None,reset_events=None,map_change_events=None,
                processing_time_statistics=benchmark.get('processing_time_statistics'),
                wall_clock_s=benchmark.get('wall_clock_s'),failure_reason=benchmark.get('failure_reason'),
                state_evidence='UNAVAILABLE', ground_truth_status='NO_GROUND_TRUTH', trajectory_metrics=None,
                note='final/keyframe pose availability is not online tracking coverage')
    if events is None:
        return result
    known={f.source_frame_id:f for f in frames}
    order=[e['source_frame_id'] for e in events]
    require(order==sorted(set(order)) and set(order)<=set(known),'EVENT_ORDER_INVALID','state observations')
    for event in events:
        require(event.get('tracking_state') in ('UNINITIALIZED','INITIALIZING','TRACKING','LOST','RELOCALIZING','RESET'),
                'EVENT_STATE_INVALID','unknown states must remain absent')
    complete=set(order)==set(known)
    tracking=[e for e in events if e['tracking_state']=='TRACKING']
    # Partial state logs cannot establish intervals/whole-input coverage.
    result.update(state_evidence='COMPLETE' if complete else 'PARTIAL',
                  processed_frame_count=len(events),
                  tracking_coverage=len(tracking)/len(frames) if complete else None)
    if complete:
        if tracking:
            first=known[tracking[0]['source_frame_id']]
            result.update(initialization_frame=first.source_frame_id,initialization_time_s=float(first.encoded_pts))
        lost=[]
        current=[]
        for event in events:
            if event['tracking_state']=='LOST':
                current.append(event['source_frame_id'])
            elif current:
                lost.append({'first_frame':current[0],'last_frame':current[-1], 'frame_count':len(current)})
                current=[]
        if current:
            lost.append({'first_frame':current[0],'last_frame':current[-1], 'frame_count':len(current)})
        result['lost_intervals']=lost
        result['relocalization_count']=sum(e['tracking_state']=='RELOCALIZING' and (i==0 or events[i-1]['tracking_state']!='RELOCALIZING') for i,e in enumerate(events))
        result['reset_events']=[e for e in events if e['tracking_state']=='RESET']
        result['map_change_events']=[e for e in events if e.get('map_event') is not None]
    return result


def trajectory_metrics(estimated, reference, alignment='Sim3', *, continuity_verified=False, reference_unit='m'):
    """One global Umeyama alignment; adjacent matched-pose RPE, no segment rescale."""
    require(alignment in ('Sim3','SE3'), 'ALIGNMENT_INVALID', alignment)
    require(continuity_verified, 'CONTINUITY_UNVERIFIED', 'reference and estimate must share one continuous coordinate frame each')
    require(reference_unit=='m','REFERENCE_UNIT_UNSUPPORTED','expected metric reference')
    def indexed(records):
        require(len({r['source_frame_id'] for r in records})==len(records),'TRAJECTORY_FRAME_DUPLICATE','evaluation')
        return {r['source_frame_id']:r for r in records}
    est,ref=indexed(estimated),indexed(reference)
    ids=sorted(set(est)&set(ref))
    require(len(ids)>=3,'REFERENCE_MATCHES_INSUFFICIENT','at least three noncollinear matches')
    require(all(abs(float(est[i]['encoded_pts'])-float(ref[i]['encoded_pts'])) <= 1e-6 for i in ids),
            'REFERENCE_TIME_MISMATCH','matched frame IDs need matching original timestamps')
    for sequence in (estimated,reference):
        for field in ('parent_frame','child_frame','localization_epoch','map_id'):
            values={r['localization_fields'].get(field) for r in sequence}
            require(len(values)==1,'CONTINUITY_UNVERIFIED','multiple coordinate frames/epochs cannot share an alignment')
    variants={r['trajectory_variant'] for r in estimated}
    require(len(variants)==1,'TRAJECTORY_VARIANT_MIXED','online and final cannot be mixed')
    e=np.asarray([pose_matrix(est[i]) for i in ids]); g=np.asarray([pose_matrix(ref[i]) for i in ids])
    x,y=e[:,:3,3],g[:,:3,3]
    xc,yc=x-x.mean(axis=0),y-y.mean(axis=0)
    require(np.linalg.matrix_rank(xc)>=2 and np.linalg.matrix_rank(yc)>=2,'ALIGNMENT_DEGENERATE','collinear/static trajectory')
    u,s,vh=np.linalg.svd(yc.T@xc/len(ids))
    signs=np.eye(3); signs[2,2]=np.linalg.det(u@vh)
    rotation=u@signs@vh
    scale=float(np.sum(s*np.diag(signs))/np.mean(np.sum(xc*xc,axis=1))) if alignment=='Sim3' else 1.0
    require(math.isfinite(scale) and scale>0,'ALIGNMENT_INVALID','nonpositive scale')
    translation=y.mean(axis=0)-scale*rotation@x.mean(axis=0)
    aligned=e.copy()
    aligned[:,:3,3]=(scale*(rotation@x.T)).T+translation
    aligned[:,:3,:3]=rotation@e[:,:3,:3]
    errors=np.linalg.norm(aligned[:,:3,3]-y,axis=1)
    rpe_t,rpe_r=[],[]
    for i in range(len(ids)-1):
        delta_e=np.linalg.inv(aligned[i])@aligned[i+1]
        delta_g=np.linalg.inv(g[i])@g[i+1]
        error=np.linalg.inv(delta_g)@delta_e
        rpe_t.append(float(np.linalg.norm(error[:3,3])))
        rpe_r.append(float(np.arccos(np.clip((np.trace(error[:3,:3])-1)/2,-1,1))))
    times=[float(est[i]['encoded_pts']) for i in ids]
    require(all(b>a for a,b in zip(times,times[1:])),'TRAJECTORY_ORDER_INVALID','evaluation times')
    return dict(status='EVALUATED_WITH_REFERENCE', trajectory_variant=next(iter(variants)), matched_frames=len(ids),
                unmatched_estimate_frames=len(est)-len(ids),unmatched_reference_frames=len(ref)-len(ids),
                alignment=alignment,global_scale=scale,rotation=rotation.tolist(),translation=translation.tolist(),
                singular_values=s.tolist(),rank=int(np.linalg.matrix_rank(xc)),
                ate_translation_rmse_m=float(np.sqrt(np.mean(errors**2))),
                rpe_translation_rmse_m=float(np.sqrt(np.mean(np.square(rpe_t)))),
                rpe_rotation_rmse_rad=float(np.sqrt(np.mean(np.square(rpe_r)))),
                rpe_pair_policy='adjacent matched source frames; gaps are explicitly listed',
                rpe_frame_pairs=list(zip(ids[:-1],ids[1:])),duration_s=times[-1]-times[0],
                estimated_aligned_distance_m=float(np.linalg.norm(np.diff(aligned[:,:3,3],axis=0),axis=1).sum()),
                reference_distance_m=float(np.linalg.norm(np.diff(y,axis=0),axis=1).sum()),
                conclusion_scope='trajectory shape after one global scale alignment; not metric scale accuracy' if alignment=='Sim3' else 'rigid alignment without scale correction')


def evaluate_reference(run_path, reference_path, continuity_path, variant, alignment, *, _fixture_token=None):
    """Explicit reference and continuity evidence required; never repairs resets."""
    from pathlib import Path
    from .common import read_json, checked_file, fixture_mode
    root=Path(run_path).resolve().parent
    run=read_json(run_path)
    reference=read_json(reference_path)
    continuity=read_json(continuity_path)
    fixture_mode(_fixture_token,run,reference,continuity)
    require(variant in ('online','final_optimized'),'TRAJECTORY_VARIANT_INVALID',variant)
    key='online_trajectory' if variant=='online' else 'final_optimized_trajectory'
    require(run.get(key) not in (None,'UNAVAILABLE'),'TRAJECTORY_UNAVAILABLE',variant)
    normalized_path,normalized_hash=checked_file(root/run[key])
    _,reference_hash=checked_file(reference_path)
    trajectory=read_json(normalized_path)
    require(trajectory['trajectory_variant']==variant and
            all(r.get('trajectory_variant')==variant for r in trajectory['records']),
            'TRAJECTORY_VARIANT_MIXED','artifact header and individual records')
    require(reference.get('reference_status')=='VERIFIED' and reference.get('timestamp_association')=='VERIFIED',
            'REFERENCE_UNVERIFIED','reference and frame/time association must be established')
    require(reference.get('dataset_id')==run['dataset_id'] and
            reference.get('dataset_manifest_sha256')==run['comparison_identity']['dataset_manifest_sha256'],
            'REFERENCE_DATASET_MISMATCH','reference association identity')
    require(continuity.get('status')=='VERIFIED_SINGLE_COORDINATE_FRAME' and
            continuity.get('run_config_sha256')==run['config_sha256'] and
            continuity.get('normalized_sha256')==normalized_hash and continuity.get('reference_sha256')==reference_hash,
            'CONTINUITY_UNVERIFIED','hashed evidence for one frame across entire evaluated trajectory required')
    records=trajectory['records']
    if variant=='online':
        records=[r for r in records if r.get('validity') is True and r.get('localization_fields') is not None]
    metrics=trajectory_metrics(records,reference['records'],alignment,
                               continuity_verified=True,reference_unit=reference.get('translation_unit'))
    return dict(schema_version=1,purpose=run['purpose'],run_id=run['run_id'],run_config_sha256=run['config_sha256'],
                normalized_sha256=normalized_hash,reference_sha256=reference_hash,
                trajectory_variant=variant,excluded_invalid_online_records=len(trajectory['records'])-len(records),
                ground_truth_status='WITH_VERIFIED_REFERENCE',trajectory_metrics=metrics)


def main():
    import argparse
    from pathlib import Path
    from .common import write_json
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--reference',type=Path,required=True)
    parser.add_argument('--continuity',type=Path,required=True)
    parser.add_argument('--variant',choices=['online','final_optimized'],required=True)
    parser.add_argument('--alignment',choices=['Sim3','SE3'],required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    report=evaluate_reference(args.run,args.reference,args.continuity,args.variant,args.alignment)
    args.out.mkdir(parents=True,exist_ok=False)
    write_json(args.out/'reference_evaluation.json',report)


if __name__=='__main__':
    main()
