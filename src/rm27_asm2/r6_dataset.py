"""Index existing autonomous transitions and construct valid R6 reset proposals."""
from __future__ import annotations
import json
import numpy as np
from .asset import ROOT, digest, read_json, write_result
from .backward_frontier import CASES, common
from . import terminal_native_eval as native
from .handover_curriculum import native_verifier

OUT=ROOT/'.agents/runs/asm2-terminal/r6'
ANCHORS=dict(zip(CASES,[725,816,702,330,346]))

def build():
    if (OUT/'dataset.json').exists(): raise FileExistsError('R6 dataset already built')
    OUT.mkdir(exist_ok=True)
    manifest,old,profile,spec,path,source,goal,command,limits=common()
    base=dict(np.load(ROOT/'.agents/runs/asm2-terminal/r3/training.npz',allow_pickle=False))
    keys=[k for k,v in base.items() if v.ndim]
    rows=[]; index=[]; pins={}; support=[[] for _ in CASES]; chains=[]
    def add(state, thrust, target, case, frame, time):
        row=dict(states=state,motor_thrust_N=thrust,motor_target_N=target,
                 case_ids=case,source_frame=frame,sample_times_s=time,phase='rotation',band='rotation_bridge')
        rows.append(row);return len(base['states'])+len(rows)-1
    # Preserve complete records by hash-addressed references rather than duplicate 90k samples.
    for run in ('r5','r5-1'):
        root=OUT.parent/run
        for p in sorted((root/'cases').glob('*/outcome.json')):
            o=read_json(p); a=dict(np.load(p.parent/'physics.npz',allow_pickle=False)); result=o['result']
            source_id=CASES.index(o['case']); n=len(a['states'])-1
            if not np.isfinite(a['states']).all():raise ValueError('Nonfinite trajectory')
            record=dict(id=o['id'],source=o['case'],source_frame=o['frame'],variant=o['variant'],
                qualified=result['qualified'],reason=result['reason'],duration_s=result['end_s'],physics_intervals=n,
                state_samples=n+1,controls=len(a['ctbr_N_rad_s']),reward_status='NOT_RECORDED_IN_NATIVE_EVALUATOR',
                source_group=o['case'],paths={})
            for name in ('physics.npz','replay.npz','outcome.json','contacts.json'):
                f=p.parent/name; rel=str(f.relative_to(ROOT)); pins[rel]=digest(f);record['paths'][name]=rel
            # These are observed successful suffix states, not independent success trials.
            if result['qualified']:
                for j in range(0,max(1,n-400),400):
                    row=add(a['states'][j],a['motor_thrust_N'][j],a['motor_target_N'][j],o['case'],o['frame'],a['sample_times_s'][j])
                    support[source_id].append(row)
            speed=np.linalg.norm(a['states'][:,7:10],axis=1)
            record.update(initial_speed=float(speed[0]),peak_speed=float(speed.max()),peak_time_s=float(a['sample_times_s'][speed.argmax()]),
                          final_speed=float(speed[-1]),motor_residual_max_N=float(np.abs(a['motor_target_N']-a['motor_thrust_N']).max()))
            index.append(record)
    # Proposal order only: complete states on source trajectories, no source commands consumed.
    for case in CASES:
        rec=source['cases'][case]; trace=path.parent/rec['trace']['path'];pins[str(trace.relative_to(ROOT))]=digest(trace)
        if digest(trace)!=rec['trace']['sha256']:raise ValueError('Source trace pin mismatch')
        a=dict(np.load(trace,allow_pickle=False)); frames=list(range(ANCHORS[case],max(rec['handover_frame']-1,ANCHORS[case]-129),-8))
        chain=[]
        for frame in frames:
            chain.append(add(a['states'][frame,0],a['motor_thrust_N'][frame,0],a['motor_target_N'][frame,0],case,frame,a['sample_times_s'][frame]))
        chains.append(chain)
    plant=native.NativeContact(profile,spec);verify=native_verifier(plant); checks=[]
    for row in rows:
        s,f,t=row['states'],row['motor_thrust_N'],row['motor_target_N']
        check=verify(s,f,t)
        if not check['passed'] or native.flight_limits(s,spec,limits) or any(np.any((v<0)|(v>profile.max_rotor_thrust_N)) for v in (f,t)):
            raise ValueError(f'Invalid R6 reset: {row["case_ids"]}:{row["source_frame"]} {check}')
        checks.append(check)
    bank={k:np.concatenate([base[k],np.asarray([r[k] for r in rows])]) for k in keys}
    bank['dt']=base['dt']
    with (OUT/'intervention-training.npz').open('xb') as f:np.savez_compressed(f,**bank)
    metadata=dict(chains=chains,support=support,base_count=len(base['states']),sources=list(CASES),
        stride_frames=8,stride_s=.04,criterion='At least 4 native-style training successes in last 8 completed episodes at current row; training feedback is not independent qualification.',
        maximum_predecessor_s=.64,frontier_mass=.125,support_mass=.125)
    write_result(OUT/'curriculum.json',metadata)
    pins[str(path.relative_to(ROOT))]=digest(path)
    for p in (ROOT/'.agents/runs/asm2-frontier/phase-r/results.json',ROOT/'.agents/runs/asm2-terminal/r3/training.npz'):
        pins[str(p.relative_to(ROOT))]=digest(p)
    write_result(OUT/'dataset.json',dict(trajectories=index,pins=pins,total_intervals=sum(r['physics_intervals'] for r in index),
        qualified=sum(r['qualified'] for r in index),correlation='30 trajectories, five source clusters; suffix samples and perturbations are correlated.',
        reward_gap='Native traces have no PPO rewards; historical R3 rollout reward/action/value arrays lack complete physical histories. These records are indexed separately, never joined as full trajectories.',
        historical_rewards=str((OUT.parent/'r3').relative_to(ROOT))+'/A|B/regional-rollouts/*.npz',
        historical_labels='.agents/runs/asm2-frontier/phase-r/results.json',
        proposals=len(rows),all_reset_restore_checks_pass=True,checks=checks,
        method_limit='Predecessor spacing is a proposal heuristic tied to 40 ms motor time constant, not a reachability guarantee or monotonic success assumption. No backward integration.',
        r52='NOT_RUN; pending recommendation, not a dependency'))
    print(json.dumps(dict(trajectories=len(index),qualified=sum(r['qualified'] for r in index),proposals=len(rows),chains=list(map(len,chains)))))

if __name__=='__main__':build()
