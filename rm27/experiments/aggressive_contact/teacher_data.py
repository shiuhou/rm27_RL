"""Versioned demonstration exporter with explicit actor/privileged separation."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .evidence import write_json
from .model import ROOT,frame,unit
from .teacher_interface import project


def read_trial(path):
    path=Path(path)
    flight=path/'flight' if (path/'flight').is_dir() else path
    summary=json.loads((path/'summary.json').read_text())
    cfg=json.loads((flight/'config.json').read_text())
    physical=json.loads((flight/'physical.json').read_text())
    archive=np.load(flight/'trajectory.npz')
    return flight,summary,cfg,physical,archive


def features(path,actuators=True):
    """Privileged fixed 120-ms post-first-contact query, independent of future labels."""
    flight,s,c,p,a=read_trial(path)
    if not s.get('first_contact'):return None
    x=a['trace']; tc=s['first_contact']['time_s']
    indices=np.flatnonzero(x[:,0]>=tc+.12-1e-9)
    if not len(indices):return None
    r=x[indices[0]]
    R=Rotation.from_quat(r[[5,6,7,4]]).as_matrix()
    Rt=frame(p['target']['normal']);n=Rt[:,2]
    normal_body=R.T@n
    center=np.array(p['target']['center_m'])
    params=s['parameters']
    # Absolute height is relevant to ground; x/y position is target-relative.
    f=np.r_[Rt.T@(r[1:4]-center),r[3],Rt.T@r[8:11],normal_body,R[2],r[11:14],
            params['retreat_m'],params['recovery_lift_m'],c['controller'].get('actuator_feedback_gain',0),
            float(c['controller'].get('adapt_specific_thrust',False))]
    if actuators:
        f=np.r_[f,r[14:18],p['draw']['mass_scale'],p['draw']['thrust_scale'],p['draw']['motor_tau_scale']]
    return f


def mechanism_metrics(path):
    flight,s,c,p,a=read_trial(path)
    x=a['trace'];t=x[:,0]
    R=Rotation.from_quat(x[:,[5,6,7,4]]).as_matrix()
    thrust=x[:,14:18].sum(axis=1)*p['draw']['thrust_scale']
    acceleration=np.gradient(x[:,8:11],t,axis=0)
    end=s.get('time_to_confirmed_recovery_s')
    use=t<=end+1e-9 if end is not None else np.ones(len(t),dtype=bool)
    out=dict(effort_proxy_N32_s=float(np.trapezoid(np.sum(np.maximum(x[use,14:18],0)**1.5,axis=1),t[use])),
             effort_end_s=float(t[use][-1]),effort_scope='through held recovery or failed trial end',
             max_preimpact_rate_rad_s=None,braking_lead_s=None,velocity_thrust_angle_deg=None,
             collective_at_contact_N=None,motor_state_at_contact_N=None)
    first=s.get('first_contact')
    if first:
        tc=first['time_s']; before=np.flatnonzero(t<tc)
        k=before[-1];v=x[k,8:11];b=R[k,:,2]
        out.update(velocity_thrust_angle_deg=float(np.rad2deg(np.arccos(np.clip(v@b/max(np.linalg.norm(v),1e-9),-1,1)))),
                   collective_at_contact_N=float(thrust[k]),motor_state_at_contact_N=x[k,14:18].tolist(),
                   max_preimpact_rate_rad_s=float(np.linalg.norm(x[before,11:14],axis=1).max()),
                   thrust_normal_component_at_contact_N=float(thrust[k]*(b@unit(p['target']['normal']))),
                   velocity_normal_at_contact_m_s=float(v@unit(p['target']['normal'])))
        power=np.sum(acceleration[before]*x[before,8:11],axis=1)
        bad=np.flatnonzero(power>=0)
        start=bad[-1]+1 if len(bad) else 0
        out['braking_lead_s']=float(tc-t[before[start]]) if start<len(before) else 0.
    return out


SCHEMA={
 'version':'rm27.teacher.v1.1',
 'clock':'seconds, simulation monotonic, sample timestamps retained; nominal control 200Hz',
 'frames':'world NWU, body FLU; quaternion WXYZ body->world',
 'actor_allowlist':['observation_ego','observation_target','observation_motor_estimate'],
 'groups':{
  'observation_ego':{'columns':['px','py','pz','vx','vy','vz','qw','qx','qy','qz','wx','wy','wz'],
                     'source':'MuJoCo truth substituted for estimator; field types deployable, measurements NOT validated','privileged_only':False},
  'observation_target':{'columns':['rel_body_x','rel_body_y','rel_body_z','normal_body_x','normal_body_y','normal_body_z','age_s','confidence'],
                       'source':'committed nominal target estimate, transformed with ego estimate; not target physical truth','privileged_only':False},
  'observation_motor_estimate':{'columns':['fhat0','fhat1','fhat2','fhat3','effective_ratio'],
                               'source':'causal controller observer; missing legacy records explicitly NaN','privileged_only':False},
  'teacher_motor_N':{'privileged_only':True,'role':'supervision; motor-force command before physical lag/scale'},
  'teacher_rate_thrust':{'columns':['p_flu_rad_s','q_flu_rad_s','r_flu_rad_s','collective_N'],
                         'privileged_only':True,'role':'supervision; nominal-inner-loop projection, not calibrated AP throttle'},
  'reference':{'privileged_only':True,'role':'teacher planned p/v/a/q/rate/force; not a default student input'},
  'physical_motor_state':{'privileged_only':True},'phase':{'privileged_only':True},
  'contact_truth':{'privileged_only':True,'role':'replace with real contact estimate/history for deployment'},
  'privileged_state':{'privileged_only':True,'role':'qpos,qvel,act,ctrl; future queries forbidden for actor'},
  'physical_draw':{'privileged_only':True},'outcome':{'privileged_only':True,'role':'future labels'},
  'event_samples':{'privileged_only':True,'role':'nearest logged before/after event, offsets retained'}},
 'action_contract':'RateThrust uses FLU rad/s and physical N; MAVLink wire FRD and normalized throttle needs an explicit calibrated map. No sender.',
 'split_contract':'split by scenario seed across all methods; teacher training/design and model validation never include new untouched holdout'
}


def export(path,output,split,teacher_id):
    from .control import Skill
    from .model import build_model,scenario,truth_estimate,TargetState
    flight,s,c,p,a=read_trial(path)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    x=a['trace'];R=Rotation.from_quat(x[:,[5,6,7,4]]).as_matrix()
    ego=np.c_[x[:,1:4],x[:,8:11],x[:,4:8],x[:,11:14]]
    target=np.array(c['target']['center_m']); n=unit(c['target']['normal'])
    relative=np.einsum('nji,nj->ni',R,target-x[:,1:4])
    normals=np.einsum('nji,j->ni',R,n)
    target_obs=np.c_[relative,normals,x[:,0]-x[0,0],np.ones(len(x))]
    m,d,info,_=build_model(scenario(c),c)
    skill=Skill(c,c['nominal_skill'],truth_estimate(m,d),TargetState(target,n,np.zeros((3,3)),0.),info)
    gain=2*c['controller']['attitude_damping']*c['controller']['attitude_frequency_rad_s']
    actions=np.array([np.r_[z.body_rates_flu_rad_s,z.collective_force_N] for z in
                      (project(row[18:22],row[11:14],skill.inertia,skill.mixer,gain) for row in x)])
    observer=a['controller_estimate'] if 'controller_estimate' in a else np.full((len(x),5),np.nan)
    phases=a['phases'] if 'phases' in a else np.array(['LEGACY_UNKNOWN']*len(x))
    np.savez_compressed(output/'demo.npz',time_s=x[:,0],observation_ego=ego,observation_target=target_obs,
                        observation_motor_estimate=observer,teacher_motor_N=x[:,18:22],teacher_rate_thrust=actions,
                        reference=np.c_[x[:,22:25],x[:,27:43]],physical_motor_state=x[:,14:18],
                        privileged_state=x[:,1:22],contact_truth=x[:,25],phase=phases)
    event_samples={}
    if s.get('first_contact'):
        tc=s['first_contact']['time_s']
        for name,time,side in [('pre_impact',tc,'left'),('post_impact_120ms',tc+.12,'right'),
                               ('recovery',s.get('time_to_confirmed_recovery_s'),'right')]:
            if time is None:
                event_samples[name]=dict(available=False)
                continue
            idx=int(np.clip(np.searchsorted(x[:,0],time,side=side)-(side=='left'),0,len(x)-1))
            event_samples[name]=dict(available=True,index=idx,requested_time_s=time,offset_s=float(x[idx,0]-time))
    source_hashes={name:hashlib.sha256((flight/name).read_bytes()).hexdigest() for name in ('trajectory.npz','config.json','physical.json')}
    source_hashes['summary.json']=hashlib.sha256((Path(path)/'summary.json').read_bytes()).hexdigest()
    model_record={}
    if (flight/'model_template.json').exists():
        model_record=json.loads((flight/'model_template.json').read_text())
        model_record['overlay_sha256']=hashlib.sha256((flight/'model_overlay.npz').read_bytes()).hexdigest()
    elif (flight/'model.mjb').exists():
        model_record={'model_sha256':hashlib.sha256((flight/'model.mjb').read_bytes()).hexdigest()}
    code_hashes={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')}
    provenance=next((p/'provenance.json' for p in [Path(path).resolve(),*Path(path).resolve().parents] if (p/'provenance.json').exists()),None)
    rollout_provenance=None if provenance is None else dict(path=str(provenance),sha256=hashlib.sha256(provenance.read_bytes()).hexdigest(),record=json.loads(provenance.read_text()))
    metadata=dict(schema=SCHEMA,rollout_provenance=rollout_provenance,teacher_id=teacher_id,split=split,scenario_group=f'{s.get("seed")}/{s.get("uncertainty_scale")}',
                  source=str(Path(path).resolve()),source_sha256=source_hashes,exporter_code_sha256=code_hashes,
                  model_record=model_record,cost_components=dict(joint_passed=s['passed'],valid_contact=s.get('valid_contact'),
                      departure_to_held_recovery_s=s.get('time_to_confirmed_recovery_s'),contact_margin_m=(s.get('first_contact') or {}).get('region_margin_m'),
                      max_post_rate_rad_s=s.get('max_post_body_rate_rad_s'),saturation_fraction=s.get('saturation_fraction'),
                      altitude_loss_m=s.get('altitude_loss_m')),
                  task_config=c,physical_draw=p,initial_state=x[0,1:18].tolist(),outcome=s,event_samples=event_samples,
                  mechanisms=mechanism_metrics(path),legacy_observer_missing='controller_estimate' not in a)
    write_json(output/'metadata.json',metadata)
    return dict(path=str(output),teacher=teacher_id,split=split,scenario_group=metadata['scenario_group'],passed=s['passed'],samples=len(x))
