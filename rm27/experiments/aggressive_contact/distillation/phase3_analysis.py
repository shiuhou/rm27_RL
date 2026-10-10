"""Distribution and local feedback audits using TRAIN bank / VALIDATION only."""
import argparse,copy,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from ..evidence import write_json
from ..model import VehicleEstimate,TargetState,unit,build_model,scenario,load_config
from ..maneuver import ManeuverSkill
from ..teacher_interface import project
from .dataset import designs,load_episode
from .network import MLP
from .observation import history_vectors


def audit(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    arrays=np.load('artifacts/phase3-training-01/training_arrays.npz');cols=np.arange(5,20)
    X,VX=arrays['X'][:,cols],arrays['VX'][:,cols];scale=np.maximum(X.std(0),.03)
    tree=cKDTree(X[::2]/scale)
    vd=tree.query(VX[::3]/scale)[0];threshold=float(np.quantile(vd,.99))
    records={}
    for name,source in [('bc','distill-bc-val-01'),('history','distill-history-val-01'),('hybrid','distill-spline-noise-filtered-val-02'),('dagger','distill-dagger-val-01')]:
        distances=[];early=[];depart=[]
        for path in sorted(Path('artifacts',source).glob('*/seed-*/student.npz')):
            a=np.load(path);t=a['time_s'];dist=tree.query(a['observation'][:,cols]/scale)[0]
            # Lasting 50 ms outside validation p99, not one isolated noisy frame.
            bad=np.convolve((dist>threshold).astype(int),np.ones(10,int),'valid')==10
            hit=np.flatnonzero(bad)
            depart.append(None if not len(hit) else float(t[hit[0]]));distances.extend(dist[::3]);early.extend(dist[t<1.][::3])
        records[name]=dict(episodes=len(depart),median_first_sustained_outside_s=None if not any(v is not None for v in depart) else float(np.median([v for v in depart if v is not None])),
                           nearest_distance_median=float(np.median(distances)),first_second_p95=float(np.quantile(early,.95)),fraction_outside=float(np.mean(np.array(distances)>threshold)),first_outside_by_episode=depart)
    write_json(output/'distribution.json',dict(passed=True,metric='nearest train state normalized Euclidean distance; p,v,R first two columns,omega; not formal manifold probability',
               validation_p99=threshold,validation_median=float(np.median(vd)),methods=records))
    # Controlled local derivatives around identical expert states. This tests
    # feedback response, not merely average nominal action accuracy.
    index=json.load(open('artifacts/distill-data-01/index.json'))
    row=next(r for r in index if r['split']=='validation' and r['teacher']=='balanced')
    path=Path(row['path']);d=np.load(path/'demo.npz');meta=json.load(open(path/'metadata.json'));cfg=meta['task_config']
    _,_,info,_=build_model(scenario(load_config()),load_config())
    target=TargetState(np.array(cfg['target']['center_m']),unit(cfg['target']['normal']),np.zeros((3,3)),0.)
    ego=d['observation_ego'];ts=d['time_s']
    states=[VehicleEstimate(r[:3],r[3:6],Rotation.from_quat(r[[7,8,9,6]]).as_matrix(),r[10:13],t) for r,t in zip(ego,ts)]
    teacher=ManeuverSkill(cfg,designs()['balanced'],states[0],target,info,states[0].position)
    obs,_,_,_=load_episode(path,1,info)
    models={n:MLP.load(f'artifacts/distill-{n}-01/policy.npz') for n in ('bc','history','dagger')}
    data={n:[] for n in ('teacher',*models)};clock=[]
    gain=2*cfg['controller']['attitude_damping']*cfg['controller']['attitude_frequency_rad_s']
    tc=meta['outcome']['first_contact']['time_s']
    for i,s in enumerate(states):
        if s.timestamp>=tc-.5:break
        if i%20==0 and s.timestamp>.1:
            derivatives={n:[] for n in data}
            for field,col,eps in [('position',5,.002),('velocity',8,.01),('omega_body',18,.01)]:
                axis=1 if field=='omega_body' else 0
                actions=[]
                for sign in (-1,1):
                    val=getattr(s,field).copy();val[axis]+=sign*eps
                    fields=dict(position=s.position,velocity=s.velocity,rotation=s.rotation,omega_body=s.omega_body,timestamp=s.timestamp);fields[field]=val
                    sp=VehicleEstimate(**fields);clone=copy.deepcopy(teacher);u,_=clone.command(sp,False)
                    a=project(u,sp.omega_body,clone.inertia,clone.mixer,gain);actions.append(np.r_[a.body_rates_flu_rad_s,a.collective_force_N])
                derivatives['teacher'].append((actions[1]-actions[0])/(2*eps))
                for n,m in models.items():
                    x=np.concatenate([obs[max(0,i-l)] for l in m.lags]);xp=x.copy();xm=x.copy();xp[col]+=eps;xm[col]-=eps
                    derivatives[n].append((m(xp)-m(xm))/(2*eps))
            clock.append(s.timestamp)
            for n in data:data[n].append(derivatives[n])
        teacher.command(s,False);teacher.previous_motor_command=d['teacher_motor_N'][i].copy()
    np.savez_compressed(output/'local_feedback.npz',time_s=clock,**data)
    teacher_j=np.array(data['teacher']);metrics={}
    for n in models:
        j=np.array(data[n]);metrics[n]=dict(pitch_derivative_median=np.median(j[:,:,1],axis=0).tolist(),
            pitch_gain_rmse_vs_teacher=np.sqrt(np.mean((j[:,:,1]-teacher_j[:,:,1])**2,axis=0)).tolist(),
            pitch_sign_agreement=np.mean(np.sign(j[:,:,1])==np.sign(teacher_j[:,:,1]),axis=0).tolist())
    write_json(output/'feedback.json',dict(passed=True,source=str(path),perturbations=['position_x .002m','velocity_x .01m/s','body_pitch_rate .01rad/s'],
            scope='current-frame partial derivatives, frozen past/controller memories; not full closed-loop stability proof',
            teacher_pitch_derivative_median=np.median(teacher_j[:,:,1],axis=0).tolist(),models=metrics))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(1,3,figsize=(13,4))
    for ax,k,label in zip(axs,range(3),['position X','velocity X','body pitch rate']):
        for n,a in data.items():ax.plot(clock,np.array(a)[:,k,1],label=n)
        ax.set(xlabel='Time (s)',ylabel='Pitch command derivative / '+label);ax.grid(alpha=.3)
    axs[0].legend();fig.tight_layout();fig.savefig(output/'feedback.png',dpi=150);plt.close(fig)
    write_json(output/'summary.json',dict(passed=True,scope='offline diagnostic, no new closed-loop success claim'))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();audit(a.output)
