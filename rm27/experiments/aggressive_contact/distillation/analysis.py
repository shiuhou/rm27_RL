"""Offline phase/error audit and closed-loop plots; labels never feed actors."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ..evidence import write_json
from ..model import build_model,scenario,load_config
from .dataset import load_episode,settings
from .observation import history_vectors
from .network import MLP


def phase_audit(dataset,models,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    cfg=load_config();_,_,info,_=build_model(scenario(cfg),cfg);records={}
    index=json.loads((Path(dataset)/'index.json').read_text())
    for name,path in models.items():
        model=MLP.load(path);errors={k:[] for k in ('approach','attitude_transition','contact_100ms','recovery')}
        for row in index:
            if row['split']!='validation':continue
            mode=settings()['modes'].index(row['teacher']);X,Y,_,meta=load_episode(row['path'],mode,info)
            t=np.load(Path(row['path'])/'demo.npz')['time_s'];tc=meta['outcome']['first_contact']['time_s']
            E=model(history_vectors(X,model.lags))-Y
            masks=[t<tc-.5,(t>=tc-.5)&(t<tc),(t>=tc)&(t<tc+.1),t>=tc+.1]
            for k,mask in zip(errors,masks):errors[k].extend(E[mask])
        records[name]={k:dict(samples=len(e),rmse=np.sqrt(np.mean(np.array(e)**2,axis=0)).tolist()) for k,e in errors.items()}
    result=dict(passed=True,scope='held-out validation expert states; action rmse rate FLU rad/s, force N; not closed-loop success',models=records)
    write_json(output/'summary.json',result);return result


def plot(source,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    source=Path(source);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    config=json.loads((source/'comparison.json').read_text())
    fig,axs=plt.subplots(3,2,figsize=(13,11));drifts={}
    for series,(name,path) in enumerate(config['methods'].items()):
        color=plt.get_cmap('tab10').colors[series]
        path=Path(path);a=np.load(path/'flight/trajectory.npz');x=a['trace'];s=json.loads((path/'summary.json').read_text())
        t=x[:,0];R=Rotation.from_quat(x[:,[5,6,7,4]]).as_matrix()
        axs[0,0].plot(x[:,1],x[:,3],label=name,color=color)
        axs[0,1].plot(t,np.linalg.norm(x[:,8:11],axis=1),label=name,color=color)
        axs[1,0].plot(t,np.rad2deg(np.arccos(np.clip(R[:,2,2],-1,1))),label=name,color=color)
        axs[1,1].plot(t,np.linalg.norm(x[:,11:14],axis=1),label=name,color=color)
        axs[2,0].plot(t,x[:,14:18].sum(1),label=name,color=color)
        if (path/'student.npz').exists():
            z=np.load(path/'student.npz');axs[2,1].plot(z['time_s'],z['innovation'],label=name,color=color)
        if s.get('first_contact'):
            for ax in axs.flat[1:]:ax.axvline(s['first_contact']['time_s'],color='gray',alpha=.2)
        # Paired teacher trajectory is supplied explicitly in the comparison manifest.
        teacher=np.load(Path(config['methods']['teacher'])/'flight/trajectory.npz')['trace']
        target=np.column_stack([np.interp(t,teacher[:,0],teacher[:,i]) for i in (1,2,3)])
        error=np.linalg.norm(x[:,1:4]-target,axis=1);bad=np.flatnonzero(error>.1)
        drifts[name]=dict(first_10cm_deviation_s=None if not len(bad) else float(t[bad[0]]),max_deviation_m=float(error.max()))
    for ax,(x,y) in zip(axs.flat,[('World X (m)','Altitude (m)'),('Time (s)','Speed (m/s)'),('Time (s)','Tilt (deg)'),('Time (s)','Body rate (rad/s)'),('Time (s)','Collective motor state (N)'),('Time (s)','Contact acceleration innovation (m/s²)')]):ax.set(xlabel=x,ylabel=y);ax.grid(alpha=.3)
    axs[2,1].set_ylim(-40,80)
    axs[0,0].legend(fontsize=7);fig.tight_layout();fig.savefig(output/'closed_loop.png',dpi=160);plt.close(fig)
    result=dict(passed=True,paired_deviation=drifts);write_json(output/'summary.json',result);return result


def summarize(source,output):
    from collections import Counter
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    source,output=Path(source),Path(output);output.mkdir(parents=True,exist_ok=False)
    if not (source/'summary.json').exists():raise ValueError('Final campaign is not complete')
    records={}
    for split in ('holdout_small','holdout_stress'):
        records[split]={}
        for method in sorted(p for p in (source/split).iterdir() if p.is_dir()):
            records[split][method.name]={}
            for mode in sorted(p for p in method.iterdir() if p.is_dir() and p.name!='plant' and p.name!='source'):
                rows=[json.loads(p.read_text()) for p in mode.glob('seed-*/summary.json')]
                if not rows:continue
                metrics={}
                def distribution(values):
                    v=[x for x in values if x is not None]
                    return None if not v else dict(n=len(v),p05=float(np.quantile(v,.05)),median=float(np.median(v)),p95=float(np.quantile(v,.95)),max=float(max(v)))
                for key in ('time_to_confirmed_recovery_s','recovery_time_s','max_post_body_rate_rad_s','altitude_loss_m','saturation_fraction','min_altitude_m','contact_detection_delay_s'):
                    metrics[key]=distribution([r.get(key) for r in rows])
                for key in ('position_error_m','region_margin_m','normal_speed_m_s','tangent_speed_m_s','attitude_error_deg','body_rate_rad_s'):
                    metrics['contact_'+key]=distribution([(r.get('first_contact') or {}).get(key) for r in rows])
                records[split][method.name][mode.name]=dict(trials=len(rows),successes=sum(r['passed'] for r in rows),valid_contacts=sum(r.get('valid_contact',False) for r in rows),qualified=sum(r['passed'] and r['first_contact']['region_margin_m']>=.008 for r in rows),failure_modes=dict(Counter(f for r in rows for f in r['failure_modes'])),metrics=metrics)
    write_json(output/'benchmark.json',records)
    methods=['teacher','bc','hybrid_state','hybrid_noisy','hybrid_noisy_delay']
    fig,axes=plt.subplots(1,3,figsize=(14,4.5))
    for ax,mode in zip(axes,['aggressive','balanced','conservative']):
        for i,method in enumerate(methods):
            for split,dx,color in [('holdout_small',-.18,'#347eb5'),('holdout_stress',.18,'#c78345')]:
                r=records[split][method][mode];ax.bar(i+dx,r['successes']/r['trials'],width=.34,color=color,label=split if i==0 else None)
                ax.text(i+dx,.03,str(r['successes'])+'/'+str(r['trials']),ha='center',fontsize=7,rotation=90)
        ax.set_xticks(range(len(methods)),['teacher','BC','hybrid\nstate','hybrid\nnoisy','hybrid\nnoise+delay']);ax.set(title=mode,ylim=(0,1.08),ylabel='Joint success fraction');ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(output/'holdout.png',dpi=160);plt.close(fig)
    fig=plt.figure(figsize=(10,6));ax=fig.add_subplot(111,projection='3d')
    for method in ['teacher','hybrid_state','hybrid_noisy','hybrid_noisy_delay']:
        p=source/'holdout_small'/method/'balanced/seed-830000/flight/trajectory.npz';x=np.load(p)['trace']
        ax.plot(x[:,1],x[:,2],x[:,3],label=method)
    ax.set(xlabel='World X (m)',ylabel='World Y (m)',zlabel='Altitude (m)');ax.legend();fig.tight_layout();fig.savefig(output/'trajectory3d.png',dpi=160);plt.close(fig)
    r=dict(passed=True,scope='descriptive fixed-holdout statistics; modes share physical seeds, do not treat all episodes as independent draws')
    write_json(output/'summary.json',r);return r
