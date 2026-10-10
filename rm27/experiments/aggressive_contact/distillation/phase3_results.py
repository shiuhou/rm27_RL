"""Aggregate frozen holdout shards, preserve failures, plot paired diagnostics."""
import argparse,csv,json
from collections import defaultdict
from pathlib import Path
import numpy as np
from ..evidence import write_json
from ..teacher import stats


def run(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    methods=defaultdict(list);records=[]
    for shard in [0,1,2,3,'adaptive']:
        root=Path(f'artifacts/phase3-holdout-{shard}-01')
        summary=json.loads((root/'summary.json').read_text())
        if not summary['passed']:raise ValueError('Incomplete/runner failure: '+str(root))
        for method in summary['results']:
            for path in sorted((root/method).glob('*/seed-*/summary.json')):
                r=json.loads(path.read_text());r['source']=str(path);r['mode_name']=path.parts[-3]
                methods[method].append(r);f=r.get('first_contact') or {}
                record=dict(method=method,mode=r['mode_name'],seed=r['seed'],scale=r['uncertainty_scale'],passed=r['passed'],
                    qualified=bool(r['passed'] and f.get('region_margin_m',-1)>=.008),failures='|'.join(r['failure_modes']),
                    margin_m=f.get('region_margin_m'),position_error_m=f.get('position_error_m'),normal_speed_m_s=f.get('normal_speed_m_s'),
                    tangent_speed_m_s=f.get('tangent_speed_m_s'),contact_tilt_error_deg=f.get('attitude_error_deg'),contact_rate_rad_s=f.get('body_rate_rad_s'),
                    recovery_time_s=r.get('recovery_time_s'),post_rate_rad_s=r.get('max_post_body_rate_rad_s'),
                    altitude_loss_m=r.get('altitude_loss_m'),saturation=r.get('saturation_fraction'),
                    maneuver_time_s=r.get('time_to_confirmed_recovery_s'),contact_detection_delay_s=r.get('contact_detection_delay_s'),
                    separated=r.get('separated'),secondary_contact=r.get('secondary_contact'),source=str(path))
                records.append(record)
    with (output/'trials.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    results={name:dict(total=stats(rows,.008),by_mode={mode:stats([r for r in rows if r['mode_name']==mode],.008) for mode in sorted(set(r['mode_name'] for r in rows))},
                         by_scale={str(scale):stats([r for r in rows if r['uncertainty_scale']==scale],.008) for scale in (1.,2.)}) for name,rows in methods.items()}
    write_json(output/'benchmark.json',results)
    paired={};rng=np.random.default_rng(92323)
    for profile in ['noise','delay10','delay20','medium']:
        a={(r['seed'],r['mode_name']):r for r in methods['current_'+profile]};b={(r['seed'],r['mode_name']):r for r in methods['improved_'+profile]}
        keys=sorted(a);seeds=sorted(set(k[0] for k in keys));blocks=np.array([np.mean([int(b[k]['passed'])-int(a[k]['passed']) for k in keys if k[0]==s]) for s in seeds])
        bootstrap=blocks[rng.integers(len(blocks),size=(10000,len(blocks)))].mean(1)
        paired[profile]=dict(episodes=len(keys),physical_draws=len(seeds),rescued=sum(not a[k]['passed'] and b[k]['passed'] for k in keys),
                            regressed=sum(a[k]['passed'] and not b[k]['passed'] for k in keys),success_rate_delta=float(blocks.mean()),
                            paired_draw_bootstrap95=np.quantile(bootstrap,[.025,.975]).tolist())
    write_json(output/'paired.json',paired)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names=list(results);fig,axs=plt.subplots(1,2,figsize=(14,6))
    success=[results[n]['total']['success_rate'] for n in names]
    axs[0].barh(names,success);axs[0].set(xlim=(0,1.03),xlabel='Joint success fraction (72 trials; adaptive 24)')
    for i,n in enumerate(names):
        x=[r['margin_m']*1000 for r in records if r['method']==n and r['margin_m'] is not None]
        axs[1].scatter(x,np.full(len(x),i),s=12,alpha=.4)
    axs[1].set(yticks=range(len(names)),yticklabels=names,xlabel='First-contact margin (mm), including failed trials');axs[1].axvline(8,color='r',linestyle='--')
    fig.tight_layout();fig.savefig(output/'holdout.png',dpi=150);plt.close(fig)
    # Fixed seed picked before looking for a visually successful holdout trial.
    sources={'teacher':Path('artifacts/phase3-holdout-0-01/teacher/balanced/seed-870000'),
             'current20':Path('artifacts/phase3-holdout-2-01/current_delay20/balanced/seed-870000'),
             'improved20':Path('artifacts/phase3-holdout-2-01/improved_delay20/balanced/seed-870000')}
    fig,axs=plt.subplots(3,2,figsize=(12,11));paths={}
    from scipy.spatial.transform import Rotation
    for name,source in sources.items():
        x=np.load(source/'flight/trajectory.npz')['trace'];t=x[:,0];paths[name]=x
        R=Rotation.from_quat(x[:,[5,6,7,4]]).as_matrix()
        axs[0,0].plot(x[:,1],x[:,3],label=name);axs[0,1].plot(t,np.linalg.norm(x[:,8:11],axis=1),label=name)
        axs[1,0].plot(t,np.rad2deg(np.arccos(np.clip(R[:,2,2],-1,1))),label=name)
        axs[1,1].plot(t,np.linalg.norm(x[:,11:14],axis=1),label=name)
        axs[2,0].plot(t,x[:,14:18].sum(1),label=name)
        if (source/'student.npz').exists():
            a=np.load(source/'student.npz');axs[2,1].plot(a['time_s'],a['innovation'],label=name)
        first=json.loads((source/'summary.json').read_text())['first_contact']
        if first:
            for ax in axs.flat[1:]:ax.axvline(first['time_s'],alpha=.2,color='gray')
    labels=[('X (m)','Height (m)'),('Time (s)','Speed (m/s)'),('Time (s)','Tilt (deg)'),('Time (s)','Body rate (rad/s)'),('Time (s)','Motor state total (N)'),('Time (s)','Contact acceleration innovation (m/s²)')]
    for ax,(xlab,ylab) in zip(axs.flat,labels):ax.set(xlabel=xlab,ylabel=ylab);ax.grid(alpha=.3)
    axs[2,1].set_ylim(-50,100);axs[0,0].legend();fig.tight_layout();fig.savefig(output/'paired_trajectory.png',dpi=160);plt.close(fig)
    fig=plt.figure(figsize=(10,6));ax=fig.add_subplot(projection='3d')
    for name,x in paths.items():ax.plot(x[:,1],x[:,2],x[:,3],label=name)
    ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Height (m)');ax.legend();fig.tight_layout();fig.savefig(output/'trajectory3d.png',dpi=160);plt.close(fig)
    write_json(output/'summary.json',dict(passed=True,trials=len(records),methods=list(methods),
              scope='independent frozen scenarios; 72 episodes reuse 24 physical draws across 3 modes. Failures retained. No held-out tuning.'))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();run(a.output)
