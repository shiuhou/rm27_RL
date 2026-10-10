"""Timestamp-aligned vector budget; correlated norms must not be added."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ..model import frame
from ..evidence import write_json


def analyze(source):
    source=Path(source);s=json.loads((source/'summary.json').read_text());a=np.load(source/'flight.npz');x=a['trace'];e=a['estimates']
    relative=e[:,1]-s['release_time_s'];valid=(relative>=x[0,0])&(relative<=x[-1,0]);idx=np.flatnonzero(valid)
    # Every state field uses the same source timestamp. Diagnostic only.
    p=np.column_stack([np.interp(relative[valid],x[:,0],x[:,k]) for k in range(1,4)])
    v=np.column_stack([np.interp(relative[valid],x[:,0],x[:,k]) for k in range(8,11)])
    current=x[valid,1:4];estimate=e[valid,3:6]
    src=estimate-p;age=p-current;total=estimate-current
    np.testing.assert_allclose(src+age,total,atol=1e-12)
    R=Rotation.from_quat(x[:,[5,6,7,4]]).as_matrix();Re=e[:,9:18].reshape(-1,3,3)
    Rsrc=Slerp(x[:,0],Rotation.from_matrix(R))(relative[valid]).as_matrix()
    angle=Rotation.from_matrix(np.einsum('nji,njk->nik',Rsrc,Re[valid])).magnitude()
    cfg=json.loads((source/'plant_config.json').read_text()) if (source/'plant_config.json').exists() else json.loads(Path('artifacts/phase3-student-bundle-02/nominal_config.json').read_text())
    offset=np.array(cfg['contact']['offset_body_m']);normal=np.array(cfg['target']['normal']);Q=frame(normal)
    lever=np.einsum('nij,j->ni',Re[valid]-R[valid],offset)
    contact=s.get('contact_evaluation',{}).get('first_contact')
    if not contact:
        # Phase4 standalone adjudications are explicitly mapped.
        m={'phase4-unshaped-telemetry-01':None}
        contact=None
    tc=contact['time_s'] if contact else 4.5
    rows=[]
    for t in [1.,2.,3.,3.5,4.,tc-.1,tc-.002]:
        i=int(np.argmin(abs(x[idx,0]-t)));j=idx[i]
        rows.append(dict(time_s=float(x[j,0]),age_s=float(e[j,2]),position_at_source_error_m=src[i].tolist(),age_displacement_m=age[i].tolist(),
                         current_position_error_m=total[i].tolist(),direct_attitude_lever_error_m=lever[i].tolist(),
                         contactpoint_error_target_frame_m=(Q.T@(total[i]+lever[i])).tolist(),
                         velocity_at_source_error_m_s=(e[j,6:9]-v[i]).tolist(),attitude_at_source_error_deg=float(np.rad2deg(angle[i]))))
    # Integrate only unique source publications; held packets aren't independent samples.
    unique=np.r_[True,np.diff(e[idx,1])>1e-9]
    r=dict(source=str(source),scope='descriptive vector decomposition; publication time not raw sensor acquisition; no additive causal percentages',
        current_position_rmse_m=float(np.sqrt(np.mean(np.sum(total[unique]**2,axis=1)))),
        aligned_position_rmse_m=float(np.sqrt(np.mean(np.sum(src[unique]**2,axis=1)))),
        velocity_aligned_rmse_m_s=float(np.sqrt(np.mean(np.sum((e[idx[unique],6:9]-v[unique])**2,axis=1)))),
        rows=rows,source_sha256={f:hashlib.sha256((source/f).read_bytes()).hexdigest() for f in ['summary.json','flight.npz']})
    return r,dict(t=x[idx,0],total=total,source=src,age=age,lever=lever,velocity=e[idx,6:9]-v,angle=np.rad2deg(angle),x=x,tc=tc)


def run(inputs,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);results=[]
    fig,axes=plt.subplots(3,2,figsize=(13,11))
    for name in inputs:
        r,a=analyze(name);results.append(r);label=Path(name).name.replace('phase5-sitl-','').replace('phase4-unshaped-','')
        t=a['t'];axes[0,0].plot(t,np.linalg.norm(a['total'],axis=1)*1000,label=label);axes[0,1].plot(t,np.linalg.norm(a['source'],axis=1)*1000)
        axes[1,0].plot(t,np.linalg.norm(a['velocity'],axis=1));axes[1,1].plot(t,a['angle'])
        axes[2,0].plot(a['x'][:,1],a['x'][:,3]);axes[2,1].plot(t,np.linalg.norm(a['lever'],axis=1)*1000)
    for ax,title in zip(axes.flat,['Position error vs now [mm]','Position error at published timestamp [mm]','Velocity error at timestamp [m/s]','Attitude error at timestamp [deg]','Full-field side view [x,z metres]','Direct bumper lever attitude error [mm]']):
        ax.set_title(title);ax.grid(alpha=.25);ax.set_xlabel('elapsed [s]' if ax is not axes[2,0] else 'world x [m]')
    axes[0,0].legend(fontsize=8);fig.tight_layout();fig.savefig(output/'error_budget.png',dpi=150);plt.close(fig)
    r=dict(passed=True,scope='offline analysis completed, not flight success',results=results);write_json(output/'summary.json',r);return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',nargs='+',required=True);p.add_argument('--output',required=True);a=p.parse_args();run(a.input,a.output)
