"""Plots and paired time statistics for the whole-maneuver study."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .evidence import write_json
from .model import unit


def plot(source, output, pose_source=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    summary = json.loads((source/'summary.json').read_text())
    methods = ['staged_v2', 'compressed_staged', 'joint_recovery']
    if pose_source:
        pose_source = Path(pose_source)
        methods.append('early_pose')
    labels = ['Staged v2', 'Compressed staged', 'Joint: fast', 'Joint: early pose'][:len(methods)]
    colors = ['#777777', '#d99832', '#3478af', '#348e6f']
    def trials(method, split):
        path = pose_source/'evaluation'/split if method == 'early_pose' else source/'evaluation'/split/method
        return [p.parent for p in sorted(path.glob('seed-*/summary.json'))]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    paired = {}
    for split in ('small', 'stress'):
        rows = {m:[json.loads((p/'summary.json').read_text()) for p in trials(m, split)] for m in methods}
        base = rows['staged_v2']
        paired[split] = {}
        for m in methods[1:]:
            eligible = [(a, b) for a,b in zip(base,rows[m],strict=True) if a['passed'] and b['passed']]
            delta = np.array([a['time_to_confirmed_recovery_s']-b['time_to_confirmed_recovery_s'] for a,b in eligible])
            rng = np.random.default_rng(311)
            bootstrap = delta[rng.integers(0,len(delta),(10000,len(delta)))].mean(axis=1) if len(delta) else np.array([np.nan])
            paired[split][m] = dict(jointly_passed_pairs=len(delta), saved_time_mean_s=float(delta.mean()) if len(delta) else None,
                                   paired_bootstrap_95_s=np.quantile(bootstrap,[.025,.975]).tolist() if len(delta) else None)
        ax = axes[0, 0]
        offset = -.18 if split == 'small' else .18
        for i,m in enumerate(methods):
            values = [r['time_to_confirmed_recovery_s'] for r in rows[m] if r['passed']]
            mid = np.median(values)
            ax.bar(i+offset, mid, width=.33, color=colors[i], alpha=1 if split=='small' else .5)
            ax.errorbar(i+offset, mid, yerr=[[mid-min(values)],[max(values)-mid]], fmt='none',color='black',capsize=3)
            ax.text(i+offset, .2, f'{sum(r["passed"] for r in rows[m])}/{len(rows[m])}',ha='center',fontsize=8,rotation=90,color='white')
    axes[0,0].set(xticks=range(len(methods)), xticklabels=labels, ylabel='Time to confirmed recovery (s)', title='Median and range of successful trials; solid=small, pale=stress')
    axes[0,0].tick_params(axis='x',labelrotation=12)
    for method,label,color in zip(methods,labels,colors):
        path = trials(method,'small')[0]
        row = json.loads((path/'summary.json').read_text())
        x = np.load(path/'flight/trajectory.npz')['trace']
        x = x[x[:,0] <= row['time_to_confirmed_recovery_s']]
        axes[0,1].plot(x[:,1], x[:,3],color=color,label=label)
        axes[1,0].plot(x[:,0],np.linalg.norm(x[:,8:11],axis=1),color=color,label=label)
        before=x[x[:,0]<row['first_contact']['time_s']]
        physical=json.loads((path/'flight/physical.json').read_text())
        n=unit(physical['target']['normal'])
        R=Rotation.from_quat(before[:,[5,6,7,4]]).as_matrix()
        angles=np.rad2deg(np.arccos(np.clip(R[:,:,2]@n,-1,1)))
        axes[1,1].plot(before[:,0]-row['first_contact']['time_s'],angles,color=color,label=label)
    axes[0,1].set(xlabel='World X (m)',ylabel='Height (m)',title='Actual continuous trajectory, first small-uncertainty seed')
    axes[0,1].legend(fontsize=8)
    axes[1,0].set(xlabel='Time from airborne dock start (s)',ylabel='Speed (m/s)',title='Transit momentum is carried into approach')
    axes[1,1].set(xlabel='Time relative to physical first contact (s)',ylabel='Body thrust-axis / target-normal error (deg)',xlim=(-1.2,0),ylim=(0,70),title='Attitude preparation before contact')
    axes[1,1].axhline(10,color='black',linestyle=':',linewidth=1)
    for ax in axes.ravel(): ax.grid(alpha=.2)
    fig.suptitle('Whole maneuver: existing RMUC plant, truth feedback, direct motors')
    fig.savefig(output/'whole-maneuver.png',dpi=170)
    plt.close(fig)
    result=dict(passed=True,paired_time=paired,scope='time intervals condition on both methods passing; reliability reported separately')
    write_json(output/'summary.json',result)
    return result
