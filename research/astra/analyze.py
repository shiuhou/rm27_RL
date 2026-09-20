"""Generate research figures and paired sensitivity estimates from saved trials."""
import argparse
import gzip
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from .scoring import paired_difference


def analyze(root):
    summary = json.loads((root/'summary.json').read_text())
    methods = ['pursuit', 'compensated_pursuit', 'lead']
    latencies, fovs = [0, .02, .05, .08, .15], [60, 90, 120]
    fig, axes = plt.subplots(1, 3, figsize=(11, 4), constrained_layout=True)
    for ax, method in zip(axes, methods):
        grid = np.array([[100*summary[f'l{l:g}_f{f}_tw2/{method}']['overall']['success_rate'] for f in fovs] for l in latencies])
        im = ax.imshow(grid, vmin=40, vmax=100, cmap='viridis', aspect='auto')
        ax.set_xticks(range(3), fovs)
        ax.set_yticks(range(5), [int(x*1000) for x in latencies])
        ax.set_xlabel('Horizontal FOV (deg)')
        ax.set_ylabel('Observation delay (ms)')
        ax.set_title(method.replace('_', ' '))
        for (row, col), val in np.ndenumerate(grid):
            ax.text(col, row, f'{val:.1f}', ha='center', va='center', color='black' if val > 82 else 'white')
    fig.colorbar(im, ax=axes, label='Geometric contact (%)', shrink=.85)
    fig.suptitle('1,000 paired trajectories per cell · nominal T/W=2 · surrogate sensor/plant')
    fig.savefig(root/'contact_heatmap.png', dpi=180)
    plt.close(fig)
    raw = root/'episodes.jsonl'
    opener = open
    if not raw.exists():
        raw = root/'episodes.jsonl.gz'
        opener = gzip.open
    needed = {'l0.05_f60_tw2', 'l0.05_f90_tw2', 'l0.05_f120_tw2', 'l0.15_f90_tw2',
              'l0.05_f90_tw1.5', 'l0.05_f90_tw3', 'occlusion_800ms', 'ego_drift_15cm_s', 'camera_pitch_up_20deg'}
    groups = {}
    with opener(raw, 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row['condition'] in needed:
                groups.setdefault((row['condition'], row['method']), []).append(row)
    baseline = 'l0.05_f90_tw2'
    sensitivity = {}
    for method in methods:
        sensitivity[method] = {name: paired_difference(groups[(baseline, method)], groups[(name, method)])
                               for name in sorted(needed-{baseline})}
    (root/'sensitivity.json').write_text(json.dumps(sensitivity, indent=2)+'\n')
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    for method in methods:
        rr = groups[(baseline, method)]
        miss = np.sort([r['miss_m'] for r in rr])
        axes[0].plot(miss, np.arange(1, len(miss)+1)/len(miss), label=method.replace('_', ' '))
        speed = np.sort([r['contact_speed_m_s'] for r in rr if r['success']])
        axes[1].plot(speed, np.arange(1, len(speed)+1)/len(speed), label=method.replace('_', ' '))
    axes[0].axvline(.17, ls='--', color='gray', label='contact radius')
    axes[0].set(xscale='log', xlabel='Minimum center separation (m)', ylabel='Empirical CDF', title='All trials')
    axes[1].set(xlabel='Relative speed at contact (m/s)', ylabel='Empirical CDF', title='Successful trials only')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('50 ms delay · 90° FOV · nominal T/W=2; impact survival unmodeled')
    fig.savefig(root/'nominal_distributions.png', dpi=180)
    plt.close(fig)
    return sensitivity


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    analyze(parser.parse_args().directory)
