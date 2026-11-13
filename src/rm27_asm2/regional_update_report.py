"""Verify realized update weighting and plot the matched native learning curves."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .asset import ROOT, digest, read_json, write_result
from .regional_update import regional_weights
from .terminal_curriculum import BANDS


def summarize(directory):
    cfg = read_json(ROOT / 'configs/rm27_asm2_regional_update.json')
    frozen = read_json(directory / 'freeze.json')
    if frozen['config_sha256'] != digest(ROOT / 'configs/rm27_asm2_regional_update.json'):
        raise ValueError('Different regional weighting configuration')
    ledger = read_json(directory / 'ledger.json')
    rows, arms, sources = [], {}, {}
    for arm in ('A', 'B'):
        folder = directory / arm
        path = folder / 'update-weights.jsonl'
        updates = [json.loads(line) for line in path.read_text().splitlines()]
        learned = [json.loads(line) for line in (folder/'learning.jsonl').read_text().splitlines()]
        if len(updates) != len(learned) or len(updates) != ledger['arms'][arm]['updates']:
            raise ValueError('Incomplete weight telemetry')
        sources[str(path.relative_to(ROOT))] = digest(path)
        for number, (record, learning) in enumerate(zip(updates, learned, strict=True), 1):
            completed = record['completed_transitions']
            if (record['updates'] != number or completed != learning['completed_transitions']
                    or record['balanced'] != (arm == 'B')):
                raise ValueError('Weight telemetry counters or arm differ')
            with np.load(folder/'regional-rollouts'/f'rollout-{completed:06d}.npz', allow_pickle=False) as data:
                regions = data['region']
            weights = regional_weights(regions, cfg['update_target']) if arm == 'B' else np.ones_like(regions)
            ess = float(weights.sum()**2 / np.square(weights).sum())
            np.testing.assert_allclose([record['weight_min'], record['weight_max'], record['effective_sample_size']],
                                       [weights.min(), weights.max(), ess], rtol=1e-12, atol=1e-12)
            for i, band in enumerate(BANDS):
                count = int((regions == i).sum())
                mass = float(weights[regions == i].sum()/weights.sum())
                if record['regions'][band]['steps'] != count:
                    raise ValueError('Region count differs from actual collected transitions')
                np.testing.assert_allclose(record['regions'][band]['loss_mass'], mass, rtol=1e-12, atol=1e-12)
                rows.append(dict(arm=arm, region=band, updates=number, completed_transitions=completed,
                                 step_fraction=count/regions.size, loss_mass=mass,
                                 effective_sample_size=ess, weight_min=float(weights.min()),
                                 weight_max=float(weights.max())))
        if not updates or updates[-1]['completed_transitions'] != cfg['per_arm_transitions']:
            raise ValueError('Arm did not complete matched pilot')
        arms[arm] = dict(updates=len(updates), completed_transitions=updates[-1]['completed_transitions'],
            learning_s=ledger['arms'][arm]['learning_s'],
            transitions_per_s=updates[-1]['completed_transitions']/ledger['arms'][arm]['learning_s'],
            min_ess=min(r['effective_sample_size'] for r in updates),
            median_ess=float(np.median([r['effective_sample_size'] for r in updates])),
            max_weight=max(r['weight_max'] for r in updates),
            regions={band: dict(
                transition_fraction=float(np.mean([r['step_fraction'] for r in rows if r['arm'] == arm and r['region'] == band])),
                loss_mass=float(np.mean([r['loss_mass'] for r in rows if r['arm'] == arm and r['region'] == band])),
                absent_updates=sum(r['regions'][band]['steps'] == 0 for r in updates)) for band in cfg['update_target']})
    result = dict(arms=arms, sources=sources, source_sha256=digest(Path(__file__)),
                  interpretation='Loss coefficient mass, not measured gradient contribution. '
                                 'ValueNorm and advantage normalization still use actual transition distribution.')
    destination = directory/'weighting-summary.json'
    if destination.exists():
        if read_json(destination) != result:
            raise ValueError('Existing report differs; preserve it')
    else:
        write_result(destination, result)
    with (directory/'update-weight-curves.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    plot(directory, rows, cfg['update_target'])
    print(json.dumps(arms, indent=2))
    return result


def plot(directory, weights, target):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    with (directory/'development-curves.csv').open() as f:
        curves = list(csv.DictReader(f))
    fields = [('bridge_qualified', 'Qualified earlier contact / 36'),
              ('bridge_exits', 'Earlier envelope exits / 36'),
              ('braking_qualified', 'Qualified braking / 36'),
              ('terminal_qualified', 'Qualified terminal / 30')]
    fig, axes = plt.subplots(3, 2, figsize=(11, 9), layout='constrained')
    labels = {'A': 'Uniform update', 'B': 'Regional weighted update'}
    for axis, (field, title) in zip(axes.flat, fields):
        for arm in ('A', 'B'):
            part = [r for r in curves if r['arm'] == arm]
            axis.plot([int(r['trained_transitions'])/1000 for r in part],
                      [int(r[field]) for r in part], 'o-', label=labels[arm])
        axis.set(title=title, xlabel='Completed transitions (thousands)')
        axis.grid(alpha=.25)
    axes[0, 0].legend(fontsize=8)
    for arm, axis in zip(('A', 'B'), axes[-1]):
        for band in target:
            part = [r for r in weights if r['arm'] == arm and r['region'] == band]
            # Eight-update smoothing only for display; exact rows remain in CSV.
            blocks = [part[i:i+8] for i in range(0, len(part), 8)]
            axis.plot([b[-1]['completed_transitions']/1000 for b in blocks],
                      [np.mean([r['loss_mass'] for r in b]) for b in blocks], label=band)
        axis.set(title=labels[arm]+' — regional loss mass', xlabel='Completed transitions (thousands)', ylim=(0, 1))
        axis.legend(fontsize=7); axis.grid(alpha=.25)
    fig.suptitle('ASM2 regional update comparison · native contact outcomes')
    fig.savefig(directory/'regional-update-curves.png', dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    summarize(parser.parse_args().directory)
