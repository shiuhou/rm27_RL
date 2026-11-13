"""Verify and report Phase R2 rollout telemetry by episode reset region."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .asset import ROOT, read_json
from .rotation_bridge_metrics import SATURATION_TANH, SUMMARY_FIELDS, _summarize
from .terminal_curriculum import BANDS


ARMS = ('A', 'B')
RUN = ROOT / '.agents/runs/asm2-terminal/phase-r2-instrumented'
ROLLOUT_FIELDS = ('region', 'done', 'action_raw', 'log_std', *SUMMARY_FIELDS)
QUANTILES = ('mean', 'p10', 'p50', 'p90')
CHANNELS = ('roll', 'pitch', 'yaw', 'collective')
PLOT_BIN_UPDATES = 8


def metric_columns(metrics):
    channels = metrics.get('command_saturation_rate', (None,) * len(CHANNELS))
    return {
        **{f'{name}_{stat}': metrics.get(name, {}).get(stat)
           for name in SUMMARY_FIELDS for stat in QUANTILES},
        **{f'near_limit_rate_{name}': value for name, value in zip(CHANNELS, channels, strict=True)},
        'near_limit_rate_mean': float(np.mean(channels)) if metrics['steps'] else None,
    }


def region_counts(row, region):
    counts = row['region_counts'][region]
    return np.array([counts['resets'], counts['episodes'], counts['qualified']], dtype=np.int64)


def read_arm(directory, arm, ledger, planned_transitions):
    folder = directory / arm
    log = folder / 'learning.jsonl'
    updates = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    result = read_json(folder / 'result.json')
    state = ledger['arms'][arm]
    if state['pending'] is not None or state != result['accounting']:
        raise ValueError(f'{arm}: final result and settled ledger differ')
    if len(updates) != state['updates']:
        raise ValueError(f'{arm}: {len(updates)} learning rows for {state["updates"]} completed updates')

    records = {p.name: p for p in (folder / 'regional-rollouts').glob('rollout-*.npz')}
    expected = {f'rollout-{row["completed_transitions"]:06d}.npz' for row in updates}
    if records.keys() != expected:
        raise ValueError(f'{arm}: missing rollout archives {sorted(expected - records.keys())}, '
                         f'unmatched archives {sorted(records.keys() - expected)}')

    curves, samples = [], []
    previous_counts = {band: np.zeros(3, dtype=np.int64) for band in BANDS}
    previous_trained = previous_charged = 0
    for index, row in enumerate(updates, 1):
        trained, charged = row['completed_transitions'], row['transitions']
        path = records[f'rollout-{trained:06d}.npz']
        with np.load(path, allow_pickle=False) as record:
            summary = json.loads(str(record['summary_json']))
            data = {name: record[name] for name in ROLLOUT_FIELDS}
            expected_counters = (1, trained, charged, index)
            counters = tuple(summary[name] for name in
                             ('format', 'completed_transitions', 'charged_transitions', 'updates'))
            if (tuple(record['band_names']) != BANDS or counters != expected_counters
                    or summary['command_saturation_abs_tanh'] != SATURATION_TANH):
                raise ValueError(f'{arm}: invalid rollout metadata at update {index}')
        stride = data['region'].size
        if (row['updates'] != index or trained != previous_trained + stride
                or charged < previous_charged + stride or not stride
                or (samples and data['region'].shape != samples[0]['region'].shape)):
            raise ValueError(f'{arm}: inconsistent update counters or shape at update {index}')
        metrics_by_band = _summarize(data)
        for band in BANDS:
            metrics = metrics_by_band[band]
            archived = summary['regions'][band]
            if any(metrics[key] != archived[key] for key in ('steps', 'ends')):
                raise ValueError(f'{arm}: {band} sample counts disagree at update {index}')
            if metrics['steps']:
                np.testing.assert_allclose(
                    list(metric_columns(metrics).values()), list(metric_columns(archived).values()),
                    rtol=1e-7, atol=1e-9, err_msg=f'{arm}: {band} summary differs at update {index}')
            cumulative_counts = region_counts(row, band)
            draws, ends, loaded = (cumulative_counts - previous_counts[band]).tolist()
            if draws < 0 or ends != metrics['ends'] or not 0 <= loaded <= ends:
                raise ValueError(f'{arm}: {band} outcome counts disagree at update {index}')
            previous_counts[band] = cumulative_counts
            curves.append(dict(
                arm=arm, region=band, updates=index, completed_transitions=trained,
                charged_transitions=charged, learning_s=row['learning_s'],
                steps=metrics['steps'], step_fraction=metrics['steps'] / stride,
                reset_draws=draws, completed_episodes=ends, isaac_designated_loaded=loaded,
                other_episode_ends=ends - loaded,
                **{f'policy_std_raw_{name}': value for name, value in
                   zip(CHANNELS, summary['policy_std_raw'], strict=True)},
                **metric_columns(metrics),
            ))
        previous_trained, previous_charged = trained, charged
        samples.append(data)

    totals = updates[-1]['episodes'] if updates else {}
    counted = np.sum(list(previous_counts.values()), axis=0)
    if (previous_trained != result['completed_transitions']
            or not previous_charged <= state['transitions'] <= planned_transitions
            or counted[1] != sum(totals.values()) or counted[2] != totals.get('DESIGNATED_LOADED', 0)):
        raise ValueError(f'{arm}: final outcomes or completed transitions do not match telemetry')
    regions = _summarize({name: np.concatenate([data[name] for data in samples], axis=0)
                          for name in ROLLOUT_FIELDS}) if samples else {
                              band: {'steps': 0, 'ends': 0} for band in BANDS}
    expected_updates = planned_transitions // samples[0]['region'].size if samples else None
    pooled = []
    for band in BANDS:
        metrics = regions[band]
        draws, ends, loaded = previous_counts[band].tolist()
        pooled.append(dict(
            arm=arm, region=band, completed_updates=len(samples), planned_updates=expected_updates,
            completed_transitions=previous_trained, charged_transitions=state['transitions'],
            planned_transitions=planned_transitions, learning_s=state['learning_s'],
            stop_reason=result['stop_reason'], steps=metrics['steps'],
            step_fraction=metrics['steps'] / previous_trained if previous_trained else None,
            reset_draws=draws, completed_episodes=ends, isaac_designated_loaded=loaded,
            other_episode_ends=ends - loaded, **metric_columns(metrics),
        ))
    return curves, pooled, totals


def write_csv(path, rows):
    temporary = path.with_suffix('.tmp')
    with temporary.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def plot_curves(path, curves):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.lines import Line2D

    panels = {
        'reward_mean': 'Reward per step',
        'critic_stored_mean': 'Stored critic value',
        'critic_denormalized_mean': 'Denormalized critic value',
        'gae_raw_mean': 'Raw GAE',
        'gae_normalized_mean': 'Normalized GAE',
        'exploration_z_l2_mean': 'Standardized exploration L2',
        'near_limit_rate_mean': f'Command fraction |tanh| ≥ {SATURATION_TANH:g}',
        'step_fraction': 'Regional step share',
        'loaded_fraction': 'Isaac designated-loaded fraction',
    }
    figure, axes = plt.subplots(3, 3, figsize=(13, 9), sharex=True)
    active = [band for band in BANDS if any(row['region'] == band and row['steps'] for row in curves)]
    colors = dict(zip(active, plt.get_cmap('tab10').colors, strict=False))
    styles = {'A': '-', 'B': '--'}
    for band in active:
        for arm in ARMS:
            series = [row for row in curves if row['arm'] == arm and row['region'] == band]
            if not any(row['steps'] for row in series):
                continue
            blocks = [series[i:i+PLOT_BIN_UPDATES] for i in range(0, len(series), PLOT_BIN_UPDATES)]
            x = [block[-1]['completed_transitions'] / 1000 for block in blocks]
            for axis, (key, title) in zip(axes.flat, panels.items(), strict=True):
                y = []
                for block in blocks:
                    steps = sum(row['steps'] for row in block)
                    if key == 'step_fraction':
                        y.append(float(np.mean([row[key] for row in block])))
                    elif key == 'loaded_fraction':
                        ends = sum(row['completed_episodes'] for row in block)
                        y.append(sum(row['isaac_designated_loaded'] for row in block) / ends if ends else np.nan)
                    else:
                        y.append(sum(row[key] * row['steps'] for row in block if row['steps']) / steps
                                 if steps else np.nan)
                axis.plot(x, y, linestyle=styles[arm], color=colors[band], marker='.', markersize=4)
                axis.set_title(title, fontsize=10)
                axis.grid(alpha=.25)
    for axis in axes[-1]:
        axis.set_xlabel('Completed transitions (thousands)')
    handles = [Line2D([0], [0], color=colors[band], label=band) for band in active]
    handles.extend(Line2D([0], [0], linestyle=styles[arm], color='#333333', label=f'Arm {arm}') for arm in ARMS)
    figure.suptitle(f'Phase R2 regional training telemetry · {PLOT_BIN_UPDATES}-update bins')
    figure.legend(handles=handles, loc='lower center', ncol=len(handles), frameon=False)
    figure.tight_layout(rect=(0, .045, 1, .95))
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, nargs='?', default=RUN)
    args = parser.parse_args()
    directory = args.directory
    planned = read_json(directory / 'freeze.json')['endpoint']
    ledger = read_json(directory / 'ledger.json')
    curves, pooled = [], []
    for arm in ARMS:
        update_rows, region_rows, outcomes = read_arm(directory, arm, ledger, planned)
        curves.extend(update_rows)
        pooled.extend(region_rows)
        row = region_rows[0]
        print(f'{arm}: {row["completed_updates"]}/{row["planned_updates"]} updates, '
              f'{row["completed_transitions"]}/{planned} trained transitions, '
              f'{row["charged_transitions"]} charged, {row["stop_reason"]}')
        print(f'{arm} Isaac episode outcomes {json.dumps(outcomes, sort_keys=True)}')
    write_csv(directory / 'regional-training-summary.csv', pooled)
    if curves:
        write_csv(directory / 'regional-training-curves.csv', curves)
        plot_curves(directory / 'regional-training-curves.png', curves)


if __name__ == '__main__':
    main()
