"""Plot recorded route flight and report independent truth-based tracking errors."""

import argparse
import json
from pathlib import Path
from rm27.telemetry.recording import open_recording
import numpy as np
from .route_experiment import path_distance


def analyze(run_dir, output):
    run_dir, output = Path(run_dir), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    meta = json.loads((run_dir/'metadata.json').read_text())
    summary = json.loads((run_dir/'summary.json').read_text())
    points = np.asarray(meta['route_audit']['waypoints_world_m'])
    errors, speeds, samples = [], [], []
    contact_frames = 0
    with open_recording(run_dir/'frames.jsonl') as stream:
        for i, line in enumerate(stream):
            row = json.loads(line)
            if row['phase'] != 'ROUTE':
                continue
            position = np.asarray(row['truth']['position_nwu'])
            error = path_distance(position, points)
            errors.append(error)
            speeds.append(np.linalg.norm(row['truth']['velocity_nwu']))
            contact_frames += bool(row['contacts'])
            if i % 50 == 0:
                samples.append([row['state_sent']['timestamp'], *position, error])
    if not errors:
        raise ValueError('No route frames to analyze')
    report = dict(passed=bool(summary.get('passed') is True and contact_frames == 0
                  and max(errors) <= meta['route_audit']['tracking_margin_m'] and max(speeds) <= .2),
                  scope='Recorded truth-based route evaluation; not perception or impact',
                  route_frames=len(errors), route_contact_frames=contact_frames,
                  path_error_max_m=max(errors), path_error_p95_m=float(np.percentile(errors, 95)),
                  speed_max_m_s=float(max(speeds)),
                  final_slot_error_m=float(np.linalg.norm(summary['final_local_position'][:2])),
                  flight_passed=summary['passed'])
    (output/'summary.json').write_text(json.dumps(report, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    a = np.asarray(samples)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    axes[0].plot(points[:, 0], points[:, 1], '--o', label='Audited centerline')
    axes[0].plot(a[:, 1], a[:, 2], label='Recorded UAV position')
    axes[0].set(xlabel='World X (m)', ylabel='World Y (m)', title='Staging transit and return')
    axes[0].axis('equal')
    axes[0].legend()
    axes[1].plot(a[:, 0], a[:, 3], label='World height')
    axes[1].set(xlabel='Physics time (s)', ylabel='World Z (m)', title='Recorded route height')
    axes[1].legend()
    for ax in axes:
        ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(output/'route.png', dpi=150)
    plt.close(fig)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.run_dir, args.output)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
