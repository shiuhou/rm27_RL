"""Audit one closed image/VIO development flight without changing its result."""
import argparse
import json
from pathlib import Path

from rm27.autopilot.px4.image_batch import assess
from rm27.autopilot.px4.return_fault_batch import digest
from rm27.experiments.recovery_membership import run as membership
from rm27.experiments.target_region_audit import audit_recording
from rm27.telemetry.ulog import analyze


def run(recording, surface, recovery, output, *, termination=False):
    recording, surface, recovery, output = map(
        lambda p: Path(p).resolve(), (recording, surface, recovery, output))
    output.mkdir(parents=True, exist_ok=False)
    def read(name):
        return json.loads((recording / name).read_text())
    inputs = [recording / name for name in
              ['summary.json', 'vio/summary.json', 'observer_rate.json', 'metadata.json']]
    inputs += [surface, recovery, Path(__file__)]
    if termination:
        inputs += [recording/'condition_summary.json',recording/'termination_summary.json']
    logs = list((recording / 'log').rglob('*.ulg'))
    if len(logs) != 1:
        raise ValueError('Exactly one original ULog required')
    inputs += logs
    manifest = dict(input_sha256={str(p): digest(p) for p in inputs},
                    recording=str(recording), runner_source=Path(__file__).read_text())
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    log = analyze(logs[0], output / 'ulog_analysis')
    surfaces = audit_recording(recording, surface, output / 'local_surface_audit')
    poses = membership(recording, recovery, output / 'native_recovery_membership')
    result = assess(read('summary.json'), read('vio/summary.json'), log,
                    surfaces=surfaces, require_surfaces=True,
                    observer=read('observer_rate.json'), required_observer_interval_us=4000)
    result['checks']['native_recovery_membership'] = poses.get('passed') is True
    if termination:
        required=['physical_hover','physical_recovery','image_target','native_vio','ulog_integrity',
            'actual_ekf2_ev_fusion','no_unexpected_failsafe','recorded_local_surfaces',
            'measured_imu_observer','native_recovery_membership']
        result['checks']={k:result['checks'][k] for k in required}
        condition=read('condition_summary.json')
        result['checks'].update(failed_task=read('summary.json')['passed'] is False,
            termination=condition.get('passed') is True and condition.get('task_success') is False and
                condition.get('safe_termination',{}).get('passed') is True,
            independent_physical_monitor=read('summary.json').get('independent_physical_monitor',{}).get('passed') is True)
        result['task_success']=False
    result['passed'] = all(result['checks'].values())
    result.update(run_type='independent closed development recording audit',
                  input_summary=str(recording / 'summary.json'))
    (output / 'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    for flag in ['surface', 'recovery', 'output']:
        parser.add_argument('--'+flag, type=Path, required=True)
    parser.add_argument('--termination',action='store_true')
    args = parser.parse_args()
    raise SystemExit(0 if run(args.recording, args.surface, args.recovery, args.output,
        termination=args.termination)['passed'] else 1)
