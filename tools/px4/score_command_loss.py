"""Independently audit closed complete-route command-loss termination evidence."""
import argparse
import json
from pathlib import Path

from rm27.autopilot.px4.image_batch import assess
from rm27.autopilot.px4.return_fault_batch import digest
from rm27.experiments.recovery_membership import run as membership
from rm27.experiments.target_region_audit import audit_recording
from rm27.telemetry.ulog import analyze


def run(recording,surface,recovery,output):
    recording,surface,recovery,output=map(lambda p:Path(p).resolve(),(recording,surface,recovery,output))
    output.mkdir(parents=True,exist_ok=False)
    def read(name):return json.loads((recording/name).read_text())
    files=[recording/name for name in ['summary.json','command_loss_summary.json',
        'termination_summary.json','command_loss_injection.json','metadata.json','vio/summary.json','observer_rate.json']]
    logs=list((recording/'log').rglob('*.ulg'))
    if len(logs)!=1:raise ValueError('Exactly one original ULog required')
    files += [surface,recovery,Path(__file__),*logs]
    manifest=dict(input_sha256={str(p):digest(p) for p in files},runner_source=Path(__file__).read_text())
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    log=analyze(logs[0],output/'ulog_analysis')
    surfaces=audit_recording(recording,surface,output/'local_surface_audit')
    poses=membership(recording,recovery,output/'native_recovery_membership')
    qualification=assess(read('summary.json'),read('vio/summary.json'),log,surfaces=surfaces,
        require_surfaces=True,observer=read('observer_rate.json'),required_observer_interval_us=4000)
    required=['physical_hover','physical_recovery','image_target','native_vio','ulog_integrity',
        'actual_ekf2_ev_fusion','recorded_local_surfaces','measured_imu_observer']
    checks={k:qualification['checks'][k] for k in required}
    checks.update(failed_task=read('summary.json')['passed'] is False,
        continued_physical_termination=read('command_loss_summary.json')['passed'] is True,
        native_recovery_membership=poses['passed'] is True)
    result=dict(passed=all(checks.values()),checks=checks,task_success=False,
        resting_horizontal_error_m=read('termination_summary.json')['resting_horizontal_error_m'],
        run_type='independent recorded complete-route upstream command-loss audit',
        scope='Image-aligned assigned dock return; no arbitrary-route or invalid-navigation claim')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('recording',type=Path)
    for flag in ['surface','recovery','output']:parser.add_argument('--'+flag,type=Path,required=True)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.recording,args.surface,args.recovery,args.output)['passed'] else 1)
