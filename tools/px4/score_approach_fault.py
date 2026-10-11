"""Independent task or pre-contact safe-return score for actual pixel faults."""
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
    summary=read('summary.json');fault=read('approach_target_fault_summary.json')
    files=[recording/n for n in ['summary.json','metadata.json','vio/summary.json',
        'observer_rate.json','approach_target_fault_summary.json','approach_target_injections.jsonl']]
    logs=list((recording/'log').rglob('*.ulg'))
    if len(logs)!=1:raise ValueError('Exactly one original ULog required')
    files.extend([surface,recovery,Path(__file__),*logs])
    (output/'manifest.json').write_text(json.dumps(dict(input_sha256={str(p):digest(p) for p in files},
        source=Path(__file__).read_text()),indent=2)+'\n')
    log=analyze(logs[0],output/'ulog_analysis')
    surfaces=audit_recording(recording,surface,output/'local_surface_audit')
    qualified=assess(summary,read('vio/summary.json'),log,surfaces=surfaces,require_surfaces=True,
        observer=read('observer_rate.json'),required_observer_interval_us=4000)
    if fault['task_success']:
        checks=qualified['checks']
        checks['native_recovery_membership']=membership(recording,recovery,output/'native_recovery_membership')['passed']
    else:
        required=['physical_hover','physical_rest','assigned_tag','auto_disarm','native_vio',
            'ulog_integrity','actual_ekf2_ev_fusion','no_unexpected_failsafe',
            'recorded_local_surfaces','measured_imu_observer']
        checks={k:qualified['checks'][k] for k in required}
        checks.update(failed_task=summary['passed'] is False,
            no_target_contact=summary.get('independent_recovery_score',{}).get('failures')==['physical_target_contact_missing'])
    checks.update(pixel_fault_handled=fault['passed'] is True,
        causal_actor=summary.get('actor_input_policy')=='PX4 telemetry and current image observations; physical monitors score after actor completion',
        independent_physical_monitor=summary.get('independent_physical_monitor',{}).get('passed') is True)
    result=dict(passed=all(checks.values()),checks=checks,task_success=fault['task_success'],
        safe_termination=not fault['task_success'] and all(checks.values()),
        resting_horizontal_error_m=summary.get('independent_resting_score',{}).get('horizontal_error_m'),
        run_type='independent recorded-image-fault flight, ULog and local surface audit',
        scope='Declared temporary target loss or pre-contact abort with normal AprilTag return. Aborted route has no contact/recovery claim.')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    for flag in ['surface','recovery','output']:parser.add_argument('--'+flag,type=Path,required=True)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.recording,args.surface,args.recovery,args.output)['passed'] else 1)
