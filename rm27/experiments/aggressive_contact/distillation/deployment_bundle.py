"""Export the selected policy, nominal task constants and deployment contract."""
import argparse,json
from pathlib import Path
from .audit import run as audit
from ..evidence import write_json
from ..model import ROOT

SCHEMA=dict(version='rm27.hybrid.deployment.v2',
 vehicle='VehicleObservation(VehicleEstimate p/v/R/body_rate/timestamp/source NWU/FLU, velocity covariance)',
 target='TargetEstimate world NWU position/normal/covariance/timestamp/confidence; commit once at reset, age<=.25s; fixed scene only',
 actuator='command-driven nominal motor force N and effective thrust/mass ratio; no measured motors required',
 action='RateThrust: FLU body rates rad/s and physical collective N; AP FRD + normalized throttle packet builder requires explicit calibration and confirmed GUID_OPTIONS semantics',
 policy='learned degree5 spline p/v/a/j(t,mode), analytic geometric tracking, causal timestamp propagation, shared adaptive thrust estimate and estimated contact reset',
 privileged_only=['simulation qpos/qvel/actual actuator state','actual physical draw','actual collision bit and geometry',
                  'actual inner-loop gyro used by plant proxy, never by portable actor observer','held-recovery evaluator gate for return handoff'],
 deployment_inputs='estimator semantics only; present evaluation uses synthetic truth-derived measurements; no validated onboard estimator',
 confidence='four-sigma velocity endpoint heuristic in contact detector; not calibrated probability',
 legacy_MLP='32/96 element vectors and masks in v1 schema/per-model features.json; not used by hybrid',
 nominal_assumptions='vehicle constants unmeasured engineering assumptions',
 state_max_age_s=.1,propagation_cap_s=.035,control_update_hz=200,
 failsafe='API rejects unsupported/stale/nonfinite inputs; caller must implement and validate flight abort/fallback. Not a complete hardware flight safety implementation.')


def run(output):
    p=Path(output);spec=json.loads((ROOT/'configs/contact_phase3/deployment_selected.json').read_text())
    result=audit(ROOT/'artifacts/distill-data-01',ROOT/spec['checkpoint'],p)
    write_json(p/'deployment_options.json',spec['deployment']);write_json(p/'schema.json',SCHEMA)
    write_json(p/'phase3_splits.json',json.loads((ROOT/'configs/contact_phase3/splits.json').read_text()))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();print(json.dumps(run(a.output),indent=2))
