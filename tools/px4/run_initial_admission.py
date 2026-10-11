"""Fresh stable first-registration development; frozen batches are unchanged."""
import argparse
import json
from pathlib import Path

from rm27.autopilot.px4.return_fault_batch import frozen_inputs,digest
from rm27.experiments import vio_initial_admission
from rm27.experiments.image_conditions import execute
from .initial_vio_admission import StableInitialAdmission
from .ground_rest import prepare as prepare_ground_rest
from .ground_rest import biased_runtime
from .landing_alignment_pulse import pulsed_runtime
from .physical_fixture import fixture_type
from rm27.autopilot.px4 import image_mission


def run(args):
    if args.causal_mission_inputs and not args.ground_rest_confirmation:
        raise ValueError('Causal mission inputs require the explicit saved landing runner')
    settings=json.loads(args.conditions.read_text())
    if settings.get('initial_vio_admission') is not True:
        raise ValueError('Explicit initial-admission configuration required')
    hashes=frozen_inputs(args.scene,args.conditions)
    files=[args.audit,args.config,args.camera_config,Path(__file__),Path(__file__).with_name('initial_vio_admission.py'),
           Path(__file__).with_name('ground_rest.py'),Path(__file__).with_name('landing_alignment_pulse.py'),
           Path(__file__).with_name('causal_flight.py'),Path(__file__).with_name('physical_fixture.py')]
    hashes.update({str(p.resolve()):digest(p) for p in files})
    guard=StableInitialAdmission()
    manifest=dict(input_sha256=hashes,guard=guard.__dict__ | {'consistent':[]},
        launcher_source=Path(__file__).read_text(),guard_source=Path(__file__).with_name('initial_vio_admission.py').read_text(),
        additional_sources={str(p):p.read_text() for p in files if p.suffix=='.py'},
        ground_rest_confirmation=args.ground_rest_confirmation,
        post_land_vertical_bias_m_s=args.post_land_vertical_bias,
        retry_final_alignment=args.retry_final_alignment,
        final_image_shift_px=args.final_image_shift_px,
        causal_mission_inputs=args.causal_mission_inputs,
        gate='DEVELOPMENT_ONLY',scope='First rigid registration and explicit post-LAND evidence; no truth in actor or ongoing realignment')
    with args.output.with_name(args.output.name+'-admission-manifest.json').open('x') as stream:
        stream.write(json.dumps(manifest,indent=2)+'\n')
    original=vio_initial_admission.initial_consistent
    original_run=image_mission.run
    selected_run=original_run
    if args.ground_rest_confirmation:
        selected_run,_=prepare_ground_rest(args.output,retry_alignment=args.retry_final_alignment,
            causal_inputs=args.causal_mission_inputs)
    elif args.retry_final_alignment:
        raise ValueError('Final alignment retry requires the explicit landing development runner')
    if settings.get('physical_fixture'):
        if not args.causal_mission_inputs or not args.ground_rest_confirmation:
            raise ValueError('Explicit saved causal runner required for physical uncertainty')
        selected_run.__globals__['MicroPlant']=fixture_type(selected_run.__globals__['MicroPlant'],settings['physical_fixture'])
    def development_run(*positional,**keywords):
        if args.final_image_shift_px:
            keywords['runtime_factory']=pulsed_runtime(keywords['runtime_factory'],args.final_image_shift_px)
        if args.post_land_vertical_bias:
            keywords['runtime_factory']=biased_runtime(keywords['runtime_factory'],args.post_land_vertical_bias)
        return selected_run(*positional,**keywords)
    image_mission.run=development_run
    vio_initial_admission.initial_consistent=guard
    try:
        code=execute(args.scene,args.audit,args.config,args.camera_config,args.conditions,args.output,instance=args.instance)
        if args.final_image_shift_px:
            summary=json.loads((args.output/'summary.json').read_text())
            pulse=json.loads((args.output/'landing_pixel_pulse.json').read_text())
            checks=dict(physical_task=summary.get('passed') is True,
                changed_pixels=bool(pulse['records']) and all(
                    r['original_sha256']!=r['delivered_sha256'] for r in pulse['records']),
                final_alignment_retry=any(e.get('event')=='landing_alignment_retry' for e in summary['events']))
            result=dict(passed=all(checks.values()),checks=checks,
                run_type='declared pixel-translation final-alignment retry development')
            (args.output/'pulse_summary.json').write_text(json.dumps(result,indent=2)+'\n')
            return 0 if result['passed'] else 1
        return code
    finally:
        vio_initial_admission.initial_consistent=original
        image_mission.run=original_run


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path)
    for flag in ['audit','config','camera-config','conditions','output']:
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--instance',type=int,default=44)
    parser.add_argument('--ground-rest-confirmation',action='store_true')
    parser.add_argument('--post-land-vertical-bias',type=float,default=0.)
    parser.add_argument('--retry-final-alignment',action='store_true')
    parser.add_argument('--final-image-shift-px',type=float,default=0.)
    parser.add_argument('--causal-mission-inputs',action='store_true')
    raise SystemExit(run(parser.parse_args()))
