"""Explicit isolated native VIO development, using the existing physical mission."""
import argparse
import json
from pathlib import Path
import subprocess

from rm27.autopilot.px4 import live_flight
from rm27.autopilot.px4.return_fault_batch import frozen_inputs, ROOT, digest
from rm27.experiments.image_conditions import execute
from rm27.autopilot.px4 import image_mission
from rm27.perception.sitl import live
from rm27.experiments import vio_initial_admission
from .native_failure_termination import guarded_runtime
from .initial_vio_admission import StableInitialAdmission


def run(args):
    installation = args.installation.resolve()
    install = json.loads(installation.read_text())
    manifest_path = args.output.with_name(args.output.name + '-variant-manifest.json')
    hashes = frozen_inputs(args.scene, args.conditions)
    files = [installation, args.config, args.camera_config, args.audit, Path(__file__),
             Path(__file__).with_name('native_failure_termination.py'),
             Path(__file__).with_name('initial_vio_admission.py')]
    hashes.update({str(p.resolve()):digest(p) for p in files})
    hashes.update(install['files_sha256'])
    binary = Path(install['workspace'])/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    hashes[str(binary)] = digest(binary)
    manifest = dict(input_sha256=hashes, variant_installation=str(installation),
        variant=install['variant'], gate='DEVELOPMENT_ONLY', flight_qualified=False,
        injected_native_delay_s=args.inject_native_failure_delay,
        termination_enabled=args.terminate_native_failure,
        stable_initial_admission=args.stable_initial_admission,
        estimator_sigma_px=args.estimator_sigma_px,
        additional_sources={str(p):p.read_text() for p in files if p.suffix=='.py'},
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        launcher_source=Path(__file__).read_text())
    with manifest_path.open('x') as stream:
        stream.write(json.dumps(manifest,indent=2)+'\n')
    original = live_flight.LiveVio
    class VariantVio(original):
        def __init__(self, *positional, **keywords):
            keywords['installation'] = installation
            super().__init__(*positional, **keywords)
    live_flight.LiveVio = VariantVio
    original_config = live.prepare_config
    original_admission = vio_initial_admission.initial_consistent
    def variant_config(*positional,**keywords):
        if args.estimator_sigma_px is not None:
            if not .5<=args.estimator_sigma_px<=4.:
                raise ValueError('Explicit bounded estimator pixel-noise sigma required')
            settings = dict(keywords.get('settings') or {})
            settings['estimator'] = dict(settings.get('estimator') or {},
                up_msckf_sigma_px=args.estimator_sigma_px,up_slam_sigma_px=args.estimator_sigma_px)
            keywords['settings'] = settings
        return original_config(*positional,**keywords)
    live.prepare_config = variant_config
    if args.stable_initial_admission:
        if json.loads(args.conditions.read_text()).get('initial_vio_admission') is not True:
            raise ValueError('Stable initial admission requires its explicit condition configuration')
        vio_initial_admission.initial_consistent = StableInitialAdmission()
    original_run = image_mission.run
    def variant_run(*positional,**keywords):
        if args.terminate_native_failure:
            keywords['runtime_factory'] = guarded_runtime(keywords['runtime_factory'],
                injected_delay_s=args.inject_native_failure_delay)
        return original_run(*positional,**keywords)
    image_mission.run = variant_run
    try:
        code = execute(args.scene,args.audit,args.config,args.camera_config,
                       args.conditions,args.output,instance=args.instance)
        result = dict(passed=code==0,task_success=code==0,safe_termination=False,
                      run_type='isolated native VIO development',flight_qualified=False)
        terminal = args.output/'termination_summary.json'
        if args.terminate_native_failure and terminal.is_file():
            score = json.loads(terminal.read_text())
            result.update(passed=score.get('passed') is True,task_success=False,
                          safe_termination=score.get('passed') is True,termination=score)
        (args.output/'native_variant_summary.json').write_text(json.dumps(result,indent=2)+'\n')
        return 0 if result['passed'] else 1
    finally:
        live_flight.LiveVio = original
        image_mission.run = original_run
        live.prepare_config = original_config
        vio_initial_admission.initial_consistent = original_admission


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    for flag in ['audit','config','camera-config','conditions','installation','output']:
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--instance',type=int,default=46)
    parser.add_argument('--terminate-native-failure',action='store_true')
    parser.add_argument('--inject-native-failure-delay',type=float)
    parser.add_argument('--stable-initial-admission',action='store_true')
    parser.add_argument('--estimator-sigma-px',type=float)
    raise SystemExit(run(parser.parse_args()))
