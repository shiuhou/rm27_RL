"""Explicit isolated native VIO development, using the existing physical mission."""
import argparse
import json
from pathlib import Path
import subprocess

from rm27.autopilot.px4 import live_flight
from rm27.autopilot.px4.return_fault_batch import frozen_inputs, ROOT, digest
from rm27.experiments.image_conditions import execute
from rm27.autopilot.px4 import image_mission
from .native_failure_termination import guarded_runtime


def run(args):
    installation = args.installation.resolve()
    install = json.loads(installation.read_text())
    manifest_path = args.output.with_name(args.output.name + '-variant-manifest.json')
    hashes = frozen_inputs(args.scene, args.conditions)
    files = [installation, args.config, args.camera_config, args.audit, Path(__file__),
             Path(__file__).with_name('native_failure_termination.py')]
    hashes.update({str(p.resolve()):digest(p) for p in files})
    hashes.update(install['files_sha256'])
    binary = Path(install['workspace'])/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    hashes[str(binary)] = digest(binary)
    manifest = dict(input_sha256=hashes, variant_installation=str(installation),
        variant=install['variant'], gate='DEVELOPMENT_ONLY', flight_qualified=False,
        injected_native_delay_s=args.inject_native_failure_delay,
        termination_enabled=args.terminate_native_failure,
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    for flag in ['audit','config','camera-config','conditions','installation','output']:
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--instance',type=int,default=46)
    parser.add_argument('--terminate-native-failure',action='store_true')
    parser.add_argument('--inject-native-failure-delay',type=float)
    raise SystemExit(run(parser.parse_args()))
