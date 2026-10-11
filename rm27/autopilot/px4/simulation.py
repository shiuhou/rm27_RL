"""Supported PX4 prototype workflow: python -m rm27 px4-sim --help."""
import argparse
from functools import partial
import hashlib
import importlib
import json
from pathlib import Path
import subprocess
import sys
import traceback

from . import COMMIT, VERSION
from .build import DEFAULT_SOURCE, verify

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = ROOT / 'configs/px4_simulation.json'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def configuration(path):
    path = Path(path).resolve()
    value = json.loads(path.read_text())
    if value.get('schema_version') != 1:
        raise ValueError('Unsupported PX4 simulation configuration')
    # Config paths are repository-relative, independently of the invoking cwd.
    names = ('scene', 'dynamics', 'base_target', 'down_camera', 'corridor',
             'mission', 'camera', 'conditions')
    inputs = {key: (ROOT / value[key]).resolve() for key in names}
    for source in inputs.values():
        if not source.is_file():
            raise FileNotFoundError(source)
    return inputs


def doctor(config=DEFAULT_CONFIG, *, perception=True):
    """Readiness only: no downloads, containers, UART or flight processes."""
    checks = {}
    def check(name, operation):
        try:
            checks[name] = dict(passed=True, evidence=operation())
        except Exception as exc:
            checks[name] = dict(passed=False, failure=f'{type(exc).__name__}: {exc}')
    inputs = configuration(config)
    def px4():
        source = verify(DEFAULT_SOURCE)
        binary = source / 'build/px4_sitl_default/bin/px4'
        if not (binary.parent.parent / 'etc/init.d-posix/rcS').is_file():
            raise FileNotFoundError('Build px4_sitl_default first')
        return dict(commit=COMMIT, version=VERSION, binary=str(binary), sha256=digest(binary))
    check('px4', px4)
    for module in ('mujoco', 'numpy', 'scipy', 'cv2', 'OCP', 'pymavlink', 'pyulog'):
        check(module, lambda module=module: getattr(importlib.import_module(module), '__version__', 'imported'))
    scene = json.loads(inputs['scene'].read_text())
    def field():
        path = ROOT / scene['field']['path']
        actual = subprocess.check_output(['git', '-C', str(path.parent), 'rev-parse', 'HEAD'], text=True).strip()
        if actual != scene['field']['commit']:
            raise ValueError('Field revision mismatch')
        subprocess.run(['git', '-C', str(path.parent), 'diff', '--quiet', 'HEAD', '--', 'model.xml', 'assets'], check=True)
        return actual
    check('field', field)
    def cad():
        path = ROOT / scene['dock']['source']
        actual = digest(path)
        if actual != scene['dock']['source_sha256']:
            raise ValueError('Dock CAD hash mismatch')
        return dict(path=str(path), sha256=actual)
    check('dock_cad', cad)
    if perception:
        from ...perception.sitl.live import DEFAULT_NATIVE
        def native():
            install = json.loads(DEFAULT_NATIVE.read_text())
            if install['commit'] != '69488123ed9362dd44b6f28e7f4680abbff1442b' or not install.get('instrumentation_sha256'):
                raise ValueError('Pinned native installation required')
            for name, expected in install['files_sha256'].items():
                if digest(name) != expected:
                    raise ValueError('Native dependency changed: ' + name)
            binary = Path(install['workspace']) / 'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
            if digest(binary) != install['binary_sha256']:
                raise ValueError('Native binary changed')
            subprocess.run(['docker', 'image', 'inspect', install['image']], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=15)
            return dict(installation=str(DEFAULT_NATIVE), binary_sha256=install['binary_sha256'], image=install['image'])
        check('fixed_simulation_perception', native)
    return dict(passed=all(row['passed'] for row in checks.values()), checks=checks,
                run_type='local dependency readiness', hardware_connected=False)


def build(output, config=DEFAULT_CONFIG):
    """Rebuild dock, provisional target, tags and surveyed-route audit here."""
    from ...__main__ import build as build_dock
    from ...integration.base_target import build as build_target
    from ...perception.sitl.dock_slots import build as build_tags
    from ...experiments.corridor_audit import audit
    inputs = configuration(config)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = dict(passed=False, run_type='scene build and static surveyed-route audit',
                  config=str(Path(config).resolve()), input_sha256={str(p): digest(p) for p in inputs.values()},
                  limitations='Provisional geometry; not continuous collision or flight certification')
    try:
        build_dock(inputs['scene'], output / 'dock', inputs['dynamics'])
        build_target(output / 'dock/scene.xml', output / 'target', inputs['base_target'])
        tags = build_tags(output / 'target/scene.xml', output / 'tags', inputs['down_camera'])
        settings = json.loads(inputs['corridor'].read_text())
        corridor = audit(output / 'tags/scene.xml', settings['waypoints_world_m'], output / 'audit',
                         spacing=settings['spacing_m'], tracking_margin=settings['tracking_margin_m'])
        result.update(passed=tags['passed'] is True and corridor['passed'] is True,
                      scene=str(output / 'tags/scene.xml'), scene_sha256=tags['scene_sha256'],
                      audit=str(output / 'audit/summary.json'))
    except Exception as exc:
        result['failure'] = f'{type(exc).__name__}: {exc}'
        (output / 'failure.txt').write_text(traceback.format_exc())
    (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def run(output, config=DEFAULT_CONFIG, *, workspace=None, instance=60, view=False, speedup=None):
    from .flight import run as flight_run, PROTOTYPE_MISSION_POLICY
    from .initial_admission import StableInitialAdmission
    from ...experiments.image_conditions import execute
    inputs = configuration(config)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    speedup = (1. if view else 2.) if speedup is None else speedup
    result = dict(passed=False, autopilot='px4', firmware=VERSION,
                  run_type=('live-view' if view else 'headless') + ' closed-loop SITL',
                  gate_version='PX4_PROTOTYPE_MIGRATION_V1',
                  command=sys.argv, config_sha256=digest(config),
                  scope='Representative R&D integration; no reliability or real-flight qualification')
    try:
        readiness = doctor(config)
        (output / 'dependencies.json').write_text(json.dumps(readiness, indent=2) + '\n')
        if not readiness['passed']:
            raise RuntimeError('Missing dependencies; inspect dependencies.json or px4-sim doctor')
        if workspace is None:
            scene_build = build(output / 'scene', config)
        else:
            scene_build = json.loads((Path(workspace) / 'summary.json').read_text())
            for name, expected in scene_build['input_sha256'].items():
                if digest(name) != expected:
                    raise ValueError('Built scene input changed: ' + name)
        if scene_build['passed'] is not True:
            raise ValueError('Scene build/audit failed')
        result['scene_build'] = scene_build
        runner = partial(flight_run, policy=PROTOTYPE_MISSION_POLICY)
        execute(scene_build['scene'], scene_build['audit'], inputs['mission'], inputs['camera'],
                inputs['conditions'], output / 'flight', instance=instance, flight_runner=runner,
                admission=StableInitialAdmission(), view=view, speedup=speedup)
        task = json.loads((output / 'flight/summary.json').read_text())
        result['task'] = task
        # Parsed flight telemetry alone does not establish actual EV fusion.
        from ...telemetry.ulog import analyze
        logs = sorted((output / 'flight').rglob('*.ulg'))
        if not logs:
            raise ValueError('No PX4 ULog was recorded')
        log_results = [analyze(path, output / f'ulog-{i:02d}') for i, path in enumerate(logs)]
        fusion = any(flags.get('cs_ev_pos', {}).get('ever') is True and
                     flags.get('cs_ev_vel', {}).get('ever') is True
                     for log in log_results for flags in log['fusion'].values())
        result['checks'] = dict(task=task.get('passed') is True,
                               parameters=task.get('parameters_verified') is True,
                               auto_disarm=task.get('auto_disarm', {}).get('confirmed') is True,
                               ulog=all(log['passed'] for log in log_results), actual_ev_fusion=fusion)
        result['passed'] = all(result['checks'].values())
    except (Exception, KeyboardInterrupt) as exc:
        result['failure'] = f'{type(exc).__name__}: {exc}'
        (output / 'failure.txt').write_text(traceback.format_exc())
    (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def add_parser(subparsers):
    parser = subparsers.add_parser('px4-sim', help='Build/run/check/replay the current PX4 prototype workflow')
    sub = parser.add_subparsers(dest='px4_action', required=True)
    p = sub.add_parser('doctor', help='Check local dependencies without launching a flight')
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--without-perception', action='store_true')
    p = sub.add_parser('build', help='Rebuild the 150 g scene and reviewed route')
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('run', help='Complete current simulation chain using fixed perception')
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--workspace', type=Path, help='Reuse an unchanged px4-sim build directory')
    p.add_argument('--instance', type=int, default=60)
    p.add_argument('--view', action='store_true')
    p.add_argument('--speedup', type=float)
    p = sub.add_parser('replay', help='Exact recorded motor-input physics check')
    p.add_argument('run', type=Path, help='Flight directory containing metadata.json')
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('log', help='Read-only PX4 ULog analysis')
    p.add_argument('input', type=Path)
    p.add_argument('--output', type=Path, required=True)


def dispatch(args):
    if args.px4_action == 'doctor':
        result = doctor(args.config, perception=not args.without_perception)
    elif args.px4_action == 'build':
        result = build(args.output, args.config)
    elif args.px4_action == 'run':
        result = run(args.output, args.config, workspace=args.workspace, instance=args.instance,
                     view=args.view, speedup=args.speedup)
    elif args.px4_action == 'replay':
        from .replay import replay
        result = replay(args.run, args.output)
    else:
        from ...telemetry.ulog import analyze
        result = analyze(args.input, args.output)
    print(json.dumps({key: value for key, value in result.items() if key != 'task'}, indent=2))
    return 0 if result['passed'] else 1
