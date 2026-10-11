"""Build an isolated OpenVINS numerical variant; preserve the accepted build."""
import argparse
import difflib
import json
import os
from pathlib import Path
import shutil
import subprocess

from rm27.perception.sitl.dataset import digest
from rm27.perception.sitl.live import DEFAULT_NATIVE


def prepare(output, installation=DEFAULT_NATIVE):
    output = Path(output).resolve()
    installation = Path(installation).resolve()
    parent = json.loads(installation.read_text())
    for p, expected in parent['files_sha256'].items():
        if digest(p) != expected:
            raise ValueError('Parent installation changed: ' + p)
    binary_relative = Path('install/ov_msckf/lib/ov_msckf/run_subscribe_msckf')
    if digest(Path(parent['workspace']) / binary_relative) != parent['binary_sha256']:
        raise ValueError('Parent binary changed')
    output.mkdir(parents=True, exist_ok=False)
    source, workspace = output / 'source', output / 'workspace'
    shutil.copytree(parent['source'], source, symlinks=True)
    shutil.copytree(parent['workspace'], workspace, symlinks=True)
    cpp = source / 'ov_msckf/src/state/StateHelper.cpp'
    before = cpp.read_text()
    start = before.index('  // Invert our S (should we use a more stable method here??)')
    end = before.index('  // We should check if we are not positive semi-definitate', start)
    block = Path(__file__).with_name('covariance_update.txt').read_text()
    after = before[:start] + block + '\n' + before[end:]
    after = after.replace('#include "StateHelper.h"', '#include "StateHelper.h"\n#include <cstdlib>\n#include <fstream>\n#include <limits>')
    cpp.write_text(after)
    patch = ''.join(difflib.unified_diff(before.splitlines(keepends=True), after.splitlines(keepends=True),
        fromfile='a/ov_msckf/src/state/StateHelper.cpp', tofile='b/ov_msckf/src/state/StateHelper.cpp'))
    (output / 'covariance.patch').write_text(patch)
    command = ['docker', 'run', '--rm', '--network', 'none', '--cpus', '2', '--memory', '6g',
        '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'OPENBLAS_NUM_THREADS=1',
        '-e', 'CMAKE_BUILD_PARALLEL_LEVEL=1', '-e', 'MAKEFLAGS=-j1',
        '-v', f'{source}:/upstream:ro', '-v', f'{workspace}:/ws', parent['image'],
        'bash', '-c', 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --base-paths /upstream '
        '--packages-up-to ov_msckf --executor sequential --cmake-args '
        '-DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON -DDISABLE_MATPLOTLIB=ON']
    manifest = dict(command=command, parent_installation=str(installation),
        parent_installation_sha256=digest(installation), parent=parent,
        algorithm='checked LLT solve and full-state Joseph update; no covariance clipping',
        diagnostic_mode='RM27_COVARIANCE_FORM=original reproduces pinned inverse/subtraction',
        covariance_patch_sha256=digest(output / 'covariance.patch'),
        builder_sha256=digest(__file__), numerical_block_sha256=digest(Path(__file__).with_name('covariance_update.txt')))
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    with (output / 'build.log').open('x') as log:
        process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    binary = workspace / binary_relative
    result = dict(passed=process.returncode == 0 and binary.is_file(), returncode=process.returncode,
        flight_qualified=False, run_type='isolated native numerical variant build')
    if result['passed']:
        files = [cpp, *source.glob('config/euroc_mav/*.yaml'),
            source / 'ov_msckf/src/ros/ROS2Visualizer.h', source / 'ov_msckf/src/ros/ROS2Visualizer.cpp',
            *(workspace / 'install').rglob('*.so')]
        install = dict(parent, source=str(source), workspace=str(workspace),
            variant='isolated Joseph covariance development with original-form diagnostic',
            binary_sha256=digest(binary), covariance_patch_sha256=manifest['covariance_patch_sha256'],
            files_sha256={str(p):digest(p) for p in files})
        (output / 'installation.json').write_text(json.dumps(install, indent=2) + '\n')
    (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--installation', type=Path, default=DEFAULT_NATIVE)
    args = parser.parse_args()
    raise SystemExit(0 if prepare(args.output, args.installation)['passed'] else 1)
