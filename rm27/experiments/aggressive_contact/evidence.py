"""Write immutable run directories, exact source/config provenance and failures."""
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback

import mujoco
import numpy as np
import scipy

from .model import ROOT


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False))


def snapshot(output, cfg):
    output = Path(output)
    source = output / "source"
    source.mkdir()
    files = list(Path(__file__).parent.rglob("*.py")) + [
        ROOT / "rm27/integration/drone.py", ROOT / "rm27/experiments/contact_metrics.py",
        ROOT / cfg["vehicle_config"], ROOT / cfg["target_config"],
        ROOT / "configs/aggressive_contact.json",
        ROOT / "configs/aggressive_contact_v2.json", ROOT / "configs/contact_arena.json",
        ROOT / "configs/contact_teacher_aggressive.json", ROOT / "configs/contact_teacher_balanced.json",
        ROOT / "configs/contact_teacher_conservative.json", ROOT / "configs/contact_distillation.json", ROOT / "tests/test_contact_distillation.py", ROOT / "configs/contact_teacher.json", ROOT / "tests/test_contact_teacher.py",
        ROOT / "configs/contact_maneuver.json", ROOT / "tests/test_contact_maneuver.py",
        ROOT / "configs/contact_maneuver_fast.json", ROOT / "configs/contact_maneuver_early_pose.json",
        ROOT / "rm27/integration/base_target.py",
        ROOT / "tests/test_aggressive_contact.py", ROOT / "tests/test_contact_arena.py"]
    files += list((ROOT / 'configs/contact_phase3').glob('*.json'))
    files += [ROOT / 'tests/test_contact_deployment.py']
    files += list((ROOT / 'configs/contact_phase4').glob('*'))
    files += [ROOT / 'tests/test_contact_flight_control.py']
    files += list((ROOT / 'configs/contact_phase5').glob('*'))
    files += [ROOT / 'tests/test_contact_estimator_quality.py']
    hashes = {}
    for path in files:
        if not path.exists():
            continue
        relative = path.relative_to(ROOT)
        dest = source / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        content = path.read_bytes()
        dest.write_bytes(content)
        hashes[str(relative)] = hashlib.sha256(content).hexdigest()
    def git(*args):
        return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True)
    write_json(output/"provenance.json", dict(
        head=git("rev-parse", "HEAD").strip(), branch=git("branch", "--show-current").strip(),
        status=git("status", "--short"), worktrees=git("worktree", "list"),
        command=sys.argv, python=sys.executable, python_version=platform.python_version(),
        mujoco=mujoco.__version__, numpy=np.__version__, scipy=scipy.__version__,
        source_sha256=hashes))
    (output/"working.diff").write_text(git("diff", "--", "rm27/experiments/aggressive_contact", "configs/aggressive_contact.json", "tests/test_aggressive_contact.py"))
    write_json(output/"config.json", cfg)


def save_failure(output, exc):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output/"traceback.log").write_text(traceback.format_exc())
    result = dict(passed=False, failure_modes=["RUNNER_ERROR"], exception=repr(exc))
    write_json(output/"summary.json", result)
    return result
