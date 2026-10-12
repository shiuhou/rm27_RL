"""Bounded content identity for the code/config actually used, including dirty trees."""
from __future__ import annotations

import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import platform
import subprocess

from rm27.experiments.aggressive_contact.model import ROOT

def source_hash(path):
    path = Path(path)
    try:
        name = str(path.relative_to(ROOT))
    except ValueError:
        name = str(path)
    return {"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def provenance(env, *, algorithm, seed=0, extra_sources=None):
    # The first-party filenames bound provenance collection; no source copies,
    # historical logs, assets or unrelated submodule scans are generated here.
    sources = dict(env.sources)
    sources["lock"] = ROOT / "requirements-contact-rl.lock"
    sources["packaging"] = ROOT / "pyproject.toml"
    sources["target_geometry"] = ROOT / "configs/base_target.json"
    code = ["rm27/__init__.py", "rm27/integration/__init__.py",
            "rm27/experiments/__init__.py", "rm27/experiments/aggressive_contact/__init__.py",
            "rm27/experiments/calibration.py", "rm27/integration/drone.py",
            "rm27/experiments/contact_metrics.py"]
    code += [f"rm27/experiments/aggressive_contact/{name}.py"
             for name in ("model", "control", "teacher_interface", "evaluation")]
    code += [f"research/contact_rl/src/rm27_contact_rl/{name}.py"
             for name in ("__init__", "env", "rm27_adapter", "baselines", "__main__", "provenance", "ppo")]
    sources.update(extra_sources or {})
    source_hashes = {name: source_hash(path) for name, path in sources.items()}
    code_hashes = [source_hash(ROOT / path) for path in code]
    def git(*args):
        result = subprocess.run(["git", *args], cwd=ROOT, capture_output=True,
                                check=True, text=True)
        return result.stdout.strip()
    # Container packaging supplies host Git identity, while the hashes above
    # always describe the code/config actually used, including source mounts.
    build_source_path = os.environ.get("CONTACT_RL_BUILD_SOURCE")
    build_source = (json.loads(Path(build_source_path).read_text())
                    if build_source_path else None)
    runtime_source = os.environ.get("CONTACT_RL_GIT_SOURCE")
    if runtime_source:
        identity = json.loads(runtime_source)
    elif build_source is not None:
        identity = build_source
    else:
        status = git("status", "--porcelain", "--untracked-files=normal")
        identity = dict(git_commit=git("rev-parse", "HEAD"), git_dirty=bool(status),
                        git_status=status.splitlines())
    versions = {name: metadata.version(name) for name in ("mujoco", "numpy", "scipy", "gymnasium")}
    result = dict(git_commit=identity["git_commit"], git_dirty=identity["git_dirty"],
                git_status=identity["git_status"],
                source_hashes=source_hashes, code_hashes=code_hashes, package_versions=versions,
                python_version=platform.python_version(), machine=platform.machine(),
                scene_seed=None, environment_random_seed=seed,
                randomization="DISABLED_FIXED_NOMINAL_SCENE",
                algorithm=algorithm)
    if algorithm == "PPO_CPU":
        versions.update({name: metadata.version(name) for name in ("torch", "stable-baselines3")})
    if build_source is not None:
        result["container"] = dict(
            image_id=os.environ.get("CONTACT_RL_IMAGE_ID", "UNKNOWN"),
            image_reference=os.environ.get("CONTACT_RL_IMAGE_REFERENCE", "UNKNOWN"),
            execution_source=os.environ.get("CONTACT_RL_EXECUTION_SOURCE", "IMAGE"),
            build_source=build_source)
    return result
