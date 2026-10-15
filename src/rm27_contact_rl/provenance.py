"""Bounded content identity for the code/config actually used, including dirty trees."""
from __future__ import annotations

import hashlib
from importlib import metadata
import importlib
import json
import os
from pathlib import Path
import platform
import subprocess

from .paths import ROOT

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
    sources.update(extra_sources or {})
    source_hashes = {name: source_hash(path) for name, path in sources.items()}
    modules = [f"rm27_contact_physics.{name}" for name in
               ("drone", "calibration", "contact_metrics", "model", "control", "teacher_interface", "evaluation")]
    if getattr(env, "task_id", None) == "agile_terminal_arrival_v1":
        modules.append("rm27_contact_physics.arrival")
    modules += ["rm27_contact_physics"]
    physics_hashes = [dict(module=name, **source_hash(importlib.import_module(name).__file__))
                      for name in modules]
    rl_root = Path(__file__).resolve().parent
    code_hashes = [source_hash(rl_root / f"{name}.py") for name in
                   ("__init__", "env", "rm27_adapter", "baselines", "__main__", "provenance", "ppo", "paths")]
    code_hashes += physics_hashes
    if getattr(env, "task_id", None) == "agile_terminal_arrival_v1":
        code_hashes.append(source_hash(rl_root / "arrival_env.py"))
    physics_dist = metadata.distribution("rm27-contact-physics")
    direct_url = physics_dist.read_text("direct_url.json")
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
        try:
            status = git("status", "--porcelain", "--untracked-files=normal")
            identity = dict(git_commit=git("rev-parse", "HEAD"), git_dirty=bool(status),
                            git_status=status.splitlines())
        except (FileNotFoundError, subprocess.CalledProcessError):
            identity = dict(git_commit="UNKNOWN", git_dirty=None, git_status=[])
    versions = {name: metadata.version(name) for name in ("mujoco", "numpy", "scipy", "gymnasium")}
    result = dict(git_commit=identity["git_commit"], git_dirty=identity["git_dirty"],
                git_status=identity["git_status"],
                source_hashes=source_hashes, code_hashes=code_hashes, package_versions=versions,
                python_version=platform.python_version(), machine=platform.machine(),
                scene_seed=None, environment_random_seed=seed,
                randomization="DISABLED_FIXED_NOMINAL_SCENE",
                algorithm=algorithm)
    result["physics_package"] = dict(version=physics_dist.version,
        direct_url=json.loads(direct_url) if direct_url else None, code_hashes=physics_hashes)
    if getattr(env, "task_id", None) == "agile_terminal_arrival_v1":
        result.update(task_id=env.task_id, observation_schema=env.observation_schema,
                      randomization="DECLARED_INITIAL_POSITION_ONLY_FIXED_DYNAMICS_AND_TARGET")
    if algorithm == "PPO_CPU":
        versions.update({name: metadata.version(name) for name in ("torch", "stable-baselines3")})
    if build_source is not None:
        result["container"] = dict(
            image_id=os.environ.get("CONTACT_RL_IMAGE_ID", "UNKNOWN"),
            image_reference=os.environ.get("CONTACT_RL_IMAGE_REFERENCE", "UNKNOWN"),
            execution_source=os.environ.get("CONTACT_RL_EXECUTION_SOURCE", "IMAGE"),
            build_source=build_source)
    return result
