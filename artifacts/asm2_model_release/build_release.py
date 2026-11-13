"""Stage the ASM2 release from the RM27 workspace using its pinned inputs.

Run with the project's native-env and PYTHONPATH=src:.  Review validation
results in the stage before calling seal.  Staging never edits the sources.
"""

import argparse
import ast
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
VERSION = "asm2-model-v1.0.0"
PHASE = Path(".agents/runs/asm2-terminal")
R2 = PHASE / "phase4r2"
R1 = PHASE / "phase4r"
CONTACT = Path(".agents/runs/asm2-contact")
STAGE1 = Path(".agents/runs/rm27-stage1")
ASSET = Path("assets/asm2")
SOURCE = ASSET / "source"
MODULE = Path("src/rm27_asm2")


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def select(path, names):
    source = (ROOT / path).read_text()
    nodes = {node.name: node for node in ast.parse(source).body
             if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
    return "\n\n\n".join(ast.get_source_segment(source, nodes[name]) for name in names)


def stage(output):
    from rm27_asm2.asset import load_inputs, read_json
    from rm27_asm2.flight import load_profile
    from rm27_asm2.contact_task import load_contact_spec
    from rm27_asm2.native_resolution import _verify_frozen_geometry
    from rm27_asm2.proxy import load_proxy
    from rm27_asm2.flight_native import _flight_mjcf
    import numpy as np

    output.mkdir()
    origins = {}

    def copy(path, target=None):
        path = Path(path)
        target = Path(target or path)
        dest = output / target
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / path, dest)
        origins[str(target)] = {"source": str(path), "sha256": digest(ROOT / path)}

    config, _, geometry, mass = load_inputs()
    spec, profile, proxy = load_contact_spec(), load_profile(), load_proxy()
    receipt = read_json(ROOT / R2 / "backend-receipt.json")
    for name, expected in receipt["run_pins"].items():
        if digest(ROOT / name) != expected:
            raise ValueError(f"Phase 4R.2 receipt pin changed: {name}")
    if digest(ROOT / SOURCE / "cad/asm2.STEP") != mass["source_sha256"]:
        raise ValueError("Original STEP differs from CAD mass properties")
    candidate = config["candidate_usd"]
    proxy_usd = CONTACT / "proxy-247f076685d8.usda"
    baseline = read_json(ROOT / CONTACT / "isaac-proxy-field.json")
    if (digest(ROOT / proxy_usd) != baseline["asset_sha256"]
            or digest(ROOT / ASSET / "proxy/manifest.json") != baseline["proxy"]["manifest_sha256"]
            or spec.contract_sha256 != receipt["contact_contract_sha256"]
            or spec.contract_sha256 != baseline["contact_config_sha256"]):
        raise ValueError("Proxy layers or scorer contract differ from the frozen source")

    mjcf = ET.parse(ROOT / SOURCE / "mujoco/model.xml")
    source_manifest = {name: sha for sha, name in
                       (line.split("  ", 1) for line in (ROOT / SOURCE / "MANIFEST_SHA256.txt").read_text().splitlines())}
    relative = {f"mujoco/{mesh.attrib['file']}" for mesh in mjcf.findall("./asset/mesh")}
    relative.update(config["pins"], proxy["source_pins"])
    relative.update(("cad/asm2.STEP", "README.txt", "metadata/parts.json", "metadata/coacd_parameters.json"))
    for name in sorted(relative):
        if name in source_manifest and digest(ROOT / SOURCE / name) != source_manifest[name]:
            raise ValueError(f"Original ASM2 source hash mismatch: {name}")
        copy(SOURCE / name)
    for row in proxy["entries"]:
        copy(ASSET / "proxy" / row["mesh"])
    for path in (
        ASSET / "proxy/manifest.json", candidate, proxy_usd, config["field_usd"],
        STAGE1 / "contact-scene.usda", R2 / "proxy-with-reference-body.usda",
        "configs/rm27_asm2_collision.json", "configs/rm27_asm2_flight.json", "configs/rm27_asm2_contact.json",
        *profile.source_pins.keys(), *(spec.config[key]["path"] for key in
                                      ("field_geometry", "registration", "field_fixture", "diagnostic_fixtures")),
        "simpleflight_lab/rm27_legacy.py", "upstream/rm27_RL/configs/flight_probe/task_b.json",
    ):
        if str(path) not in origins:
            copy(path)
    # Relative references are part of the frozen USD byte stream.
    for path in (Path(candidate), proxy_usd, Path(config["field_usd"]), R2 / "proxy-with-reference-body.usda"):
        for reference in re.findall(r"@([^@]+)@", (output / path).read_text()):
            target = (output / path.parent / reference).resolve()
            if not target.is_relative_to(output.resolve()) or not target.is_file():
                raise ValueError(f"Unresolved USD dependency: {path} -> {reference}")
    for name in ("backend-receipt.json", "geometry-distance.json", "isaac-corrected-layout0.json",
                 "isaac-corrected-layout1.json"):
        copy(R2 / name, Path("evidence/phase4r2") / name)
    copy(R1 / "native-gate-braking.json", "evidence/phase4r2/native-gate-braking.json")

    params = {}
    field, meshes, field_origin = _verify_frozen_geometry(contact_parameters=params)
    if field != spec.field_geometry:
        raise ValueError("Verified native Field differs from the contact contract")
    arrays = {f"{key}_{suffix}": value for key, pair in sorted(meshes.items())
              for suffix, value in zip(("vertices", "faces"), pair)}
    np.savez_compressed(output / "evidence/field-hulls.npz", **arrays)
    write_json(output / "evidence/field-materials.json", {
        "provenance": field_origin,
        "hulls_sha256": digest(output / "evidence/field-hulls.npz"),
        "contact_parameters": params,
    })
    # Load only the already registered fixture snapshots and tape from the
    # historical gate. The release does not carry reset corpora or checkpoints.
    from importlib.util import module_from_spec, spec_from_file_location
    loader = spec_from_file_location("phase4r_contact_gate", ROOT / R1 / "contact_gate.py")
    gate = module_from_spec(loader)
    loader.loader.exec_module(gate)
    _, _, controls = gate.load_controls(include_braking=True)
    write_json(output / "evidence/phase4r2/controls.json", [
        {key: np.asarray(value).tolist() if isinstance(value, (np.ndarray, np.generic)) else value
         for key, value in row.items()} for row in controls
    ])

    for name in ("__init__", "asset", "contacts", "contact_task", "flight", "flight_native",
                 "flight_suite", "native", "contact_native", "native_resolution", "proxy", "isaac", "terminal_isaac"):
        copy(MODULE / f"{name}.py")
    # Keep the simulator and scorer unchanged. Point the portable native
    # plant at the frozen, verified Field snapshot instead of the historical MJB.
    path = output / MODULE / "native_resolution.py"
    old = select(MODULE / "native_resolution.py", ("_verify_frozen_geometry",))
    source = path.read_text()
    assert source.count(old) == 1
    path.write_text(source.replace(old, (HERE / "build/field_loader.py").read_text().rstrip()))
    (output / MODULE / "contact_isaac.py").write_text(
        '"""Shared paths for the portable Isaac contact regression."""\n\nfrom .asset import ROOT\n\n'
        'EVIDENCE = ROOT / ".agents/runs/asm2-contact"\n'
        'PROXY_MANIFEST = ROOT / "assets/asm2/proxy/manifest.json"\n'
    )
    # Retain the baseline receipt used by TerminalSim's optional bare-proxy loader.
    copy(CONTACT / "isaac-proxy-field.json")
    reused = {
        R2 / "matched_contact.py": ("corrected_asset", "TraceSim", "native", "isaac"),
        MODULE / "terminal_preflight.py": ("batch",),
        MODULE / "terminal_native_eval.py": ("restore",),
    }
    harness = (HERE / "build/gate.py").read_text()
    body = "\n\n\n".join(select(path, names) for path, names in reused.items())
    body = body.replace("    from rm27_asm2.terminal_native_eval import restore\n", "")
    body = body.replace("def restore(plant, state, motor_thrust, motor_target):\n", "def restore(plant, state, motor_thrust, motor_target):\n    import mujoco\n    from .native import records\n\n")
    body = body.replace('read_json(SOURCE / "native-gate-braking.json")', 'read_json(EVIDENCE_DIR / "native-gate-braking.json")')
    body = body.replace("    order = np.arange(sim.worlds)", "    runtime = runtime_settings(sim)\n    order = np.arange(sim.worlds)")
    body = body.replace("                rollout_seconds=", "                runtime=runtime, rollout_seconds=")
    (output / MODULE / "release_gate.py").write_text(harness.replace("# INSERT_FROZEN_FUNCTIONS", body))
    for name in ("validate_release.py", "load_aircraft.py", "verify_manifest.py", "run_isaac_gate.sh", "requirements.txt", "README.md"):
        shutil.copyfile(HERE / "build" / name, output / name)
    (output / "models").mkdir()
    (output / "models/free_flight.xml").write_text(_flight_mjcf(profile) + "\n")
    write_json(output / "models/plant.json", {
        "mass_kg": profile.mass_kg, "com_body_m": [0., 0., 0.],
        "inertia_body_kg_m2": profile.inertia_kg_m2.tolist(),
        "rotor_positions_body_m": profile.hubs_m.tolist(), "rotor_axes_body": profile.axes.tolist(),
        "yaw_torque_per_thrust_m": profile.yaw_torque_per_thrust_m.tolist(),
        "max_rotor_thrust_N": profile.max_rotor_thrust_N, "motor_time_constant_s": profile.motor_time_constant_s,
        "rate_gain_per_s": profile.rate_gain_per_s, "native_rate_limit_rad_s": profile.native_rate_limit_rad_s,
        "gravity_m_s2": profile.gravity_m_s2, "control_dt_s": profile.policy_dt_s,
        "free_flight_dt_s": profile.physics_dt_s, "contact_dt_s": spec.dt,
        "mixer_collective_and_body_torque_per_rotor_N": profile.mixer.tolist(),
        "hover_motor_thrust_N": profile.hover_motor_thrust_N.tolist(), "provenance": profile.provenance,
        "cad_to_body": mass["cad_to_body"], "cad_com_mm": mass["cad_com_mm"],
        "battery_translation_cad_mm": mass["battery_translation_cad_mm"],
        "contact_patches": [entry for entry in geometry["entries"] if entry["group"] != "body"],
        "field_translation_m": spec.field_translation.tolist(),
        "target_rotation_to_world": spec.target_rotation.tolist(),
    })
    write_json(output / "evidence/source_provenance.json", {
        "release": VERSION, "files": origins,
        "frozen_harness_sources": {str(path): digest(ROOT / path) for path in (*reused, R1 / "contact_gate.py")},
        "upstream_revisions": {
            repo: subprocess.check_output(["git", "-C", str(ROOT / "upstream" / repo), "rev-parse", "HEAD"],
                                          text=True).strip()
            for repo in ("rm27_RL", "rm27_sim")
        },
    })


def seal(stage_path, release_dir):
    files = {path.relative_to(stage_path).as_posix(): digest(path)
             for path in sorted(stage_path.rglob("*")) if path.is_file() and path.name != "manifest.json"}
    # Nested manifests are payload files too.
    for path in stage_path.rglob("manifest.json"):
        if path.parent != stage_path:
            files[path.relative_to(stage_path).as_posix()] = digest(path)
    if not {"README.md", "validation_report.md"} <= files.keys():
        raise ValueError("Stage must include README and validation report before sealing")
    payload = json.dumps(files, sort_keys=True, separators=(",", ":")).encode()
    write_json(stage_path / "manifest.json", {
        "schema": "asm2_model_release_v1", "version": VERSION,
        "payload_sha256": hashlib.sha256(payload).hexdigest(), "files": files,
    })
    archive = release_dir / f"{VERSION}.tar.gz"
    with archive.open("xb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0, compresslevel=6) as gz:
            with tarfile.open(fileobj=gz, mode="w|", format=tarfile.PAX_FORMAT) as tar:
                for relative in sorted([*files, "manifest.json"]):
                    path = stage_path / relative
                    info = tarfile.TarInfo(f"{VERSION}/{relative}")
                    info.size, info.mode = path.stat().st_size, 0o444
                    with path.open("rb") as stream:
                        tar.addfile(info, stream)
    print(digest(archive), archive)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("stage", "seal"))
    parser.add_argument("stage_path", type=Path)
    parser.add_argument("--release-dir", type=Path, default=HERE)
    args = parser.parse_args()
    if args.operation == "stage":
        stage(args.stage_path.resolve())
    else:
        seal(args.stage_path.resolve(), args.release_dir.resolve())
