"""Export and audit the Phase 4R.2 aircraft geometry as a separate MuJoCo plant."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import replace
from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from .asset import ROOT, digest, load_inputs, obj_mesh, read_json, write_result
from .flight import load_profile
from .flight_suite import rotation_matrix
from .proxy import PROXY, load_proxy

ASM2 = Path("assets/asm2")
MODEL = Path("models/asm2_isaac_proxy.xml")
PHASE = ROOT / ".agents/runs/asm2-terminal"
R2 = PHASE / "phase4r2"
RECEIPT = R2 / "backend-receipt.json"
OVERLAY = R2 / "proxy-with-reference-body.usda"
PROXY_USD = ROOT / ".agents/runs/asm2-contact/proxy-247f076685d8.usda"
ORIGINAL_HULL = "collision_0_570"
GROUP_COUNTS = {"body": 3507, "contact_front": 14, "contact_rear": 1}


def inputs():
    config, source, geometry, _ = load_inputs()
    manifest = load_proxy()
    receipt = read_json(RECEIPT)
    baseline = read_json(PROXY_USD.parent / "isaac-proxy-field.json")
    if (digest(OVERLAY) != receipt["asset_sha256"]
            or digest(PROXY_USD) != baseline["asset_sha256"]
            or digest(PROXY) != baseline["proxy"]["manifest_sha256"]
            or digest(ROOT / config["candidate_usd"]) != config["candidate_sha256"]):
        raise ValueError("Phase 4R.2 geometry pins differ")
    original = {entry["name"]: entry for entry in geometry["entries"]}
    hull = original[ORIGINAL_HULL]
    if hull["group"] != "body" or any(entry["name"] == ORIGINAL_HULL for entry in manifest["entries"]):
        raise ValueError("Expected one original hull restored over the proxy")
    hashes = {name: sha for sha, name in
              (line.split("  ", 1) for line in (source / "MANIFEST_SHA256.txt").read_text().splitlines())}
    file = Path("source/mujoco") / hull["mesh"]
    entries = [dict(entry, file=str(Path("proxy") / entry["mesh"])) for entry in manifest["entries"]]
    entries.append(dict(hull, file=file.as_posix(), sha256=hashes[str(Path("mujoco") / hull["mesh"])]))
    if Counter(entry["group"] for entry in entries) != GROUP_COUNTS:
        raise ValueError("Restored proxy collider groups differ")
    return source, hashes, entries


def export(output):
    """Keep relative mesh paths compatible with the v1.0.0 release layout."""
    from .flight_native import _flight_mjcf

    source, source_hashes, entries = inputs()
    profile = load_profile()
    flight = ET.fromstring(_flight_mjcf(profile))
    root = ET.parse(source / "mujoco/model.xml").getroot()
    root.set("model", "asm2_isaac_phase4r2_proxy")
    root.find("compiler").set("meshdir", "../assets/asm2")
    option = flight.find("option")
    option.set("timestep", str(receipt_dt()))
    root.insert(1, option)
    asset = root.find("asset")
    drone = root.find("./worldbody/body[@name='drone']")
    old_geoms = {geom.get("name"): geom for geom in drone.findall("geom")}
    visual_assets = {geom.get("mesh") for geom in old_geoms.values() if geom.get("group") == "2"}
    if len(visual_assets) != 14:
        raise ValueError("Expected 14 original visuals")

    files = {}
    for mesh in asset.findall("mesh"):
        if mesh.get("name") not in visual_assets:
            asset.remove(mesh)
            continue
        original = Path("mujoco") / mesh.attrib["file"]
        path = (Path("source") / original).as_posix()
        mesh.set("file", path)
        files[path] = source_hashes[original.as_posix()]
    for geom in old_geoms.values():
        if geom.get("group") != "2":
            drone.remove(geom)
    for entry in entries:
        name = entry["name"]
        asset.append(ET.Element("mesh", name=name, file=entry["file"]))
        template = old_geoms[ORIGINAL_HULL] if entry["group"] == "body" else old_geoms[name]
        drone.append(ET.Element("geom", dict(template.attrib, name=name, mesh=name)))
        files[entry["file"]] = entry["sha256"]
    flight_body = flight.find("./worldbody/body")
    drone.remove(drone.find("inertial"))
    drone.insert(0, flight_body.find("inertial"))
    drone.extend(flight_body.findall("site"))
    root.append(flight.find("actuator"))

    output.mkdir(parents=True)
    for relative, expected in files.items():
        original = ROOT / ASM2 / relative
        dest = output / ASM2 / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, dest)
        if digest(dest) != expected:
            raise ValueError(f"Exported mesh hash mismatch: {relative}")
    model = output / MODEL
    model.parent.mkdir()
    root.insert(0, ET.Comment("Phase 4R.2 Isaac hull geometry with native MuJoCo contact settings."))
    ET.indent(root)
    model.write_text(ET.tostring(root, encoding="unicode") + "\n")
    print(model, flush=True)
    print(f"SHA-256 {digest(model)}", flush=True)


def receipt_dt():
    return float(read_json(RECEIPT)["physics_dt_s"])


def audit_usd(output):
    """Compare authored USD collider faces and points in the /Drone frame."""
    from pxr import Usd, UsdGeom, UsdPhysics

    _, _, entries = inputs()
    stage = Usd.Stage.Open(str(OVERLAY))
    drone = stage.GetDefaultPrim()
    if (str(drone.GetPath()) != "/Drone" or UsdGeom.GetStageMetersPerUnit(stage) != 1.
            or UsdGeom.GetStageUpAxis(stage) != "Z"):
        raise ValueError("Unexpected Phase 4R.2 USD frame")
    colliders = {prim.GetName(): prim for prim in Usd.PrimRange(drone)
                 if prim.HasAPI(UsdPhysics.CollisionAPI)}
    if set(colliders) != {entry["name"] for entry in entries}:
        raise ValueError("Collider names differ from composed Phase 4R.2 USD")
    cache = UsdGeom.XformCache()
    worst = 0.
    for entry in entries:
        prim = colliders[entry["name"]]
        vertices, faces = obj_mesh(output / ASM2 / entry["file"])
        mesh = UsdGeom.Mesh(prim)
        transform = np.asarray(cache.ComputeRelativeTransform(prim, drone)[0])
        points = (np.c_[np.asarray(mesh.GetPointsAttr().Get()), np.ones(len(vertices))] @ transform)[:, :3]
        if prim.GetCustomDataByKey("contact_group") != entry["group"]:
            raise ValueError(f"USD contact class differs: {entry['name']}")
        np.testing.assert_array_equal(mesh.GetFaceVertexCountsAttr().Get(), 3)
        np.testing.assert_array_equal(mesh.GetFaceVertexIndicesAttr().Get(), faces.ravel())
        worst = max(worst, float(np.max(np.abs(points - vertices))))
    if worst > 1e-8:
        raise ValueError("USD points exceed the existing 10 nm cooking-normalization bound")
    write_result(output / "usd_geometry.json", dict(usd_version=list(Usd.GetVersion()),
                 overlay_sha256=digest(OVERLAY), base_usd_sha256=digest(PROXY_USD),
                 groups=GROUP_COUNTS, collider_count=len(colliders), max_coordinate_error_m=worst))
    print(f"USD {len(colliders)} colliders, max coordinate error {worst:.4g} m", flush=True)


def check_model(model_path, entries, profile):
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    if (model.ngeom, model.nmesh, model.nu, model.na) != (3536, 3536, 4, 4):
        raise ValueError("Expected 3522 collision hulls, 14 visual meshes and four filtered motors")
    if (model.opt.timestep != receipt_dt() or np.any(model.actuator_actearly)
            or np.any(model.actuator_dyntype != mujoco.mjtDyn.mjDYN_FILTEREXACT)):
        raise ValueError("Physics interval or actuator update order changed")
    np.testing.assert_array_equal(model.actuator_dynprm[:, 0], profile.motor_time_constant_s)
    np.testing.assert_array_equal(model.actuator_ctrlrange, np.tile([0., profile.max_rotor_thrust_N], (4, 1)))
    body = model.body("drone")
    np.testing.assert_allclose(body.mass, profile.mass_kg, atol=1e-12, rtol=0)
    np.testing.assert_array_equal(body.ipos, np.zeros(3))
    axes = rotation_matrix(body.iquat)
    np.testing.assert_allclose(axes @ np.diag(body.inertia) @ axes.T, profile.inertia_kg_m2, atol=1e-12, rtol=0)
    for i in range(4):
        np.testing.assert_array_equal(model.site(f"rotor_{i}").pos, profile.hubs_m[i])
        np.testing.assert_array_equal(model.actuator_gear[i],
                                      np.r_[profile.axes[i], profile.yaw_torque_per_thrust_m[i]*profile.axes[i]])
    data.act[:] = data.ctrl[:] = profile.hover_motor_thrust_N
    mujoco.mj_forward(model, data)
    ids = {model.geom(entry["name"]).id for entry in entries}
    assert ids == set(np.flatnonzero((model.geom_contype != 0) | (model.geom_conaffinity != 0)))
    worst = 0.
    for entry in entries:
        g = model.geom(entry["name"]).id
        mesh = model.geom_dataid[g]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        actual = model.mesh_vert[start:start+count] @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        path = model_path.parents[1] / ASM2 / entry["file"]
        if digest(path) != entry["sha256"]:
            raise ValueError(f"Portable collision mesh hash mismatch: {path}")
        expected, _ = obj_mesh(path)
        distances = np.linalg.norm(actual[:, None] - expected[None, :], axis=-1)
        worst = max(worst, float(max(distances.min(axis=0).max(), distances.min(axis=1).max())))
    if worst > 2e-7:
        raise ValueError("Compiled aircraft vertices exceed the existing 0.2 µm native mesh coordinate bound")
    mujoco.mj_step(model, data, nstep=round(1. / model.opt.timestep))
    if data.ncon or any(w.number for w in data.warning):
        raise ValueError("Unexpected contact or warning during stationary hover")
    np.testing.assert_allclose(data.qpos, model.qpos0, rtol=0, atol=1e-9)
    np.testing.assert_allclose(data.qvel, 0., rtol=0, atol=1e-9)
    return dict(mass_kg=float(body.mass[0]), colliders=len(ids), visuals=model.ngeom-len(ids),
                groups=GROUP_COUNTS, compiled_max_vertex_error_m=worst,
                hover_1s_position_error_m=float(np.linalg.norm(data.qpos[:3]-model.qpos0[:3])))


def replay_controls(plant, controls):
    from .contact_task import ContactScorer
    from .terminal_native_eval import restore

    results = {}
    for row in controls:
        name = row["name"]
        state, thrust, target = (np.asarray(row[key]) for key in ("state", "thrust", "target"))
        initial = restore(plant, state, thrust, target)
        scorer = ContactScorer(plant.spec, dt=plant.dt)
        result = scorer.observe(initial, plant.pre_state, 0)
        substeps = round(plant.profile.policy_dt_s / plant.dt)
        tape = row["tape"]
        for tick in range(math.ceil(row["source_window_s"] / plant.dt - 1e-9)):
            if result is not None:
                break
            if tick % substeps == 0:
                plant.set_ctbr(tape[min(tick // substeps, len(tape)-1)])
            records_now = plant.step_physics()
            result = scorer.observe(records_now, plant.pre_state, plant.tick)
        result = result or scorer.finish()
        for key in ("first_touch", "first_loaded"):
            if result[key] is not None:
                result[key].pop("records")
        results[name] = result
    return results


def validate(output):
    """Rerun detailed and proxy contact controls with the same native scorer."""
    import mujoco

    from .contact_native import NativeContact
    from .contact_task import load_contact_spec
    from .native_resolution import _field_model, _native_options, _verify_frozen_geometry

    source, _, entries = inputs()
    spec, profile = load_contact_spec(), load_profile()
    model_path = output / MODEL
    model_check = check_model(model_path, entries, profile)

    # Read the same frozen resets and command tapes as the Phase 4R.2 gate.
    loader = spec_from_file_location("phase4r2_matched_contact", R2 / "matched_contact.py")
    if digest(Path(loader.origin)) != read_json(RECEIPT)["run_pins"][str(Path(loader.origin).relative_to(ROOT))]:
        raise ValueError("Historical control loader differs from the Phase 4R.2 receipt")
    phase = module_from_spec(loader)
    loader.loader.exec_module(phase)
    _, _, controls = phase.load_cases()
    plant = NativeContact(profile, spec)
    reference = replay_controls(plant, controls)
    proxy = ET.parse(model_path).getroot()
    meshdir = (model_path.parent / proxy.find("compiler").attrib["meshdir"]).resolve()
    params = {}
    field, hulls, _ = _verify_frozen_geometry(contact_parameters=params)
    if field != spec.field_geometry:
        raise ValueError("Field inputs differ from the frozen native contact contract")

    def configure(root):
        # Keep the existing Field hulls and collider order. Use the exact
        # standalone XML's aircraft, actuators, and physics options.
        root.remove(root.find("option"))
        root.insert(1, proxy.find("option"))
        world = root.find("worldbody")
        world.remove(world.find("body[@name='drone']"))
        world.insert(0, proxy.find("./worldbody/body[@name='drone']"))
        root.append(proxy.find("actuator"))
        asset = root.find("asset")
        for mesh in asset.findall("mesh"):
            if mesh.get("file") is not None:
                asset.remove(mesh)
        for index, mesh in enumerate(proxy.findall("./asset/mesh")):
            mesh.set("file", str(meshdir / mesh.attrib["file"]))
            asset.insert(index, mesh)
        for geom in world.findall("geom"):
            contact = params[geom.attrib["name"]]
            geom.set("priority", str(contact["priority"]))
            geom.set("solmix", str(contact["solmix"]))

    plant.model, plant.data, field_error = _field_model(source, field, hulls, spec.field_translation, spec.dt, configure)
    plant.spec = replace(spec, entries={entry["name"]: entry for entry in entries})
    candidate = replay_controls(plant, controls)
    comparisons = []
    for row in controls:
        name = row["name"]
        old, new = reference[name], candidate[name]
        expected = "QUALIFIED" if row["expected"] == "DESIGNATED_LOADED" else row["expected"]
        if old["reason"] != expected:
            raise ValueError(f"Detailed baseline differs for {name}: {old['reason']}")
        first_old, first_new = old["first_touch"], new["first_touch"]
        delta = (first_new["sample_time_s"]-first_old["sample_time_s"]
                 if first_new is not None and first_old is not None else None)
        comparisons.append(dict(name=name, reference=old, proxy=new,
                                same_label=old["reason"] == new["reason"], first_touch_delta_s=delta))
        print(name, old["reason"], new["reason"], delta, flush=True)
    report = dict(mujoco=mujoco.__version__, numpy=np.__version__, model_sha256=digest(model_path),
                  detailed_mjcf_sha256=digest(source / "mujoco/model.xml"), proxy_manifest_sha256=digest(PROXY),
                  overlay_sha256=digest(OVERLAY), contact_contract_sha256=spec.contract_sha256,
                  physics=_native_options(plant.model), control_dt_s=profile.policy_dt_s,
                  model=model_check, field_recompiled_max_vertex_error_m=field_error,
                  same_labels=sum(row["same_label"] for row in comparisons),
                  cases=len(comparisons), controls=comparisons)
    write_result(output / "validation.json", report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("export", "audit-usd", "validate"))
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    action = {"export": export, "audit-usd": audit_usd, "validate": validate}
    action[args.action](args.output.resolve())


if __name__ == "__main__":
    main()
