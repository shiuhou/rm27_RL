"""Bounded native time-resolution and frozen-Field collision diagnostics.

Only isolated in-memory models are built.  The old aircraft, contact sphere,
scene files, task scorer and candidate source asset are never changed.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .asset import RUN, digest, load_inputs, read_json, write_result
from .contacts import scorer_input, summarize
from .native import fixture_model, records, reset


def _text(values):
    return " ".join(format(float(value), ".17g") for value in np.asarray(values).ravel())


def _before(data):
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, data.qpos[3:7])
    return (data.qpos[:3].copy(), data.qvel[:3].copy(),
            rotation.reshape(3, 3) @ data.qvel[3:6])


def _event(rows, tick, dt, before):
    touching = [row for row in rows if row["distance_m"] <= 0]
    return dict(tick=tick, collision_sample_time_s=(tick - 1) * dt,
                completed_interval_end_s=tick * dt,
                pre_position_m=before[0].tolist(), pre_velocity_m_s=before[1].tolist(),
                **summarize(rows),
                maximum_normal_force_N=max((row["normal_force_N"] for row in rows), default=0),
                minimum_touching_distance_m=min((row["distance_m"] for row in touching), default=None),
                records=rows)


def _rollout(model, data, entries, steps):
    events, trajectory, first_reported, first_touching = [], [], None, None
    dt = float(model.opt.timestep)
    for tick in range(1, steps + 1):
        before = _before(data)
        mujoco.mj_step(model, data)
        rows = records(model, data, entries, tick, before)
        events.extend(rows)
        if rows and first_reported is None:
            first_reported = _event(rows, tick, dt, before)
        if any(row["distance_m"] <= 0 for row in rows) and first_touching is None:
            first_touching = _event(rows, tick, dt, before)
        trajectory.append(dict(tick=tick, interval_end_s=tick * dt,
                               position_m=data.qpos[:3].tolist(),
                               velocity_m_s=data.qvel[:3].tolist(), ncon=int(data.ncon),
                               **summarize(rows),
                               maximum_normal_force_N=max((r["normal_force_N"] for r in rows), default=0)))
    return dict(**summarize(events), first_reported=first_reported,
                first_touching=first_touching, records=events, trajectory=trajectory,
                warnings={name: dict(count=int(data.warning[int(kind)].number),
                                     lastinfo=int(data.warning[int(kind)].lastinfo))
                          for name, kind in mujoco.mjtWarning.__members__.items()
                          if name != "mjNWARNING" and data.warning[int(kind)].number},
                finite=bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()))


def _native_options(model):
    return dict(timestep_s=float(model.opt.timestep), gravity_m_s2=model.opt.gravity.tolist(),
                solver=mujoco.mjtSolver(int(model.opt.solver)).name,
                integrator=mujoco.mjtIntegrator(int(model.opt.integrator)).name,
                iterations=int(model.opt.iterations), tolerance=float(model.opt.tolerance))


def descending(source, entries, case):
    results = []
    for dt in (.001, .0001):
        start = time.perf_counter()
        model = fixture_model(source, dt)
        data = mujoco.MjData(model)
        reset(model, data, case)
        steps = round(.030 / dt)
        row = _rollout(model, data, entries, steps)
        row["per_group"] = {}
        for group in ("contact_front", "contact_rear", "body"):
            group_rows = [event for event in row["records"] if event["group"] == group]
            touching = [event for event in group_rows if event["distance_m"] <= 0]
            row["per_group"][group] = dict(
                **summarize(group_rows),
                first_touching=touching[0] if touching else None,
                maximum_force_record=max(group_rows, key=lambda e: e["normal_force_N"], default=None),
                positive_normal_force_points=sum(e["normal_force_N"] > 0 for e in group_rows))
        # Retain complete manifolds for first events, representative records
        # by group and all per-tick summaries, instead of 50k repeated points.
        del row["records"]
        row["records_storage"] = "first manifolds, per-group first/max force records, and every tick's complete classification summary"
        row.update(physics=_native_options(model), steps=steps, duration_s=steps * dt,
                   initial_case=case, build_and_run_s=time.perf_counter() - start)
        results.append(row)
        print("ASM2_NATIVE_RESOLUTION", dt,
              row["first_touching"]["touching_groups"] if row["first_touching"] else None,
              flush=True)
    return results


def _verify_frozen_geometry(*, contact_parameters=None):
    """Verify exported hull arrays against the hash-pinned original MJB.

    The historical loader is used only for immutable-input verification; its
    retired learning authorization is not consumed as a new research budget.
    The NPZ has no independent pin in that configuration, so every selected
    static hull is compared directly with the pinned MJB instead.
    """
    from simpleflight_lab.rm27_contact_geometry import _mesh_hull
    from simpleflight_lab.stage1_experiment import load_experiment

    _, assets, _, geometry = load_experiment()
    arrays_path = assets["geometry"].with_suffix(".npz")
    original = mujoco.MjModel.from_binary_path(str(assets["native_scene"]))
    original_data = mujoco.MjData(original)
    mujoco.mj_forward(original, original_data)
    meshes = {}
    with np.load(arrays_path, allow_pickle=False) as arrays:
        for shape in geometry["static"]:
            index = int(shape["id"])
            if original.geom(index).name != shape["name"]:
                raise ValueError("Frozen Field geom identity differs from source MJB")
            if contact_parameters is not None:
                contact_parameters[shape["name"]] = {
                    "priority": int(original.geom_priority[index]),
                    "solmix": float(original.geom_solmix[index]),
                }
            if not np.allclose(original_data.geom_xpos[index], shape["position_m"], atol=1e-12, rtol=0):
                raise ValueError(f"Frozen Field position differs: {shape['name']}")
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, np.asarray(shape["quaternion_wxyz"]))
            if not np.allclose(original_data.geom_xmat[index], rotation, atol=1e-12, rtol=0):
                raise ValueError(f"Frozen Field orientation differs: {shape['name']}")
            for field, expected in (("geom_contype", shape["contype"]),
                                    ("geom_conaffinity", shape["conaffinity"]),
                                    ("geom_condim", shape["condim"])):
                if int(getattr(original, field)[index]) != expected:
                    raise ValueError(f"Frozen Field contact parameter differs: {shape['name']}: {field}")
            for field, expected in (("geom_friction", shape["friction"]),
                                    ("geom_solref", shape["solref"]),
                                    ("geom_solimp", shape["solimp"]),
                                    ("geom_size", shape["size"]),
                                    ("geom_margin", shape["margin_m"]),
                                    ("geom_gap", shape["gap_m"])):
                if not np.array_equal(getattr(original, field)[index], expected):
                    raise ValueError(f"Frozen Field physical parameter differs: {shape['name']}: {field}")
            key = shape["mesh"]
            if key is not None and key not in meshes:
                vertices, faces = _mesh_hull(original, int(original.geom_dataid[index]))
                exported = arrays[f"{key}_vertices"], arrays[f"{key}_faces"]
                if not all(np.array_equal(a, b) for a, b in zip((vertices, faces), exported)):
                    raise ValueError(f"Frozen Field NPZ differs from source MJB hull: {key}")
                meshes[key] = tuple(value.copy() for value in exported)
    if len(geometry["static"]) != 1822 or sum(row["legal"] for row in geometry["static"]) != 25:
        raise ValueError("Frozen Field selection or collider registration changed")
    provenance = dict(verification="load_experiment pins; selected NPZ hulls exactly match pinned MJB",
                      geometry_json_sha256=digest(assets["geometry"]),
                      geometry_npz_sha256=digest(arrays_path),
                      native_scene_sha256=digest(assets["native_scene"]),
                      registration_sha256=digest(assets["registration"]),
                      static_count=len(geometry["static"]), legal_count=25,
                      unique_verified_static_meshes=len(meshes))
    return geometry, meshes, provenance


def _field_model(source, geometry, meshes, translation, dt, configure=None):
    root = ET.parse(source / "mujoco/model.xml").getroot()
    asset = root.find("asset")
    for mesh in asset.findall("mesh"):
        mesh.set("file", str(source / "mujoco" / mesh.get("file")))
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", str(dt))
    option.set("gravity", "0 0 0")
    for key, (vertices, faces) in meshes.items():
        ET.SubElement(asset, "mesh", name="frozen_field_" + key,
                      vertex=_text(vertices), face=" ".join(map(str, faces.ravel())))
    world = root.find("worldbody")
    for shape in geometry["static"]:
        attributes = dict(name=shape["name"], type=shape["type"],
                          pos=_text(np.asarray(shape["position_m"]) + translation),
                          quat=_text(shape["quaternion_wxyz"]),
                          contype=str(shape["contype"]), conaffinity=str(shape["conaffinity"]),
                          friction=_text(shape["friction"]), solref=_text(shape["solref"]),
                          solimp=_text(shape["solimp"]), condim=str(shape["condim"]),
                          margin=str(shape["margin_m"]), gap=str(shape["gap_m"]))
        if shape["mesh"] is None:
            attributes["size"] = _text(shape["size"])
        else:
            attributes["mesh"] = "frozen_field_" + shape["mesh"]
        ET.SubElement(world, "geom", **attributes)
    if configure is not None:
        configure(root)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    # The target-centered origin intersects the aircraft at its default
    # placement. Keep it clear while querying only fixed geom transforms.
    data.qpos[:3] = [0., 0., 10.]
    mujoco.mj_forward(model, data)
    # Recompilation may recenter the mesh. Verify the actual resulting world
    # vertices, rather than assuming unchanged local mesh frames.
    worst = 0.
    for shape in geometry["static"]:
        gid = model.geom(shape["name"]).id
        if shape["mesh"] is None:
            continue
        expected_rotation = np.empty(9)
        mujoco.mju_quat2Mat(expected_rotation, np.asarray(shape["quaternion_wxyz"]))
        expected = (meshes[shape["mesh"]][0] @ expected_rotation.reshape(3, 3).T
                    + np.asarray(shape["position_m"]) + translation)
        mid = int(model.geom_dataid[gid])
        first, count = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
        actual = (model.mesh_vert[first:first + count] @ data.geom_xmat[gid].reshape(3, 3).T
                  + data.geom_xpos[gid])
        distances = np.linalg.norm(actual[:, None] - expected[None, :], axis=-1)
        error = float(max(distances.min(axis=0).max(), distances.min(axis=1).max()))
        worst = max(worst, error)
        if error > 2e-7:
            raise ValueError(f"Recompiled Field exceeds original export's 0.2 um coordinate bound: {shape['name']}: {error}")
    return model, data, worst


def field(source, entries, case):
    start = time.perf_counter()
    geometry, meshes, provenance = _verify_frozen_geometry()
    model, data, worst = _field_model(source, geometry, meshes,
                                      np.asarray(case["field_origin_translation_m"]), .001)
    active = {model.geom(name).id for name in entries}
    if (len(active) != len(entries) or model.njnt != 1
            or int(model.jnt_type[0]) != int(mujoco.mjtJoint.mjJNT_FREE)
            or "drone_1_stage1_contact" in {model.geom(i).name for i in range(model.ngeom)}):
        raise ValueError("Isolated ASM2 candidate unexpectedly includes a legacy aircraft or joint")
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = case["position"]
    mujoco.mju_mat2Quat(data.qpos[3:7], np.asarray(case["rotation"]).ravel())
    data.qvel[:3], data.qvel[3:6] = case["velocity"], case["omega"]
    mujoco.mj_forward(model, data)
    initial = records(model, data, entries, 1, _before(data))
    result = _rollout(model, data, entries, case["steps"])
    legal = {shape["name"] for shape in geometry["static"] if shape["legal"]}
    result.update(case=case, case_sha256=digest(RUN / "field-case.json"),
                  physics=_native_options(model), provenance=provenance,
                  field_recompiled_max_vertex_error_m=worst,
                  active_colliders=len(active), drone_mass_kg=float(model.body("drone").mass[0]),
                  initial_reported=initial,
                  expected_groups=case["expected_groups"],
                  geometric_classification_agrees=result["touching_groups"] == case["expected_groups"],
                  scorer_inputs=[scorer_input(row, legal) for row in result["records"]],
                  contacted_field_geometry=[dict(name=name, registered_legal=name in legal)
                                            for name in sorted({row["other_geom"] for row in result["records"]})],
                  build_verify_and_run_s=time.perf_counter() - start)
    print("ASM2_NATIVE_FIELD", result["touching_groups"], result["geometric_classification_agrees"], flush=True)
    return result


def run(output):
    if mujoco.__version__ != "3.12.0":
        raise RuntimeError("Use the frozen native-env MuJoCo 3.12.0 runtime")
    start = time.perf_counter()
    config, source, geometry, _ = load_inputs()
    entries = {entry["name"]: entry for entry in geometry["entries"]}
    fixtures = read_json(RUN / "fixtures.json")["fixtures"]
    case = next(row for row in fixtures if row["name"] == "descending")
    result = dict(backend="MuJoCo", version=mujoco.__version__,
                  task_qualification="NOT_DEFINED_FOR_ASM2", training_transitions=0,
                  source_pins=config["pins"], fixture_sha256=digest(RUN / "fixtures.json"),
                  descending=descending(source, entries, case),
                  field=field(source, entries, read_json(RUN / "field-case.json")))
    result["wall_s"] = time.perf_counter() - start
    write_result(output, result)
    print("ASM2_NATIVE_RESOLUTION_COMPLETE", str(output), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=RUN / "native-resolution.json")
    run(parser.parse_args().output)


if __name__ == "__main__":
    main()
