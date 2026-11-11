"""Matched MuJoCo ASM2 collision tests; source MJCF and legacy scorer stay intact."""
from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import resource
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .asset import RUN, load_inputs, obj_mesh, read_json, write_result
from .contacts import contact_record, summarize


def fixture_model(source, dt):
    root = ET.parse(source / "mujoco/model.xml").getroot()
    for mesh in root.findall("asset/mesh"):
        mesh.set("file", str(source / "mujoco" / mesh.get("file")))
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", str(dt))
    option.set("gravity", "0 0 0")
    for name, kind, size in (("test_plane", "plane", "1 1 .01"), ("test_box", "box", ".0002 .0002 .002")):
        anchor = ET.SubElement(root.find("worldbody"), "body", name=name+"_anchor", mocap="true", pos="0 0 -10")
        ET.SubElement(anchor, "geom", name=name, type=kind, size=size,
                      friction=".6 .005 .0001", contype="1", conaffinity="1")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def reset(model, data, case):
    mujoco.mj_resetData(model, data)
    for name in ("test_plane", "test_box"):
        selected = name == "test_" + case["fixture"]
        mocap = model.body(name+"_anchor").mocapid[0]
        data.mocap_pos[mocap] = case["fixture_position"] if selected else [0., 0., -10.]
    data.qpos[:3] = case["position"]
    mujoco.mju_mat2Quat(data.qpos[3:7], np.asarray(case["rotation"]).ravel())
    data.qvel[:3] = case["velocity"]
    data.qvel[3:6] = case["omega"]
    mujoco.mj_forward(model, data)


def records(model, data, entries, tick, before):
    output = []
    for i in range(data.ncon):
        contact = data.contact[i]
        names = [model.geom(int(g)).name for g in contact.geom]
        owners = [name in entries for name in names]
        if sum(owners) != 1:
            raise RuntimeError(f"Unexpected collision pair: {names}")
        side = int(owners[1])
        sign = 1 if side == 1 else -1
        wrench = np.zeros(6)
        if contact.efc_address >= 0:
            mujoco.mj_contactForce(model, data, i, wrench)
        frame = contact.frame.reshape(3, 3)
        output.append(contact_record(tick=tick, dt=float(model.opt.timestep), group=entries[names[side]]["group"],
                                     geom=names[side], other_geom=names[1-side], point=contact.pos,
                                     normal=sign * frame[0], impulse=sign * frame.T @ wrench[:3] * model.opt.timestep,
                                     separation=float(contact.dist), pre_position=before[0], pre_linear=before[1],
                                     pre_angular_world=before[2]))
    return output


def nearest_surface_distance(point, triangles):
    """Exact point/triangle distance, vectorized over supplied visual triangles."""
    a, b, c = triangles.transpose(1, 0, 2)
    ab, ac = b - a, c - a
    normal = np.cross(ab, ac)
    norm2 = np.einsum("ij,ij->i", normal, normal)
    valid = norm2 > 1e-30
    a, b, c, ab, ac, normal, norm2 = [x[valid] for x in (a, b, c, ab, ac, normal, norm2)]
    projection = point - (np.einsum("ij,ij->i", point - a, normal) / norm2)[:, None] * normal
    v2 = projection - a
    d00, d01, d11, d20, d21 = [np.einsum("ij,ij->i", x, y) for x, y in
                             ((ab, ab), (ab, ac), (ac, ac), (v2, ab), (v2, ac))]
    denominator = norm2
    v = (d11 * d20 - d01 * d21) / denominator
    w = (d00 * d21 - d01 * d20) / denominator
    inside = (v >= 0) & (w >= 0) & (v + w <= 1)
    best = np.linalg.norm(point - projection[inside], axis=1).min(initial=np.inf)
    for start, end in ((a, b), (b, c), (c, a)):
        edge = end - start
        denominator = np.einsum("ij,ij->i", edge, edge)
        t = np.clip(np.einsum("ij,ij->i", point - start, edge) / np.maximum(denominator, 1e-30), 0, 1)
        closest = start + t[:, None] * edge
        best = min(best, float(np.linalg.norm(point - closest, axis=1).min()))
    return float(best)


def guard_rays(model, data, source, mass):
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    visual, collision = np.array([0, 0, 1, 0, 0, 0], np.uint8), np.array([0, 0, 0, 1, 0, 0], np.uint8)
    gid = np.array([-1], np.int32)
    rays, false = [], []
    for cx in [-.0275, .0275]:
        for cy in [-.041, .041]:
            for radius in [.012, .017, .022]:
                for angle in np.linspace(0, 2*np.pi, 24, endpoint=False):
                    xy = np.array([cx, cy]) + radius * np.array([np.cos(angle), np.sin(angle)])
                    origin = np.r_[xy, -.06]
                    vd = mujoco.mj_ray(model, data, origin, np.array([0., 0., 1.]), visual, True, -1, gid)
                    cd = mujoco.mj_ray(model, data, origin, np.array([0., 0., 1.]), collision, True, -1, gid)
                    row = dict(index=len(rays), origin_m=origin.tolist(), direction=[0., 0., 1.],
                               visual_distance_m=float(vd), collision_distance_m=float(cd),
                               collision_geom=model.geom(int(gid[0])).name if cd >= 0 else None)
                    rays.append(row)
                    if vd < 0 <= cd:
                        false.append(dict(row))
    if false:
        triangles = []
        for part in mass["parts"]:
            vertices, faces = obj_mesh(source / "mujoco" / part["mesh"])
            triangles.append(vertices[faces])
        triangles = np.concatenate(triangles)
        for row in false:
            point = np.asarray(row["origin_m"]) + row["collision_distance_m"] * np.asarray(row["direction"])
            row["point_m"] = point.tolist()
            row["nearest_visual_surface_distance_m"] = nearest_surface_distance(point, triangles)
    ray_result = dict(rays=rays, false_positive_rays=false)
    ray_path = RUN / "guard-rays.json"
    if ray_path.exists():
        frozen = read_json(ray_path)
        if frozen["rays"] != rays or len(frozen["false_positive_rays"]) != len(false):
            raise ValueError("The prospectively recorded guard rays changed")
        for old, new in zip(frozen["false_positive_rays"], false):
            # This derived triangle-distance diagnostic can differ by floating
            # roundoff across NumPy versions; ray identity/hits stay exact.
            key = "nearest_visual_surface_distance_m"
            np.testing.assert_allclose(old[key], new[key], atol=1e-15, rtol=0)
            if {k: v for k, v in old.items() if k != key} != {k: v for k, v in new.items() if k != key}:
                raise ValueError("The frozen false-positive ray identity or hit changed")
    else:
        write_result(ray_path, ray_result)
    return dict(probes=len(rays), visual_open=sum(r["visual_distance_m"] < 0 for r in rays),
                false_positive_count=len(false), false_positive_rays=false,
                strict_guard_gate="FAIL" if false else "PASS")


def run(output):
    config, source, geometry, mass = load_inputs()
    started = time.perf_counter()
    model = fixture_model(source, config["physics_dt_s"])
    load_s = time.perf_counter() - started
    data = mujoco.MjData(model)
    entries = {e["name"]: e for e in geometry["entries"]}
    assert abs(float(model.body("drone").mass[0]) - .142) < 1e-9
    fixtures = read_json(RUN / "fixtures.json")["fixtures"]
    results = {}
    for case in fixtures:
        reset(model, data, case)
        events, first = [], None
        trajectory = []
        for tick in range(1, case.get("steps", 1) + 1):
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, data.qpos[3:7])
            before = data.qpos[:3].copy(), data.qvel[:3].copy(), rotation.reshape(3, 3) @ data.qvel[3:6]
            mujoco.mj_step(model, data)
            rows = records(model, data, entries, tick, before)
            events.extend(rows)
            if rows and first is None:
                first = dict(tick=tick, contacts=rows)
            trajectory.append(dict(tick=tick, position=data.qpos[:3].tolist(), velocity=data.qvel[:3].tolist()))
        summary = summarize(events)
        summary.update(expected_groups=case["expected_groups"],
                       classification_pass=summary["touching_groups"] == case["expected_groups"],
                       first_contact=first, records=events, trajectory=trajectory,
                       finite=bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()))
        results[case["name"]] = summary
        print("ASM2_NATIVE", case["name"], summary["touching_groups"], summary["classification_pass"], flush=True)
    rays = guard_rays(model, data, source, mass)
    benchmark = {}
    for contact in (False, True):
        case = next(c for c in fixtures if c["name"] == ("both" if contact else "clear_1mm"))
        reset(model, data, case)
        model.opt.gravity[2] = -9.81
        if not contact:
            for name in ("test_plane", "test_box"):
                data.mocap_pos[model.body(name+"_anchor").mocapid[0]] = [0., 0., -10.]
            mujoco.mj_forward(model, data)
        for _ in range(20):
            mujoco.mj_step(model, data)
        start = time.perf_counter()
        for _ in range(200):
            mujoco.mj_step(model, data)
        seconds = time.perf_counter() - start
        benchmark["contact" if contact else "no_contact"] = dict(steps=200, wall_s=seconds, world_steps_s=200/seconds,
                                                                    finite=bool(np.isfinite(data.qpos).all()), final_ncon=data.ncon)
    result = dict(backend="MuJoCo", version=mujoco.__version__, physics_dt_s=float(model.opt.timestep),
                  fixture_gravity=[0., 0., 0.], load_s=load_s, collider_count=len(entries), mass_kg=float(model.body("drone").mass[0]),
                  fixtures=results, guard=rays, benchmark=benchmark, max_rss_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                  wall_s=time.perf_counter()-started, task_qualification="NOT_DEFINED_FOR_ASM2", training_transitions=0)
    write_result(output, result)
    print("ASM2_NATIVE_COMPLETE", "fixtures",sum(r["classification_pass"] for r in results.values()),"/",len(results),
          "guard",rays["strict_guard_gate"], flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=RUN / "native.json")
    run(parser.parse_args().output)


if __name__ == "__main__":
    main()
