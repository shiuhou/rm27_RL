"""Query frozen body and armor meshes at archived Phase 4R.2 pre-contact poses."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from rm27_asm2.asset import ROOT, digest, obj_mesh, read_json, write_result
from rm27_asm2.contact_task import load_contact_spec
from rm27_asm2.flight_suite import rotation_matrix
from rm27_asm2.native_resolution import _verify_frozen_geometry


RUN = Path(__file__).resolve().parent
CASE = "dev_lateral_positive:763"
SOURCE_BODY = "collision_0_570"
PROXY = ROOT / "assets/asm2/proxy/manifest.json"
LOCAL_RADIUS_M = .004


def distances(state, targets, bodies):
    model = ET.Element("mujoco")
    asset = ET.SubElement(model, "asset")
    world = ET.SubElement(model, "worldbody")
    rotation, position = rotation_matrix(np.asarray(state[3:7])), np.asarray(state[:3])
    for name, (vertices, faces) in (targets | {
            name: (vertices @ rotation.T + position, faces)
            for name, (vertices, faces) in bodies.items()}).items():
        ET.SubElement(asset, "mesh", name=name, vertex=" ".join(map(str, vertices.ravel())),
                      face=" ".join(map(str, faces.ravel())))
        ET.SubElement(world, "geom", name=name, type="mesh", mesh=name)
    model = mujoco.MjModel.from_xml_string(ET.tostring(model, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    report = {}
    for target in targets:
        rows = []
        for name in bodies:
            closest = np.zeros(6)
            separation = mujoco.mj_geomDistance(
                model, data, model.geom(name).id, model.geom(target).id, .010, closest)
            rows.append(dict(name=name, signed_distance_m=separation,
                             closest_world_m=closest.reshape(2, 3).tolist()))
        report[target] = dict(detailed=next(row for row in rows if row["name"] == SOURCE_BODY),
                              proxy_nearest=sorted((row for row in rows if row["name"] != SOURCE_BODY),
                                                   key=lambda row: row["signed_distance_m"])[:3])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    spec = load_contact_spec()
    native_path = RUN / "native-matched.json"
    native = read_json(native_path)
    if native["contact_config_sha256"] != spec.contract_sha256:
        raise ValueError("Archived native contact config differs")
    contact = native["controls"][CASE]["first"]
    body_rows = [row for row in contact["signed"] if row["geom"] == SOURCE_BODY]
    if not body_rows:
        raise ValueError("Missing reference body contact")
    target_names = {row["other_geom"] for row in body_rows}
    rotation = rotation_matrix(np.asarray(contact["pre_state"][3:7]))
    position = np.asarray(contact["pre_state"][:3])
    points = (np.array([row["position_world_m"] for row in body_rows]) - position) @ rotation
    geometry, mesh, provenance = _verify_frozen_geometry()
    if geometry != spec.field_geometry:
        raise ValueError("Geometry differs from the frozen Field")
    targets = {}
    for shape in geometry["static"]:
        if shape["name"] in target_names:
            vertices, faces = mesh[shape["mesh"]]
            r = rotation_matrix(np.asarray(shape["quaternion_wxyz"]))
            targets[shape["name"]] = (vertices @ r.T + shape["position_m"] + spec.field_translation, faces)
    if set(targets) != target_names:
        raise ValueError("Missing one or more referenced armor geoms")
    exact = spec.entries[SOURCE_BODY]
    body_path = spec.source / "mujoco" / exact["mesh"]
    source_manifest = dict((path, sha) for sha, path in (
        line.split("  ", 1) for line in (spec.source / "MANIFEST_SHA256.txt").read_text().splitlines()))
    if digest(body_path) != source_manifest[str(body_path.relative_to(spec.source))]:
        raise ValueError("Reference body mesh changed")
    bodies = {SOURCE_BODY: obj_mesh(body_path)}
    for entry in read_json(PROXY)["entries"]:
        if entry["group"] != "body":
            continue
        path = PROXY.parent / entry["mesh"]
        vertices, faces = obj_mesh(path)
        if any(np.all((point >= vertices.min(0)-LOCAL_RADIUS_M) & (point <= vertices.max(0)+LOCAL_RADIUS_M))
               for point in points):
            if digest(path) != entry["sha256"]:
                raise ValueError(f"Proxy mesh changed: {path}")
            bodies[entry["name"]] = (vertices, faces)
    states = {"native": contact["pre_state"]}
    hashes = {"native": digest(native_path), "proxy_manifest": digest(PROXY), "detailed_body": digest(body_path)}
    for asset in ("proxy", "detailed"):
        path = RUN / f"isaac-{asset}-focused.json"
        evidence = read_json(path)
        row = next(row for row in evidence["batches"][0]["rows"] if row["name"] == CASE)
        states[f"isaac_{asset}"] = row["first_signed"]["pre_state"]
        hashes[f"isaac_{asset}"] = digest(path)
    result = dict(case=CASE, contact_interval_s=[contact["time_s"], contact["time_s"]+native["physics_dt_s"]],
                  detail_signed_reports_m={row["other_geom"]: row["distance_m"] for row in body_rows},
                  local_aabb_radius_m=LOCAL_RADIUS_M, local_proxy_hulls=len(bodies)-1,
                  hashes=hashes, field=provenance,
                  geometry_queries={label: distances(state, targets, bodies) for label, state in states.items()})
    write_result(args.output, result)


if __name__ == "__main__":
    main()
