"""Immutable ASM2 inputs, USD/mesh conformance, and matched collision fixtures."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "configs/rm27_asm2_collision.json"
RUN = ROOT / ".agents/runs/asm2-collision"


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def write_result(path, result):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")


def load_inputs():
    config = read_json(CONFIG)
    if config["schema"] != "rm27_asm2_collision_candidate_v1" or config["training_allowed"] is not False:
        raise ValueError("This entry point is exclusively collision engineering")
    source = ROOT / config["asset_root"]
    for name, expected in config["pins"].items():
        if digest(source / name) != expected:
            raise ValueError(f"ASM2 input changed: {name}")
    if digest(ROOT / config["field_usd"]) != config["field_sha256"]:
        raise ValueError("The existing Field collision source changed")
    geometry = read_json(source / "metadata/collision_geometry.json")
    if Counter(e["group"] for e in geometry["entries"]) != config["groups"]:
        raise ValueError("ASM2 collider semantic counts changed")
    return config, source, geometry, read_json(source / "metadata/mass_properties.json")


def obj_mesh(path):
    vertices, faces = [], []
    for line in Path(path).read_text().splitlines():
        words = line.split()
        if words and words[0] == "v":
            vertices.append([float(x) for x in words[1:4]])
        elif words and words[0] == "f":
            indices = [int(x.split("/")[0]) - 1 for x in words[1:]]
            if len(indices) != 3:
                raise ValueError(f"Expected a triangulated source mesh: {path}")
            faces.append(indices)
    return np.asarray(vertices), np.asarray(faces, dtype=np.int32)


def rotation(axis, angle):
    vector = np.eye(3)[axis]
    cross = np.cross(vector, np.eye(3)).T
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * cross @ cross


def fixture_suite(vertices):
    """Prospective fixture states; all distances are body-frame metres.

    Ten-micrometre initial penetration is an intentional contact fixture,
    never an admitted flight reset or a learned success.
    """
    identity = np.eye(3)
    bottom = float(vertices[:, 2].min())
    patch_bottom = -0.01986282  # Supplied CAD-extracted patch plane, not a new tolerance.
    # Front arm interior and rear strip interior; the 0.4 mm square probe fits
    # inside each existing 1.4 mm-wide footprint without enlarging it.
    rows = []
    for name, xy, groups in (("front", [.02059486, .01121001], ["contact_front"]),
                             ("rear", [-.0275, 0.], ["contact_rear"])):
        rows.append(dict(name=name, rotation=identity.tolist(), position=[0., 0., 0.],
                         velocity=[.02, -.01, -.1], omega=[0., 0., .2],
                         fixture="box", fixture_position=[*xy, patch_bottom + 1e-5 - .002],
                         fixture_halfsize=[.0002, .0002, .002], expected_groups=groups))
    rows.append(dict(name="both", rotation=identity.tolist(), position=[0., 0., 0.],
                     velocity=[.02, -.01, -.1], omega=[0., 0., .2], fixture="plane",
                     fixture_position=[0., 0., patch_bottom + 1e-5], expected_groups=["contact_front", "contact_rear"]))
    for axis in (0, 1):
        for sign in (-1, 1):
            matrix = rotation(axis, sign * np.pi / 2)
            low = float((vertices @ matrix.T)[:, 2].min())
            rows.append(dict(name=f"body_{axis}_{sign:+d}", rotation=matrix.tolist(), position=[0., 0., 0.],
                             velocity=[0., 0., -.1], omega=[0., 0., 0.], fixture="plane",
                             fixture_position=[0., 0., low + 1e-5], expected_groups=["body"]))
    for name, gap in (("clear_1mm", .001), ("clear_20um", .00002)):
        rows.append(dict(name=name, rotation=identity.tolist(), position=[0., 0., 0.],
                         velocity=[0., 0., 0.], omega=[0., 0., 0.], fixture="plane",
                         fixture_position=[0., 0., bottom - gap], expected_groups=[]))
    rows.append(dict(name="descending", rotation=identity.tolist(), position=[0., 0., .001],
                     velocity=[0., 0., -.1], omega=[0., 0., 0.], fixture="plane",
                     fixture_position=[0., 0., patch_bottom], expected_groups=["contact_front", "contact_rear"],
                     steps=30))
    return rows


def audit(output):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    started = time.perf_counter()
    config, source, geometry, mass = load_inputs()
    manifest = (source / "MANIFEST_SHA256.txt").read_text().splitlines()
    for line in manifest:
        expected, relative = line.split("  ", 1)
        if digest(source / relative) != expected:
            raise ValueError(f"Source manifest mismatch: {relative}")
    stage = Usd.Stage.Open(str(source / "usd/drone.usdc"))
    root = stage.GetDefaultPrim()
    assert str(root.GetPath()) == "/Drone"
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0 and UsdGeom.GetStageUpAxis(stage) == "Z"
    assert np.array_equal(np.asarray(UsdGeom.Xformable(root).GetLocalTransformation()), np.eye(4))
    api = UsdPhysics.MassAPI(root)
    principal = api.GetPrincipalAxesAttr().Get()
    q = np.r_[principal.GetReal(), principal.GetImaginary()]
    # Gf matrices use row vectors. Transpose to the body-column convention.
    axes = np.asarray(Gf.Matrix3d(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:].astype(float).tolist())))).T
    diagonal = np.asarray(api.GetDiagonalInertiaAttr().Get())
    inertia = axes @ np.diag(diagonal) @ axes.T
    expected_inertia = np.asarray(mass["inertia_body_kg_m2"])
    np.testing.assert_allclose(inertia, expected_inertia, atol=2e-11, rtol=1e-6)
    assert abs(api.GetMassAttr().Get() - config["expected_mass_kg"]) < 1e-8
    np.testing.assert_allclose(api.GetCenterOfMassAttr().Get(), [0., 0., 0.], atol=1e-9)
    cad_to_body = np.asarray(mass["cad_to_body"], dtype=float)
    assert np.linalg.det(cad_to_body) == 1.
    reconstructed_inertia = np.zeros((3, 3))
    reconstructed_com = np.zeros(3)
    for part in mass["parts"]:
        center = np.asarray(part["center_body_m"])
        np.testing.assert_allclose(cad_to_body @ (np.asarray(part["center_m"]) - np.asarray(mass["cad_com_mm"]) * .001),
                                   center, atol=1e-12, rtol=0)
        reconstructed_com += part["mass_kg"] * center
        reconstructed_inertia += cad_to_body @ np.asarray(part["inertia_kg_m2"]) @ cad_to_body.T
        reconstructed_inertia += part["mass_kg"] * ((center @ center) * np.eye(3) - np.outer(center, center))
    np.testing.assert_allclose(reconstructed_inertia, expected_inertia, atol=1e-12)
    np.testing.assert_allclose(reconstructed_com, 0., atol=1e-12)
    entries = {e["name"]: e for e in geometry["entries"]}
    counts, offsets, vertex_counts = Counter(), Counter(), []
    vertices, worst_mesh_error, worst_transform_error = [], 0., 0.
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        entry = entries[prim.GetName()]
        group = prim.GetCustomDataByKey("contact_group")
        assert group == entry["group"] and prim.GetCustomDataByKey("source_part_index") == entry["source_part"]
        assert str(prim.GetPath()) == "/Drone/Collision/" + entry["name"]
        assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "convexHull"
        points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        expected, _ = obj_mesh(source / "mujoco" / entry["mesh"])
        assert points.shape == expected.shape
        error = float(np.max(np.abs(points - expected)))
        if error > 1e-8:
            raise ValueError(f"USD mesh differs from source OBJ: {prim.GetPath()}: {error}")
        transform_error = float(np.max(np.abs(np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)) - np.eye(4))))
        assert transform_error < 1e-12
        counts[group] += 1
        offset = float(prim.GetAttribute("physxCollision:contactOffset").Get())
        rest = float(prim.GetAttribute("physxCollision:restOffset").Get())
        assert rest == 0.0
        offsets[f"{offset:.9g}"] += 1
        vertex_counts.append(len(points))
        vertices.append(expected)
        worst_mesh_error = max(worst_mesh_error, error)
        worst_transform_error = max(worst_transform_error, transform_error)
    assert counts == config["groups"]
    all_vertices = np.concatenate(vertices)
    fixtures = fixture_suite(all_vertices)
    fixture_contract = {"schema": "asm2_matched_collision_fixtures_v1", "physics_dt_s": config["physics_dt_s"],
                        "fixtures": fixtures, "qualification": "GEOMETRY_DIAGNOSTICS_ONLY"}
    fixture_path = RUN / "fixtures.json"
    if fixture_path.exists():
        if read_json(fixture_path) != fixture_contract:
            raise ValueError("Computed fixtures differ from the frozen matched-state contract")
    else:
        write_result(fixture_path, fixture_contract)
    result = dict(status="PASS_ASSET_CONFORMANCE; GEOMETRY_ACCEPTANCE_PENDING", source_files_verified=len(manifest),
                  usd_version=Usd.GetVersion(), units="m", up_axis="Z", body_frame="COM-origin FLU, inferred from CAD",
                  root_identity=True, mass_kg=float(api.GetMassAttr().Get()), com_m=list(api.GetCenterOfMassAttr().Get()),
                  inertia_body_kg_m2=inertia.tolist(), inertia_max_error_kg_m2=float(np.max(np.abs(inertia-expected_inertia))),
                  mass_properties_recomputed=True, battery_translation_applied_again=False,
                  battery_translation_cad_mm=mass["battery_translation_cad_mm"],
                  collider_groups=dict(counts), source_offsets_m=dict(offsets), rest_offset_m=0.,
                  collider_vertices_max=max(vertex_counts), colliders_over_64_vertices=sum(n > 64 for n in vertex_counts),
                  usd_obj_max_coordinate_error_m=worst_mesh_error, collider_transform_max_error=worst_transform_error,
                  bounds_body_m=[all_vertices.min(0).tolist(), all_vertices.max(0).tolist()],
                  mass_provenance="CAD_ESTIMATE; rk3576_body surface approximation; 4.5g point mass; not calibrated hardware",
                  supplied_validation_passed=read_json(source/"metadata/collision_validation.json")["passed"],
                  wall_s=time.perf_counter()-started, training_transitions=0)
    write_result(output, result)
    print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=RUN / "asset-audit.json")
    audit(parser.parse_args().output)


if __name__ == "__main__":
    main()
