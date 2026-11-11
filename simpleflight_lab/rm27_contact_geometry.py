"""Export collision geometry from the verified RM27 Stage 1 MuJoCo archive.

Static poses are in the archived NWU world frame. Aircraft shapes are in its
FLU body frame. A mesh's local vertices already include the asset scale and
mesh frame used by MuJoCo. Compose only the exported geom pose with them.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np


_ROOT = Path(__file__).resolve().parents[1]
_RL = _ROOT / "upstream/rm27_RL"
_FROZEN_CONFIG = _RL / "configs/stage1/task_contact_geometry.json"
# Per-shape PhysX contact generation offset. Both shapes contribute to a pair.
PHYSX_CONTACT_OFFSET_M = 0.00001
# Reserve for acceleration during the step and cross-backend rounding.
_ONE_TICK_TRAVEL_RESERVE_M = 0.00098


def _sha(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _unit(vector) -> np.ndarray:
    array = np.asarray(vector, dtype=np.float64)
    return array / np.linalg.norm(array)


def _mesh_hull(model, mesh_id: int) -> tuple[np.ndarray, np.ndarray]:
    """Decode the compiled collision hull, separate from the render mesh.

    MuJoCo 3.12 API Types / Convex hulls defines the graph record layout:
    https://mujoco.readthedocs.io/en/3.12.0/APIreference/APItypes.html#convex-hulls
    """
    address = int(model.mesh_graphadr[mesh_id])
    if address < 0:
        raise ValueError(f"No compiled convex hull for collision mesh {mesh_id}")
    graph = model.mesh_graph
    n_vertices, n_faces = map(int, graph[address:address + 2])
    first_vertex = address + 2 + n_vertices
    vertex_ids = graph[first_vertex:first_vertex + n_vertices]
    first_face = address + 2 + 3 * n_vertices + 3 * n_faces
    faces = graph[first_face:first_face + 3 * n_faces].reshape(-1, 3)
    n_model_vertices = int(model.mesh_vertnum[mesh_id])
    if (
        n_vertices < 4 or n_faces < 4 or len(faces) != n_faces
        or np.any(vertex_ids < 0) or np.any(vertex_ids >= n_model_vertices)
        or len(np.unique(vertex_ids)) != n_vertices
        or np.any(faces < 0) or np.any(faces >= n_model_vertices)
    ):
        raise ValueError(f"Invalid compiled convex hull for mesh {mesh_id}")
    first = int(model.mesh_vertadr[mesh_id])
    vertices = model.mesh_vert[first + vertex_ids].copy()
    remap = np.full(n_model_vertices, -1, dtype=np.int32)
    remap[vertex_ids] = np.arange(n_vertices, dtype=np.int32)
    triangles = remap[faces]
    if np.any(triangles < 0) or len(np.unique(triangles)) != n_vertices:
        raise ValueError(f"Convex hull faces do not match vertices of mesh {mesh_id}")
    # Verify the native winding without changing the collision topology.
    a, b, c = vertices[triangles].astype(np.float64).transpose(1, 0, 2)
    outward = np.einsum("ij,ij->i", np.cross(b - a, c - a), a - vertices.mean(axis=0, dtype=np.float64))
    if not np.all(outward > 0):
        raise ValueError(f"Non-outward convex hull faces for mesh {mesh_id}")
    return vertices, triangles


def export(cache_dir: Path, output: Path) -> dict:
    import mujoco

    if mujoco.__version__ != "3.12.0":
        raise RuntimeError("The frozen Stage 1 MJB requires MuJoCo 3.12.0")
    manifest_path = cache_dir / "stage1_scene.json"
    model_path = cache_dir / "scene.mjb"
    manifest = json.loads(manifest_path.read_text())
    config = json.loads(_FROZEN_CONFIG.read_text())
    registry_path = _RL / config["contact_surface_registration"]
    registry_sha = _sha(registry_path)
    registry = json.loads(registry_path.read_text())
    model_sha = _sha(model_path)
    if (
        registry_sha != config["expected_contact_surface_registration_sha256"]
        or not manifest["compiled"] or manifest["mujoco_version"] != mujoco.__version__
        or model_sha != manifest["compiled_model_sha256"] or model_sha != registry["physics_model_sha256"]
        or manifest["inputs"]["source_scene"]["sha256"] != config["expected_scene_sha256"]
        or manifest["inputs"]["target_mapping"]["sha256"] != registry["original_target_mapping_sha256"]
    ):
        raise ValueError("MJB or contact registration does not match the frozen Stage 1 archive")

    target = manifest["resolved_target_mapping"]
    if any(target[key] != value for key, value in registry["target_reference"].items()):
        raise ValueError("Archived target frame differs from the frozen registration")
    legal = set(target["registered_contact_geoms"]) | {r["geom"] for r in registry["additional_contact_geoms"]}
    if len(legal) != 25 or not set(target["registered_contact_geoms"]) <= set(registry["original_registered_contact_geoms"]):
        raise ValueError("The archive must retain all 25 verified selected-side contact colliders")
    center = np.asarray(target["armor_surface_center_m"])
    normal = _unit(target["armor_normal"])
    frame = np.column_stack((-_unit(target["slope_axis"]), _unit(target["across_axis"]), normal))
    if not np.allclose(frame.T @ frame, np.eye(3), atol=1e-5, rtol=0) or not np.isclose(np.linalg.det(frame), 1.0):
        raise ValueError("Frozen target frame is not a proper rotation")

    model = mujoco.MjModel.from_binary_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    active = model.body("drone_1").id
    collidable = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
    active_mask = collidable & (model.body_rootid[model.geom_bodyid] == active)
    active_ids = np.flatnonzero(active_mask)
    if len(active_ids) != 247 or not np.all(model.geom_bodyid[active_ids] == active):
        raise ValueError("Expected 247 archived active shapes attached to drone_1")

    safety = config["safety"]
    low = np.asarray(safety["position_min_target_frame_m"])
    high = np.asarray(safety["position_max_target_frame_m"])
    # Cover every possible orientation within the COM safety box, expanded
    # by the greatest active body radius and one interval at the speed limit.
    # Include both native margins and both PhysX contact-generation offsets.
    # The travel reserve avoids culling contacts near the swept boundary.
    reach = float(np.max(np.linalg.norm(model.geom_pos[active_ids], axis=1) + model.geom_rbound[active_ids]))
    one_tick = safety["max_speed_m_s"] * model.opt.timestep
    padding = (reach + one_tick + float(model.geom_margin[active_ids].max())
               + 2 * PHYSX_CONTACT_OFFSET_M + _ONE_TICK_TRAVEL_RESERVE_M)
    geometry_rotation = data.geom_xmat.reshape(-1, 3, 3)
    obb_center = data.geom_xpos + np.einsum("nij,nj->ni", geometry_rotation, model.geom_aabb[:, :3])
    target_center = (obb_center - center) @ frame
    target_rotation = np.einsum("ij,njk->nik", frame.T, geometry_rotation)
    target_halfsize = np.einsum("nij,nj->ni", np.abs(target_rotation), model.geom_aabb[:, 3:])
    halfsize = target_halfsize + padding + model.geom_margin[:, None]
    within = ((target_center + halfsize >= low) & (target_center - halfsize <= high)).all(axis=1)
    planes = model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE
    static_ids = np.flatnonzero(collidable & ~active_mask & (within | planes))
    if np.any(model.body_weldid[model.geom_bodyid[static_ids]] != 0):
        raise ValueError("A movable non-active collider is within the bounded safety scene")
    static_names = {model.geom(int(i)).name for i in static_ids}
    if not legal <= static_names:
        raise ValueError(f"Missing legal target hulls: {sorted(legal - static_names)}")

    mesh_arrays = {}

    def shape(index: int, dynamic: bool) -> dict:
        name = model.geom(index).name
        kind = mujoco.mjtGeom(int(model.geom_type[index])).name.removeprefix("mjGEOM_").lower()
        mesh_key = None
        if kind == "mesh":
            mesh_id = int(model.geom_dataid[index])
            mesh_key = f"mesh_{mesh_id}"
            if f"{mesh_key}_vertices" not in mesh_arrays:
                vertices, triangles = _mesh_hull(model, mesh_id)
                mesh_arrays[f"{mesh_key}_vertices"] = vertices
                mesh_arrays[f"{mesh_key}_faces"] = triangles
        elif kind not in {"plane", "sphere", "capsule", "box", "cylinder"}:
            raise ValueError(f"Unsupported frozen collision shape {name}: {kind}")
        body_id = int(model.geom_bodyid[index])
        position = model.geom_pos[index] if dynamic else data.geom_xpos[index]
        quat = model.geom_quat[index]
        if not dynamic and body_id != 0:
            quat = np.empty(4)
            mujoco.mju_mulQuat(quat, data.xquat[body_id], model.geom_quat[index])
        return {
            "name": name, "id": index, "type": kind, "body": model.body(body_id).name,
            "position_m": position.tolist(), "quaternion_wxyz": quat.tolist(),
            "size": model.geom_size[index].tolist(), "mesh": mesh_key, "legal": name in legal,
            "contype": int(model.geom_contype[index]), "conaffinity": int(model.geom_conaffinity[index]),
            "friction": model.geom_friction[index].tolist(), "solref": model.geom_solref[index].tolist(),
            "solimp": model.geom_solimp[index].tolist(), "condim": int(model.geom_condim[index]),
            "margin_m": float(model.geom_margin[index]), "gap_m": float(model.geom_gap[index]),
        }

    aircraft = [shape(int(i), True) for i in active_ids]
    static = [shape(int(i), False) for i in static_ids]
    if len({row["name"] for row in (*aircraft, *static)}) != len(aircraft) + len(static):
        raise ValueError("Collision geom names must be distinct")
    if [r["name"] for r in aircraft if r["type"] == "sphere"] != ["drone_1_stage1_contact"]:
        raise ValueError("The archived active underside sphere is absent or ambiguous")

    # Validate the decoded collision coordinates against the registered
    # original geometry. The source's 0.2 µm bound is float32 storage error.
    selected = {row["name"]: row for row in static}
    registry_meshes = [registry["reconstruction"]["existing_proxy_record"], *registry["additional_contact_geoms"]]
    worst_vertex_error = 0.0
    for record in registry_meshes:
        row = selected[record["geom"]]
        local = mesh_arrays[f"{row['mesh']}_vertices"].astype(np.float64)
        world = local @ geometry_rotation[row["id"]].T + row["position_m"]
        expected = np.asarray(record["expected_world_vertices_m"], dtype=np.float64)
        distances = np.linalg.norm(world[:, None] - expected[None, :], axis=-1)
        error = float(max(distances.min(axis=0).max(), distances.min(axis=1).max()))
        if world.shape != expected.shape or error > 2e-7:
            raise ValueError(f"Exported hull differs from audited geometry: {record['geom']}: {error:.9g} m")
        worst_vertex_error = max(worst_vertex_error, error)

    archived_aircraft = {}
    for i in range(1, 5):
        name = f"drone_{i}"
        body_id = model.body(name).id
        archived_aircraft[name] = {
            "position_m": data.xpos[body_id].tolist(), "quaternion_wxyz": data.xquat[body_id].tolist(),
        }
    metadata = {
        "schema": "rm27_stage1_collision_hulls_v1", "mujoco_version": mujoco.__version__,
        "model_sha256": model_sha, "registry_sha256": registry_sha,
        "source_scene_sha256": config["expected_scene_sha256"],
        "target": {
            "center_m": center.tolist(), "target_rotation": frame.tolist(),
            "normal": normal.tolist(), "halfsize_m": target["contact_region_halfsize_slope_across_m"],
        },
        "body": {
            "name": "drone_1", "mass_kg": float(model.body_mass[active]),
            "diagonal_inertia_kg_m2": model.body_inertia[active].tolist(),
            "com_m": model.body_ipos[active].tolist(),
            "inertial_quaternion_wxyz": model.body_iquat[active].tolist(),
        },
        "archived_aircraft": archived_aircraft,
        "selection": {
            "safety_lower_target_frame_m": low.tolist(), "safety_upper_target_frame_m": high.tolist(),
            "active_max_reach_m": reach, "max_speed_one_tick_travel_m": one_tick,
            "physx_contact_offset_per_shape_m": PHYSX_CONTACT_OFFSET_M,
            "one_tick_travel_reserve_m": _ONE_TICK_TRAVEL_RESERVE_M,
            "padding_m_excluding_static_margin": padding,
            "included_static_types": dict(Counter(row["type"] for row in static)),
            "included_static_bodies": dict(Counter(row["body"] for row in static)),
            "excluded_nonactive_colliders": int(np.count_nonzero(collidable & ~active_mask)) - len(static),
            "legal_static_count": len(legal), "audited_world_hull_max_vertex_error_m": worst_vertex_error,
        },
        "aircraft": aircraft, "static": static,
    }
    metadata_path = output.with_suffix(".json")
    arrays_path = output.with_suffix(".npz")
    if metadata_path.exists() or arrays_path.exists():
        raise FileExistsError(f"Choose a new export prefix: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(arrays_path, **mesh_arrays)
    with metadata_path.open("x") as stream:
        stream.write(json.dumps(metadata, separators=(",", ":"), allow_nan=False) + "\n")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="New output prefix for JSON and NPZ")
    args = parser.parse_args()
    metadata = export(args.cache_dir, args.output)
    print(f"RM27_STAGE1_GEOMETRY {len(metadata['aircraft'])} active, {len(metadata['static'])} static, "
          f"{metadata['selection']['legal_static_count']} registered target colliders")


if __name__ == "__main__":
    main()
