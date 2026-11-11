"""Build and measure a compact per-part ASM2 convex decomposition."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import importlib.metadata
from pathlib import Path
import shutil
import time

import numpy as np
from scipy.spatial import ConvexHull, cKDTree
import trimesh

from .asset import ROOT, digest, obj_mesh, read_json, write_result
from .proxy import GEOMETRY, PROXY, SCHEMA, SOURCE, load_proxy


RUN = ROOT / ".agents/runs/asm2-contact"
ELECTRONICS = {1, 4, 5, 12, 13}
MOTOR_COPIES = {6, 7, 8}
DETAILED_CAMERA = 3
AXIAL_DETAIL_BAND_M = 0.0003
COACD = dict(
    preprocess_resolution=90, resolution=1500, mcts_iterations=10,
    mcts_nodes=8, mcts_max_depth=2, decimate=True, max_ch_vertex=32, seed=42,
)


def coacd_params(index):
    if index == 2:
        return dict(COACD, threshold=.035, preprocess_mode="auto")
    return dict(COACD, threshold=.010 if index == 0 else .006,
                preprocess_resolution=145 if index == 0 else 180,
                resolution=1600, mcts_iterations=12, preprocess_mode="auto")


def decompose(index_path):
    import coacd

    index, path = index_path
    coacd.set_log_level("warn")
    mesh = trimesh.load_mesh(path)
    hulls = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **coacd_params(index))
    if not hulls:
        raise ValueError(f"CoACD produced no hulls for {path}")
    return index, [vertices for vertices, _ in hulls]


def convex_mesh(points):
    hull = ConvexHull(points)
    vertices = points[hull.vertices]
    inverse = np.full(len(points), -1, dtype=int)
    inverse[hull.vertices] = np.arange(len(vertices))
    faces = inverse[hull.simplices]
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    flip = np.einsum("ij,ij->i", normals, hull.equations[:, :3]) < 0
    faces[flip] = faces[flip, ::-1]
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)


def clip_hull(mesh, axis, value, sign=1):
    """Clip a convex polytope using exact edge/plane intersections."""
    vertices = mesh.vertices
    offsets = sign * (vertices[:, axis] - value)
    if offsets.min() >= 0:
        return mesh
    if offsets.max() <= 0:
        return None
    first, second = mesh.edges_unique.T
    crosses = (offsets[first] < 0) != (offsets[second] < 0)
    first, second = first[crosses], second[crosses]
    ratio = offsets[first] / (offsets[first] - offsets[second])
    intersections = vertices[first] + ratio[:, None] * (vertices[second] - vertices[first])
    intersections[:, axis] = value
    points = np.concatenate((vertices[offsets >= 0], intersections))
    if len(points) < 4 or np.ptp(points[:, axis]) < 1e-8:
        return None
    return convex_mesh(points)


def gpu_pieces(mesh):
    """Partition exact hulls to at most 64 triangular faces per piece."""
    if len(mesh.faces) <= 64 and len(mesh.vertices) <= 64:
        yield mesh
        return
    axis = int(np.argmax(np.ptp(mesh.vertices, axis=0)))
    low, high = mesh.bounds[:, axis]
    value = (low + high) / 2
    for sign in (-1, 1):
        piece = clip_hull(mesh, axis, value, sign)
        if piece is None or np.ptp(piece.vertices[:, axis]) >= np.ptp(mesh.vertices[:, axis]):
            raise ValueError("Unable to partition a large convex piece")
        yield from gpu_pieces(piece)


def axial_detail(geometry, expected_digests):
    """Keep the camera and outer side bands from the frozen detailed hulls."""
    raw = []
    for entry in geometry["entries"]:
        if entry["group"] != "body":
            continue
        relative = "mujoco/" + entry["mesh"]
        path = SOURCE / relative
        if digest(path) != expected_digests[relative]:
            raise ValueError(f"Detailed source mesh changed: {relative}")
        vertices, _ = obj_mesh(path)
        raw.append((entry["source_part"], vertices))
    lows = np.array([v.min(axis=0) for _, v in raw]).min(axis=0)
    highs = np.array([v.max(axis=0) for _, v in raw]).max(axis=0)
    planes = [(axis, low + AXIAL_DETAIL_BAND_M, 1)
              for axis, low in enumerate(lows[:2])]
    planes += [(axis, high - AXIAL_DETAIL_BAND_M, -1)
               for axis, high in enumerate(highs[:2])]
    detail = {}
    for index, vertices in raw:
        if index == DETAILED_CAMERA:
            detail.setdefault(index, []).append(convex_mesh(vertices))
        elif any(np.any(sign * (vertices[:, axis] - value) < 0)
                 for axis, value, sign in planes):
            hull = convex_mesh(vertices)
            # Clip adjacent bands against each other to avoid creating
            # duplicate volume where two outer side bands meet.
            for axis, value, sign in planes:
                outer = clip_hull(hull, axis, value, -sign)
                if outer is not None:
                    detail.setdefault(index, []).append(outer)
                hull = clip_hull(hull, axis, value, sign)
                if hull is None:
                    break
    return planes, detail


def build(output, cache_parts=None):
    started = time.perf_counter()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "meshes").mkdir()
    mass_path = SOURCE / "metadata/mass_properties.json"
    parts = read_json(mass_path)["parts"]
    geometry = read_json(GEOMETRY)
    original_manifest = SOURCE / "MANIFEST_SHA256.txt"
    expected_digests = {path: sha for sha, path in (
        line.split("  ", 1) for line in original_manifest.read_text().splitlines())}
    visual_paths = [SOURCE / "mujoco" / part["mesh"] for part in parts]
    for path in (GEOMETRY, mass_path, *visual_paths):
        relative = str(path.relative_to(SOURCE))
        if digest(path) != expected_digests[relative]:
            raise ValueError(f"Source mesh or metadata changed: {relative}")
    jobs = [(i, p) for i, p in enumerate(visual_paths)
            if i not in ELECTRONICS | MOTOR_COPIES | {DETAILED_CAMERA}]
    if cache_parts is None:
        with ProcessPoolExecutor(max_workers=len(jobs)) as executor:
            hulls = dict(executor.map(decompose, jobs))
    else:
        if set(cache_parts) != {i for i, _ in jobs}:
            raise ValueError("Supply one decomposition array file per decomposed source part")
        hulls = {}
        for index, archive in cache_parts.items():
            with np.load(archive) as values:
                hulls[index] = [values[name] for name in values.files]
    planes, detailed_pieces = axial_detail(geometry, expected_digests)
    entries = []
    part_counts = {}
    for index, path in enumerate(visual_paths):
        if index in ELECTRONICS:
            vertices, _ = obj_mesh(path)
            point_sets = [vertices]
        elif index in MOTOR_COPIES:
            delta = np.asarray(parts[index]["center_body_m"]) - parts[2]["center_body_m"]
            # Verify the copied rotor meshes really are translated instances.
            vertices, faces = obj_mesh(path)
            reference, reference_faces = obj_mesh(visual_paths[2])
            error, vertex_ids = cKDTree(reference).query(vertices - delta)
            source_triangles = sorted(map(tuple, np.sort(reference_faces, axis=1)))
            mapped_triangles = sorted(map(tuple, np.sort(vertex_ids[faces], axis=1)))
            if (len(set(vertex_ids)) != len(reference) or error.max() > 1e-7
                    or mapped_triangles != source_triangles):
                raise ValueError(f"Rotor {index} is not a translated source instance")
            point_sets = [points + delta for points in hulls[2]]
        elif index == DETAILED_CAMERA:
            point_sets = []
        else:
            point_sets = hulls[index]
        body = []
        for points in point_sets:
            hull = convex_mesh(points)
            if index == 0:
                hull = clip_hull(hull, 2, geometry["general_carrier_cut_z_m"])
            for axis, value, sign in planes:
                if hull is not None:
                    hull = clip_hull(hull, axis, value, sign)
            if hull is not None:
                body.append(hull)
        body.extend(detailed_pieces.get(index, ()))
        count = 0
        for hull in body:
            for piece in gpu_pieces(hull):
                name = f"collision_proxy_{index:02d}_{count:03d}"
                relative = f"meshes/{name}.obj"
                mesh_path = output / relative
                piece.export(mesh_path, digits=10)
                entries.append(dict(name=name, group="body", mesh=relative,
                                    sha256=digest(mesh_path), source_part=index))
                count += 1
        part_counts[index] = count
    for frozen in geometry["entries"]:
        if frozen["group"] == "body":
            continue
        relative = f"meshes/{frozen['name']}.obj"
        destination = output / relative
        source_mesh = SOURCE / "mujoco" / frozen["mesh"]
        if digest(source_mesh) != expected_digests["mujoco/" + frozen["mesh"]]:
            raise ValueError(f"Source contact patch changed: {frozen['name']}")
        shutil.copyfile(source_mesh, destination)
        entries.append(dict(name=frozen["name"], group=frozen["group"],
                            source_part=frozen["source_part"], mesh=relative,
                            sha256=digest(destination)))
    manifest = dict(
        schema=SCHEMA, training_allowed=False, units="m",
        frame="ASM2 COM-centred body FLU", entries=entries,
        source_pins={str(path.relative_to(SOURCE)): digest(path)
                     for path in (GEOMETRY, original_manifest, mass_path, *visual_paths)},
        generator=dict(
            implementation_sha256=digest(__file__),
            dependencies={name: importlib.metadata.version(name) for name in (
                "coacd", "numpy", "scipy", "trimesh", "embreex", "rtree")},
            coacd_parameters={str(i): coacd_params(i) for i, _ in jobs},
            decomposition_cache_sha256=({i: digest(path) for i, path in cache_parts.items()}
                                        if cache_parts is not None else {}),
            detailed_camera_source_part=DETAILED_CAMERA,
            frozen_axial_detail_band_m=AXIAL_DETAIL_BAND_M,
            frozen_axial_detail_planes=planes,
            electronics_envelopes=sorted(ELECTRONICS),
            motor_translation_reuse=sorted(MOTOR_COPIES),
            carrier_cut_z_m=geometry["general_carrier_cut_z_m"],
            max_body_hull_triangles=64, convex_body_counts=part_counts,
        ),
    )
    write_result(output / "manifest.json", manifest)
    load_proxy(output / "manifest.json")
    print(f"ASM2 proxy: {len(entries)} hulls, elapsed {time.perf_counter() - started:.2f} s", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROXY.parent)
    parser.add_argument("--cached-part", action="append", default=[], metavar="INDEX=NPZ")
    args = parser.parse_args()
    cached = {int(index): Path(path) for index, path in
              (pair.split("=", 1) for pair in args.cached_part)}
    build(args.output, cached or None)


if __name__ == "__main__":
    main()
