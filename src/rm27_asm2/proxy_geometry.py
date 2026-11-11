"""Measure geometric errors and guard-clearance regression before simulator tests."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector

from .asset import digest, obj_mesh, read_json, write_result
from .proxy import GEOMETRY, PROXY, SOURCE, load_proxy


# Prospective engineering gates for the sampled geometry checks.
GUARD_MATERIAL_ERROR_M = 0.00025
BODY_SUPPORT_ERROR_M = 0.0008
CONTACT_SUPPORT_ERROR_M = 0.00025
SILHOUETTE_ERROR_M = 0.001
CORRIDOR_RADIUS_M = 0.0005
GRID_M = 0.002


def triangle_mesh(paths):
    all_vertices, all_faces = [], []
    count = 0
    for path in paths:
        vertices, faces = obj_mesh(path)
        all_vertices.append(vertices)
        all_faces.append(faces + count)
        count += len(vertices)
    return trimesh.Trimesh(vertices=np.concatenate(all_vertices),
                           faces=np.concatenate(all_faces), process=False)


def ray_hits(intersector, origins, directions):
    locations, ray_ids, _ = intersector.intersects_location(
        origins, directions, multiple_hits=False
    )
    distance = np.full(len(origins), -1.0)
    distance[ray_ids] = np.einsum(
        "ij,ij->i", locations - origins[ray_ids], directions[ray_ids]
    )
    return distance


def guard_origins():
    """Match the source package's 288 historical probe positions and order."""
    rows = []
    for cx in (-0.0275, 0.0275):
        for cy in (-0.041, 0.041):
            for radius in (0.012, 0.017, 0.022):
                for angle in np.linspace(0, 2 * np.pi, 24, endpoint=False):
                    rows.append([cx + radius * np.cos(angle), cy + radius * np.sin(angle), -0.06])
    return np.asarray(rows)


def closest_distances(mesh, points):
    batches = []
    for start in range(0, len(points), 256):
        _, distances, _ = trimesh.proximity.closest_point(mesh, points[start:start + 256])
        batches.append(distances)
    return np.concatenate(batches) if batches else np.empty(0)


def inside_proxy(mesh_paths, points):
    """Resolve sampled points against the union of authored convex pieces."""
    covered = np.zeros(len(points), dtype=bool)
    for path in mesh_paths:
        vertices, _ = obj_mesh(path)
        nearby = np.flatnonzero(~covered & (
            (points >= vertices.min(axis=0)) & (points <= vertices.max(axis=0))
        ).all(axis=1))
        if len(nearby):
            planes = ConvexHull(vertices).equations
            covered[nearby] = (points[nearby] @ planes[:, :3].T + planes[:, 3] <= 1e-9).all(axis=1)
    return covered


def distribution(distances):
    if not len(distances):
        return dict(count=0, median_m=None, p95_m=None, max_m=None)
    return dict(count=len(distances), median_m=float(np.median(distances)),
                p95_m=float(np.quantile(distances, 0.95)), max_m=float(np.max(distances)))


def ray_comparison(visual, candidate, origins, directions, reference_hit, candidate_hit):
    false = (reference_hit < 0) & (candidate_hit >= 0)
    missed = (reference_hit >= 0) & (candidate_hit < 0)
    common = (reference_hit >= 0) & (candidate_hit >= 0)
    false_points = origins[false] + candidate_hit[false, None] * directions[false]
    missed_points = origins[missed] + reference_hit[missed, None] * directions[missed]
    false_error = closest_distances(visual, false_points)
    missed_error = closest_distances(candidate, missed_points)
    depth_error = candidate_hit[common] - reference_hit[common]
    return dict(
        rays=len(origins), reference_open=int(np.sum(reference_hit < 0)),
        false_positive_ids=np.flatnonzero(false).tolist(),
        false_positive_distance_to_cad_m=false_error.tolist(),
        miss_ids=np.flatnonzero(missed).tolist(),
        missed_cad_distance_to_proxy_m=missed_error.tolist(),
        abs_common_first_hit_depth_error=distribution(np.abs(depth_error)),
        earliest_depth_shift_m=float(depth_error.min(initial=0)),
        latest_depth_shift_m=float(depth_error.max(initial=0)),
    )


def silhouette_rays(bounds):
    origins, directions = [], []
    for axis in range(3):
        i, j = (d for d in range(3) if d != axis)
        a, b = np.meshgrid(
            np.arange(bounds[0, i] - GRID_M, bounds[1, i] + GRID_M, GRID_M),
            np.arange(bounds[0, j] - GRID_M, bounds[1, j] + GRID_M, GRID_M),
            indexing="ij",
        )
        points = np.zeros((a.size, 3))
        points[:, i], points[:, j] = a.ravel(), b.ravel()
        for sign in (-1, 1):
            points[:, axis] = bounds[int(sign < 0), axis] - sign * .025
            direction = np.zeros(3)
            direction[axis] = sign
            origins.append(points.copy())
            directions.append(np.broadcast_to(direction, points.shape))
    return np.concatenate(origins), np.concatenate(directions)


def directions_for_support():
    n = 256
    index = np.arange(n) + .5
    z = 1 - 2 * index / n
    radius = np.sqrt(1 - z * z)
    angle = index * np.pi * (3 - np.sqrt(5))
    broad = np.vstack((np.eye(3), -np.eye(3),
                       np.column_stack((radius * np.cos(angle), radius * np.sin(angle), z))))
    tilt, azimuth = np.meshgrid(
        np.deg2rad([0, 1, 3, 5, 10, 15]),
        np.linspace(0, 2 * np.pi, 36, endpoint=False), indexing="ij",
    )
    near_patches = np.column_stack((
        (np.sin(tilt) * np.cos(azimuth)).ravel(),
        (np.sin(tilt) * np.sin(azimuth)).ravel(),
        -np.cos(tilt).ravel(),
    ))
    return broad, near_patches


def support(vertices, directions):
    result = np.full(len(directions), -np.inf)
    for start in range(0, len(vertices), 4096):
        result = np.maximum(result, (vertices[start:start + 4096] @ directions.T).max(axis=0))
    return result


def measure(manifest_path, output):
    manifest_path = Path(manifest_path).resolve()
    manifest = load_proxy(manifest_path)
    original = read_json(GEOMETRY)
    visual = triangle_mesh([
        SOURCE / "mujoco" / part["mesh"]
        for part in read_json(SOURCE / "metadata/mass_properties.json")["parts"]
    ])
    detailed = triangle_mesh([SOURCE / "mujoco" / entry["mesh"] for entry in original["entries"]])
    candidate_paths = [manifest_path.parent / entry["mesh"] for entry in manifest["entries"]]
    candidate = triangle_mesh(candidate_paths)
    body = triangle_mesh([
        path for path, entry in zip(candidate_paths, manifest["entries"]) if entry["group"] == "body"
    ])
    detailed_body = triangle_mesh([
        SOURCE / "mujoco" / entry["mesh"] for entry in original["entries"] if entry["group"] == "body"
    ])
    vis_ray = RayMeshIntersector(visual)
    full_ray = RayMeshIntersector(detailed)
    proxy_ray = RayMeshIntersector(candidate)

    origins = guard_origins()
    ray_direction = np.array([0.0, 0.0, 1.0])
    directions = np.broadcast_to(ray_direction, origins.shape)
    visual_hit = ray_hits(vis_ray, origins, directions)
    detailed_hit = ray_hits(full_ray, origins, directions)
    proxy_hit = ray_hits(proxy_ray, origins, directions)
    guard_detailed = ray_comparison(visual, detailed, origins, directions, visual_hit, detailed_hit)
    guard_proxy = ray_comparison(visual, candidate, origins, directions, visual_hit, proxy_hit)

    angles = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    offsets = CORRIDOR_RADIUS_M * np.column_stack((np.cos(angles), np.sin(angles), np.zeros(8)))
    offsets = np.vstack((np.zeros((1, 3)), offsets))
    corridor_origins = (origins[:, None, :] + offsets).reshape(-1, 3)
    corridor_directions = np.broadcast_to(ray_direction, corridor_origins.shape)
    vis_corridor = ray_hits(vis_ray, corridor_origins, corridor_directions)
    detailed_corridor = ray_hits(full_ray, corridor_origins, corridor_directions)
    proxy_corridor = ray_hits(proxy_ray, corridor_origins, corridor_directions)
    corridor = ray_comparison(visual, candidate, corridor_origins, corridor_directions,
                              vis_corridor, proxy_corridor)
    corridors_open_in_cad = (vis_corridor.reshape(-1, 9) < 0).all(axis=1)
    corridor.update(
        radius_m=CORRIDOR_RADIUS_M, rays_per_corridor=9,
        cad_open_count=int(corridors_open_in_cad.sum()),
        detailed_blocked_ids=np.flatnonzero(corridors_open_in_cad &
                                           (detailed_corridor.reshape(-1, 9) >= 0).any(axis=1)).tolist(),
        proxy_blocked_ids=np.flatnonzero(corridors_open_in_cad &
                                        (proxy_corridor.reshape(-1, 9) >= 0).any(axis=1)).tolist(),
    )

    grid_origins, grid_directions = silhouette_rays(visual.bounds)
    grid_visual_hit = ray_hits(vis_ray, grid_origins, grid_directions)
    silhouette = ray_comparison(
        visual, candidate, grid_origins, grid_directions,
        grid_visual_hit,
        ray_hits(proxy_ray, grid_origins, grid_directions),
    )
    silhouette["spacing_m"] = GRID_M
    silhouette_detailed = ray_comparison(
        visual, detailed, grid_origins, grid_directions, grid_visual_hit,
        ray_hits(full_ray, grid_origins, grid_directions),
    )
    # Area-weighted point samples supplement the directional first-hit probes.
    sample_visual, _ = trimesh.sample.sample_surface(visual, 4096, seed=42)
    sample_proxy, _ = trimesh.sample.sample_surface(candidate, 4096, seed=43)
    visual_sample_distances = closest_distances(candidate, sample_visual)
    sample_outlier_ids = np.flatnonzero(visual_sample_distances > SILHOUETTE_ERROR_M)
    covered = inside_proxy(candidate_paths, sample_visual[sample_outlier_ids])
    outside_ids = sample_outlier_ids[~covered]
    surface = dict(
        cad_to_proxy=distribution(visual_sample_distances),
        proxy_facets_to_cad=distribution(closest_distances(visual, sample_proxy)),
        outlier_threshold_m=SILHOUETTE_ERROR_M,
        cad_outliers_inside_proxy=int(covered.sum()),
        cad_outliers_outside_proxy=distribution(visual_sample_distances[outside_ids]),
        outside_outlier_points_m=sample_visual[outside_ids].tolist(),
        interpretation="Nearest sampled triangle distance. Proxy samples include internal facets.",
    )
    broad, near_patches = directions_for_support()
    support_error = support(body.vertices, broad) - support(detailed_body.vertices, broad)
    critical_error = (support(body.vertices, near_patches)
                      - support(detailed_body.vertices, near_patches))
    support_summary = dict(
        abs_error=distribution(np.abs(support_error)),
        body_axis_errors_m=support_error[:6].tolist(),
        worst_direction=broad[np.argmax(np.abs(support_error))].tolist(),
        critical_abs_error=distribution(np.abs(critical_error)),
        critical_worst_direction=near_patches[np.argmax(np.abs(critical_error))].tolist(),
        min_body_z_m=float(body.bounds[0, 2]),
        carrier_cut_z_m=original["general_carrier_cut_z_m"],
        reference="frozen detailed body colliders",
    )

    def worst(rows, key):
        return max(rows[key], default=0)

    mesh_sizes = [(len(v), len(f)) for v, f in (obj_mesh(path) for path in candidate_paths)]
    body_mesh_sizes = [size for size, entry in zip(mesh_sizes, manifest["entries"])
                       if entry["group"] == "body"]
    gates = dict(
        gpu_body_convex_face_budget=all(v <= 64 and f <= 64 for v, f in body_mesh_sizes),
        guard_false_positive=max(
            worst(guard_proxy, "false_positive_distance_to_cad_m"),
            worst(corridor, "false_positive_distance_to_cad_m"),
        ) <= GUARD_MATERIAL_ERROR_M,
        guard_miss=max(worst(guard_proxy, "missed_cad_distance_to_proxy_m"),
                       worst(corridor, "missed_cad_distance_to_proxy_m")) <= GUARD_MATERIAL_ERROR_M,
        body_support=np.max(np.abs(support_error)) <= BODY_SUPPORT_ERROR_M,
        patch_relative_body_support=(np.max(np.abs(critical_error)) <= CONTACT_SUPPORT_ERROR_M
                                     and body.bounds[0, 2] >= original["general_carrier_cut_z_m"] - 1e-9),
        silhouette_error=max(worst(silhouette, "false_positive_distance_to_cad_m"),
                             worst(silhouette, "missed_cad_distance_to_proxy_m")) <= SILHOUETTE_ERROR_M,
    )
    gates = {name: bool(value) for name, value in gates.items()}
    report = dict(
        manifest_sha256=digest(manifest_path), exact_patch_count=15,
        source_colliders=len(original["entries"]), proxy_colliders=len(manifest["entries"]),
        guard_detailed=guard_detailed, guard_proxy=guard_proxy, guard_corridors=corridor,
        silhouette=silhouette, silhouette_detailed=silhouette_detailed,
        surface_samples=surface, support=support_summary,
        max_hull_vertices=max(v for v, _ in mesh_sizes), max_hull_faces=max(f for _, f in mesh_sizes),
        max_body_hull_vertices=max(v for v, _ in body_mesh_sizes),
        max_body_hull_faces=max(f for _, f in body_mesh_sizes),
        tolerances_m=dict(guard_material=GUARD_MATERIAL_ERROR_M,
                          body_support=BODY_SUPPORT_ERROR_M, critical_body_support=CONTACT_SUPPORT_ERROR_M,
                          silhouette_material=SILHOUETTE_ERROR_M),
        gates=gates, engineering_geometry_pass=all(gates.values()),
    )
    write_result(output, report)
    print("ASM2 geometry gates", gates, flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=PROXY)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(0 if measure(args.manifest, args.output)["engineering_geometry_pass"] else 2)


if __name__ == "__main__":
    main()
