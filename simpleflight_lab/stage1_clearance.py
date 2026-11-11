"""Native collision-shape clearance queries for the full-field Stage 1 plant."""

import math

import mujoco
import numpy as np


def geom_clearance(plant, *, max_distance_m: float = 0.05, forbidden_only: bool = True) -> dict:
    """Return the nearest eligible pair below the cap, or the cap as a lower bound.

    Use after mj_forward, at an integration endpoint. Forbidden clearance
    excludes only the designated bumper versus a registered surface. Contact
    qualification still belongs to the native completed-interval scorer.
    """
    if not math.isfinite(max_distance_m) or max_distance_m <= 0:
        raise ValueError("Clearance query cap must be positive and finite")
    m, d = plant.model, plant.data
    geom_ids = np.arange(m.ngeom)
    collidable = (m.geom_contype != 0) | (m.geom_conaffinity != 0)
    aircraft = np.zeros(m.ngeom, dtype=bool)
    aircraft[list(plant.active_geom_ids)] = True
    own = geom_ids[collidable & aircraft]
    other = geom_ids[collidable & ~aircraft]
    plane = int(mujoco.mjtGeom.mjGEOM_PLANE)
    supported = {int(kind) for kind in (
        mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CAPSULE,
        mujoco.mjtGeom.mjGEOM_ELLIPSOID, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_BOX,
        mujoco.mjtGeom.mjGEOM_MESH,
    )}
    if not set(np.unique(m.geom_type[collidable])) <= supported or np.any(m.geom_type[own] == plane):
        raise ValueError("Stage 1 clearance requires finite convex shapes and fixed planes")
    planes = other[m.geom_type[other] == plane]
    field = other[m.geom_type[other] != plane]
    legal = {m.geom(name).id for name in plant.target_mapping["registered_contact_geoms"]} if forbidden_only else set()

    # Each world AABB conservatively contains the native collision geom.
    boxes = m.geom_aabb.reshape(-1, 2, 3)
    rotation = d.geom_xmat.reshape(-1, 3, 3)
    centers = d.geom_xpos + np.einsum("ijk,ik->ij", rotation, boxes[:, 0])
    half = np.einsum("ijk,ik->ij", np.abs(rotation), boxes[:, 1])
    own_centers, own_half = centers[own], half[own]
    body_low, body_high = (own_centers - own_half).min(axis=0), (own_centers + own_half).max(axis=0)
    outer_delta = np.maximum(
        np.maximum(body_low - centers[field] - half[field], centers[field] - half[field] - body_high), 0
    )
    outer_bound = np.linalg.norm(outer_delta, axis=1)

    best, pair = max_distance_m, None
    queried = 0
    best_segment = np.zeros(6)
    segment = np.zeros(6)
    rounding_m = 1e-8

    def consider(a, b):
        nonlocal best, pair, queried
        if a == plant.bottom_geom_id and b in legal:
            return
        if not (m.geom_contype[a] & m.geom_conaffinity[b] or m.geom_contype[b] & m.geom_conaffinity[a]):
            return
        queried += 1
        distance = mujoco.mj_geomDistance(m, d, int(a), int(b), max(best, rounding_m), segment)
        if not math.isfinite(distance):
            raise ValueError(f"Nonfinite signed geom distance for {a}, {b}")
        if distance < best:
            best, pair = distance, (int(a), int(b))
            best_segment[:] = segment

    for b in planes:
        normal = rotation[b, :, 2]
        lower = (own_centers - d.geom_xpos[b]) @ normal - own_half @ np.abs(normal)
        for a in own[lower <= max(0.0, best) + rounding_m]:
            consider(a, b)
    candidates = np.flatnonzero(outer_bound <= max(0.0, best) + rounding_m)
    for i in candidates[np.argsort(outer_bound[candidates])]:
        if outer_bound[i] > max(0.0, best) + rounding_m:
            break
        b = field[i]
        delta = np.maximum(np.abs(own_centers - centers[b]) - own_half - half[b], 0)
        for a in own[np.linalg.norm(delta, axis=1) <= max(0.0, best) + rounding_m]:
            consider(a, b)

    return {
        "clearance_m": best, "is_lower_bound": pair is None,
        "geom_ids": None if pair is None else list(pair),
        "geoms": None if pair is None else [m.geom(i).name for i in pair],
        "closest_points_m": None if pair is None else best_segment.reshape(2, 3).tolist(),
        "geoms_considered": {"active": len(own), "nonactive": len(other)},
        "native_pair_distance_queries": queried,
    }
