"""Recorded outbound/return swept-sphere audit against retained visual surfaces."""
import json
from pathlib import Path

import mujoco
import numpy as np

from rm27.experiments.corridor_audit import mesh_surface_hits
from .evidence import write_json
from .recording import load_model


def audit_routes(source, output, spacing=.05, approach=False):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    model = load_model(source)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    archive = np.load(source/"trajectory.npz")
    trace = archive["trace"]
    phases = archive['phases'] if 'phases' in archive else np.array(['OUTBOUND']*len(trace))
    if approach:
        result = json.loads((source/'summary.json').read_text())
        first = result.get('first_contact')
        stop = float(first['time_s']-.5) if first else trace[-1, 0]
        phases = np.where(trace[:, 0] < stop, 'OUTBOUND', 'EXCLUDED_TERMINAL')
    uid = model.body("uav").id
    bounds = []
    for g in np.flatnonzero(model.geom_bodyid == uid):
        offset = data.geom_xpos[g]-data.xpos[uid]
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
            axis, half = data.geom_xmat[g].reshape(3, 3)[:, 2], model.geom_size[g, 1]
            bounds.append(max(np.linalg.norm(offset-axis*half), np.linalg.norm(offset+axis*half))+model.geom_size[g, 0])
        else:
            bounds.append(np.linalg.norm(offset)+model.geom_rbound[g])
    radius = float(max(bounds))
    groups, max_jump = {}, 0.
    for phase in ("OUTBOUND", "RETURN"):
        path = trace[phases == phase, 1:4]
        if not len(path):
            continue
        selected = [path[0]]
        for p in path[1:]:
            jump = np.linalg.norm(p-selected[-1])
            if jump >= spacing:
                max_jump = max(max_jump, float(jump))
                selected.append(p)
        selected.append(path[-1])
        groups[phase] = np.array(selected)
    # Add full sample spacing, conservatively covering the skipped recorded path.
    inflated = radius+max_jump
    hits = []
    checked = 0
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] == uid or model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        for phase, points in groups.items():
            candidates = np.flatnonzero(np.linalg.norm(points-data.geom_xpos[geom], axis=1) < model.geom_rbound[geom]+inflated)
            if not len(candidates):
                continue
            found = mesh_surface_hits(model, data, geom, points[candidates], inflated)
            hits.extend(dict(phase=phase, geom=model.geom(geom).name,
                             point_world_m=points[candidates[h['sample']]].tolist(), **h) for h in found)
        checked += 1
    result = dict(passed=bool(groups and not hits), source=str(source), mesh_geoms_checked=checked,
                  vehicle_radius_m=radius, sample_inflation_m=max_jump, sphere_radius_m=inflated,
                  samples={k:len(v) for k, v in groups.items()}, hits=hits,
                  scope=("recorded approach visual triangle surface clearance; excludes final 0.5 s before contact and recovery; not solid-volume or real-world certification"
                         if approach else "recorded outbound/return visual triangle surface clearance; excludes intended contact terminal segment; not solid-volume or real-world certification"))
    write_json(output/"summary.json", result)
    return result
