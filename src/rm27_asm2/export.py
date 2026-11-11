"""Export the validated ASM2 cooking recipe as a standalone collision candidate.

The source USD is referenced unchanged. Only the already measured diagnostic
recipe's stronger opinions are copied; this does not qualify a new flight task.
Run with PYTHONPATH=src python -m rm27_asm2.export --source-recipe PATH --output PATH.
"""
from __future__ import annotations

import argparse
from collections import Counter
import os
from pathlib import Path
import tempfile

import numpy as np

from .asset import ROOT, RUN, digest, load_inputs, write_result


RECIPE_DRONE = "/World/ASM2Envs/env_0/Drone"
RECIPE_MATERIAL = "/World/ASM2Envs/env_0/PhysicsMaterial"


def validate_asset(path, raw_path, config):
    """Check composed geometry, semantics, rigid-body properties and recipe."""
    from pxr import Usd, UsdGeom, UsdPhysics

    stage, raw = Usd.Stage.Open(str(path)), Usd.Stage.Open(str(raw_path))
    if not stage or not raw or stage.GetCompositionErrors():
        raise ValueError("Exported asset has unresolved USD composition")
    if stage.GetDefaultPrim().GetPath() != "/Drone":
        raise ValueError("Default prim must be /Drone")
    if (UsdGeom.GetStageMetersPerUnit(stage) != 1.0
            or UsdGeom.GetStageUpAxis(stage) != "Z"
            or UsdPhysics.GetStageKilogramsPerUnit(stage) != 1.0):
        raise ValueError("Candidate must use metres, kilograms and Z-up")
    root = stage.GetPrimAtPath("/Drone")
    cache, raw_cache = UsdGeom.XformCache(), UsdGeom.XformCache()
    if np.max(np.abs(np.asarray(cache.GetLocalToWorldTransform(root)) - np.eye(4))) > 1e-12:
        raise ValueError("Candidate root transform changed")
    for name in ("physics:mass", "physics:centerOfMass", "physics:diagonalInertia", "physics:principalAxes"):
        if root.GetAttribute(name).Get() != raw.GetPrimAtPath("/Drone").GetAttribute(name).Get():
            raise ValueError(f"Source mass properties changed: {name}")
    mass = float(UsdPhysics.MassAPI(root).GetMassAttr().Get())
    if abs(mass - config["expected_mass_kg"]) > 1e-8:
        raise ValueError("Candidate mass differs from supplied 142 g CAD estimate")
    raw_colliders = {str(p.GetPath()): p for p in raw.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}
    groups, count, worst = Counter(), 0, 0.0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        name = str(prim.GetPath())
        source_prim = raw_colliders.pop(name)
        for key in ("contact_group", "source_part_index"):
            if prim.GetCustomDataByKey(key) != source_prim.GetCustomDataByKey(key):
                raise ValueError(f"Source collider identity changed: {name}: {key}")
        groups[prim.GetCustomDataByKey("contact_group")] += 1
        mesh, source_mesh = UsdGeom.Mesh(prim), UsdGeom.Mesh(source_prim)
        if (list(mesh.GetFaceVertexCountsAttr().Get()) != list(source_mesh.GetFaceVertexCountsAttr().Get())
                or list(mesh.GetFaceVertexIndicesAttr().Get()) != list(source_mesh.GetFaceVertexIndicesAttr().Get())):
            raise ValueError(f"Collider topology changed: {name}")
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
        original = np.asarray(source_mesh.GetPointsAttr().Get(), dtype=np.float64)
        if points.shape != original.shape:
            raise ValueError(f"Collider vertex count changed: {name}")
        # USD matrices use row vectors. Include all composed source transforms.
        transformed = np.c_[points, np.ones(len(points))] @ np.asarray(cache.GetLocalToWorldTransform(prim))
        expected = np.c_[original, np.ones(len(original))] @ np.asarray(raw_cache.GetLocalToWorldTransform(source_prim))
        error = float(np.max(np.abs(transformed[:, :3] - expected[:, :3])))
        if error > 1e-8:
            raise ValueError(f"Collider coordinates differ by {error} m: {name}")
        worst = max(worst, error)
        if UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != "convexHull":
            raise ValueError(f"Collision approximation changed: {name}")
        expected_attributes = {
            "physxCollision:contactOffset": config["diagnostic_contact_offset_m"],
            "physxCollision:restOffset": config["rest_offset_m"],
            "physxConvexHullCollision:minThickness": config["convex_min_thickness_m"],
            "physxConvexHullCollision:hullVertexLimit": config["convex_hull_vertex_limit"],
        }
        for key, value in expected_attributes.items():
            observed = prim.GetAttribute(key).Get()
            if observed is None or not np.isclose(observed, value, rtol=1e-7, atol=1e-14):
                raise ValueError(f"Invalid candidate cooking setting: {name}: {key}={observed}")
        expected_scale = np.full(3, 1.0 / config["cook_coordinate_scale"])
        if not np.allclose(prim.GetAttribute("xformOp:scale").Get(), expected_scale, rtol=1e-7, atol=0):
            raise ValueError(f"Normalized cooking scale missing: {name}")
        material = prim.GetRelationship("material:binding:physics").GetTargets()
        if material != [root.GetPath().AppendChild("PhysicsMaterial")]:
            raise ValueError(f"Physics material binding does not resolve locally: {name}")
        count += 1
    if raw_colliders or groups != config["groups"]:
        raise ValueError("Source colliders or semantic groups missing")
    return dict(collider_count=count, collider_groups=dict(groups),
                collider_world_coordinate_max_error_m=worst, root_transform_identity=True,
                mass_kg=mass, mass_properties_identical_to_source=True,
                topology_identical_to_source=True, material_bindings_local=True,
                source_layers_resolved=sorted(l.realPath for l in stage.GetUsedLayers() if l.realPath))


def export_candidate(source_recipe, output, evidence):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

    source_recipe, output, evidence = Path(source_recipe).resolve(), Path(output).resolve(), Path(evidence).resolve()
    if output.exists() or evidence.exists():
        raise FileExistsError("Candidate asset and evidence are immutable; choose absent output paths")
    config, source, _, _ = load_inputs()
    before = digest(source_recipe)
    recipe = Sdf.Layer.FindOrOpen(str(source_recipe))
    if not recipe or not recipe.GetPrimAtPath(RECIPE_DRONE) or not recipe.GetPrimAtPath(RECIPE_MATERIAL):
        raise ValueError("Expected verified one-world Drone and PhysicsMaterial recipe")
    candidate = Sdf.Layer.CreateAnonymous("asm2-candidate.usda")
    if not Sdf.CopySpec(recipe, RECIPE_DRONE, candidate, "/Drone"):
        raise ValueError("Failed to copy the candidate aircraft recipe")
    if not Sdf.CopySpec(recipe, RECIPE_MATERIAL, candidate, "/Drone/PhysicsMaterial"):
        raise ValueError("Failed to copy the candidate physics material")
    resolved = {}

    def rebase(asset_path):
        if not asset_path:
            return asset_path
        path = Path(Sdf.ComputeAssetPathRelativeToLayer(recipe, asset_path)).resolve()
        if not path.is_file():
            raise ValueError(f"Referenced source asset is missing: {asset_path}")
        relative = os.path.relpath(path, output.parent)
        resolved[asset_path] = dict(source=str(path), candidate_reference=relative, sha256=digest(path))
        return relative

    UsdUtils.ModifyAssetPaths(candidate, rebase)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        descriptor, name = tempfile.mkstemp(prefix=".asm2-export-", suffix=".usda", dir=output.parent)
        os.close(descriptor)
        temporary = Path(name)
        if not candidate.Export(str(temporary)):
            raise ValueError("USD candidate export failed")
        # Compose only after writing in the destination directory: relative
        # references cannot use an anonymous layer as their filesystem anchor.
        stage = Usd.Stage.Open(str(temporary))
        drone = stage.GetPrimAtPath("/Drone")
        stage.SetDefaultPrim(drone)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
        # The original material was a sibling outside the copied subtree.
        for prim in stage.Traverse():
            binding = prim.GetRelationship("material:binding:physics")
            if binding and binding.GetTargets():
                binding.SetTargets([Sdf.Path("/Drone/PhysicsMaterial")])
        stage.SetMetadata("customLayerData", dict(
            scope="ASM2_COLLISION_ENGINEERING_ONLY", training_allowed=False,
            source_recipe=str(source_recipe.relative_to(ROOT)), source_recipe_sha256=before,
            cook_coordinate_scale=config["cook_coordinate_scale"], geometry_reduction=False))
        stage.GetRootLayer().Save()
        checks = validate_asset(temporary, source / "usd/drone.usdc", config)
        if digest(source_recipe) != before:
            raise ValueError("Source recipe changed during export")
        # A hard link publishes the fully checked file without overwriting any
        # historical target, even if another process creates it concurrently.
        os.link(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    checks["source_layers_resolved"] = [str(output) if p == str(temporary) else p for p in checks["source_layers_resolved"]]
    result = dict(status="PASS_COMPOSED_CANDIDATE_EXPORT; PHYSX_NOT_RERUN", training_transitions=0,
                  output=str(output.relative_to(ROOT)), output_sha256=digest(output),
                  source_recipe=str(source_recipe.relative_to(ROOT)), source_recipe_sha256=before,
                  export_implementation="src/rm27_asm2/export.py", export_implementation_sha256=digest(__file__),
                  referenced_assets=resolved, validation=checks,
                  limitation="Preserves verified fixture recipe; not new-airframe task qualification or training acceptance")
    write_result(evidence, result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-recipe", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, default=RUN / "candidate-export.json")
    args = parser.parse_args()
    result = export_candidate(args.source_recipe, args.output, args.evidence)
    print(f"{result['status']}: {result['output']} ({result['output_sha256']})")


if __name__ == "__main__":
    main()
