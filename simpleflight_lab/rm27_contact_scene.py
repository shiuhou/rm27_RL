"""Author and reference the local USD collision scene for RM27 Stage 1."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

from .rm27_contact_geometry import PHYSX_CONTACT_OFFSET_M


SCENE_ROOT = "/Stage1Scene"
# PhysX defaults for convex hull cooking. Use the original hull triangles for
# static shapes outside these limits, where convex cooking could change the surface.
CONVEX_VERTEX_LIMIT = 64
CONVEX_THICKNESS_M = 0.001


def _pose(geometry, position, quat):
    geometry.AddTranslateOp().Set(Gf.Vec3d(*position))
    geometry.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(quat[0], Gf.Vec3d(*quat[1:])))


def _physx_attribute(prim, schema, name, type_name, value):
    # The authoring host has OpenUSD. Kit registers these PhysX schemas when it loads the file.
    prim.AddAppliedSchema(schema)
    prim.CreateAttribute(name, type_name, custom=False).Set(value)


def _physics_material(stage, root, materials, sliding_friction):
    if sliding_friction not in materials:
        material = UsdShade.Material.Define(stage, root.AppendChild(f"Friction{len(materials)}"))
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(sliding_friction)
        physics.CreateDynamicFrictionAttr(sliding_friction)
        physics.CreateRestitutionAttr(0.0)
        _physx_attribute(material.GetPrim(), "PhysxMaterialAPI", "physxMaterial:frictionCombineMode",
                         Sdf.ValueTypeNames.Token, "max")
        materials[sliding_friction] = material
    return materials[sliding_friction]


def _collider(stage, path, shape, meshes):
    size = shape["size"]
    kind = shape["type"]
    if kind == "mesh":
        geom = UsdGeom.Mesh.Define(stage, path)
        points = np.ascontiguousarray(meshes[f"{shape['mesh']}_vertices"], dtype=np.float32)
        faces = np.ascontiguousarray(meshes[f"{shape['mesh']}_faces"], dtype=np.int32)
        geom.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
        geom.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
        geom.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
        geom.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0).tolist()), Gf.Vec3f(*points.max(axis=0).tolist())])
        geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        geom.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
        prim = geom.GetPrim()
        triangle_mesh = len(points) > CONVEX_VERTEX_LIMIT or np.ptp(points, axis=0).min() < CONVEX_THICKNESS_M
        approximation = UsdPhysics.Tokens.none if triangle_mesh else UsdPhysics.Tokens.convexHull
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
        if triangle_mesh:
            _physx_attribute(prim, "PhysxTriangleMeshCollisionAPI", "physxTriangleMeshCollision:weldTolerance",
                             Sdf.ValueTypeNames.Float, 0.0)
        else:
            _physx_attribute(prim, "PhysxConvexHullCollisionAPI", "physxConvexHullCollision:minThickness",
                             Sdf.ValueTypeNames.Float, 0.0)
    elif kind == "sphere":
        geom = UsdGeom.Sphere.Define(stage, path)
        geom.CreateRadiusAttr(size[0])
    elif kind in ("capsule", "cylinder"):
        geom_type = UsdGeom.Capsule if kind == "capsule" else UsdGeom.Cylinder
        geom = geom_type.Define(stage, path)
        geom.CreateAxisAttr(UsdGeom.Tokens.z)
        geom.CreateRadiusAttr(size[0])
        geom.CreateHeightAttr(2.0 * size[1])
    elif kind == "box":
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(2.0)
    elif kind == "plane":
        geom = UsdGeom.Plane.Define(stage, path)
        geom.CreateAxisAttr(UsdGeom.Tokens.z)
    else:
        raise ValueError(f"Unsupported collision geometry {kind!r} for {shape['name']}")

    _pose(geom, shape["position_m"], shape["quaternion_wxyz"])
    if kind == "box":
        geom.AddScaleOp().Set(Gf.Vec3f(*size))
    prim = geom.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    _physx_attribute(prim, "PhysxCollisionAPI", "physxCollision:contactOffset", Sdf.ValueTypeNames.Float,
                     PHYSX_CONTACT_OFFSET_M)
    _physx_attribute(prim, "PhysxCollisionAPI", "physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.0)
    prim.CreateAttribute("stage1:geomId", Sdf.ValueTypeNames.Int).Set(shape["id"])
    prim.CreateAttribute("stage1:geomName", Sdf.ValueTypeNames.String).Set(shape["name"])
    prim.CreateAttribute("stage1:registered", Sdf.ValueTypeNames.Bool).Set(shape["legal"])
    return prim


def _add_colliders(stage, root, shapes, meshes):
    root_path = root.GetPath()
    materials_path = root_path.AppendChild("Materials")
    UsdGeom.Scope.Define(stage, materials_path)
    materials = {}
    for shape in shapes:
        if (shape["contype"], shape["conaffinity"]) != (1, 1):
            raise ValueError(f"Unsupported contact mask on {shape['name']}")
        name = Tf.MakeValidIdentifier(shape["name"]) if shape["name"] else f"Geom_{shape['id']}"
        path = root_path.AppendChild(name)
        if stage.GetPrimAtPath(path):
            raise ValueError(f"Duplicate collider path {path}")
        collider = _collider(stage, path, shape, meshes)
        material = _physics_material(stage, materials_path, materials, shape["friction"][0])
        UsdShade.MaterialBindingAPI.Apply(collider).Bind(material, materialPurpose="physics")


def author_scene(geometry_path: Path, meshes_path: Path, output: Path) -> None:
    """Write a Z-up, meter-scale USD. The vehicle root is a body-frame reference at identity."""
    if output.exists():
        raise FileExistsError(output)
    geometry = json.loads(geometry_path.read_text())
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, SCENE_ROOT).GetPrim()
    stage.SetDefaultPrim(root)
    root.SetCustomDataByKey("stage1:modelSha256", geometry["model_sha256"])
    root.SetCustomDataByKey("stage1:registrySha256", geometry["registry_sha256"])
    field = UsdGeom.Xform.Define(stage, f"{SCENE_ROOT}/Field").GetPrim()
    vehicle = UsdGeom.Xform.Define(stage, f"{SCENE_ROOT}/Drone")
    _pose(vehicle, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    drone = vehicle.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(drone)
    body = geometry["body"]
    mass = UsdPhysics.MassAPI.Apply(drone)
    mass.CreateMassAttr(body["mass_kg"])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*body["com_m"]))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*body["diagonal_inertia_kg_m2"]))
    axes = body["inertial_quaternion_wxyz"]
    mass.CreatePrincipalAxesAttr(Gf.Quatf(axes[0], Gf.Vec3f(*axes[1:])))
    _physx_attribute(drone, "PhysxRigidBodyAPI", "physxRigidBody:enableGyroscopicForces", Sdf.ValueTypeNames.Bool, True)
    _physx_attribute(drone, "PhysxRigidBodyAPI", "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 0.0)
    _physx_attribute(drone, "PhysxRigidBodyAPI", "physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float, 0.0)
    _physx_attribute(drone, "PhysxRigidBodyAPI", "physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float, 0.0)
    _physx_attribute(drone, "PhysxContactReportAPI", "physxContactReport:threshold", Sdf.ValueTypeNames.Float, 0.0)
    with np.load(meshes_path, allow_pickle=False) as meshes:
        _add_colliders(stage, field, geometry["static"], meshes)
        _add_colliders(stage, drone, geometry["aircraft"], meshes)
    if not stage.GetRootLayer().Export(str(output.resolve())):
        raise OSError(f"USD export failed: {output}")


def reference_scene(stage, scene_path: Path, parent_path: str):
    """Compose the fixed field and active vehicle. Set the vehicle pose before starting simulation."""
    scene_path = scene_path.resolve(strict=True)
    parent = Sdf.Path(parent_path)
    prims = []
    for name in ("Field", "Drone"):
        path = parent.AppendChild(name)
        if stage.GetPrimAtPath(path):
            raise ValueError(f"Scene prim already exists at {path}")
        prim = stage.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(str(scene_path), f"{SCENE_ROOT}/{name}")
        prims.append(prim)
    return tuple(prims)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", required=True, type=Path)
    parser.add_argument("--meshes", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    author_scene(args.geometry, args.meshes, args.output)


if __name__ == "__main__":
    main()
