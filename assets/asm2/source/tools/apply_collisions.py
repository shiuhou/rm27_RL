"""Apply the same decomposed geometry and contact identities to MJCF and USD."""
from pathlib import Path
import json
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Vt
from contact_geometry import build_contact_geometry

ROOT = (Path(__file__).resolve().parents[1] / 'mujoco')

def clip_convex_above(mesh, z):
    vertices = mesh.vertices
    edges = vertices[mesh.edges_unique]
    crosses = (edges[:, 0, 2] < z) != (edges[:, 1, 2] < z)
    crossing = edges[crosses]
    if len(crossing):
        t = (z - crossing[:, 0, 2]) / (crossing[:, 1, 2] - crossing[:, 0, 2])
        intersections = crossing[:, 0] + t[:, None] * (crossing[:, 1] - crossing[:, 0])
    else:
        intersections = np.empty((0, 3))
    points = np.vstack([vertices[vertices[:, 2] >= z], intersections])
    if len(points) < 4 or np.ptp(points[:, 2]) < 1e-8:
        return None
    return trimesh.convex.convex_hull(points)

def collision_sources_match():
    manifest = ROOT / '../metadata/collision_manifest.json'
    if not manifest.exists():
        return False
    parts = json.loads((ROOT / '../metadata/mass_properties.json').read_text())['parts']
    digests = [hashlib.sha256((ROOT / part['mesh']).read_bytes()).hexdigest() for part in parts]
    entries = json.loads(manifest.read_text())
    return bool(entries) and all(entry.get('source_mesh_sha256') == digests[entry['source_part']] for entry in entries)

def apply():
    if not collision_sources_match():
        raise ValueError('Collision sources changed: run decompose.py before apply_collisions.py')
    contacts, cut = build_contact_geometry()
    entries = json.loads((ROOT / '../metadata/collision_manifest.json').read_text())
    retained = []
    for entry in entries:
        if entry['source_part'] == 0:
            mesh = trimesh.load(ROOT / entry['mesh'])
            mesh = clip_convex_above(mesh, cut)
            if mesh is None:
                continue
            path = Path(entry['mesh']).with_stem(Path(entry['mesh']).stem + '_trimmed')
            mesh.export(ROOT / path)
            entry['mesh'] = str(path)
        retained.append(entry)
    entries = retained + contacts
    tree = ET.parse(ROOT / 'model.xml')
    root = tree.getroot()
    body = root.find('worldbody/body')
    assets = root.find('asset')
    for geom in list(body.findall('geom')):
        if geom.get('name', '').startswith(('collision_', 'contact_')):
            body.remove(geom)
        else:
            geom.set('contype', '0'); geom.set('conaffinity', '0'); geom.set('group', '2')
    for asset in list(assets):
        if asset.get('name', '').startswith(('collision_', 'contact_')):
            assets.remove(asset)
    stage = Usd.Stage.Open(str(ROOT / '../usd/drone.usda'))
    stage.RemovePrim('/Drone/Collision')
    UsdGeom.Scope.Define(stage, '/Drone/Collision')
    material = UsdShade.Material.Define(stage, '/Drone/PhysicsMaterial')
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(.6)
    physics_material.CreateDynamicFrictionAttr(.6)
    physics_material.CreateRestitutionAttr(0)
    for prim in stage.Traverse():
        if prim.GetPath().pathString.startswith('/Drone/part_'):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
            prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
            prim.RemoveProperty('physics:collisionEnabled')
            prim.RemoveProperty('physics:approximation')
    for entry in entries:
        name = entry['name']
        is_contact = entry['group'].startswith('contact_')
        ET.SubElement(assets, 'mesh', name=name, file=entry['mesh'])
        ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name, group='3',
                      rgba='0 .7 1 .6' if is_contact else '.9 .5 .1 .25',
                      friction='0.6 0.005 0.0001', margin='0', gap='0')
        mesh = trimesh.load(ROOT / entry['mesh'])
        usd_mesh = UsdGeom.Mesh.Define(stage, f'/Drone/Collision/{name}')
        usd_mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(mesh.vertices.astype(np.float32)))
        usd_mesh.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
        usd_mesh.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
        usd_mesh.CreateSubdivisionSchemeAttr('none')
        usd_mesh.CreateVisibilityAttr('invisible')
        prim = usd_mesh.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
        prim.AddAppliedSchema('PhysxCollisionAPI')
        prim.CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.00001 if entry['source_part'] == 0 else .0001)
        prim.CreateAttribute('physxCollision:restOffset', Sdf.ValueTypeNames.Float).Set(0)
        prim.SetCustomDataByKey('contact_group', entry['group'])
        prim.SetCustomDataByKey('source_part_index', entry['source_part'])
    drone = stage.GetPrimAtPath('/Drone')
    drone.AddAppliedSchema('PhysxContactReportAPI')
    drone.CreateAttribute('physxContactReport:threshold', Sdf.ValueTypeNames.Float).Set(0)
    ET.indent(tree)
    tree.write(ROOT / 'model.xml')
    stage.GetRootLayer().Save()
    stage.Export(str(ROOT / '../usd/drone.usdc'))
    (ROOT / '../metadata/collision_geometry.json').write_text(json.dumps(dict(
        entries=entries, contact_skin_thickness_m=.00005,
        general_carrier_cut_z_m=cut,
        notes=['Contact bottom footprint extracted from original CAD tessellation.',
               'Lowest 0.05 mm carrier layer replaced by exact flat contact patches; bottom edge fillets in this layer omitted.',
               'CoACD approximations require spatial validation; friction is an engineering default.']), indent=2))
    print('Applied',len(entries),'colliders including',len(contacts),'contact patches')

if __name__ == '__main__':
    apply()
