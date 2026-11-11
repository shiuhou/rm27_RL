"""Extract the existing planar underside contact patches, in body metres."""
from pathlib import Path
import json
import numpy as np
import trimesh
from shapely.geometry import Polygon

ROOT = (Path(__file__).resolve().parents[1] / 'mujoco')
CONTACT_THICKNESS = 0.00005

def build_contact_geometry():
    mesh = trimesh.load(ROOT / 'meshes/0_rk3576_body.obj')
    bottom = float(mesh.vertices[:, 2].min())
    selected = np.all(np.abs(mesh.triangles[:, :, 2] - bottom) < 2e-7, axis=1)
    entries = []
    for group, is_front in [('contact_front', True), ('contact_rear', False)]:
        triangles = mesh.triangles[selected]
        triangles = triangles[(triangles[:, :, 0].mean(axis=1) > 0) == is_front]
        polygons = [Polygon(t[:, :2]) for t in triangles if Polygon(t[:, :2]).area > 1e-14]
        # Merge only adjacent triangles whose union remains convex; retain holes and bends.
        changed = True
        while changed:
            changed = False
            for i in range(len(polygons)):
                for j in range(i + 1, len(polygons)):
                    if polygons[i].intersection(polygons[j]).length < 1e-8:
                        continue
                    union = polygons[i].union(polygons[j])
                    if union.geom_type == 'Polygon' and union.convex_hull.area - union.area < 1e-13:
                        polygons[i] = union.convex_hull
                        polygons.pop(j)
                        changed = True
                        break
                if changed:
                    break
        for index, polygon in enumerate(polygons):
            xy = np.array(polygon.exterior.coords[:-1])
            lower = np.column_stack([xy, np.full(len(xy), bottom)])
            upper = lower + [0, 0, CONTACT_THICKNESS]
            hull = trimesh.convex.convex_hull(np.vstack([lower, upper]))
            name = f'{group}_{index:03d}'
            path = ROOT / 'collision_meshes' / f'{name}.obj'
            hull.export(path)
            entries.append(dict(name=name, group=group, source_part=0,
                                mesh=str(path.relative_to(ROOT)), footprint_area_m2=polygon.area))
    return entries, bottom + CONTACT_THICKNESS

if __name__ == '__main__':
    entries, cut = build_contact_geometry()
    (ROOT / '../metadata/contact_geometry.json').write_text(json.dumps(dict(entries=entries,cut_z_m=cut), indent=2))
    print(len(entries), cut, sum(e['footprint_area_m2'] for e in entries))
