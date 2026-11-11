from pathlib import Path
import json,time,xml.etree.ElementTree as ET
import numpy as np
import mujoco
from pxr import Usd,UsdPhysics
from contact_records import collect_contacts
ROOT=(Path(__file__).resolve().parents[1] / 'mujoco')
start=time.perf_counter()
model=mujoco.MjModel.from_xml_path(str(ROOT/'model.xml'))
data=mujoco.MjData(model);mujoco.mj_forward(model,data)
visual=np.array([0,0,1,0,0,0],np.uint8)
collision=np.array([0,0,0,1,0,0],np.uint8)
gid=np.array([-1],np.int32)
probes=[]
for cx in [-.0275,.0275]:
 for cy in [-.041,.041]:
  for radius in [.012,.017,.022]:
   for angle in np.linspace(0,2*np.pi,24,endpoint=False):
    xy=np.array([cx,cy])+radius*np.array([np.cos(angle),np.sin(angle)])
    origin=np.r_[xy,-.06]
    v=mujoco.mj_ray(model,data,origin,np.array([0.,0.,1.]),visual,1,-1,gid)
    c=mujoco.mj_ray(model,data,origin,np.array([0.,0.,1.]),collision,1,-1,gid)
    probes.append(dict(xy_m=xy.tolist(),visual_hit=float(v),collision_hit=float(c)))
free=[r for r in probes if r['visual_hit']<0]
false=[r for r in free if r['collision_hit']>=0]
if false:
    import trimesh
    parts = json.loads((ROOT/'../metadata/mass_properties.json').read_text())['parts']
    visual_mesh = trimesh.util.concatenate([trimesh.load(ROOT / part['mesh']) for part in parts])
    for probe in false:
        point = np.r_[probe['xy_m'], probe['collision_hit'] - .06]
        _, distance, _ = trimesh.proximity.closest_point(visual_mesh, [point])
        probe['nearest_visual_surface_distance_m'] = float(distance[0])
# Near-floor contact must belong to the extracted underside patches only.
root=ET.parse(ROOT/'model.xml').getroot()
for asset in root.findall('asset/mesh'):asset.set('file',str(ROOT/asset.get('file')))
z=-.01986282
ET.SubElement(root.find('worldbody'),'geom',name='test_floor',type='plane',size='1 1 .01',pos=f'0 0 {z+.00001}')
m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
records=collect_contacts(m,d)
groups=sorted(set(r['group'] for r in records))
assert groups==['contact_front','contact_rear'],groups
# A downward velocity, solved for one step, should yield upward support impulse.
d.qvel[2]=-.1;mujoco.mj_step(m,d)
records=collect_contacts(m,d)
assert all(r['normal_other_to_drone_world'][2]>.99 for r in records)
assert sum(r['impulse_on_drone_world_Ns'][2] for r in records)>0
start_step=time.perf_counter()
for _ in range(100):mujoco.mj_step(m,d)
step_time=time.perf_counter()-start_step
assert np.isfinite(d.qpos).all()
assert abs(model.body_mass.sum()-.142)<1e-9
stage=Usd.Stage.Open(str(ROOT/'../usd/drone.usdc'))
colliders=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
manifest=json.loads((ROOT/'../metadata/collision_geometry.json').read_text())
assert len(colliders)==len(manifest['entries'])
assert set(e['source_part'] for e in manifest['entries']) == set(range(14))
side_contacts = []
for axis in [0, 1]:
    for angle in [-np.pi / 2, np.pi / 2]:
        mujoco.mj_resetData(m, d)
        d.qpos[3] = np.cos(angle / 2)
        d.qpos[4 + axis] = np.sin(angle / 2)
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, d.qpos[3:7])
        import trimesh
        low = min((trimesh.load(ROOT / e['mesh']).vertices @ rotation.reshape(3, 3).T)[:, 2].min() for e in manifest['entries'])
        m.geom_pos[m.geom('test_floor').id, 2] = low + .00001
        mujoco.mj_forward(m, d)
        side_groups = sorted(set(r['group'] for r in collect_contacts(m, d)))
        assert 'body' in side_groups, side_groups
        side_contacts.append(dict(axis=axis, angle_rad=float(angle), groups=side_groups))
assert all(p.GetPath().pathString.startswith('/Drone/Collision/') for p in colliders)
report=dict(geometry_count=model.ngeom,collision_count=len(colliders),visual_count=14,
 mass_kg=float(model.body_mass.sum()),guard_probes=len(probes),visual_open_probes=len(free),
 blocked_open_probes=len(false),blocked_points=false,contact_groups=groups,
 side_collision_checks=side_contacts,contact_events_example=records[:4],headless_contact_steps=100,steps_wall_seconds=step_time,
 total_validation_seconds=time.perf_counter()-start,isaac_runtime='NOT_RUN',
 passed=not false)
(ROOT/'../metadata/collision_validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ['blocked_points','contact_events_example']},indent=2))

raise SystemExit(0 if report["passed"] else 1)
