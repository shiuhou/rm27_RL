from pathlib import Path
import json, hashlib
import numpy as np
import trimesh,mujoco
import xml.etree.ElementTree as ET
p=(Path(__file__).resolve().parents[1] / 'mujoco')
records=json.loads((p/'../metadata/parts.json').read_text());meshes=[trimesh.load(p/r['mesh']) for r in records]
# CAD +Y up, +Z toward camera. Right-handed body +X forward, +Y left, +Z up.
R=np.array([[0,0,1],[1,0,0],[0,1,0]])
for r,mesh in zip(records,meshes):
 if not np.isfinite(r['inertia_kg_m2']).all() or min(np.linalg.eigvalsh(r['inertia_kg_m2']))<=0:
  centers=mesh.triangles_center;weights=mesh.area_faces/mesh.area
  c=weights@centers;delta=centers-c
  inertia=sum(w*(np.dot(v,v)*np.eye(3)-np.outer(v,v)) for w,v in zip(weights,delta))*r['mass_kg']
  r.update(center_m=c.tolist(),inertia_kg_m2=inertia.tolist(),estimate='area-weighted triangle surface approximation; invalid STEP volume')
 else:r['estimate']='uniform-density CAD volume'
center=sum(r['mass_kg']*np.array(r['center_m']) for r in records)/.1375
I=np.zeros((3,3))
for r in records:
 delta=np.array(r['center_m'])-center
 I+=np.array(r['inertia_kg_m2'])+r['mass_kg']*(delta@delta*np.eye(3)-np.outer(delta,delta))
 r['center_body_m']=(R@delta).tolist()
I=R@I@R.T
root=ET.Element('mujoco',model='asm2_142g');ET.SubElement(root,'compiler',angle='radian');assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');body=ET.SubElement(world,'body',name='drone');ET.SubElement(body,'freejoint');fmt=lambda a:' '.join(f'{v:.10g}' for v in a)
ET.SubElement(body,'inertial',pos='0 0 0',mass='.142',fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
vals,vec=np.linalg.eigh(I)
if np.linalg.det(vec)<0:vec[:,0]*=-1
q=np.empty(4);mujoco.mju_mat2Quat(q,vec.ravel())
usd=['#usda 1.0','(\ndefaultPrim = "Drone"\nmetersPerUnit = 1\nupAxis = "Z"\n)','def Xform "Drone" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]) {','bool physics:rigidBodyEnabled = true','float physics:mass = 0.142','point3f physics:centerOfMass = (0,0,0)',f'float3 physics:diagonalInertia = {tuple(float(x) for x in vals)}',f'quatf physics:principalAxes = ({q[0]}, {q[1]}, {q[2]}, {q[3]})']
for i,(r,mesh) in enumerate(zip(records,meshes)):
 mesh.vertices=(mesh.vertices-center)@R.T;r['mesh']=str(Path(r['mesh']).with_suffix('.obj'));mesh.export(p/r['mesh']);name=f'part_{i}_{r["name"]}'
 ET.SubElement(assets,'mesh',name=name,file=r['mesh']);ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,rgba='.5 .6 .7 1',contype='0',conaffinity='0',group='2')
 ET.SubElement(body,'site',name=name+'_com',pos=fmt(r['center_body_m']),size='.001',rgba='1 0 0 1',group='4')
 usd += [f'def Mesh "{name}" {{','uniform token subdivisionScheme = "none"','point3f[] points = ['+','.join(str(tuple(float(x) for x in v)) for v in mesh.vertices)+']','int[] faceVertexCounts = ['+','.join(['3']*len(mesh.faces))+']','int[] faceVertexIndices = ['+','.join(str(int(x)) for x in mesh.faces.ravel())+']','}']
usd+=['}'];(p/'../usd/drone.usda').write_text('\n'.join(usd));ET.indent(root);ET.ElementTree(root).write(p/'model.xml')
report=dict(total_mass_kg=.142,cad_com_mm=(center*1000).tolist(),cad_to_body=R.tolist(),inertia_body_kg_m2=I.tolist(),parts=records,extra_mass_kg=.0045,extra_mass_model='point mass at combined COM, zero intrinsic inertia',limitations=['rk3576_body uses surface approximation because STEP volume is invalid; one face failed triangulation','Collision geometry is generated separately by decompose.py and apply_collisions.py; see collision_validation.json','CAD +Y up and +Z camera-forward inferred from assembly geometry','Motor/propeller internal mass uses uniform CAD density','No thrust model or flight controller supplied'],source_sha256=hashlib.sha256((Path(__file__).resolve().parents[1] / 'cad' / 'asm2.STEP').read_bytes()).hexdigest())
report['battery_translation_cad_mm'] = [0.041446297937902325, 0.0, -2.142733906478395]
report['balance_target'] = 'XY centroid of four motor assemblies; battery height unchanged; extra mass follows final COM'
(p/'../metadata/mass_properties.json').write_text(json.dumps(report,indent=2));model=mujoco.MjModel.from_xml_path(str(p/'model.xml'));data=mujoco.MjData(model)
for _ in range(100):mujoco.mj_step(model,data)
assert np.isfinite(data.qpos).all();assert abs(model.body_mass.sum()-.142)<1e-9
print('MJCF PASS',report['cad_com_mm'],I)

from apply_collisions import apply, collision_sources_match
if collision_sources_match():
    apply()
else:
    print("Visual export complete; run decompose.py then apply_collisions.py for collisions")
