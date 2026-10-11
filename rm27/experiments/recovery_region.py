"""Bounded continuous pose-region clearance, independent of mission guidance.

Every accepted cell has a clearance proof using a conservative displacement
bound over all translations and Euler rotations in that cell. Unknown cells
remain failures. This checks surfaces, not containment inside solid CAD.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from .target_region_audit import (capsule_probes,local_meshes,
    outside_patch_triangles,segment_triangle_distance)

RUNNER_BYTES=Path(__file__).read_bytes()


def subdivide_capsule(start,end,count):
    if type(count) is not int or not 1<=count<=64:raise ValueError('1..64 capsule segments required')
    points=np.linspace(start,end,count+1)
    return list(zip(points[:-1],points[1:]))


def displacement_bound(half_widths,radius):
    half=np.asarray(half_widths,float)
    # The rotation distance of a product is at most the sum of the three
    # component angle distances, irrespective of Euler order or centre.
    angle=min(np.pi,float(np.deg2rad(half[3:]).sum()))
    return float(np.linalg.norm(half[:3])+2*radius*np.sin(angle/2))


class RegionAudit:
    def __init__(self,scene,settings):
        self.settings=settings
        self.model=mujoco.MjModel.from_xml_path(str(scene));self.data=mujoco.MjData(self.model)
        model,data=self.model,self.data;mujoco.mj_forward(model,data)
        self.body=model.body('drone_1').id
        joint=model.body_jntadr[self.body];self.qadr=model.jnt_qposadr[joint]
        if model.jnt_type[joint]!=mujoco.mjtJoint.mjJNT_FREE:raise ValueError('free vehicle root required')
        site=model.site(settings['armor_site']).id
        self.center=data.site_xpos[site].copy();self.plane=data.site_xmat[site].reshape(3,3).copy()
        self.point=np.asarray(settings['contact_point_body_flu_m'],float)
        self.vehicle=np.flatnonzero(model.geom_bodyid==self.body)
        self.armor=model.geom(settings['armor_geom']).id
        width,height=settings['allowed_surface_dimensions_m']
        half=[width/2,height/2,settings['backing_surface_tolerance_m']]
        bounds=np.asarray(settings['pose_bounds'],float)
        # The entire family lies inside this conservative sphere/AABB about
        # the patch. Include all triangles crossing it, not just mesh centres.
        self.radii={}
        for geom in self.vehicle:
            start,end,radius,_=capsule_probes(model,data,geom)
            body_rotation=data.xmat[self.body].reshape(3,3)
            local=np.vstack([start,end])-data.xpos[self.body]
            local=local@body_rotation-self.point
            self.radii[int(geom)]=float(np.linalg.norm(local,axis=1).max()+radius)
        radius=np.linalg.norm(np.max(np.abs(bounds[:3]),axis=1))+max(self.radii.values())+.01
        self.meshes=local_meshes(model,data,self.center-radius,self.center+radius)
        self.guard_meshes=[]
        for geom,triangles,tree in self.meshes:
            forbidden=outside_patch_triangles(triangles,self.center,self.plane,half)
            if len(forbidden):
                boxes=np.column_stack((forbidden.min(axis=1),forbidden.max(axis=1)))
                self.guard_meshes.append((geom,forbidden,trimesh.util.bounds_tree(boxes)))
        # The designated primitive armor is allowed only for guards. Its
        # forbidden face fragments must also be audited (not just a witness).
        if model.geom_type[self.armor]!=mujoco.mjtGeom.mjGEOM_BOX:
            raise ValueError('reviewed box armor required')
        box=trimesh.creation.box(extents=2*model.geom_size[self.armor])
        vertices=box.vertices@data.geom_xmat[self.armor].reshape(3,3).T+data.geom_xpos[self.armor]
        forbidden=outside_patch_triangles(vertices[box.faces],self.center,self.plane,half)
        if len(forbidden):
            boxes=np.column_stack((forbidden.min(axis=1),forbidden.max(axis=1)))
            self.guard_meshes.append((self.armor,forbidden,trimesh.util.bounds_tree(boxes)))
        self.others=[g for g in range(model.ngeom) if model.geom_bodyid[g]!=self.body and
            model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH and
            (model.geom_contype[g] or model.geom_conaffinity[g])]

    def assess(self,low,high):
        model,data=self.model,self.data
        pose=(low+high)/2;half=(high-low)/2
        rotation=Rotation.from_euler('xyz',pose[3:],degrees=True)
        # Pose axes: normal, patch tangent X/Y, body roll/pitch/yaw.
        tip=self.center+self.plane@pose[[1,2,0]]
        data.qpos[self.qadr:self.qadr+3]=tip-rotation.apply(self.point)
        data.qpos[self.qadr+3:self.qadr+7]=rotation.as_quat()[[3,0,1,2]]
        mujoco.mj_forward(model,data)
        tolerance=self.settings['penetration_tolerance_m'];certain=True
        for geom in self.vehicle:
            start,end,radius,method=capsule_probes(model,data,geom)
            epsilon=displacement_bound(half,self.radii[int(geom)])
            guard=model.geom(geom).name.startswith('drone_1_guard_')
            segments=subdivide_capsule(start,end,self.settings.get('capsule_segments',1)) if guard else [(start,end)]
            tip_world=data.xpos[self.body]+data.xmat[self.body].reshape(3,3)@self.point
            for first,last in segments:
                # A capsule is exactly the union of these shorter capsules.
                # Its isotropic radius is invariant under rotation; only the
                # axis displacement needs an angular movement bound.
                lever=float(max(np.linalg.norm(first-tip_world),np.linalg.norm(last-tip_world)))
                segment_epsilon=displacement_bound(half,lever) if guard else epsilon
                for other,triangles,tree in (self.guard_meshes if guard else self.meshes):
                    expanded=radius+segment_epsilon
                    indices=list(tree.intersection(np.r_[np.minimum(first,last)-expanded,np.maximum(first,last)+expanded]))
                    if not indices:continue  # AABB separation proves this pair.
                    distances,nearest=segment_triangle_distance(first,last,triangles[indices])
                    index=int(np.argmin(distances));clearance=float(distances[index]-radius+tolerance)
                    if clearance<0:
                        return dict(status='collision',pose=pose.tolist(),vehicle_geom=model.geom(geom).name,
                            surface_geom=model.geom(other).name,point_world_m=nearest[index].tolist(),
                            overlap_m=float(radius-distances[index]),method=method)
                    if clearance<segment_epsilon:certain=False
            for other in self.others:
                if guard and other==self.armor:continue
                # Far separated enclosing spheres are a proof even for
                # geometry pairs the compiled narrow phase cannot evaluate.
                if model.geom_type[other]!=mujoco.mjtGeom.mjGEOM_PLANE:
                    gap=(np.linalg.norm(data.geom_xpos[other]-data.geom_xpos[geom])-
                        model.geom_rbound[other]-model.geom_rbound[geom])
                    if gap>epsilon:continue
                distance=float(mujoco.mj_geomDistance(model,data,geom,other,epsilon+.001,np.zeros(6)))
                if distance < -tolerance:
                    return dict(status='collision',pose=pose.tolist(),vehicle_geom=model.geom(geom).name,
                        surface_geom=model.geom(other).name,overlap_m=-distance,method='compiled primitive distance')
                if distance+tolerance<epsilon:certain=False
        return dict(status='certified' if certain else 'unknown',pose=pose.tolist(),
            maximum_displacement_bound_m=displacement_bound(half,max(self.radii.values())))


def run(scene,configuration,output):
    scene,configuration,output=map(lambda p:Path(p).resolve(),(scene,configuration,output))
    settings=json.loads(configuration.read_text());output.mkdir(parents=True,exist_ok=False)
    auditor=RegionAudit(scene,settings);bounds=np.asarray(settings['pose_bounds'],float)
    if bounds.shape!=(6,2) or not np.isfinite(bounds).all() or np.any(bounds[:,1]<bounds[:,0]):
        raise ValueError('six finite ordered pose bounds required')
    stack=[(bounds[:,0],bounds[:,1],0)];certified=0;checked=0;failure=None
    radius=max(auditor.radii.values())
    with (output/'cells.jsonl').open('x',buffering=1) as stream:
        while stack and checked<settings['maximum_cells']:
            low,high,depth=stack.pop();result=auditor.assess(low,high);checked+=1
            stream.write(json.dumps(dict(low=low.tolist(),high=high.tolist(),depth=depth,**result))+'\n')
            if result['status']=='certified':certified+=1;continue
            if result['status']=='collision':failure=result;break
            if depth>=settings['maximum_depth']:
                failure=dict(status='unresolved',reason='refinement depth exhausted',cell=result);break
            extent=high-low;weights=np.r_[extent[:3],np.deg2rad(extent[3:])*radius]
            axis=int(np.argmax(weights));middle=(low[axis]+high[axis])/2
            first_high=high.copy();first_high[axis]=middle
            last_low=low.copy();last_low[axis]=middle
            stack.append((last_low,high,depth+1));stack.append((low,first_high,depth+1))
            if checked%1000==0:print(json.dumps(dict(checked=checked,certified=certified,pending=len(stack))),flush=True)
    if stack and failure is None:failure=dict(status='unresolved',reason='cell budget exhausted')
    result=dict(passed=failure is None and not stack,gate_version=settings['gate_version'],
        s1_complete=False,run_type='continuous bounded pose-region surface clearance',
        checked_cells=checked,certified_cells=certified,failure=failure,configuration=settings,
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        configuration_sha256=hashlib.sha256(configuration.read_bytes()).hexdigest(),
        runner_sha256=hashlib.sha256(RUNNER_BYTES).hexdigest(),
        surface_runner_sha256=hashlib.sha256(Path(__file__).with_name('target_region_audit.py').read_bytes()).hexdigest(),
        limitations='Bounded configured pose family only; conservatively enclosed shapes may cause false failures. Surface clearance is not CAD solid containment. Flight membership, contact classification and force convergence remain separate.')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'runner.py').write_bytes(RUNNER_BYTES)
    print(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();return 0 if run(a.scene,a.config,a.output)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
