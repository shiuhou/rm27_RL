"""Independent local surface audit; visual meshes remain visual-only in physics."""
import argparse
import hashlib
import json
from pathlib import Path
from rm27.telemetry.recording import open_recording,recording_sha256

import mujoco
import numpy as np
import trimesh

RUNNER_BYTES=Path(__file__).read_bytes()


def segment_triangle_distance(start,end,triangles):
    """Exact capsule-axis distance to triangles, including face/edge intersection."""
    start,end=np.asarray(start,float),np.asarray(end,float)
    triangles=np.asarray(triangles,float)
    if not len(triangles):return np.empty(0),np.empty((0,3))
    points=np.broadcast_to(start,(len(triangles),3))
    closest=trimesh.triangles.closest_point(triangles,points)
    best=np.sum((closest-start)**2,axis=1)
    other=trimesh.triangles.closest_point(triangles,np.broadcast_to(end,points.shape))
    distance=np.sum((other-end)**2,axis=1);mask=distance<best
    best[mask]=distance[mask];closest[mask]=other[mask]
    axis=end-start;length=float(axis@axis)
    if length>1e-18:
        # Closest points between the finite axis and each finite triangle edge.
        for first,last in ((0,1),(1,2),(2,0)):
            begin=triangles[:,first];edge=triangles[:,last]-begin;delta=start-begin
            b=edge@axis;c=np.sum(edge*edge,axis=1);d=delta@axis;e=np.sum(delta*edge,axis=1)
            denominator=length*c-b*b
            t=np.divide(b*e-c*d,denominator,out=np.full_like(b,np.nan),where=denominator>1e-18)
            s=np.divide(length*e-b*d,denominator,out=np.full_like(b,np.nan),where=denominator>1e-18)
            on_edge=begin+s[:,None]*edge;on_axis=start+t[:,None]*axis
            distance=np.sum((on_axis-on_edge)**2,axis=1)
            mask=(t>=0)&(t<=1)&(s>=0)&(s<=1)&(distance<best)
            best[mask]=distance[mask];closest[mask]=on_edge[mask]
        # Boundary optima are axis endpoints (above) or triangle vertices.
        # This also covers parallel edges without iterative clamp error.
        for vertex in range(3):
            point=triangles[:,vertex]
            t=np.clip((point-start)@axis/length,0,1)
            distance=np.sum((start+t[:,None]*axis-point)**2,axis=1);mask=distance<best
            best[mask]=distance[mask];closest[mask]=point[mask]
        normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        denominator=normal@axis
        t=np.divide(np.sum(normal*(triangles[:,0]-start),axis=1),denominator,
            out=np.full(len(normal),np.nan),where=np.abs(denominator)>1e-18)
        valid=np.isfinite(t)&(t>=0)&(t<=1)
        crossing=start+np.nan_to_num(t)[:,None]*axis
        nearest=trimesh.triangles.closest_point(triangles,crossing)
        valid&=np.sum((nearest-crossing)**2,axis=1)<1e-18
        best[valid]=0;closest[valid]=crossing[valid]
    return np.sqrt(np.maximum(best,0)),closest


def capsule_probes(model,data,geom):
    position=data.geom_xpos[geom];kind=model.geom_type[geom]
    if kind==mujoco.mjtGeom.mjGEOM_CAPSULE:
        axis=data.geom_xmat[geom].reshape(3,3)[:,2]*model.geom_size[geom,1]
        return position-axis,position+axis,float(model.geom_size[geom,0]),'exact capsule'
    if kind==mujoco.mjtGeom.mjGEOM_SPHERE:
        return position,position,float(model.geom_size[geom,0]),'exact sphere'
    return position,position,float(model.geom_rbound[geom]),'conservative enclosing sphere'


def local_meshes(model,data,low,high):
    result=[]
    for geom in range(model.ngeom):
        if model.geom_type[geom]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        mesh=model.geom_dataid[geom];va,fa=model.mesh_vertadr[mesh],model.mesh_faceadr[mesh]
        vertices=model.mesh_vert[va:va+model.mesh_vertnum[mesh]]
        vertices=vertices@data.geom_xmat[geom].reshape(3,3).T+data.geom_xpos[geom]
        faces=model.mesh_face[fa:fa+model.mesh_facenum[mesh]];triangles=vertices[faces]
        keep=np.all(triangles.max(axis=1)>=low,axis=1)&np.all(triangles.min(axis=1)<=high,axis=1)
        if np.any(keep):
            triangles=triangles[keep]
            bounds=np.column_stack((triangles.min(axis=1),triangles.max(axis=1)))
            result.append((geom,triangles,trimesh.util.bounds_tree(bounds)))
    return result


def outside_patch_triangles(triangles,center,rotation,half_extent):
    """Clip out the allowed patch; retain every forbidden triangle fragment.

    Testing only a triangle's closest point misses a guard that covers both
    the allowed patch and the forbidden surface just beyond its edge.
    """
    triangles=np.asarray(triangles,float)
    if not len(triangles):return triangles.copy()
    local=(triangles-center)@rotation
    half=np.asarray(half_extent,float)
    fully_outside=np.any((local.min(axis=1)>half)|(local.max(axis=1)<-half),axis=1)
    fully_inside=np.all(np.abs(local)<=half,axis=(1,2))
    fragments=list(triangles[fully_outside])

    def clip(polygon,axis,bound,sign):
        result=[]
        for first,last in zip(polygon,polygon[1:]+polygon[:1]):
            a=sign*(first[axis]-bound);b=sign*(last[axis]-bound)
            if a<=0:result.append(first)
            if (a<0<b) or (b<0<a):result.append(first+(last-first)*a/(a-b))
        return result

    for triangle in local[~(fully_outside|fully_inside)]:
        remaining=list(triangle)
        for axis in range(3):
            for sign in (1.,-1.):
                bound=sign*half[axis]
                outside=clip(remaining,axis,bound,-sign)
                for i in range(1,len(outside)-1):
                    fragment=np.array([outside[0],outside[i],outside[i+1]])
                    if np.linalg.norm(np.cross(fragment[1]-fragment[0],fragment[2]-fragment[0]))>1e-16:
                        fragments.append(fragment@rotation.T+center)
                remaining=clip(remaining,axis,bound,sign)
                if len(remaining)<3:break
            if len(remaining)<3:break
    return np.asarray(fragments,float).reshape(-1,3,3)


def audit_recording(recording,configuration,output):
    recording,configuration,output=map(lambda p:Path(p).resolve(),(recording,configuration,output))
    settings=json.loads(configuration.read_text());metadata=json.loads((recording/'metadata.json').read_text())
    scene=Path(metadata['scene']);scene_hash=hashlib.sha256(scene.read_bytes()).hexdigest()
    if scene_hash!=metadata['scene_sha256']:raise ValueError('Recorded scene changed')
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    body=model.body('drone_1').id;armor=model.geom(settings['armor_geom']).id
    site=model.site(settings['armor_site']).id;center=data.site_xpos[site].copy()
    rotation=data.site_xmat[site].reshape(3,3).copy();width,length=settings['allowed_surface_dimensions_m']
    low,high=np.asarray(settings['local_bounds_world_m'])
    meshes=local_meshes(model,data,low-.15,high+.15)
    guard_meshes=[]
    for geom,triangles,tree in meshes:
        forbidden=outside_patch_triangles(triangles,center,rotation,
            [width/2,length/2,settings['backing_surface_tolerance_m']])
        if len(forbidden):
            bounds=np.column_stack((forbidden.min(axis=1),forbidden.max(axis=1)))
            guard_meshes.append((geom,forbidden,trimesh.util.bounds_tree(bounds)))
    vehicle=np.flatnonzero(model.geom_bodyid==body)
    others=[g for g in range(model.ngeom) if model.geom_bodyid[g]!=body and
        model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH and
        (model.geom_contype[g] or model.geom_conaffinity[g])]
    hits=[];allowed=[];outside=[];frames=0;next_time=0
    output.mkdir(parents=True,exist_ok=False)
    def allowed_patch(geom,point):
        local=rotation.T@(point-center)
        return (model.geom(geom).name.startswith('drone_1_guard_') and
            abs(local[0])<=width/2 and abs(local[1])<=length/2 and abs(local[2])<=settings['backing_surface_tolerance_m'])
    with open_recording(recording/'frames.jsonl') as stream:
        for line in stream:
            row=json.loads(line);position=np.asarray(row['truth']['position_nwu'])
            if not np.all((position>=low)&(position<=high)):continue
            time=row['timestamp']
            if time<next_time:continue
            next_time=time+settings['recording_sample_period_s']-1e-9
            data.qpos[:]=row['all_qpos'];mujoco.mj_forward(model,data);frames+=1
            for geom in vehicle:
                start,end,radius,method=capsule_probes(model,data,geom)
                is_guard=model.geom(geom).name.startswith('drone_1_guard_')
                for other,triangles,tree in (guard_meshes if is_guard else meshes):
                    # Conservative AABB cull; no triangle near the full capsule
                    # axis is omitted, even when its centre is far away.
                    indices=list(tree.intersection(np.r_[np.minimum(start,end)-radius,np.maximum(start,end)+radius]))
                    if not indices:continue
                    distance,nearest=segment_triangle_distance(start,end,triangles[indices])
                    collisions=np.flatnonzero(distance<radius-settings['penetration_tolerance_m'])
                    for index in collisions:
                        event=dict(time_s=time,vehicle_geom=model.geom(geom).name,
                            surface_geom=model.geom(other).name,point_world_m=nearest[index].tolist(),
                            overlap_m=float(radius-distance[index]),method=method,
                            physical_collision_enabled=bool(model.geom_contype[other] or model.geom_conaffinity[other]))
                        hits.append(event)
                if is_guard:
                    # Diagnostic allowed overlaps use the original triangles;
                    # acceptance above uses all forbidden clipped fragments.
                    for other,triangles,tree in meshes:
                        indices=list(tree.intersection(np.r_[np.minimum(start,end)-radius,np.maximum(start,end)+radius]))
                        if not indices:continue
                        distance,nearest=segment_triangle_distance(start,end,triangles[indices])
                        for index in np.flatnonzero(distance<radius-settings['penetration_tolerance_m']):
                            if allowed_patch(geom,nearest[index]):
                                allowed.append(dict(time_s=time,vehicle_geom=model.geom(geom).name,
                                    surface_geom=model.geom(other).name,point_world_m=nearest[index].tolist(),
                                    overlap_m=float(radius-distance[index]),method=method))
                for other in others:
                    if model.geom_type[other]!=mujoco.mjtGeom.mjGEOM_PLANE and np.linalg.norm(
                        data.geom_xpos[other]-data.geom_xpos[geom])>model.geom_rbound[other]+model.geom_rbound[geom]:continue
                    witness=np.zeros(6)
                    distance=mujoco.mj_geomDistance(model,data,geom,other,.001,witness)
                    if distance < -settings['penetration_tolerance_m']:
                        if other==armor and allowed_patch(geom,witness[3:]):continue
                        hits.append(dict(time_s=time,vehicle_geom=model.geom(geom).name,
                            surface_geom=model.geom(other).name,overlap_m=float(-distance),method='compiled primitive distance',
                            physical_collision_enabled=True))
    result=dict(passed=bool(frames and not hits),s1_complete=False,
        run_type='recorded-pose static local collider and visual-surface audit',
        gate_version=settings['gate_version'],frames=frames,forbidden_hits=hits,
        allowed_target_backing_intersections=allowed,local_mesh_triangle_count=sum(len(t) for _,t,_ in meshes),
        scene_sha256=scene_hash,configuration_sha256=hashlib.sha256(configuration.read_bytes()).hexdigest(),
        recording_sha256=recording_sha256(recording/'frames.jsonl'),
        runner_sha256=hashlib.sha256(RUNNER_BYTES).hexdigest(),
        limitations='Recorded poses only; enclosing spheres can give conservative false hits. Visual surfaces are not physical colliders or solid-containment proof. Full static recovery-volume sweep remains required.')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'runner.py').write_bytes(RUNNER_BYTES)
    print(json.dumps({k:v for k,v in result.items() if k not in ('forbidden_hits','allowed_target_backing_intersections')},indent=2))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('recording',type=Path)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();return 0 if audit_recording(a.recording,a.config,a.output)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
