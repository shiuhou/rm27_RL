"""Compose the current dock field with an independent kinematic dart experiment."""

import contextlib
import hashlib
import json
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .audit_cad import export, sha256
from .trajectory import BallisticPath, basis, target_frame

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = Path(__file__).with_name('config.json')


def fmt(v):
    return ' '.join(f'{float(x):.12g}' for x in v)


def quat(rotation):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(rotation).reshape(9))
    return q


def load_model(path):
    m = mujoco.MjModel.from_xml_path(str(Path(path).resolve()))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    return m,d


def ray(m,d,start,direction,groups):
    identifier = np.array([-1],dtype=np.int32)
    normal = np.zeros(3)
    distance = mujoco.mj_ray(m,d,np.array(start,dtype=float),
                            np.array(direction,dtype=float),np.array(groups,dtype=np.uint8),
                            1,-1,identifier,normal)
    return distance,int(identifier[0]),normal


def swept_checks(m,d,points,source_geom_count):
    """Swept centerline checks, separately against source colliders and visuals.

    These are geometric diagnostics, not finite-size or impact dynamics checks.
    Exclude only the added dart, annotations and target patch, never source geoms.
    """
    saved = m.geom_group.copy()
    result = {}
    try:
        m.geom_group[:] = 5
        for i in range(source_geom_count):
            m.geom_group[i] = 0 if m.geom_contype[i] or m.geom_conaffinity[i] else 1
        for name,groups in [('colliders',[1,0,0,0,0,0]),('visuals',[0,1,0,0,0,0])]:
            first = None
            for j,(a,b) in enumerate(zip(points[:-1],points[1:])):
                delta = b-a
                length = np.linalg.norm(delta)
                distance,identifier,_ = ray(m,d,a,delta/length,groups)
                if 0 <= distance <= length+1e-8:
                    first = dict(segment=j,geom=m.geom(identifier).name,
                                 position_m=(a+distance*delta/length).tolist())
                    break
            result[name] = dict(clear=first is None,first_intersection=first)
    finally:
        m.geom_group[:] = saved
    return result


def build(output, config=DEFAULT_CONFIG, cad_assets=None):
    import trimesh
    from rm27.__main__ import build as build_dock

    output,config = Path(output).resolve(),Path(config).resolve()
    cfg = json.loads(config.read_text())
    source = ROOT/cfg['cad_source']
    if sha256(source) != cfg['cad_sha256']:
        raise ValueError('Dart STEP changed; audit and update research/Darts/config.json')
    output.mkdir(parents=True,exist_ok=False)
    (output/'config.json').write_text(json.dumps(cfg,indent=2)+'\n')
    cad_assets = Path(cad_assets).resolve() if cad_assets else output/'cad'
    if cad_assets == output/'cad':
        export(source,cad_assets)
    audit = json.loads((cad_assets/'audit.json').read_text())
    if audit['source_sha256'] != cfg['cad_sha256']:
        raise ValueError('Cached CAD belongs to another STEP')
    mesh = trimesh.load(cad_assets/'cad_m.obj',force='mesh')
    # A rigid basis change, with no scaling beyond mm->m performed by export.
    mesh.vertices = (mesh.vertices-np.array(cfg['cad_front_center_mm'])*.001)@basis(cfg['cad_forward_axis'])
    if abs(mesh.vertices[:,0].max()) > .0002:
        raise ValueError('Front reference no longer matches the CAD nose plane')
    mesh.export(output/'dart_visual.obj')

    # Rebuild from current reviewed configuration, rather than selecting by folder mtime.
    with (output/'dock-build.log').open('w') as log,contextlib.redirect_stdout(log):
        build_dock(ROOT/cfg['scene_config'],output/'dock')
    source_scene = output/'dock/scene.xml'
    original,initial = load_model(source_scene)
    if original.body('rm27_dock').id < 0:
        raise ValueError('A dock field is required')
    launcher = original.body(cfg['launcher_body']).id
    start = initial.xpos[launcher]+initial.xmat[launcher].reshape(3,3)@np.array(cfg['launch_offset_local_m'])
    direction = np.array(cfg['target_ray_direction_world'],dtype=float)
    direction /= np.linalg.norm(direction)
    distance,gid,normal = ray(original,initial,cfg['target_ray_start_world_m'],direction,[0,0,1,0,0,0])
    if distance < 0 or original.geom(gid).name != cfg['target_visual_geom']:
        raise ValueError('Upper target ray no longer hits the reviewed field surface')
    raw_target = np.array(cfg['target_ray_start_world_m'])+distance*direction
    target = raw_target+cfg['target_surface_offset_m']*normal
    marker_cfg = json.loads((ROOT/cfg['marker_config']).read_text())
    marker = np.array(marker_cfg['marker_center_m'])
    if target[2] <= marker[2]+marker_cfg['marker_radius_m']:
        raise ValueError('Dart target must be above the green marker')
    path = BallisticPath.aimed(start,target,cfg['flight_time_s'],cfg['gravity_m_s2'])
    t,points,velocities = path.samples(cfg['sample_dt_s'])

    tree = ET.parse(source_scene)
    root = tree.getroot()
    world,asset = root.find('worldbody'),root.find('asset')
    visual = root.find('visual')
    if visual is None:
        visual = ET.SubElement(root,'visual')
    glob = visual.find('global')
    if glob is None:
        glob = ET.SubElement(visual,'global')
    glob.set('offwidth','1280');glob.set('offheight','900')
    ET.SubElement(asset,'mesh',name='darts_mesh',file=str(output/'dart_visual.obj'))
    body = ET.SubElement(world,'body',name='darts_projectile',mocap='true',pos=fmt(start),quat=fmt(quat(basis(path.velocity))))
    ET.SubElement(body,'geom',name='darts_visual',type='mesh',mesh='darts_mesh',
                  contype='0',conaffinity='0',group='2',rgba='.9 .45 .08 1')
    ET.SubElement(body,'site',name='darts_front',size='.003',rgba='1 1 1 1')
    # Deliberately visual: the terminal event is a finite-plane crossing test.
    target_body = ET.SubElement(world,'body',name='darts_target',pos=fmt(target),quat=fmt(quat(target_frame(normal))))
    width,height = cfg['target_width_height_m']
    depth = cfg['target_thickness_m']
    ET.SubElement(target_body,'geom',name='darts_upper_armor',type='box',
                  pos=fmt([0,0,-depth/2]),size=fmt([width/2,height/2,depth/2]),
                  rgba='.03 .3 1 1',contype='0',conaffinity='0',group='2')
    ET.SubElement(target_body,'site',name='darts_target_center',size='.003',rgba='1 1 .2 1')
    ET.SubElement(world,'geom',name='darts_green_lamp',type='cylinder',pos=fmt(marker),
                  quat=fmt(quat(target_frame(marker_cfg['marker_normal']))),
                  size=fmt([marker_cfg['marker_radius_m'],.003]),rgba='.05 1 .04 1',
                  contype='0',conaffinity='0',group='2')
    ET.SubElement(world,'geom',name='darts_launch_marker',type='sphere',pos=fmt(start),size='.035',
                  rgba='.1 .9 1 .8',contype='0',conaffinity='0',group='4')
    tree.write(output/'scene.xml',encoding='unicode')
    model,data = load_model(output/'scene.xml')
    # Compare by name: adding world geoms can change compiled geom indexing.
    preserved = model.nq == original.nq and model.nv == original.nv and model.nu == original.nu
    for i in range(original.ngeom):
        j = model.geom(original.geom(i).name).id
        preserved &= bool(np.allclose(initial.geom_xpos[i],data.geom_xpos[j])
                          and np.allclose(initial.geom_xmat[i],data.geom_xmat[j])
                          and np.array_equal(original.geom_size[i],model.geom_size[j])
                          and original.geom_contype[i]==model.geom_contype[j]
                          and original.geom_conaffinity[i]==model.geom_conaffinity[j])
    # Run sweep in the untouched original model, so appended annotations never interfere.
    clear = swept_checks(original,initial,points,original.ngeom)
    hit = path.plane_hit(target,normal,width,height)
    height_time = np.clip(-path.velocity[2]/path.gravity[2],0,path.duration) if cfg['gravity_m_s2'] else 0
    q = np.array([quat(basis(v)) for v in velocities])
    np.savetxt(output/'trajectory.csv',np.column_stack((t,points,velocities,q)),delimiter=',',
               header='time_s,front_x_m,front_y_m,front_z_m,vx_m_s,vy_m_s,vz_m_s,qw,qx,qy,qz',comments='')
    report = dict(passed=bool(hit and preserved and all(x['clear'] for x in clear.values())),
                  mode='headless analytic trajectory; MuJoCo scene/geometry validation; no SITL or mj_step',
                  config=cfg,config_sha256=sha256(config),source_cad_sha256=sha256(source),
                  cad=dict(dimensions_mm=audit['dimensions_mm'],solid_occurrences=audit['solid_occurrences'],
                           triangles=audit['triangles'],axial_length_m=float(-mesh.vertices[:,0].min()),
                           frame_evidence=cfg['cad_frame_evidence']),
                  scene_config_sha256=sha256(ROOT/cfg['scene_config']),
                  marker_config_sha256=sha256(ROOT/cfg['marker_config']),
                  field_commit=json.loads((output/'dock/audit.json').read_text())['field_commit'],
                  source_scene_sha256=sha256(source_scene),scene_sha256=sha256(output/'scene.xml'),
                  dart_mesh_sha256=sha256(output/'dart_visual.obj'),
                  source_geometries_preserved=bool(preserved),source_geom_count=original.ngeom,
                  launch_m=start.tolist(),target_m=target.tolist(),target_native_surface_m=raw_target.tolist(),
                  target_normal=normal.tolist(),green_lamp_m=marker.tolist(),
                  dock_m=data.xpos[model.body('rm27_dock').id].tolist(),
                  flight_time_s=path.duration,launch_velocity_m_s=path.velocity.tolist(),
                  launch_speed_m_s=float(np.linalg.norm(path.velocity)),
                  final_speed_m_s=float(np.linalg.norm(velocities[-1])),
                  apex_height_m=float(path.state(height_time)[0][2]),samples=len(t),
                  impact=hit,centerline_checks=clear,
                  limitations=[cfg['launch_evidence'],cfg['target_evidence'],cfg['model'],
                               'Swept centerline ray clearance does not certify body/tail clearance, solid containment, collision impulses or referee damage.',
                               'Existing aircraft remain at their initial parked poses; no interception controller.',
                               'Real muzzle speed, aerodynamics and mass are uncalibrated.'])
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def playback_spec(output, report):
    """Resolve the documented repository migration without rewriting saved runs.

    Relocated field meshes must match the recorded Git commit byte for byte.
    Unknown missing assets fail instead of silently substituting a new scene.
    """
    root=ET.parse(output/'scene.xml').getroot()
    field=ROOT/'rm27/simulation/field/rmuc2026_battlefield_mjcf'
    old_field=str(ROOT/'rm_uav_lab_m600/rmuc2026_battlefield_mjcf')+'/'
    old_run=str(ROOT/'Darts/artifacts'/output.name)+'/'
    blobs=None;relocated=[]
    for element in root.iter():
        name=element.get('file')
        if not name:
            continue
        original=Path(name)
        resolved=original if original.is_absolute() else output/original
        if not resolved.exists() and name.startswith(old_field):
            relative=name[len(old_field):]
            resolved=field/relative
            if blobs is None:
                rows=subprocess.check_output(['git','-C',str(field),'ls-tree','-r',report['field_commit']],text=True)
                blobs={row.split('\t',1)[1]:row.split()[2] for row in rows.splitlines()}
            raw=resolved.read_bytes()
            digest=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
            if blobs.get(relative)!=digest:
                raise ValueError(f'Relocated field asset differs from recorded commit: {relative}')
        elif not resolved.exists() and name.startswith(old_run):
            resolved=output/name[len(old_run):]
        if not resolved.exists():
            raise FileNotFoundError(f'Missing playback asset: {name}')
        element.set('file',str(resolved.resolve()))
        if resolved!=original:
            relocated.append(dict(original=name,resolved=str(resolved.resolve()),sha256=sha256(resolved)))
    report['playback_asset_relocations']=relocated
    return mujoco.MjSpec.from_string(ET.tostring(root,encoding='unicode'))


def load_run(output, fovy=60.):
    output=Path(output).resolve()
    report=json.loads((output/'summary.json').read_text())
    if sha256(output/'scene.xml') != report['scene_sha256'] or sha256(output/'dart_visual.obj') != report['dart_mesh_sha256']:
        raise ValueError('Generated scene or dart mesh changed; rebuild this experiment')
    if not np.isfinite(fovy) or not 1 < fovy < 179:
        raise ValueError('Vertical field of view must be finite and in (1,179) degrees')
    # Add the camera in memory: earlier, hashed experiment artifacts stay intact.
    spec=playback_spec(output,report)
    # MuJoCo cameras look down -Z with +Y up. Dart +X is forward, +Z up.
    camera_rotation=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
    spec.body('darts_projectile').add_camera(name='darts_fpv',pos=[0,0,0],
        quat=quat(camera_rotation),fovy=float(fovy),ipd=0.)
    m=spec.compile()
    # An absolute 1 mm near plane, rather than the whole-field default clip.
    m.vis.map.znear=.001/m.stat.extent
    d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    path=BallisticPath.aimed(report['launch_m'],report['target_m'],report['flight_time_s'],report['config']['gravity_m_s2'])
    return m,d,path,report


def pose(m,d,path,t):
    p,v=path.state(np.clip(t,0,path.duration))
    mid=m.body_mocapid[m.body('darts_projectile').id]
    d.mocap_pos[mid]=p
    d.mocap_quat[mid]=quat(basis(v))
    d.time=float(np.clip(t,0,path.duration))
    mujoco.mj_forward(m,d)
    return p,v


def camera(cam,mode,path,report,t,model=None):
    if mode=='fpv':
        if model is None:
            raise ValueError('First-person camera requires the loaded model')
        cam.type=mujoco.mjtCamera.mjCAMERA_FIXED
        cam.fixedcamid=model.camera('darts_fpv').id
        return
    cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    p,_=path.state(np.clip(t,0,path.duration))
    views={
        'overview':([0,0,1.4],34,90,-65),
        'follow':(p,1.8,115,-18),
        'target':(report['target_m'],1.0,180,-12),
        'launch':(report['launch_m'],3.3,0,-10),
        'side':((path.start+np.array(report['target_m']))/2,30,90,-8),
    }
    look,dist,az,el=views[mode]
    cam.lookat[:]=look;cam.distance=dist;cam.azimuth=az;cam.elevation=el
