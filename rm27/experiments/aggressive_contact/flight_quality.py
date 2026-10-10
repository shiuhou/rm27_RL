"""Flight-control diagnostics. Camera angles are geometry proxies, not vision."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .model import ROOT, frame, marker_center, target_frame, unit


def view_geometry(position, rotation, target, mount):
    """Exact rectangular 16:9 pinhole frustum of the unchanged preview mount.

    No occlusion, pixels, lens model or detector is evaluated. This function
    consumes geometry only and is never an estimator or a contact detector.
    """
    f = unit(mount['forward_body'])
    right = unit(np.cross(f, mount['up_reference_body']))
    up = np.cross(right, f)
    position = np.asarray(position, float)
    rotation = np.asarray(rotation, float)
    target = np.asarray(target, float)
    camera = position + np.einsum('...ij,j->...i', rotation, np.array(mount['position_body_m']))
    if target.ndim == 2 and position.ndim == 2:
        delta = target[None, :, :] - camera[:, None, :]
        body = np.einsum('...ji,...j->...i', rotation[:, None, :, :], delta)
    else:
        delta = target-camera
        body = np.einsum('...ji,...j->...i', rotation, delta)
    depth = body@f
    vertical = np.rad2deg(np.arctan2(body@up, depth))
    horizontal = np.rad2deg(np.arctan2(body@right, depth))
    vhalf = mount['vertical_fov_degrees']/2
    hhalf = np.rad2deg(np.arctan(np.tan(np.deg2rad(vhalf))*16/9))
    inside = (depth>0) & (abs(vertical)<vhalf) & (abs(horizontal)<hhalf)
    angle = np.rad2deg(np.arccos(np.clip(depth/np.maximum(np.linalg.norm(delta,axis=-1),1e-12),-1,1)))
    return inside, angle, vertical, horizontal


def marker_geometry(position, rotation, target, mount, radius_m=0.0275):
    """Conservative geometry proxy for the circular guidance lamp.

    The centre and eight perimeter points must all be in the rectangular frustum
    for ``inside`` to be true. This models a visible footprint, not brightness,
    occlusion, lens distortion, or a detector.
    """
    position = np.asarray(position, float)
    rotation = np.asarray(rotation, float)
    center = marker_center(target)
    # The lamp is circular, so its perimeter frame follows only the lamp/plane
    # normal.  Do not inherit the rectangular armor patch heading here.
    R = frame(target['normal'])
    points = [center]
    for angle in np.linspace(0., 2*np.pi, 8, endpoint=False):
        points.append(center + radius_m*(np.cos(angle)*R[:, 0] + np.sin(angle)*R[:, 1]))
    points = np.asarray(points)
    inside, angle, vertical, horizontal = view_geometry(position, rotation, points, mount)
    # view_geometry broadcasts only through the target subtraction; normalize
    # the returned shape for trajectory arrays.
    if inside.ndim == 2:
        footprint_inside = np.all(inside, axis=1)
        center_inside, center_angle = inside[:, 0], angle[:, 0]
        center_vertical, center_horizontal = vertical[:, 0], horizontal[:, 0]
        vertical_max = np.max(np.abs(vertical), axis=1)
        horizontal_max = np.max(np.abs(horizontal), axis=1)
    else:
        footprint_inside = np.all(inside, axis=0)
        center_inside, center_angle = inside[0], angle[0]
        center_vertical, center_horizontal = vertical[0], horizontal[0]
        vertical_max = np.max(np.abs(vertical), axis=0)
        horizontal_max = np.max(np.abs(horizontal), axis=0)
    return dict(inside=footprint_inside, center_inside=center_inside,
                angle=center_angle, vertical=center_vertical,
                horizontal=center_horizontal, vertical_max=vertical_max,
                horizontal_max=horizontal_max, center=center,
                radius_m=float(radius_m))


def arrays(source):
    source = Path(source)
    flight = source/'flight' if (source/'flight').exists() else source
    archive = np.load(source/'trajectory.npz' if (source/'trajectory.npz').exists() else flight/'trajectory.npz')
    x = archive['trace']; t = x[:,0]
    if len(t)<3 or not np.all(np.diff(t)>0):
        raise ValueError('At least three strictly increasing flight samples required')
    summary = json.loads((source/'summary.json').read_text())
    physical = json.loads((flight/'physical.json').read_text())
    mount = json.loads((ROOT/'configs/contact_fpv_camera.json').read_text())
    R = Rotation.from_quat(x[:,[5,6,7,4]]).as_matrix()
    speed = np.linalg.norm(x[:,8:11],axis=1)
    acc = np.gradient(x[:,8:11],t,axis=0)
    jerk = np.gradient(acc,t,axis=0)
    target = physical['target']
    inside,angle,vertical,horizontal = view_geometry(x[:,1:4],R,np.array(target['center_m']),mount)
    marker = marker_geometry(x[:,1:4], R, target, mount)
    forward = R@np.array([-1.,0.,0.])
    horizontal_speed = np.linalg.norm(x[:,8:10],axis=1)
    backward = np.sum(forward[:,:2]*x[:,8:10],axis=1)<0
    tc = (summary.get('first_contact') or {}).get('time_s',float(t[-1]))
    return dict(trace=x,time=t,rotation=R,summary=summary,physical=physical,
                phases=archive['phases'],speed=speed,acc=acc,jerk=jerk,inside=marker['inside'],
                armor_inside=inside,marker=marker,
                angle=angle,vertical=vertical,horizontal=horizontal,backward=backward,
                moving=horizontal_speed>.2,pre=t<tc-.05,contact_time=tc,
                distance=np.linalg.norm(x[:,1:4]-physical['target']['center_m'],axis=1),
                tilt=np.rad2deg(np.arccos(np.clip(R[:,2,2],-1,1))),rate=np.linalg.norm(x[:,11:14],axis=1))


def metrics(source):
    a=arrays(source);x=a['trace'];t=a['time'];pre=a['pre'];s=a['summary']
    mount=json.loads((ROOT/'configs/contact_fpv_camera.json').read_text())
    approach = t<a['contact_time']
    ret = a['phases']=='RETURN'
    dt = np.diff(t,append=t[-1])
    def fraction(mask, condition):
        duration=float(dt[mask].sum())
        return None if duration<=0 else float(dt[mask & condition].sum()/duration)
    far = approach & (a['distance']>=.5)
    near = approach & (a['distance']<.5)
    first=s.get('first_contact') or {}
    missing=np.flatnonzero(approach & ~a['inside'])
    valid=np.flatnonzero(approach & a['inside'])
    out=dict(passed=s['passed'],failure_modes=s['failure_modes'],
        contact_time_s=s.get('time_to_contact_s'),recovery_time_s=s.get('recovery_time_s'),
        maneuver_time_s=s.get('time_to_confirmed_recovery_s'),mission_time_s=s.get('mission_duration_s'),
        contact_margin_mm=None if not first else 1000*first['region_margin_m'],
        pre_speed_max_m_s=float(a['speed'][pre].max()),pre_tilt_max_deg=float(a['tilt'][pre].max()),
        pre_rate_max_rad_s=float(a['rate'][pre].max()),pre_acceleration_max_m_s2=float(np.linalg.norm(a['acc'][pre],axis=1).max()),
        pre_jerk_max_m_s3=float(np.linalg.norm(a['jerk'][pre],axis=1).max()),
        pre_jerk_rms_m_s3=float(np.sqrt(np.mean(np.sum(a['jerk'][pre]**2,axis=1)))),
        outbound_altitude_max_m=float(x[approach,3].max()),
        outbound_backward_fraction=fraction(approach & a['moving'],a['backward']),
        return_backward_fraction=fraction(ret & a['moving'],a['backward']),
        return_moving_time_s=float(dt[ret & a['moving']].sum()),
        return_rate_max_rad_s=None if not ret.any() else float(a['rate'][ret].max()),
        whole_rate_max_rad_s=float(a['rate'].max()),
        view_outside_far_fraction=fraction(far,~a['inside']),
        view_outside_far_s=float(dt[far & ~a['inside']].sum()),
        view_outside_near_s=float(dt[near & ~a['inside']].sum()),
        marker_center_outside_far_fraction=fraction(far,~a['marker']['center_inside']),
        marker_vertical_rms_deg=None if not far.any() else float(np.sqrt(np.average(a['marker']['vertical'][far]**2, weights=dt[far]))),
        marker_vertical_max_deg=None if not far.any() else float(np.max(np.abs(a['marker']['vertical'][far]))),
        marker_horizontal_rms_deg=None if not far.any() else float(np.sqrt(np.average(a['marker']['horizontal'][far]**2, weights=dt[far]))),
        marker_view_margin_deg=None if not far.any() else float(min(np.rad2deg(np.arctan(np.tan(np.deg2rad(mount['vertical_fov_degrees']/2))*16/9))
                                         - np.max(a['marker']['horizontal_max'][far]),
                                         mount['vertical_fov_degrees']/2 - np.max(a['marker']['vertical_max'][far]))),
        terminal_view_loss_s=None if not len(valid) else float(a['contact_time']-t[valid[-1]]),
        post_rate_max_rad_s=s.get('max_post_body_rate_rad_s'),altitude_loss_m=s.get('altitude_loss_m'),
        saturation_fraction=s.get('saturation_fraction'),
        whole_saturation_fraction=float(np.sum(dt*x[:,26])/dt.sum()),
        continuous_slow_time_s=float(dt[(t>.8)&(t<a['contact_time']-.5)&(a['speed']<.4)].sum()),
        status='MuJoCo truth closed-loop; camera geometry proxy uses unchanged assumed CAD mount; no visual estimator',
        derivative_scope='actual finite-difference acceleration/jerk, t < impact-50ms; excludes impulse and recovery reset',
        travel_axis_body=[-1,0,0],camera_mount_sha256=hashlib.sha256((ROOT/'configs/contact_fpv_camera.json').read_bytes()).hexdigest())
    return out
