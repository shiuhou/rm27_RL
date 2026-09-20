import numpy as np

G = 9.81


def unit(x):
    return x / np.maximum(np.linalg.norm(x, axis=-1, keepdims=True), 1e-12)


def camera_basis(up, yaw, mount_pitch_deg):
    """Columns are camera right, down, forward expressed in local coordinates."""
    forward_heading = np.stack([np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)], axis=1)
    body_x = unit(forward_heading - np.sum(forward_heading*up, axis=1)[:, None]*up)
    right = -unit(np.cross(up, body_x))
    pitch = np.deg2rad(mount_pitch_deg)
    forward = np.cos(pitch)*body_x + np.sin(pitch)*up
    down = np.cross(forward, right)
    return np.stack([right, down, forward], axis=2)


def project(relative, basis, radius, hfov, width, height, min_pixels):
    camera = np.einsum('nji,nj->ni', basis, relative)
    distance = np.linalg.norm(relative, axis=1)
    angle = np.arcsin(np.clip(radius/np.maximum(distance, radius), 0, 1))
    f = width / (2*np.tan(np.deg2rad(hfov/2)))
    uv = f*camera[:, :2]/np.maximum(camera[:, 2:3], 1e-12)
    # Full angular contour required for range-from-size: clipped blobs unavailable.
    half_v = np.arctan(height/(2*f))
    half_h = np.deg2rad(hfov/2)
    visible = ((camera[:, 2] > radius) & (camera[:, 2]*np.sin(half_h)-np.abs(camera[:, 0])*np.cos(half_h) > radius)
               & (camera[:, 2]*np.sin(half_v)-np.abs(camera[:, 1])*np.cos(half_v) > radius)
               & (2*f*np.tan(angle) >= min_pixels))
    return uv, angle, visible


def unproject(uv, angle, basis, radius, hfov, width):
    f = width / (2*np.tan(np.deg2rad(hfov/2)))
    ray = unit(np.column_stack([uv/f, np.ones(len(uv))]))
    distance = radius/np.maximum(np.sin(angle), 1e-6)
    return np.einsum('nij,nj->ni', basis, ray)*distance[:, None]


def limit_specific_force(force, tw, tilt_deg):
    force = force.copy()
    force[:, 2] = np.maximum(force[:, 2], .1*G)
    xy = np.linalg.norm(force[:, :2], axis=1)
    force[:, :2] *= np.minimum(1., force[:, 2]*np.tan(np.deg2rad(tilt_deg))/np.maximum(xy, 1e-12))[:, None]
    force *= np.minimum(1., tw*G/np.maximum(np.linalg.norm(force, axis=1), 1e-12))[:, None]
    return force


def swept_distance(r0, r1):
    step = r1-r0
    fraction = np.clip(-np.sum(r0*step, axis=1)/np.maximum(np.sum(step*step, axis=1), 1e-15), 0, 1)
    return np.linalg.norm(r0+fraction[:, None]*step, axis=1), fraction
