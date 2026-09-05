"""Vectorized closed-loop surrogate. Truth stays in plant, sensors and scoring.

Vehicle is a thrust-vector/attitude-lag surrogate, not six-DOF rotor dynamics.
Ego estimates are an explicitly abstracted error process, not a flow estimator.
Camera output is noisy geometric features, not image detector validation.
"""
from collections import deque
import numpy as np
from .autonomy import Guidance
from .geometry import G, camera_basis, limit_specific_force, project, swept_distance, unit
from .interfaces import EgoEstimate, ImageObservation
from .scenario import Scenario


def simulate(cfg, method, n, seed, scenario='mixed', trace=None):
    s = Scenario(n, seed, scenario)
    camera_rng = np.random.default_rng(s.noise_seed)
    ego_rng = np.random.default_rng(s.noise_seed+1)
    p = np.tile([0., 0., 2.], (n, 1))
    v = np.zeros((n, 3))
    up = np.tile([0., 0., 1.], (n, 1))
    yaw = np.zeros(n)
    thrust = np.full(n, G)
    tw = cfg.tw*(1+cfg.performance_spread*s.performance)
    mass = cfg.mass_g*(1+cfg.performance_spread*s.mass_error)/1000
    # Independent effects: fixed nominal max thrust, uncertain mass and supply.
    tw *= cfg.mass_g/(mass*1000)
    g = Guidance(n, method, cfg)
    queue = deque()
    alive = np.ones(n, bool)
    success = np.zeros(n, bool)
    crashed = np.zeros(n, bool)
    contact_time = np.full(n, np.nan)
    contact_speed = np.full(n, np.nan)
    miss = np.full(n, np.inf)
    maxima = np.zeros((n, 3))  # speed, acceleration, tilt
    effort = np.zeros(n)
    observations = np.zeros(n, int)
    samples = np.zeros(n, int)
    invisible = np.zeros(n, int)
    track_error = np.zeros(n)
    track_samples = np.zeros(n, int)
    max_age = np.zeros(n)
    stale_steps = np.zeros(n, int)
    active_steps = np.zeros(n, int)
    next_frame = 0
    next_control = 0
    command = np.zeros((n, 3))
    desired_yaw = np.zeros(n)
    last_ego_noise_frame = -1
    velocity_noise = np.zeros((n, 3))
    radius = cfg.target_radius_m+cfg.vehicle_radius_m
    for step in range(int(np.ceil(cfg.duration_s/cfg.dt))):
        t = step*cfg.dt
        dt = min(cfg.dt, cfg.duration_s-t)
        target, target_velocity = s.target(t)
        basis = camera_basis(up, yaw, cfg.camera_pitch_deg)
        # Simulated local estimate only; known initial pose is a declared launch prior.
        drift = cfg.ego_drift_m_s*s.drift_direction
        yaw_error = np.deg2rad(cfg.ego_yaw_drift_deg_s*t)*s.performance
        estimated_basis = camera_basis(up, yaw+yaw_error, cfg.camera_pitch_deg)
        ego_frame = int(t*100+1e-8)
        if ego_frame != last_ego_noise_frame:
            velocity_noise = ego_rng.normal(0, cfg.ego_velocity_noise, (n, 3))
            last_ego_noise_frame = ego_frame
        estimate_p = p + drift*t + cfg.ego_scale_error*(p-np.array([0, 0, 2]))
        estimate_v = v*(1+cfg.ego_scale_error)+drift+velocity_noise
        ego = EgoEstimate(t, estimate_p, estimate_v, estimated_basis)
        if t+1e-9 >= next_frame/cfg.camera_hz:
            uv, angle, visible = project(target-p, basis, cfg.target_radius_m, cfg.fov_deg, cfg.width, cfg.height, cfg.min_diameter_px)
            uv += camera_rng.normal(0, cfg.pixel_noise, (n, 2))
            f = cfg.width/(2*np.tan(np.deg2rad(cfg.fov_deg/2)))
            angle += camera_rng.normal(0, cfg.radius_noise_fraction, n)*angle + camera_rng.normal(0, cfg.pixel_noise/(2*f), n)
            valid = visible & (angle > 0) & (camera_rng.random(n) >= cfg.camera_dropout)
            valid &= ~((t >= s.occlusion_start) & (t < s.occlusion_start+cfg.occlusion_s))
            samples += alive
            invisible += alive & ~visible
            queue.append(ImageObservation(t, t+cfg.latency_s, uv, np.clip(angle, 1e-6, np.pi/2), valid,
                                          estimate_p.copy(), estimated_basis.copy()))
            next_frame += 1
        while queue and queue[0].delivery_time <= t+1e-9:
            obs = queue.popleft()
            g.observe(obs, t)
            observations += obs.valid & alive
        if t+1e-9 >= next_control/cfg.guidance_hz:
            command, desired_yaw = g.command(ego)
            next_control += 1
        measured_age = np.where(g.seen, t-g.epoch, 0.)
        max_age = np.maximum(max_age, np.where(alive, measured_age, 0.))
        valid_track = alive & g.seen
        track_error += np.where(valid_track, np.linalg.norm(g.target_estimate(t)-target, axis=1)**2, 0)
        track_samples += valid_track
        stale_steps += alive & (~g.seen | (measured_age > cfg.track_timeout_s))
        active_steps += alive
        force_request = limit_specific_force(command+np.array([0, 0, G]), tw, cfg.tilt_deg)
        force_magnitude = np.linalg.norm(force_request, axis=1)
        up_next = unit(up+(1-np.exp(-dt/cfg.attitude_tau_s))*(unit(force_request)-up))
        thrust_next = thrust+(1-np.exp(-dt/cfg.motor_tau_s))*(force_magnitude-thrust)
        delta_yaw = (desired_yaw-yaw+np.pi)%(2*np.pi)-np.pi
        yaw_next = yaw + np.clip(delta_yaw, -np.deg2rad(cfg.yaw_rate_deg_s)*dt, np.deg2rad(cfg.yaw_rate_deg_s)*dt)
        acceleration = .5*(thrust[:, None]*up+thrust_next[:, None]*up_next)-np.array([0, 0, G])-cfg.drag_per_s*v
        v_next = v+dt*acceleration
        p_next = p+.5*dt*(v+v_next)
        future_target, future_velocity = s.target(t+dt)  # evaluator/plant only
        distance, fraction = swept_distance(target-p, future_target-p_next)
        miss = np.minimum(miss, np.where(alive, distance, np.inf))
        hit = alive & (distance <= radius)
        # Earliest segment/sphere entry for time/contact speed; closest point only scores miss.
        rel0 = target-p
        rel_step = future_target-p_next-rel0
        qa = np.sum(rel_step**2, axis=1)
        qb = 2*np.sum(rel0*rel_step, axis=1)
        qc = np.sum(rel0**2, axis=1)-radius**2
        entry = np.clip((-qb-np.sqrt(np.maximum(qb*qb-4*qa*qc, 0)))/(2*np.maximum(qa, 1e-15)), 0, 1)
        entry[qc <= 0] = 0
        ground_fraction = np.where(p_next[:, 2] < cfg.vehicle_radius_m,
                                   (p[:, 2]-cfg.vehicle_radius_m)/np.maximum(p[:, 2]-p_next[:, 2], 1e-12), np.inf)
        hit &= entry <= ground_fraction
        crash = alive & ~hit & ((p_next[:, 2] < cfg.vehicle_radius_m) | (p_next[:, 2] > 12) | (np.linalg.norm(p_next[:, :2], axis=1) > 100))
        contact_time[hit] = t+dt*entry[hit]
        contact_speed[hit] = np.linalg.norm((v-target_velocity)+entry[:, None]*((v_next-future_velocity)-(v-target_velocity)), axis=1)[hit]
        success |= hit
        crashed |= crash
        tilt = np.rad2deg(np.arccos(np.clip(up_next[:, 2], -1, 1)))
        values = np.column_stack([np.linalg.norm(v_next, axis=1), np.linalg.norm(acceleration, axis=1), tilt])
        maxima = np.maximum(maxima, np.where(alive[:, None], values, 0))
        effort += np.where(alive, dt*np.sum(command**2, axis=1), 0.)
        if trace is not None:
            trace.append({'t': t, 'p': p[0].tolist(), 'target': target[0].tolist(), 'estimate': g.target_estimate(t)[0].tolist(),
                          'visible': bool(g.seen[0] and measured_age[0] < .1), 'tilt_deg': float(tilt[0]), 'active': bool(alive[0])})
        p[alive], v[alive], up[alive], thrust[alive], yaw[alive] = p_next[alive], v_next[alive], up_next[alive], thrust_next[alive], yaw_next[alive]
        alive &= ~hit & ~crash
        if not alive.any():
            break
    rows = []
    for i in range(n):
        reason = 'contact' if success[i] else ('flight_envelope' if crashed[i] else ('never_detected' if observations[i] == 0 else 'timeout'))
        rows.append(dict(episode=i, seed=seed, scenario=['static', 'crossing', 'receding', 'maneuver'][s.kind[i]], method=method,
                         success=bool(success[i]), reason=reason, miss_m=float(miss[i]), clearance_m=float(max(0, miss[i]-radius)),
                         contact_time_s=float(contact_time[i]) if success[i] else None,
                         contact_speed_m_s=float(contact_speed[i]) if success[i] else None,
                         max_speed_m_s=float(maxima[i, 0]), max_acceleration_m_s2=float(maxima[i, 1]), max_tilt_deg=float(maxima[i, 2]),
                         control_effort_m2_s3=float(effort[i]), observations=int(observations[i]),
                         invisible_fraction=float(invisible[i]/max(samples[i], 1)), max_observation_age_s=float(max_age[i]),
                         stale_fraction=float(stale_steps[i]/max(active_steps[i], 1)),
                         tracking_rmse_m=float(np.sqrt(track_error[i]/track_samples[i])) if track_samples[i] else None,
                         actual_tw=float(tw[i]), mass_g=float(mass[i]*1000), initial_range_m=float(np.linalg.norm(s.start[i]-[0, 0, 2])),
                         nominal_target_speed_m_s=float(np.linalg.norm(s.velocity[i]))))
    return rows
