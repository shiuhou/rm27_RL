"""MuJoCo contact data for an independent scorer; no task success decisions."""
import numpy as np
import mujoco

def collect_contacts(model, data):
    records = []
    jacobian = np.zeros((3, model.nv))
    rotational = np.zeros_like(jacobian)
    for index in range(data.ncon):
        contact = data.contact[index]
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or '' for g in contact.geom]
        for side, name in enumerate(names):
            if not name.startswith(('collision_', 'contact_')):
                continue
            velocities = []
            for geom in contact.geom:
                mujoco.mj_jac(model, data, jacobian, rotational, contact.pos, int(model.geom_bodyid[geom]))
                velocities.append(jacobian @ data.qvel)
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, data, index, wrench)
            group = 'contact_front' if name.startswith('contact_front_') else (
                'contact_rear' if name.startswith('contact_rear_') else 'body')
            sign = 1 if side == 1 else -1
            frame = contact.frame.reshape(3, 3)
            records.append(dict(group=group, geom=name, other_geom=names[1-side],
                position_world_m=contact.pos.copy().tolist(),
                normal_other_to_drone_world=(sign*frame[0]).tolist(),
                relative_velocity_world_m_s=(velocities[side]-velocities[1-side]).tolist(),
                force_on_drone_world_N=(sign*frame.T@wrench[:3]).tolist(),
                impulse_on_drone_world_Ns=(sign*frame.T@wrench[:3]*model.opt.timestep).tolist(),
                physics_dt_s=float(model.opt.timestep), distance_m=float(contact.dist)))
            break
    return records
