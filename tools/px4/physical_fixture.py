"""Declared simulation uncertainty applied before the first physics step.

Aircraft mass and geometry stay fixed. The actor cannot read these values;
the same fixture is used for recorded-input replay and independent convergence.
"""
import json

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


def validate(configuration):
    supported={'initial_xy_m','initial_yaw_error_deg','motor_lag_factor','maximum_thrust_factor','contact_time_factor'}
    if set(configuration)-supported:
        raise ValueError('Unsupported physical fixture field')
    xy=np.asarray(configuration.get('initial_xy_m',[0.,0.]),float)
    yaw=configuration.get('initial_yaw_error_deg',0.)
    factors=[configuration.get(k,1.) for k in ['motor_lag_factor','maximum_thrust_factor','contact_time_factor']]
    if (xy.shape!=(2,) or not np.isfinite(np.r_[xy,yaw,factors]).all() or
        np.linalg.norm(xy)>.004 or abs(yaw)>3 or any(not .9<=f<=1.1 for f in factors)):
        raise ValueError('Explicit bounded physical fixture required')
    return json.loads(json.dumps(configuration,allow_nan=False))


def configure_model(model,motors,configuration):
    settings=validate(configuration)
    maximum_before=model.actuator_ctrlrange[motors,1].copy()
    lag_before=model.actuator_dynprm[motors,0].copy()
    model.actuator_ctrlrange[motors,1]*=settings.get('maximum_thrust_factor',1.)
    model.actuator_dynprm[motors,0]*=settings.get('motor_lag_factor',1.)
    geoms=[g for g in range(model.ngeom) if (model.geom(g).name or '').startswith('drone_1_guard_')
        or model.geom(g).name=='rm27_base_armor']
    contacts={}
    for geom in geoms:
        before=model.geom_solref[geom].copy()
        if before[0]<=0:
            raise ValueError('Positive time-reference contact model required')
        model.geom_solref[geom,0]*=settings.get('contact_time_factor',1.)
        contacts[model.geom(geom).name]=dict(before=before.tolist(),after=model.geom_solref[geom].tolist())
    return dict(maximum_thrust_before_N=maximum_before.tolist(),maximum_thrust_after_N=model.actuator_ctrlrange[motors,1].tolist(),
        motor_lag_before_s=lag_before.tolist(),motor_lag_after_s=model.actuator_dynprm[motors,0].tolist(),contact_parameters=contacts)


def fixture_type(base,configuration):
    settings=validate(configuration)
    class FixturePlant(base):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            parameters=configure_model(self.model,self.motors,settings)
            self.maximum=self.model.actuator_ctrlrange[self.motors,1].copy()
            initial=self.qpos.copy()
            self.qpos[:2]+=settings.get('initial_xy_m',[0.,0.])
            rotation=Rotation.from_euler('z',settings.get('initial_yaw_error_deg',0.),degrees=True)
            original=Rotation.from_quat(self.qpos[[4,5,6,3]])
            self.qpos[3:7]=(rotation*original).as_quat()[[3,0,1,2]]
            self.initial_quaternion=self.qpos[3:7].copy()
            mujoco.mj_forward(self.model,self.data)
            self.physical_fixture_evidence=dict(settings=settings,
                mass_kg=float(self.model.body_mass[self.body]),initial_qpos_before=initial.tolist(),initial_qpos_after=self.qpos.tolist(),
                **parameters,
                scope='Declared engineering uncertainty in simulation only; no actor input, hardware write or measured calibration')
    return FixturePlant
