import mujoco
import numpy as np
import pytest

from tools.px4.physical_fixture import fixture_type,validate
from tools.px4.convergence_fixture import loaded_classification


class SmallPlant:
    def __init__(self):
        actuators=''.join(f'<general joint="free" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="0 1.103625" dyntype="filterexact" dynprm=".04"/>' for _ in range(4))
        self.model=mujoco.MjModel.from_xml_string(f'<mujoco><option timestep=".001"/><worldbody><body name="drone_1" pos="0 0 .02"><freejoint name="free"/><geom name="drone_1_guard_0" type="box" size=".01 .01 .01" mass=".15"/></body><geom name="rm27_base_armor" type="box" pos="1 0 0" size=".01 .01 .01"/></worldbody><actuator>{actuators}</actuator></mujoco>')
        self.data=mujoco.MjData(self.model);self.body=self.model.body('drone_1').id
        self.motors=np.arange(4);self.maximum=self.model.actuator_ctrlrange[:,1].copy()
    @property
    def qpos(self):return self.data.qpos


def test_declared_uncertainty_changes_motor_response_and_keeps_mass():
    plant=fixture_type(SmallPlant,dict(initial_xy_m=[.002,-.002],motor_lag_factor=1.1,
        maximum_thrust_factor=.95,contact_time_factor=.9))()
    assert plant.model.body_mass[plant.body]==pytest.approx(.15)
    np.testing.assert_allclose(plant.qpos[:2],[.002,-.002],atol=1e-15)
    plant.data.ctrl[:]=plant.maximum
    for _ in range(10):mujoco.mj_step(plant.model,plant.data)
    expected=1.103625*.95*(1-np.exp(-.01/.044))
    np.testing.assert_allclose(plant.data.act,expected,rtol=1e-13)
    with pytest.raises(ValueError):validate(dict(initial_xy_m=[.01,0]))


def test_zero_force_candidates_do_not_become_back_impacts_but_tiny_real_loads_do():
    contacts=[dict(force_on_armor_world_N=[0.,0.,0.],signed_normal_force_N=-0.),
        dict(force_on_armor_world_N=[-1.,0.,0.],signed_normal_force_N=1.)]
    assert loaded_classification(contacts)==({'front'},1)
    contacts.append(dict(force_on_armor_world_N=[1e-20,0.,0.],signed_normal_force_N=-1e-20))
    assert loaded_classification(contacts)==({'front','back'},1)
