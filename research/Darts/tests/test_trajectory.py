import mujoco
import numpy as np
import pytest

from research.Darts.trajectory import BallisticPath,basis
from research.Darts.experiment import swept_checks


def test_known_vacuum_projectile_and_energy():
    path=BallisticPath.aimed([0,0,1],[20,0,1],2,10)
    np.testing.assert_allclose(path.velocity,[10,0,10])
    p,v=path.state(np.array([0,1,2]))
    np.testing.assert_allclose(p,[[0,0,1],[10,0,6],[20,0,1]])
    energy=.5*np.sum(v*v,axis=1)+10*p[:,2]
    np.testing.assert_allclose(energy,energy[0])


def test_true_hit_miss_and_wrong_direction_are_distinct():
    path=BallisticPath.aimed([0,0,1],[20,0,1],2,10)
    hit=path.plane_hit([20,0,1],[-1,0,0],.1,.1)
    assert hit['time_s']==pytest.approx(2)
    assert hit['normal_speed_m_s']==pytest.approx(10)
    assert path.plane_hit([20,1,1],[-1,0,0],.1,.1) is None
    assert path.plane_hit([20,0,1],[1,0,0],.1,.1) is None


def test_thin_obstacle_detected_between_samples():
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="thin_wall" type="box" pos=".5 0 0" size=".0001 1 1"/></worldbody></mujoco>')
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    groups=m.geom_group.copy()
    result=swept_checks(m,d,np.array([[0.,0,0],[1.,0,0]]),m.ngeom)
    assert not result['colliders']['clear']
    assert result['colliders']['first_intersection']['geom']=='thin_wall'
    assert result['visuals']['clear']
    np.testing.assert_array_equal(m.geom_group,groups)


def test_visual_only_obstacle_reported_separately():
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="visual_wall" type="box" pos=".5 0 0" size=".001 1 1" contype="0" conaffinity="0"/></worldbody></mujoco>')
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    result=swept_checks(m,d,np.array([[0.,0,0],[1.,0,0]]),m.ngeom)
    assert result['colliders']['clear']
    assert not result['visuals']['clear']


def test_heading_and_sampling_endpoint():
    for v in [[1,2,3],[0,0,1],[0,0,-1],[-3,0,2]]:
        r=basis(v)
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
        assert np.linalg.det(r)==pytest.approx(1)
        np.testing.assert_allclose(r[:,0],np.array(v)/np.linalg.norm(v))
    path=BallisticPath.aimed([0,0,1],[2,3,4],1.003)
    t,p,v=path.samples(.02)
    assert t[-1]==1.003
    assert np.max(np.diff(t))<=.02
    np.testing.assert_allclose(p[-1],[2,3,4])


@pytest.mark.parametrize('duration',[0,-1,float('nan')])
def test_invalid_flight_time(duration):
    with pytest.raises(ValueError):
        BallisticPath.aimed([0,0,0],[1,0,0],duration)
