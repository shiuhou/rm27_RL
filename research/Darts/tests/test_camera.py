import json
import mujoco
import numpy as np
import pytest

from research.Darts.capture import intrinsics, camera_truth
from research.Darts.audit_cad import sha256
from research.Darts.experiment import camera, pose, load_run
from research.Darts.trajectory import BallisticPath


@pytest.fixture
def camera_scene(tmp_path):
    scene='<mujoco><statistic extent="20"/><worldbody><body name="darts_projectile" mocap="true"/></worldbody></mujoco>'
    (tmp_path/'scene.xml').write_text(scene)
    (tmp_path/'dart_visual.obj').write_text('# unused mesh in camera-only fixture\n')
    report=dict(scene_sha256=sha256(tmp_path/'scene.xml'),
        dart_mesh_sha256=sha256(tmp_path/'dart_visual.obj'),launch_m=[0,0,1],
        target_m=[20,5,1],flight_time_s=2,config=dict(gravity_m_s2=9.81))
    (tmp_path/'summary.json').write_text(json.dumps(report))
    m,d,_,_=load_run(tmp_path)
    assert (tmp_path/'scene.xml').read_text()==scene
    assert m.cam_fovy[m.camera('darts_fpv').id]==60
    assert m.vis.map.znear*m.stat.extent==pytest.approx(.001)
    return m,d


@pytest.mark.parametrize('fraction',[0,.5,1,1.5])
def test_camera_follows_nose_and_heading_and_freezes(fraction,camera_scene):
    m,d=camera_scene
    path=BallisticPath.aimed([0,0,1],[20,5,1],2)
    p,v=pose(m,d,path,path.duration*fraction)
    truth=camera_truth(m,d)
    R=np.array(truth['R_world_from_camera'])
    np.testing.assert_allclose(truth['position_world_m'],p,atol=1e-12)
    np.testing.assert_allclose(R[:,2],v/np.linalg.norm(v),atol=1e-12)
    np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12)
    assert np.linalg.det(R)==pytest.approx(1)
    assert R[2,1]<0  # image down points toward world down
    # A point straight ahead must project onto the principal point.
    K=intrinsics(1280,720,60)
    pc=R.T@(p+3*v/np.linalg.norm(v)-p)
    uv=K@pc;uv=uv[:2]/uv[2]
    np.testing.assert_allclose(uv,[639.5,359.5],atol=1e-9)


def test_switching_back_restores_free_camera(camera_scene):
    m,d=camera_scene
    path=BallisticPath.aimed([0,0,1],[20,5,1],2)
    cam=mujoco.MjvCamera()
    report=dict(target_m=[20,5,1],launch_m=[0,0,1])
    camera(cam,'fpv',path,report,0,m)
    assert cam.type==mujoco.mjtCamera.mjCAMERA_FIXED
    assert cam.fixedcamid==m.camera('darts_fpv').id
    for mode in ('overview','follow','target','launch','side'):
        camera(cam,mode,path,report,0,m)
        assert cam.type==mujoco.mjtCamera.mjCAMERA_FREE


def test_intrinsics_vertical_fov_and_aspect():
    K=intrinsics(1280,720,60)
    assert K[0,0]==pytest.approx(K[1,1])
    assert 2*np.arctan(720/(2*K[1,1]))==pytest.approx(np.deg2rad(60))
