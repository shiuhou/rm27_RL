"""Rigid lens pose and display isolation, including camera switching."""
import copy
from contextlib import nullcontext
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from rm27.experiments.aggressive_contact.fpv import configure_camera,load_mount


def plant():
    model=mujoco.MjModel.from_xml_string('''<mujoco><statistic extent="20"/>
    <worldbody><camera name="overview_top" pos="0 0 10"/>
    <body name="uav"><freejoint/><geom type="sphere" size=".02" mass=".18"/></body>
    </worldbody></mujoco>''')
    return model,mujoco.MjData(model)


@pytest.mark.parametrize('q',[[1,0,0,0],[.5,.5,.5,.5],[2**-.5,2**-.5,0,0]])
def test_lens_tracks_body_rotation_including_roll(q):
    m,d=plant();mount=load_mount();cid=configure_camera(m,mount)
    d.qpos[:3]=[2,-3,4];d.qpos[3:7]=q
    mujoco.mj_forward(m,d)
    body=m.body('uav').id;R=d.xmat[body].reshape(3,3)
    C=d.cam_xmat[cid].reshape(3,3)
    np.testing.assert_allclose(d.cam_xpos[cid],d.xpos[body]+R@mount['position_body_m'],atol=1e-12)
    np.testing.assert_allclose(-C[:,2],R@mount['forward_body'],atol=1e-12)
    np.testing.assert_allclose(C.T@C,np.eye(3),atol=1e-12)
    assert np.linalg.det(C)==pytest.approx(1.)
    assert m.cam_fovy[cid]==60
    assert m.vis.map.znear*m.stat.extent==pytest.approx(.001)


def test_mount_matches_cad_face_with_explicit_preview_alignment():
    mount=load_mount();cad=mount['cad_evidence']
    B=np.array(cad['cad_to_display_body']);R=np.array(cad['display_body_to_teacher_rotation'])
    p=B@(np.array(cad['front_face_center_cad_mm'])-cad['cad_origin_mm'])/1000
    p[2]-=cad['display_body_height_m']/2
    np.testing.assert_allclose(R@p,mount['position_body_m'],atol=1e-12)
    np.testing.assert_allclose(R@B@cad['outward_axis_cad'],mount['forward_body'],atol=1e-12)
    assert np.rad2deg(np.arcsin(-mount['forward_body'][2]))==pytest.approx(20)


def test_live_camera_switch_does_not_modify_control_plant(monkeypatch):
    import mujoco.viewer
    from rm27.experiments.aggressive_contact.viewing import LiveView
    model,data=plant();mujoco.mj_forward(model,data)
    saved_model=copy.deepcopy(model);saved_qpos=data.qpos.copy()
    def launch(m,d,**kwargs):
        return SimpleNamespace(lock=nullcontext,cam=mujoco.MjvCamera(),opt=mujoco.MjvOption(),
            is_running=lambda:True,sync=lambda **kw:None,set_texts=lambda x:None,close=lambda:None)
    monkeypatch.setattr(mujoco.viewer,'launch_passive',launch)
    view=LiveView(model,data,speed=1e9,camera='fpv')
    data.qpos[:3]=[1,2,3];data.qpos[3:7]=[.5,.5,.5,.5]
    mujoco.mj_forward(model,data)
    view.update(data,'TEST',force=True)
    cid=view.fpv_id;body=model.body('uav').id
    np.testing.assert_allclose(-view.data.cam_xmat[cid].reshape(3,3)[:,2],
        data.xmat[body].reshape(3,3)@load_mount()['forward_body'],atol=1e-12)
    assert view.handle.cam.type==mujoco.mjtCamera.mjCAMERA_FIXED
    for key,overview in [('1',False),('2',True),('V',False)]:
        view.key(ord(key))
        assert view.overview==overview
        assert view.handle.cam.type==mujoco.mjtCamera.mjCAMERA_FREE
    view.key(ord('6'));assert view.handle.cam.type==mujoco.mjtCamera.mjCAMERA_FIXED
    for field in ('cam_bodyid','cam_pos','cam_quat','body_mass','body_inertia','geom_pos'):
        np.testing.assert_array_equal(getattr(model,field),getattr(saved_model,field))
    np.testing.assert_array_equal(data.qpos[:3],[1,2,3])
    assert not np.array_equal(data.qpos,saved_qpos)
    view.close()


@pytest.mark.parametrize('fovy',[0,180,float('nan')])
def test_invalid_fov_rejected(fovy):
    m,_=plant()
    with pytest.raises(ValueError):configure_camera(m,load_mount(),fovy)
