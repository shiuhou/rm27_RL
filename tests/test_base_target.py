import json
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np
import pytest
from rm27.base_target import add


def test_patch_contact_and_marker_noncollision():
    cfg=json.loads(Path('configs/base_target.json').read_text())
    root=ET.fromstring('<mujoco><option timestep=".002" gravity="0 0 0" integrator="RK4"/><worldbody/></mujoco>')
    p,n=add(root,cfg)
    body=ET.SubElement(root.find('worldbody'),'body',pos=' '.join(map(str,p+n*.05)))
    ET.SubElement(body,'freejoint')
    ET.SubElement(body,'geom',name='probe',type='sphere',size='.01',mass='.18')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    d.qvel[:3]=-n*.08
    touched=False
    for _ in range(400):
        mujoco.mj_step(m,d)
        for contact in d.contact:
            names={m.geom(contact.geom1).name,m.geom(contact.geom2).name}
            if names=={'probe','rm27_base_armor'}: touched=True
        assert np.isfinite(d.qpos).all()
    assert touched
    assert np.dot(d.qpos[:3]-p,n)>0
    assert m.geom_contype[m.geom('rm27_base_marker').id]==0
    assert m.geom_conaffinity[m.geom('rm27_base_marker').id]==0
    mujoco.mj_forward(m,d)
    np.testing.assert_allclose(d.site_xpos[m.site('rm27_base_armor_center').id],p)
    np.testing.assert_allclose(d.site_xmat[m.site('rm27_base_armor_center').id].reshape(3,3)[:,2],n,atol=1e-12)
    with pytest.raises(ValueError,match='already present'): add(root,cfg)
