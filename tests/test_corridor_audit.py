import numpy as np
import pytest
from rm27.experiments.corridor_audit import audit, path_samples
from rm27.experiments.corridor_audit import mesh_surface_hits
import mujoco


def test_path_spacing_covers_every_segment():
    points = path_samples([[0, 0, 0], [1, 0, 0], [1, 1, 0]], .13)
    assert np.max(np.linalg.norm(np.diff(points, axis=0), axis=1)) <= .13
    np.testing.assert_allclose(points[-1], [1, 1, 0])
    with pytest.raises(ValueError):
        path_samples([[0, 0, 0], [1, 0, 0]], 0)


def test_visual_wall_blocks_between_coarse_samples(tmp_path):
    scene = tmp_path/'scene.xml'
    scene.write_text('''<mujoco><worldbody>
      <body name="drone_1" pos="0 0 1"><freejoint/>
        <geom type="box" size=".06 .06 .02" mass=".18"/>
      </body>
      <geom name="visual_wall" type="box" pos=".25 0 1" size=".001 .3 .3"
            contype="0" conaffinity="0"/>
      </worldbody></mujoco>''')
    result = audit(scene, [[0, 0, 1], [1, 0, 1]], tmp_path/'blocked', spacing=.5)
    assert not result['passed']
    assert not result['obstructions']['visual_wall']['collision_enabled']
    clear = audit(scene, [[0, 1, 1], [1, 1, 1]], tmp_path/'clear', spacing=.1)
    assert clear['passed']


def test_triangle_distance_uses_compiled_world_transform():
    model = mujoco.MjModel.from_xml_string('''<mujoco><asset>
      <mesh name="tetra" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            face="0 2 1  0 1 3  0 3 2  1 2 3"/>
      </asset><worldbody><geom name="mesh" type="mesh" mesh="tetra" pos="2 3 4"/></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hits = mesh_surface_hits(model, data, 0, np.array([[2.2, 3.2, 3.95], [10, 10, 10]]), .1)
    assert len(hits) == 1 and hits[0]['sample'] == 0
    assert hits[0]['gap_m'] == pytest.approx(-.05, abs=1e-6)
