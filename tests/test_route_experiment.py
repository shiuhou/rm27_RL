import hashlib
import json

import numpy as np
import pytest

from rm27.experiments.route_experiment import RouteExperiment, path_distance


def make_route(tmp_path):
    scene = tmp_path/'scene.xml'
    scene.write_text('''<mujoco><worldbody>
      <body name="drone_1" pos="1 0 0"><freejoint/><geom size=".01" mass=".18"/></body>
      <geom name="rm27_base_armor" type="box" pos="-1.01 0 0" size=".01 .1 .1"/>
      <site name="rm27_base_armor_center" pos="-1 0 0" zaxis="1 0 0" size=".001"/>
    </worldbody></mujoco>''')
    manifest = dict(
        passed=True, surface_route_clear=True, asset_sha256={},
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        tracking_margin_m=.08, waypoints_world_m=[[1, 0, 0], [-.4, 0, 0]],
    )
    audit = tmp_path/'audit.json'
    audit.write_text(json.dumps(manifest))
    mission = json.loads(open('configs/fast_armor_mission.json').read())
    mission['cruise_reference_speed_m_s'] = .05
    mission['command_waypoint_indices'] = [0, 1]
    mission['stop_waypoint_indices'] = [1]
    mission['inbound_stop_waypoint_indices'] = []
    mission['runtime_tracking_abort_m'] = .08
    mission_path = tmp_path/'mission.json'
    mission_path.write_text(json.dumps(mission))
    return scene, audit, mission_path


def test_route_guard_and_bounded_reference(tmp_path):
    scene, audit, mission = make_route(tmp_path)
    route = RouteExperiment(scene, audit, mission)
    position = np.array([1., 0., 0.])
    route.target(0, position, 0)
    target = route.target(1, position, 0)
    np.testing.assert_allclose(target, [.95, 0, 0])
    assert path_distance(np.array([.5, .05, 0]), route.points) == pytest.approx(.05)
    with pytest.raises(RuntimeError, match='tube'):
        route.check(np.array([.5, .09, 0]))
    scene.write_text('changed')
    with pytest.raises(ValueError, match='hash'):
        RouteExperiment(scene, audit, mission)


def test_route_requires_enemy_contact_and_return(tmp_path):
    scene, audit, mission = make_route(tmp_path)
    route = RouteExperiment(scene, audit, mission)
    start, staging = route.points
    route.target(0, start, 0)
    route.target(1, staging, 0)
    assert route.state == 'IMPACT'
    result = route.target(
        2, route.armor_center + route.armor_normal*.01, .8,
        [{'geoms': ['drone_1_guard_0', 'rm27_base_armor'], 'force_N': 1, 'distance_m': 0}],
        -route.armor_normal*.8,
    )
    assert route.state == 'RETREAT'
    assert route.impact['armor_normal_speed_m_s'] == pytest.approx(.8)
    assert route.contact_allowed({'geoms': ['rm27_base_armor', 'drone_1_guard_0']})
    route.target(3, staging, 0)
    assert route.state == 'INBOUND'
    result = route.target(4, start, 0)
    assert result is None
    assert any(row['state'] == 'armor_contact' for row in route.visited)


def test_asset_change_rejects_route(tmp_path):
    scene, audit, mission = make_route(tmp_path)
    asset = tmp_path/'mesh.bin'
    asset.write_bytes(b'original mesh')
    manifest = json.loads(audit.read_text())
    manifest['asset_sha256'] = {str(asset): hashlib.sha256(asset.read_bytes()).hexdigest()}
    audit.write_text(json.dumps(manifest))
    RouteExperiment(scene, audit, mission)
    asset.write_bytes(b'changed mesh')
    with pytest.raises(ValueError, match='asset hash'):
        RouteExperiment(scene, audit, mission)


def test_rejects_wrong_team_contract(tmp_path):
    scene, audit, mission = make_route(tmp_path)
    config = json.loads(mission.read_text())
    config['target_team'] = 'red'
    mission.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='target blue'):
        RouteExperiment(scene, audit, mission)
