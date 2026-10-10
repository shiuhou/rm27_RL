"""Scope, dependency closure and immutability for the independent dart scene."""

import json
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import trimesh

from rm27.integration.isolated_red_dart import build, sha256


@pytest.fixture
def scene_config(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'files').mkdir()
    trimesh.creation.box(extents=[.2, .3, .4]).export(source / 'files' / 'station.obj')
    baseline = source / 'v6.xml'
    baseline.write_text('''<mujoco>
      <compiler angle="radian"/>
      <statistic center="0 0 0" extent="4"/>
      <asset/>
      <worldbody>
        <geom name="floor" type="plane" size="20 10 .1"/>
        <geom name="field__visual__hangar_red_column" type="box" size=".2 .2 .5"
              pos="0 0 .5" contype="0" conaffinity="0"/>
        <geom name="test_column_footing" type="box" size=".2 .2 .1"/>
        <geom name="rm27_dock_collision_bottom" type="box" size=".15 .15 .08" pos="0 0 1.1"/>
        <site name="slot_1" pos="0 0 1.18"/>
      </worldbody>
    </mujoco>''')
    field = source / 'field.xml'
    field.write_text('''<mujoco>
      <compiler angle="radian" assetdir="files"/>
      <default>
        <default class="visual"><geom group="2" contype="0" conaffinity="0"/></default>
        <default class="collision"><geom group="3" friction=".8 .01 .001" rgba="0 0 0 0"/></default>
      </default>
      <asset>
        <texture name="station_texture" type="2d" builtin="checker" width="16" height="16"/>
        <material name="station_red" texture="station_texture" rgba="1 .1 .1 1"/>
        <material name="unused_blue" rgba="0 0 1 1"/>
        <mesh name="station_mesh" file="station.obj"/>
      </asset>
      <worldbody>
        <body name="launch_station_red" pos="12.5905 -6.2265 0" euler="0 0 .4">
          <geom name="station_visual" type="mesh" mesh="station_mesh" material="station_red" class="visual"/>
          <geom name="station_collision" type="box" size=".1 .15 .2" class="collision"/>
        </body>
        <body name="launch_station_blue" pos="-12 6 0">
          <geom name="blue_station" type="box" size=".1 .15 .2" material="unused_blue"/>
        </body>
      </worldbody>
    </mujoco>''')
    cfg = dict(source_scene=str(baseline), source_scene_sha256=sha256(baseline),
               field_scene=str(field), field_scene_sha256=sha256(field),
               launcher_body='launch_station_red',
               remove_geom_prefixes=['field__visual__hangar_red_', 'field__collision__hangar_red_'],
               remove_geom_names=['test_column_footing'])
    config = tmp_path / 'config.json'
    config.write_text(json.dumps(cfg))
    return config, source


def test_scene_scope_assets_and_portability(scene_config, tmp_path):
    config, source = scene_config
    original_hashes = {p: sha256(p) for p in source.rglob('*') if p.is_file()}
    output = tmp_path / 'output'
    report = build(output, config)
    assert report['passed'] is True
    assert report['geoms'] == 4
    assert report['station_colliders'] == 1
    assert report['removed_geoms'] == ['field__visual__hangar_red_column', 'test_column_footing']
    assert report['retained_geoms'] == ['floor', 'rm27_dock_collision_bottom']
    for path, digest in original_hashes.items():
        assert sha256(path) == digest
    root = ET.parse(output / 'scene.xml').getroot()
    assert root.find("asset/texture[@name='station_texture']") is not None
    assert root.find("asset/material[@name='unused_blue']") is None
    assert root.find("worldbody/body[@name='launch_station_blue']") is None
    assert root.find("default/default[@class='red_dart_visual']") is not None
    moved = tmp_path / 'relocated'
    shutil.copytree(output, moved)
    shutil.rmtree(source)
    model = mujoco.MjModel.from_xml_path(str(moved / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.site_xpos[model.site('slot_1').id], [0, 0, 1.18])
    np.testing.assert_allclose(data.xpos[model.body('launch_station_red').id], [12.5905, -6.2265, 0])
    assert model.nq == model.nu == 0


def test_changed_source_fails_before_creating_output(scene_config, tmp_path):
    config, source = scene_config
    with (source / 'v6.xml').open('a') as handle:
        handle.write('\n<!-- changed -->')
    output = tmp_path / 'output'
    with pytest.raises(ValueError, match='Source changed'):
        build(output, config)
    assert not output.exists()


def test_existing_output_is_never_overwritten(scene_config, tmp_path):
    config, _ = scene_config
    output = tmp_path / 'existing'
    output.mkdir()
    sentinel = output / 'scene.xml'
    sentinel.write_text('preserve existing scene')
    with pytest.raises(FileExistsError):
        build(output, config)
    assert sentinel.read_text() == 'preserve existing scene'
