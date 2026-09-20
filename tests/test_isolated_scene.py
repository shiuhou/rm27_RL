import numpy as np
import trimesh

from rm27.integration.isolated_scene import contained_components


def test_selection_keeps_whole_components_without_clipping():
    inside = trimesh.creation.box(extents=[.4, .4, .4])
    crossing = trimesh.creation.box(extents=[1, 1, 1])
    crossing.apply_translation([1, 0, 0])
    source = trimesh.util.concatenate([inside, crossing])
    parts = contained_components(source.vertices, source.faces, [[-1]*3, [1]*3])
    assert len(parts) == 1
    assert len(parts[0].faces) == len(inside.faces)
    np.testing.assert_allclose(parts[0].bounds, inside.bounds)


def test_open_surface_is_not_repaired():
    vertices = [[0, 0, 0], [.1, 0, 0], [0, .1, 0]]
    parts = contained_components(vertices, [[0, 1, 2]], [[-1]*3, [1]*3])
    assert len(parts) == 1
    assert len(parts[0].faces) == 1
