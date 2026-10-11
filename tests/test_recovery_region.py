import numpy as np
from scipy.spatial.transform import Rotation

from rm27.experiments.recovery_region import displacement_bound,subdivide_capsule
from rm27.experiments.recovery_corridor import prior_corridor_region
from rm27.experiments.target_region_audit import segment_triangle_distance


def test_displacement_proof_covers_points_across_translation_and_rotation_cell():
    rng=np.random.default_rng(417)
    centre=np.array([25.,-35.,45.])
    half=np.array([.002,.003,.001,10.,15.,20.])
    point=np.array([.12,-.05,.03]);radius=np.linalg.norm(point)
    origin=Rotation.from_euler('xyz',centre,degrees=True).apply(point)
    maximum=displacement_bound(half,radius)
    for _ in range(500):
        offset=rng.uniform(-1,1,6)*half
        actual=Rotation.from_euler('xyz',centre+offset[3:],degrees=True).apply(point)+offset[:3]
        assert np.linalg.norm(actual-origin)<=maximum
    # A rotation-free cell reduces to the exact translational corner bound.
    np.testing.assert_allclose(displacement_bound([.003,.004,0,0,0,0],.2),.005)


def test_short_capsule_union_preserves_all_original_triangle_distances():
    start=np.array([-.1,0.,0.]);end=np.array([.1,0.,0.])
    triangles=np.array([[[0.,-.01,-.1],[0.,.01,-.1],[0.,0.,.1]],
        [[.101,0.,0.],[.101,1.,0.],[.101,0.,1.]]])
    original,_=segment_triangle_distance(start,end,triangles)
    shorter=np.array([segment_triangle_distance(a,b,triangles)[0] for a,b in subdivide_capsule(start,end,8)])
    np.testing.assert_allclose(shorter.min(axis=0),original,atol=1e-15)


def test_reusing_continuous_region_requires_verified_matching_geometry(tmp_path):
    import hashlib,json
    import rm27.experiments.recovery_corridor as corridor
    box=np.array([[0,.2],[-.1,.1],[-.1,.1],[-7,7],[-7,7],[42,48]])
    settings=dict(armor_geom='armor',armor_site='site',contact_point_body_flu_m=[0,0,0],
        allowed_surface_dimensions_m=[.09,.055],backing_surface_tolerance_m=.01,
        penetration_tolerance_m=.0002,capsule_segments=8,pose_regions=[box.tolist()])
    (tmp_path/'region_dependency.py').write_bytes(corridor.DEPENDENCY_BYTES)
    (tmp_path/'surface_dependency.py').write_bytes(corridor.SURFACE_BYTES)
    prior=dict(passed=True,scene_sha256='scene',asset_sha256={'mesh':'hash'},configuration=settings,
        regions=[dict(passed=True,region=0)],
        dependency_sha256=hashlib.sha256(corridor.DEPENDENCY_BYTES).hexdigest(),
        surface_dependency_sha256=hashlib.sha256(corridor.SURFACE_BYTES).hexdigest())
    path=tmp_path/'summary.json';path.write_text(json.dumps(prior))
    assert prior_corridor_region(path,settings,box,'scene',{'mesh':'hash'})
    assert not prior_corridor_region(path,settings,box,'changed-scene',{'mesh':'hash'})
    assert not prior_corridor_region(path,settings,box,'scene',{'mesh':'changed'})
    assert not prior_corridor_region(path,dict(settings,penetration_tolerance_m=.01),box,'scene',{'mesh':'hash'})
    larger=box.copy();larger[0,1]=.3
    assert not prior_corridor_region(path,settings,larger,'scene',{'mesh':'hash'})
    (tmp_path/'surface_dependency.py').write_text('modified geometry')
    assert not prior_corridor_region(path,settings,box,'scene',{'mesh':'hash'})
