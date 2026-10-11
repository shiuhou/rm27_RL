import numpy as np

from rm27.experiments.target_region_audit import segment_triangle_distance, outside_patch_triangles


def test_capsule_axis_crossing_face_and_near_edge_are_not_missed_by_endpoint_checks():
    triangles=np.array([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]])
    distance,point=segment_triangle_distance([.2,.2,-1],[.2,.2,1],triangles)
    assert distance[0]==0
    np.testing.assert_allclose(point[0],[.2,.2,0.])
    distance,point=segment_triangle_distance([.5,-.1,-1],[.5,-.1,1],triangles)
    np.testing.assert_allclose(distance,[.1])
    np.testing.assert_allclose(point,[[.5,0,0]])


def test_degenerate_axis_is_a_point_and_empty_triangle_set_stays_empty():
    triangles=np.array([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]])
    distance,_=segment_triangle_distance([.2,.2,1],[.2,.2,1],triangles)
    np.testing.assert_allclose(distance,[1.])
    distance,point=segment_triangle_distance([0,0,0],[0,0,1],np.empty((0,3,3)))
    assert distance.shape==(0,) and point.shape==(0,3)


def test_guard_overlap_outside_patch_is_detected_even_when_nearest_point_is_inside():
    triangle=np.array([[[-1.,-1.,0.],[1.,-1.,0.],[0.,1.,0.]]])
    start=np.array([.095,0,0])
    original,_=segment_triangle_distance(start,start,triangle)
    assert original[0]<1e-15  # The old nearest-witness check would allow this.
    forbidden=outside_patch_triangles(triangle,np.zeros(3),np.eye(3),[.1,.1,.01])
    distances,points=segment_triangle_distance(start,start,forbidden)
    np.testing.assert_allclose(distances.min(),.005)
    assert distances.min()<.02  # A 2 cm guard crosses the forbidden boundary.
    inside=np.array([[[-.01,0,0],[.01,0,0],[0,.01,0]]])
    assert outside_patch_triangles(inside,np.zeros(3),np.eye(3),[.1,.1,.01]).shape==(0,3,3)
