"""Independent geometry diagnostic checks, no native scorer changes."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
import fcl
p=Path(__file__).resolve().parents[1]/'.agents/runs/asm2-g311/audit.py'
spec=importlib.util.spec_from_file_location('g311',p);a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
finish_spec=importlib.util.spec_from_file_location('g311_finish',p.parent/'finish_contact_order.py')
finish=importlib.util.module_from_spec(finish_spec);finish_spec.loader.exec_module(finish)
report_spec=importlib.util.spec_from_file_location('g311_report',p.parent/'report.py')
report=importlib.util.module_from_spec(report_spec);report_spec.loader.exec_module(report)
class AuditTests(unittest.TestCase):
    def test_convex_closed_solid_distance(self):
        v=np.array([[x,y,z] for x in [-.01,.01] for y in [-.01,.01] for z in [-.000025,.000025]])
        shape,_=a.hull(v);x=fcl.CollisionObject(shape);y=fcl.CollisionObject(shape,fcl.Transform(np.array([0.,0.,.0001])))
        d=fcl.distance(x,y,fcl.DistanceRequest(),fcl.DistanceResult());self.assertAlmostEqual(d,.00005,places=12)
        y.setTranslation(np.array([0.,0.,.00004]));self.assertLessEqual(fcl.distance(x,y,fcl.DistanceRequest(),fcl.DistanceResult()),0)
    def test_sat_thin_solid_and_fcl_agree(self):
        v=np.array([[x,y,z] for x in [-.01,.01] for y in [-.01,.01] for z in [-.000025,.000025]])
        poly=a.poly(v)
        self.assertAlmostEqual(a.sat_gap(poly,poly,np.eye(3),np.array([0,0,.0001])),.00005,places=12)
        self.assertAlmostEqual(a.sat_gap(poly,poly,np.eye(3),np.array([0,0,.00004])),-.00001,places=12)
        states=np.zeros((2,13));states[:,3]=1;states[:,2]=[.0001,0]
        gap=lambda r,p:a.sat_gap(poly,poly,r,p)
        result=a.first_interval(None,None,states,.015,gap_fn=gap)
        self.assertLess(abs(np.mean(result)-(.2+a.DT*.5)),1e-7)
    def test_crossing_with_clear_endpoints(self):
        shape=fcl.Box(.01,.01,.01);target=fcl.CollisionObject(shape)
        states=np.zeros((2,13));states[:,3]=1;states[:,0]=[-.03,.03]
        result=a.first_interval(shape,target,states,np.sqrt(3)*.005)
        expected=.2+a.DT/3
        self.assertLessEqual(result[0],expected);self.assertGreaterEqual(result[1],expected-1e-8)
    def test_rotation_interpolation(self):
        x=np.zeros(13);x[3]=1;y=x.copy();y[3:7]=[0,0,0,1]
        r,p=a.interpolate(x,y,.5);np.testing.assert_allclose(r@np.array([1,0,0]),[0,1,0],atol=1e-14)
    def test_signed_root_and_symmetric_uncertainty(self):
        states=np.zeros((3,13));states[:,3]=1;states[:,0]=[20e-6,5e-6,-10e-6]
        gap=lambda r,p:float(p[0])
        root=finish.root_interval(gap,states,.2001332,.2001333)
        self.assertAlmostEqual(root['time_s'],.20013333333333333,places=12)
        self.assertGreater(root['before_gap_m'],0)
        self.assertLess(root['after_gap_m'],0)
        upper=finish.negative_band_witness(gap,states,root,1e-6)
        self.assertAlmostEqual(upper,.20014+1e-10,places=12)
        self.assertIsNone(finish.negative_band_witness(gap,states,root,20e-6))
    def test_overlapping_intervals_stay_unresolved(self):
        self.assertEqual(finish.compare_order([1,2],[1.5,3]),'UNRESOLVED_OVERLAPPING_INTERVALS')
        self.assertEqual(finish.compare_order([1,2],[3,None]),'DESIGNATED_BEFORE_BODY')
        self.assertEqual(finish.compare_order([3,4],[1,2]),'BODY_BEFORE_DESIGNATED')
        self.assertEqual(finish.compare_order(None,[1,2]),'UNRESOLVED_NO_DESIGNATED_EVENT')
    def test_velocity_reconstruction_matches_constant_motion(self):
        x=np.zeros(13);x[3]=1;x[7]=.1;x[12]=.3;y=x.copy();y[0]=.1*a.DT
        q=a.Rotation.from_rotvec([0,0,.3*a.DT]).as_quat();y[3:7]=np.r_[q[3],q[:3]]
        for u in [0,.25,.5,.75,1]:
            r,p=a.interpolate(x,y,u);rr,pp=finish.hermite_pose(x,y,u)
            np.testing.assert_allclose(r,rr,atol=2e-15)
            np.testing.assert_allclose(p,pp,atol=2e-15)
    def test_analytic_sweep_catches_interior_rotational_minimum(self):
        x=np.zeros(13);y=x.copy()
        for state,angle in [(x,-np.pi/2),(y,np.pi/2)]:
            q=a.Rotation.from_rotvec([0,0,angle]).as_quat();state[3:7]=np.r_[q[3],q[:3]]
        value=report.swept_plane_minimum(np.array([[1.,0,0]]),x,y,np.array([-1.,0,0]))
        self.assertAlmostEqual(value,-1,places=12)
    def test_analytic_sweep_with_translation_matches_dense_sampling(self):
        rng=np.random.default_rng(311);v=rng.normal(size=(8,3))*.1;x=np.zeros(13);x[3]=1;y=x.copy();y[:3]=[.012,-.008,.004]
        q=a.Rotation.from_rotvec([.3,-.2,.1]).as_quat();y[3:7]=np.r_[q[3],q[:3]];normal=np.array([.3,.4,np.sqrt(.75)])
        exact=report.swept_plane_minimum(v,x,y,normal)
        sampled=min(np.min((v@r.T+p)@normal) for r,p in [a.interpolate(x,y,u) for u in np.linspace(0,1,2001)])
        self.assertLessEqual(exact,sampled+1e-13);self.assertLess(sampled-exact,1e-7)
    def test_wrench_and_region_force(self):
        rows=[dict(position_world_m=[1,0,0],impulse_on_drone_world_Ns=[0,0,2],normal_impulse_N_s=2),dict(position_world_m=[-1,0,0],impulse_on_drone_world_Ns=[0,0,2],normal_impulse_N_s=2)]
        j,l,c=a.wrench(rows,np.zeros(3));np.testing.assert_array_equal(j,[0,0,4]);np.testing.assert_array_equal(l,[0,0,0]);np.testing.assert_array_equal(c,[0,0,0])
if __name__=='__main__':unittest.main()
