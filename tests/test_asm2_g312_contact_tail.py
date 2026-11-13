"""Wrench/load diagnostics only; these tests launch no native experiment."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

path=Path(__file__).resolve().parents[1]/'.agents/runs/asm2-g312/contact_tail.py'
spec=importlib.util.spec_from_file_location('g312_tail',path)
tail=importlib.util.module_from_spec(spec)
spec.loader.exec_module(tail)


def point(group,position,force,normal_force,distance=-1e-6,torque=(0.,0.,0.)):
    return dict(group=group,geom=group,other_geom='plate',position_world_m=position,
                force_on_drone_world_N=force,normal_force_N=normal_force,
                distance_m=distance,torque_on_drone_world_Nm=torque)


class ContactTailTests(unittest.TestCase):
    def test_unloaded_forbidden_intersection_does_not_become_load(self):
        points=[point('body',[0,0,0],[0,0,0],0),point('contact_front',[.01,0,0],[0,0,2],2)]
        result=tail.aggregate(points,np.zeros(3),np.array([0,0,1]),.0001)
        self.assertEqual(result['body']['signed_points'],1)
        self.assertEqual(result['body']['loaded_points'],0)
        self.assertIsNone(result['body']['normal_force_centroid_world_m'])
        self.assertEqual(result['contact_rear']['normal_impulse_Ns'],0)
        np.testing.assert_allclose(result['all']['angular_impulse_about_com_world_Nms'],[0,-2e-6,0])

    def test_force_couple_and_intrinsic_torque_about_nonzero_com(self):
        com=np.array([1.,2.,3.])
        points=[point('contact_front',(com+[.01,0,0]).tolist(),[0,2,0],2,torque=[0,0,.003]),
                point('contact_rear',(com+[-.01,0,0]).tolist(),[0,-2,0],2)]
        result=tail.aggregate(points,com,np.array([0,1,0]),.0001)
        np.testing.assert_allclose(result['all']['impulse_world_Ns'],0,atol=1e-18)
        np.testing.assert_allclose(result['all']['angular_impulse_about_com_world_Nms'],[0,0,4.3e-6])
        np.testing.assert_allclose(result['all']['normal_force_centroid_world_m'],com)

    def test_positive_separation_force_is_retained_separately(self):
        result=tail.aggregate([point('body',[0,0,0],[0,0,1],1,distance=1e-8)],np.zeros(3),np.array([0,0,1]),.0001)
        self.assertEqual(result['body']['signed_points'],0)
        self.assertEqual(result['body']['signed_normal_force_N'],0)
        self.assertEqual(result['body']['positive_separation_loaded_points'],1)
        self.assertEqual(result['body']['normal_impulse_Ns'],.0001)


if __name__=='__main__':
    unittest.main()
