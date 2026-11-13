"""Perturbation signs and invariants in an oblique registered-style frame."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from rm27_asm2.cross_track_validation import perturb


class CrossTrackTests(unittest.TestCase):
    def test_oblique_frame_sign_and_unchanged_components(self):
        r=Rotation.from_euler('xyz',[.3,.5,-.2]).as_matrix()
        x=np.arange(13,dtype=float)/20;x[3:7]=[1,0,0,0]
        for sign in [-1,1]:
            y,check=perturb(x,r,sign)
            np.testing.assert_allclose((y[:3]-x[:3])@r,[0,sign*.0005,0],atol=1e-15,rtol=0)
            np.testing.assert_allclose((y[7:10]-x[7:10])@r,[0,sign*.005,0],atol=1e-15,rtol=0)
            np.testing.assert_array_equal(y[3:7],x[3:7]);np.testing.assert_array_equal(y[10:],x[10:])
        minus,_=perturb(x,r,-1);plus,_=perturb(x,r,1)
        np.testing.assert_allclose((minus+plus)/2,x,atol=1e-15,rtol=0)

    def test_no_nominal_or_extra_magnitude_variant(self):
        for sign in [0,2]:
            with self.assertRaises(ValueError):perturb(np.zeros(13),np.eye(3),sign)


if __name__=='__main__':unittest.main()
