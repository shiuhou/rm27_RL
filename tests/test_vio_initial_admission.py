from dataclasses import replace

import numpy as np

from test_px4_odometry_clocks import estimate
from rm27.experiments.vio_initial_admission import initial_consistent


def test_initial_gate_uses_fresh_navigation_velocity_and_observed_covariance():
    e=estimate()
    e=replace(e,velocity=(0.,0.,0.),orientation_wxyz=(1.,0.,0.,0.),
        quality=dict(e.quality,pose_covariance=(np.eye(6)*.0025).tolist()))
    history=[(1.,np.zeros(3),[1.,0.,0.,0.],np.zeros(3))]
    assert initial_consistent(e,history)==(True,None)
    assert not initial_consistent(replace(e,velocity=(1.,0.,0.)),history)[0]
    assert not initial_consistent(e,[(.8,*history[0][1:])])[0]
    assert not initial_consistent(e,[])[0]
    noisy=replace(e,quality=dict(e.quality,pose_covariance=(np.eye(6)*.1).tolist()))
    assert not initial_consistent(noisy,history)[0]
    # Initial ascent is permitted; only horizontal agreement is used here.
    assert initial_consistent(replace(e,velocity=(0.,0.,.7)),history)[0]
