"""Development first-registration guard with 3D velocity and temporal evidence."""
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from rm27.experiments.vio_initial_admission import initial_consistent


class StableInitialAdmission:
    def __init__(self, *, maximum_vertical_disagreement=.15, minimum_duration_s=.2,
                 minimum_samples=5):
        self.maximum_vertical_disagreement=maximum_vertical_disagreement
        self.minimum_duration_s=minimum_duration_s
        self.minimum_samples=minimum_samples
        self.consistent=deque(maxlen=50)
        self.localization_epoch=None

    def __call__(self,estimate,history):
        if estimate.localization_epoch!=self.localization_epoch:
            self.consistent.clear()
            self.localization_epoch=estimate.localization_epoch
        accepted,reason=initial_consistent(estimate,history)
        if not accepted:
            self.consistent.clear()
            return False,reason
        source=estimate.source_time.raw_value*1e-9
        timestamp,_,q,velocity=min(history,key=lambda r:abs(r[0]-source))
        parent=(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()@
            np.diag([1.,-1.,-1.])@Rotation.from_quat(np.asarray(estimate.orientation_wxyz)[[1,2,3,0]]).as_matrix().T)
        native_velocity=parent@np.asarray(estimate.velocity)
        if abs(native_velocity[2]-velocity[2])>self.maximum_vertical_disagreement:
            self.consistent.clear()
            return False,'initial vertical VIO/navigation velocity disagreement'
        if self.consistent and (source<=self.consistent[-1] or source-self.consistent[-1]>.15):
            self.consistent.clear()
        self.consistent.append(source)
        if len(self.consistent)<self.minimum_samples or source-self.consistent[0]<self.minimum_duration_s-1e-9:
            return False,'initial VIO evidence stability window incomplete'
        return True,None
