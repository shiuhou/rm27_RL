"""Development initial VIO admission against fresh independent flow navigation.

Only the first rigid registration is gated. No truth, ongoing pose alignment
or camera restamping is available to this boundary.
"""
from dataclasses import asdict
import json

import numpy as np
from scipy.spatial.transform import Rotation


def initial_consistent(estimate,history,*,maximum_xy_velocity_disagreement=.2,
                       maximum_position_variance=.01):
    if not estimate.valid:return False,'native tracking invalid'
    if not history:return False,'no valid PX4 flow navigation'
    source=estimate.source_time.raw_value*1e-9
    time,position,quaternion,velocity=min(history,key=lambda row:abs(row[0]-source))
    if abs(time-source)>.05:return False,'initial navigation timestamp mismatch'
    q=np.asarray(estimate.orientation_wxyz)
    parent=(Rotation.from_quat(np.asarray(quaternion)[[1,2,3,0]]).as_matrix()@
        np.diag([1.,-1.,-1.])@Rotation.from_quat(q[[1,2,3,0]]).as_matrix().T)
    v=parent@np.asarray(estimate.velocity)
    if not np.isfinite(v).all() or np.linalg.norm(v[:2]-np.asarray(velocity)[:2])>maximum_xy_velocity_disagreement:
        return False,'initial horizontal VIO/flow velocity disagreement'
    covariance=np.asarray(estimate.quality['pose_covariance'])[:3,:3]
    if np.linalg.eigvalsh(covariance).max()>maximum_position_variance:
        return False,'initial VIO position covariance above declared bound'
    return True,None


class InitialContract:
    def __init__(self,contract,vio,stream,*,admission=None):
        self.contract,self.vio,self.stream=contract,vio,stream
        self.admission=admission
        self.admitted=False;self.history=[]

    def estimate(self,row):
        estimate=self.contract.estimate(row)
        if self.admitted or not estimate.valid:return estimate
        check=self.admission if self.admission is not None else initial_consistent
        accepted,reason=check(estimate,self.history)
        self.stream.write(json.dumps(dict(source_time_ns=estimate.source_time.raw_value,
            admitted=accepted,reason=reason,estimate=asdict(estimate)),allow_nan=False)+'\n')
        if not accepted:raise ValueError(reason)
        self.admitted=True
        return estimate
