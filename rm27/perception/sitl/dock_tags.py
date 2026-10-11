"""Pixel-only dock/slot tag poses with explicit simulated calibration."""
from dataclasses import dataclass

import cv2
import numpy as np

from .dataset import intrinsics


@dataclass(frozen=True)
class TagObservation:
    tag_id: int
    source_time_us: int
    corners_px: tuple
    translation_camera_m: tuple
    rotation_camera_tag: tuple
    reprojection_rmse_px: float
    safe_for_control: bool
    reason: str


class DockTagDetector:
    def __init__(self, tag_sizes, *, fovy_deg=60., maximum_age_us=200_000, maximum_reprojection_px=1.):
        if not tag_sizes or any(type(k) is not int or not np.isfinite(v) or v<=0 for k,v in tag_sizes.items()):
            raise ValueError('explicit metric tag sizes required')
        self.sizes=tag_sizes;self.fovy=fovy_deg;self.max_age=maximum_age_us;self.max_error=maximum_reprojection_px
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters())

    def detect(self, frame, *, now_us):
        age=now_us-frame.timestamp_us
        if age<0:raise ValueError('future tag image')
        gray=cv2.cvtColor(frame.image_rgb,cv2.COLOR_RGB2GRAY)
        corners,ids,_=self.detector.detectMarkers(gray)
        if ids is None:return []
        ids=np.asarray(ids).reshape(-1)
        fx,fy,cx,cy=intrinsics(frame.width,frame.height,self.fovy)
        calibration=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
        output=[]
        for points,tag_id in zip(corners,ids):
            tag_id=int(tag_id)
            if tag_id not in self.sizes:continue
            half=self.sizes[tag_id]/2
            object_points=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]])
            pixels=np.asarray(points,dtype=float).reshape(4,2)
            # IPPE can degenerate for a perfectly centered frontal square.
            # Select the positive-depth solution by measured reprojection;
            # iterative PnP handles that singular fixture without truth input.
            candidates=[]
            for method in [cv2.SOLVEPNP_IPPE_SQUARE,cv2.SOLVEPNP_ITERATIVE]:
                ok,rvec,tvec=cv2.solvePnP(object_points,pixels,calibration,np.zeros(4),flags=method)
                if not ok or not np.isfinite(tvec).all() or tvec[2,0]<=0:continue
                projected,_=cv2.projectPoints(object_points,rvec,tvec,calibration,np.zeros(4))
                error=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-pixels)**2,axis=1))))
                candidates.append((error,rvec,tvec))
            if not candidates:continue
            error,rvec,tvec=min(candidates,key=lambda c:c[0])
            rotation=cv2.Rodrigues(rvec)[0]
            duplicate=np.count_nonzero(ids==tag_id)>1
            valid=bool(np.isfinite(tvec).all() and np.isfinite(rotation).all() and tvec[2,0]>0
                and age<=self.max_age and error<=self.max_error and not duplicate)
            reason='valid' if valid else 'duplicate ID' if duplicate else 'stale' if age>self.max_age else 'invalid pose/reprojection'
            output.append(TagObservation(tag_id,frame.timestamp_us,tuple(map(tuple,pixels)),
                tuple(tvec[:,0]),tuple(map(tuple,rotation)),error,valid,reason))
        return output
