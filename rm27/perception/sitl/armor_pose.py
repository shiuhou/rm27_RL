"""Metric armor pose from a known rectangular patch and current RGB pixels.

Rectangle dimensions and pinhole intrinsics are explicit simulation assumptions.
This detector has no simulator pose, contact or target identity input.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from .dataset import intrinsics
from .image_target_detector import DetectorConfig, ImageTargetDetector, _mask


@dataclass(frozen=True)
class ArmorPose:
    source_time_us: int
    valid: bool
    reason: str
    translation_camera_m: tuple = ()
    normal_camera: tuple = ()
    corners_px: tuple = ()
    reprojection_rmse_px: float | None = None
    position_std_m: float | None = None
    metric_scale_source: str = 'configured simulation armor rectangle'


def ordered_quad(points):
    p=np.asarray(points,dtype=np.float32).reshape(4,2)
    # Clockwise order in image coordinates, beginning at the upper left.
    center=p.mean(0);angles=np.arctan2(p[:,1]-center[1],p[:,0]-center[0])
    p=p[np.argsort(angles)]
    p=np.roll(p,-int(np.argmin(p.sum(1))),axis=0)
    return p


class ArmorPoseDetector:
    def __init__(self, width_m=.09, height_m=.055, *, fovy_deg=60., maximum_age_us=200_000):
        if not np.isfinite([width_m,height_m]).all() or min(width_m,height_m)<=0:
            raise ValueError('known positive metric armor dimensions required')
        self.width,self.height=width_m,height_m
        self.config=DetectorConfig(fovy_deg=fovy_deg,max_measurement_age_us=maximum_age_us)
        self.bearing=ImageTargetDetector(self.config)

    def detect(self, frame, *, now_us):
        bearing=self.bearing.detect(frame,now_us=now_us)
        invalid=lambda reason:ArmorPose(frame.timestamp_us,False,reason)
        if not bearing.safe_for_control:return invalid('stale, missing or ambiguous armor/marker association')
        fx,fy,cx,cy=intrinsics(frame.width,frame.height,self.config.fovy_deg)
        expected=np.array([cx+fx*math.tan(bearing.yaw_rad),cy-fy*math.tan(bearing.pitch_rad)])
        hsv=cv2.cvtColor(frame.image_rgb,cv2.COLOR_RGB2HSV)
        mask=_mask(hsv,self.config.blue_hue)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        candidates=[c for c in contours if cv2.pointPolygonTest(c,tuple(expected),False)>=0]
        if len(candidates)!=1:return invalid('armor contour association')
        contour=candidates[0]
        polygon=cv2.approxPolyDP(contour,.02*cv2.arcLength(contour,True),True)
        if len(polygon)!=4 or not cv2.isContourConvex(polygon):return invalid('armor is not a visible quadrilateral')
        pixels=ordered_quad(polygon)
        if np.any(pixels<2) or np.any(pixels[:,0]>frame.width-3) or np.any(pixels[:,1]>frame.height-3):
            return invalid('armor clipped by image boundary')
        w,h=self.width/2,self.height/2
        objects=np.array([[-w,h,0],[w,h,0],[w,-h,0],[-w,-h,0]],np.float64)
        calibration=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
        solutions=[]
        for method in (cv2.SOLVEPNP_IPPE,cv2.SOLVEPNP_ITERATIVE):
            ok,rvec,tvec=cv2.solvePnP(objects,pixels.astype(np.float64),calibration,np.zeros(4),flags=method)
            if not ok or not np.isfinite(np.r_[rvec.ravel(),tvec.ravel()]).all() or tvec[2,0]<=0:continue
            rotation=cv2.Rodrigues(rvec)[0];normal=rotation[:,2]
            if float(normal@-tvec[:,0])<=0:continue
            projected,_=cv2.projectPoints(objects,rvec,tvec,calibration,np.zeros(4))
            error=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-pixels)**2,axis=1))))
            if error<=1.25:solutions.append((error,tvec[:,0],normal))
        if not solutions:return invalid('metric pose or reprojection rejected')
        error,translation,normal=min(solutions,key=lambda s:s[0])
        span=float(min(np.linalg.norm(pixels[1]-pixels[0]),np.linalg.norm(pixels[3]-pixels[0])))
        if span<8:return invalid('insufficient metric pose resolution')
        return ArmorPose(frame.timestamp_us,True,'valid',tuple(map(float,translation)),tuple(map(float,normal)),
            tuple(tuple(map(float,p)) for p in pixels),error,max(.003,float(np.linalg.norm(translation))/span))
