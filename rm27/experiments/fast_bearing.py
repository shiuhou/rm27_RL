"""Equivalent image bearing detection using component statistics and one ROI.

Avoid allocating/scanning one full image mask for every small noisy component.
Thresholds, association, ambiguity, confidence and source-age rules are retained.
"""
import math

import cv2
import numpy as np

from ..perception.guidance import TargetEstimate
from ..perception.sitl.mujoco_camera import CameraFrame
from ..perception.sitl.image_target_detector import _mask


def detect(self,frame,*,now_us=None):
    if not isinstance(frame,CameraFrame):
        raise ValueError('timestamped camera frame required by the development detector')
    image=frame.image_rgb;hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV);c=self.config
    kernel=np.ones((3,3),np.uint8)
    blue=cv2.morphologyEx(_mask(hsv,c.blue_hue),cv2.MORPH_OPEN,kernel)
    green=cv2.morphologyEx(_mask(hsv,c.green_hue),cv2.MORPH_OPEN,kernel)
    count,labels,stats,centroids=cv2.connectedComponentsWithStats(blue,connectivity=8)
    green_count,_,green_stats,green_centroids=cv2.connectedComponentsWithStats(green,connectivity=8)
    blues=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=c.minimum_area_px]
    greens=[i for i in range(1,green_count) if green_stats[i,cv2.CC_STAT_AREA]>=c.minimum_area_px]
    candidates=[]
    for i in blues:
        width,height=stats[i,cv2.CC_STAT_WIDTH],stats[i,cv2.CC_STAT_HEIGHT]
        if not .25<=width/height<=6 or stats[i,cv2.CC_STAT_AREA]/(width*height)<.4:continue
        for j in greens:
            dx=abs(green_centroids[j,0]-centroids[i,0]);dy=centroids[i,1]-green_centroids[j,1]
            if dx<=1.5*width and .2*height<=dy<=5*height:
                candidates.append(i);break
    associated=len(candidates)==1
    selected=max(candidates or blues,key=lambda i:stats[i,cv2.CC_STAT_AREA],default=None)
    age=self._age(now_us,frame.timestamp_us)
    if selected is None:return TargetEstimate(timestamp_us=frame.timestamp_us,measurement_age_us=age,valid=False)
    x,y,width,height,area=stats[selected]
    saturation=float(hsv[y:y+height,x:x+width,1][labels[y:y+height,x:x+width]==selected].mean())/255.
    u,v=centroids[selected];f=(frame.height/2)/math.tan(math.radians(c.fovy_deg)/2)
    yaw=math.atan2(float(u-(frame.width-1)/2),f);pitch=-math.atan2(float(v-(frame.height-1)/2),f)
    confidence=float(np.clip(area/c.confidence_area_px*saturation,0.,1.))
    valid=bool(math.isfinite(yaw) and math.isfinite(pitch) and area>=c.minimum_area_px)
    safe=bool(valid and (associated or not c.require_marker) and confidence>=c.minimum_confidence and age<=c.max_measurement_age_us)
    return TargetEstimate(timestamp_us=frame.timestamp_us,measurement_age_us=age,valid=valid,
        safe_for_control=safe,predicted=False,yaw_rad=yaw,pitch_rad=pitch,confidence=confidence)
