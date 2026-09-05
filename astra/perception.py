"""Pixel-only baseline for an isolated designated red spherical test fixture.

This deliberately abstains on multiple plausible red components. It is not a
general RoboMaster detector. No fixture coordinate or renderer state is accepted.
"""
import cv2
import numpy as np


def detect_red_sphere(rgb, hfov_deg):
    height, width = rgb.shape[:2]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = (((hsv[:, :, 0] < 10) | (hsv[:, :, 0] > 170)) & (hsv[:, :, 1] > 100) & (hsv[:, :, 2] > 50)).astype(np.uint8)*255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    candidates = [c for c in contours if cv2.contourArea(c) >= 8 and len(c) >= 8]
    if len(candidates) != 1:
        return None
    contour = candidates[0][:, 0, :].astype(float)
    if ((contour <= 0).any() or (contour[:, 0] >= width-1).any() or (contour[:, 1] >= height-1).any()):
        return None
    perimeter = cv2.arcLength(candidates[0], True)
    circularity = 4*np.pi*cv2.contourArea(candidates[0])/max(perimeter**2, 1e-12)
    if circularity < .65:
        return None
    f = width/(2*np.tan(np.deg2rad(hfov_deg/2)))
    rays = np.column_stack([(contour-[width/2, height/2])/f, np.ones(len(contour))])
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    # A sphere's silhouette rays lie on a cone; their unit directions lie in a
    # plane. Fit its normal rather than treating an off-axis ellipse as a circle.
    _, _, vh = np.linalg.svd(rays-rays.mean(axis=0), full_matrices=False)
    axis = vh[-1]
    if axis[2] < 0:
        axis *= -1
    angle = float(np.arccos(np.clip(np.mean(rays@axis), -1, 1)))
    if angle < 1e-5:
        return None
    return dict(uv_px=(f*axis[:2]/axis[2]).tolist(), angular_radius_rad=angle,
                circularity=float(circularity), contour_pixels=len(contour))
