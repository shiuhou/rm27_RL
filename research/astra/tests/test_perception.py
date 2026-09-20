import numpy as np
import cv2
from research.astra.perception import detect_red_sphere


def test_pixel_detector_needs_visible_pixels():
    assert detect_red_sphere(np.zeros((480, 640, 3), np.uint8), 90) is None


def test_pixel_detector_estimates_spherical_angular_extent():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(image, (320, 240), 32, (255, 0, 0), -1)  # RGB
    result = detect_red_sphere(image, 90)
    assert result is not None
    assert np.linalg.norm(result['uv_px']) < 1
    assert abs(result['angular_radius_rad']-np.arctan(.1)) < .005


def test_pixel_detector_rejects_clipped_contour_and_ambiguity():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(image, (0, 240), 32, (255, 0, 0), -1)
    assert detect_red_sphere(image, 90) is None
    image[:] = 0
    cv2.circle(image, (200, 240), 32, (255, 0, 0), -1)
    cv2.circle(image, (400, 240), 32, (255, 0, 0), -1)
    assert detect_red_sphere(image, 90) is None
