import numpy as np
import pytest

from rm27.perception.sitl.dock_tags import TagObservation
from rm27.perception.sitl.return_camera import slot_relative_position


def observation(*, valid=True):
    return TagObservation(21, 100, (), (-.01, .02, .5), tuple(map(tuple, np.eye(3))),
                          0., valid, 'valid' if valid else 'stale')


def test_return_position_uses_camera_mount_and_px4_attitude():
    # Level NED/FRD attitude; tag 10 mm left, 20 mm down in the image.
    np.testing.assert_allclose(slot_relative_position(observation(), [1, 0, 0, 0],
                              [0, 0, -.025]), [.01, .02], atol=1e-9)
    # PX4 90-degree NED yaw rotates the FLU displacement into NWU.
    yaw_ned_90=[np.sqrt(.5), 0, 0, np.sqrt(.5)]
    np.testing.assert_allclose(slot_relative_position(observation(), yaw_ned_90,
                              [0, 0, -.025]), [.02, -.01], atol=1e-9)


def test_return_position_rejects_unsafe_detection_and_attitude():
    with pytest.raises(ValueError, match='unsafe'):
        slot_relative_position(observation(valid=False), [1, 0, 0, 0], [0, 0, -.025])
    with pytest.raises(ValueError, match='attitude'):
        slot_relative_position(observation(), [0, 0, 0, 0], [0, 0, -.025])
