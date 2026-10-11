"""Distinct acquisition and native monotonic publication clocks stay distinct."""
from dataclasses import replace

import numpy as np
import pytest

from rm27.autopilot.px4.odometry import OdometryAdapter, Registration
from rm27.perception.vision.localization import LocalizationEstimate,TimestampEvidence,TrackingState,ScaleState


def estimate():
    return LocalizationEstimate(
        source_time=TimestampEvidence(1_000_000_000,'ns','sim','IMU_STATE','VERIFIED'),
        publish_time=TimestampEvidence(9_000_000_000,'ns','native_monotonic','NATIVE_PUBLISH','VERIFIED'),
        parent_frame='vio',child_frame='imu',translation=(0.,0.,0.),orientation_wxyz=(1.,0.,0.,0.),
        tracking_state=TrackingState.TRACKING,valid=True,initialized=True,relocalizing=False,
        localization_epoch=0,scale_state=ScaleState.METRIC,translation_unit='m',
        velocity_unit='m/s',velocity=(0.,0.,0.),velocity_frame='vio',
        quality=dict(flight_qualified=True,pose_covariance=np.eye(6).tolist(),velocity_covariance=np.eye(6).tolist()))


def test_two_clocks_need_separate_current_times():
    r=Registration('vio','imu','sim',np.eye(3),(0.,0.,0.),np.eye(3),'native_monotonic')
    e=estimate();a=OdometryAdapter(r)
    with pytest.raises(ValueError,match='current time'):a.packet(e,1_020_000_000)
    with pytest.raises(ValueError,match='publication'):a.packet(e,1_020_000_000,publication_now_ns=9_500_000_000)
    with pytest.raises(ValueError,match='publication'):a.packet(e,1_020_000_000,publication_now_ns=8_990_000_000)
    p=a.packet(e,1_020_000_000,publication_now_ns=9_020_000_000)
    assert p.time_usec==1_000_000


def test_unregistered_publication_domain_is_rejected():
    e=estimate();a=OdometryAdapter(Registration('vio','imu','sim',np.eye(3),(0.,0.,0.),np.eye(3)))
    with pytest.raises(ValueError,match='registered clock'):a.packet(e,1_020_000_000)
