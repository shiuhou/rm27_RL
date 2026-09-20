import json
import math

import pytest

from rm27.perception.gazebo.truth_adapter import TruthTargetAdapter, vision_yaw_rad
from rm27.perception.guidance import (
    GuidanceCommand,
    GuidanceState,
    TargetEstimate,
    UAVGuidance,
)
from rm27.perception.sitl.mavlink_adapter import GuidanceMavlinkAdapter
from rm27.perception.sitl.synthetic_target import SyntheticTargetProducer


def estimate(yaw=0.0, **overrides):
    values = dict(
        timestamp_us=1_000_000,
        measurement_age_us=20_000,
        valid=True,
        safe_for_control=True,
        predicted=False,
        yaw_rad=yaw,
        pitch_rad=0.0,
        confidence=.9,
    )
    values.update(overrides)
    return TargetEstimate(**values)


def test_guidance_state_and_safety_contract():
    guidance = UAVGuidance()
    out = guidance.update(estimate(yaw=math.radians(10)), 1_000_000)
    assert out.state is GuidanceState.ALIGN
    assert out.command.yaw_rate_rad_s > 0
    out = guidance.update(estimate(yaw=0.0), 1_000_000)
    assert out.state is GuidanceState.APPROACH
    assert out.command.vx_body_mps == pytest.approx(.30)
    out = guidance.update(estimate(valid=False), 1_033_000)
    assert out.state is GuidanceState.SAFE_HOLD
    assert out.command.vx_body_mps == 0 and not out.command.valid


def test_schema_adapter_accepts_json_v2_subset():
    line = json.dumps({
        "schema_version": 2, "timestamp_us": 123, "measurement_age_us": 20,
        "valid": True, "safe_for_control": True, "predicted": False,
        "yaw_rad": .1, "pitch_rad": -.02, "confidence": .8,
    })
    parsed = TargetEstimate.from_json(line)
    assert parsed.schema_version == 2 and parsed.confidence == pytest.approx(.8)
    with pytest.raises(ValueError, match="schema_version"):
        TargetEstimate.from_json(line.replace('"schema_version": 2', '"schema_version": 9'))


def test_synthetic_producer_and_mavlink_boundary():
    producer = SyntheticTargetProducer(
        [("bearing", 1.0, {"yaw_deg": 10}), ("dropout", .5, {})], 0
    )
    assert producer.update(0).yaw_rad == pytest.approx(math.radians(10))
    assert producer.update(1_100_000) is None

    class Mav:
        def __init__(self): self.sent = []
        def set_position_target_local_ned_send(self, *args): self.sent.append(args)
    class Master:
        target_system, target_component = 1, 1
        def __init__(self): self.mav = Mav()
    master = Master()
    GuidanceMavlinkAdapter(master).send(GuidanceCommand(valid=False))
    args = master.mav.sent[0]
    assert (args[8], args[9], args[10], args[15]) == (0.0, 0.0, 0.0, 0.0)


def test_truth_adapter_only_produces_schema_estimate():
    class Source:
        def poses(self):
            return {
                "iris_with_gimbal": {"pos": (0, 0, 1.5), "quat": (0, 0, 0, 1)},
                "rm27_target": {"pos": (1, 5, .25), "quat": (0, 0, 0, 1)},
            }
    assert vision_yaw_rad((0, 0, 1.5), (0, 0, 0, 1), (1, 5, .25)) < 0
    parsed = TruthTargetAdapter(Source()).update(1_000_000)
    assert parsed.valid and parsed.safe_for_control
