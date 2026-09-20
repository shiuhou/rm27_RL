import json
import struct

import numpy as np
import pytest

from rm27.autopilot.bridge.actuators import AP_ROTORS, pwm_to_thrust
from rm27.autopilot.bridge.bridge import Bridge
from rm27.autopilot.bridge.frames import attitude_to_ap, to_ap
from rm27.autopilot.bridge.launch import EXPECTED_PARAMS, verify_parameters
from rm27.autopilot.bridge.plant_adapter import Plant
from rm27.autopilot.bridge.protocol import ProtocolError, decode_servos, encode_state


def packet(count, pwm=1500, rate=500):
    return struct.pack("<HHI16H", 18458, rate, count, *([pwm] * 16))


def test_migrated_wire_contract_and_frames():
    decoded = decode_servos(packet(12))
    assert decoded.frame_count == 12 and len(decoded.pwm) == 16
    assert AP_ROTORS == ("rotor_fr", "rotor_rl", "rotor_fl", "rotor_rr")
    np.testing.assert_allclose(pwm_to_thrust([1100, 1200, 1300, 1400]), [2.4, .8, 3.2, 1.6])
    np.testing.assert_array_equal(to_ap([0, 1, 0]), [0, -1, 0])
    np.testing.assert_allclose(abs(np.dot(attitude_to_ap([1, 0, 0, 0]), [1, 0, 0, 0])), 1)


def test_migrated_bridge_rejects_invalid_rate_and_duplicates():
    bridge = Bridge(Plant())
    first, row = bridge.process(packet(0))
    assert json.loads(first)["timestamp"] == .002 and row["epoch"] == 0
    assert bridge.process(packet(0)) == (first, None)
    with pytest.raises(ProtocolError):
        bridge.process(packet(1, rate=400))


def test_migrated_plant_advances_and_reports_state():
    plant = Plant()
    state = plant.step([1500] * 4)
    assert state["timestamp"] == pytest.approx(.002)
    assert set(plant.truth()) >= {"position_nwu", "actual_thrust_n"}


def test_migrated_launch_parameter_contract():
    assert verify_parameters(EXPECTED_PARAMS)
    changed = EXPECTED_PARAMS.copy()
    changed["ARMING_SKIPCHK"] = 1
    assert not verify_parameters(changed)


def test_migrated_bridge_rejects_bad_packet():
    with pytest.raises(ProtocolError):
        decode_servos(b"bad")
    with pytest.raises(ValueError):
        encode_state({"timestamp": float("nan")})
