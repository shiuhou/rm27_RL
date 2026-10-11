"""Actual allocator torque directions and meaningful response scoring."""
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from rm27.autopilot.px4.response import plant, reference, score

ROOT=Path(__file__).resolve().parents[1]


def test_custom_allocator_matches_compiled_frd_rotor_geometry(tmp_path):
    p=plant(tmp_path,ROOT/'configs/dynamics_150g.json')
    allocator=json.loads((ROOT/'configs/px4/diagnostic.json').read_text())
    for i in range(4):
        actuator=p.model.actuator(f'uav_motor_{i}').id
        site=p.model.actuator_trnid[actuator,0]
        physical=p.model.site_pos[site]*[1,-1,-1]
        np.testing.assert_allclose(physical[:2],[allocator[f'CA_ROTOR{i}_PX'],allocator[f'CA_ROTOR{i}_PY']])
        # PX4 moment = ct*(position cross axis - km*axis). With the
        # upward FRD axis (0,0,-1), yaw moment is +ct*km in FRD and -ct*km
        # in FLU. This checks actual upstream convention, including yaw.
        assert p.model.actuator_gear[actuator,5]==pytest.approx(-allocator[f'CA_ROTOR{i}_KM'])
        p.data.act[:]=0;p.data.act[actuator]=1
        mujoco.mj_forward(p.model,p.data)
        expected=np.r_[np.cross(p.model.site_pos[site],[0,0,1])[:2],p.model.actuator_gear[actuator,5]]
        np.testing.assert_allclose(p.data.qfrc_actuator[3:],expected,atol=1e-9)


@pytest.mark.parametrize('layer',['rate','attitude'])
def test_response_scorer_rejects_no_tracking(layer):
    times=np.arange(.004,5.001,.004)
    samples=[dict(elapsed_s=t,desired=reference(t,layer,0),measured=[0,0,0],controls=[.333]*4) for t in times]
    assert not score(samples,layer,0)['passed']
    for s in samples:s['measured']=s['desired']
    assert score(samples,layer,0)['passed']
