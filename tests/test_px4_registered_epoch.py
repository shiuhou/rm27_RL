from dataclasses import replace
from types import SimpleNamespace

import pytest

from test_px4_odometry_clocks import estimate
from tools.px4.registered_epoch import RegisteredEpochContract


def test_new_native_epoch_cannot_reuse_existing_registration_or_resume_silently():
    previous=estimate()
    contract=SimpleNamespace(estimate=lambda row:row)
    vio=SimpleNamespace(adapter=SimpleNamespace(previous=previous))
    records=[]
    boundary=RegisteredEpochContract(contract,vio,records.append)
    assert boundary.estimate(previous)==previous
    new=replace(previous,localization_epoch=1,translation=(100.,0.,0.))
    with pytest.raises(ValueError,match='requires new registration'):
        boundary.estimate(new)
    assert boundary.frame_changed and records[0]['observed_epoch']==1
    with pytest.raises(ValueError,match='requires new registration'):
        boundary.estimate(previous)
    assert vio.adapter.previous==previous


def test_initial_native_epoch_is_owned_by_first_registration():
    e=replace(estimate(),localization_epoch=4)
    boundary=RegisteredEpochContract(SimpleNamespace(estimate=lambda row:row),
        SimpleNamespace(adapter=None),lambda row:None)
    assert boundary.estimate(e)==e and not boundary.frame_changed
