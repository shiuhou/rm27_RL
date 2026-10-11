import json

import numpy as np

from rm27.autopilot.px4.image_mission import independent_recovery


def test_image_mission_scores_actual_first_contact_even_when_inference_is_late(tmp_path):
    rows=[]
    for millisecond in range(100, 3201):
        time=millisecond/1000
        contacts=[]
        if millisecond in (100, 170):
            contacts=[dict(geoms=['drone_1_guard_1','rm27_base_armor'])]
        rows.append(dict(timestamp=time, contacts=contacts, truth=dict(
            position_nwu=[0.,0.,1.], velocity_nwu=[0.,0.,0.],
            body_rates_flu=[0.,0.,0.], quaternion_flu_to_nwu=[1.,0.,0.,0.])))
    (tmp_path/'physical_audit.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    result=independent_recovery(tmp_path,dict(time_s=.308),np.array([1.,0.,0.]))
    assert result['impact_time_s']==.1
    assert result['recovery_confirmation_time_s'] is not None
    assert result['passed'] is False
    assert set(result['failures'])=={'secondary_collision','impact_detection_outside_100ms_gate'}
