from test_px4_full_return_command_loss import observations
from tools.px4.low_alignment_command_loss import eligible


def test_low_alignment_interruption_requires_fresh_low_assigned_slot():
    tag,tel=observations(10.)
    tel['local_position']=[0.,0.,-.12]
    assert eligible(tag,tel,'ALIGN',10.)
    assert not eligible(tag,tel,'RETURN',10.)
    tel['local_position'][2]=-.6
    assert not eligible(tag,tel,'ALIGN',10.)
    tel['local_position'][2]=-.12
    tag['source_time_s']=9.
    assert not eligible(tag,tel,'ALIGN',10.)
