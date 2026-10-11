from rm27.autopilot.px4.image_batch import assess


def test_packet_reception_cannot_substitute_for_actual_ekf2_fusion():
    summary=dict(passed=True,events=[],image_route={'passed':True},
        independent_hover_score={'passed':True},independent_resting_score={'passed':True},
        independent_recovery_score={'passed':True},landing_commitment={'assigned_tag_id':21},
        auto_disarm={'confirmed':True})
    vio={'passed':True,'counts':{'odometry':100}}
    log={'passed':True,'fusion':{}}
    assert assess(summary,vio,log)['passed'] is False
    log['fusion']={'0':{name:{'ever':True} for name in ('cs_ev_pos','cs_ev_vel','cs_ev_hgt')}}
    assert assess(summary,vio,log)['passed'] is True
    summary['events']=[{'event':'STATUSTEXT','text':'Failsafe activated'}]
    assert assess(summary,vio,log)['passed'] is False
