"""PX4 contracts use native actuator order and sensor frames, never AP PWM order."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np
import pytest
from pymavlink.dialects.v20 import common as mavlink
from rm27.autopilot.px4.bridge import ActuatorDecoder, SensorProducer
from rm27.experiments.calibration import load_profile
from rm27.simulation.micro_plant import MicroPlant
from rm27.integration import drone, dock
from rm27.experiments.calibration import with_profile

ROOT=Path(__file__).resolve().parents[1]

def actuator(stamp, controls, *, armed=True, lockstep=True):
    return mavlink.MAVLink_hil_actuator_controls_message(
        stamp,list(controls)+[0.]*12,128 if armed else 0,1 if lockstep else 0)

def test_actuator_order_disarm_and_clock():
    d=ActuatorDecoder()
    np.testing.assert_array_equal(d.decode(actuator(1,[.1,.2,.3,.4])),[.1,.2,.3,.4])
    np.testing.assert_array_equal(d.decode(actuator(2,[.1,.2,.3,.4],armed=False)),[0,0,0,0])
    for message in [actuator(2,[.1,.2,.3,.4]),actuator(1,[.1,.2,.3,.4]),actuator(3,[.1,.2,.3,.4],lockstep=False),actuator(3,[np.nan,0,0,0])]:
        with pytest.raises(ValueError):d.decode(message)

def test_150g_profile_inertia_and_force():
    profile=load_profile(ROOT/'configs/dynamics_150g.json')
    assert profile['values']['mass_kg']==.15
    expected=(.15-.032)/12*np.array([.03**2+.02**2,.04**2+.02**2,.04**2+.03**2])+.008*np.array([4*.03**2,4*.03**2,8*.03**2])
    np.testing.assert_allclose(profile['values']['inertia_kg_m2'],expected)


@pytest.fixture
def px4_plant(tmp_path):
    root=ET.fromstring('<mujoco><option timestep=".002" integrator="RK4"/><asset/><worldbody/></mujoco>')
    body=ET.SubElement(root.find('worldbody'),'body',name='dock')
    for name,center,size in dock.boxes():
        ET.SubElement(body,'geom',name='rm27_dock_collision_'+name,type='box',pos=' '.join(map(str,center)),size=' '.join(map(str,size)))
    cfg=with_profile(json.loads((ROOT/'configs/drone.json').read_text()),ROOT/'configs/dynamics_150g.json')
    for i,(name,center) in enumerate(dock.slot_centers().items(),1):
        ET.SubElement(body,'site',name=name,pos=' '.join(map(str,center)))
        drone.add(root,f'drone_{i}',np.array(center)+[0,0,.021],[1,0,0,0],cfg)
    path=tmp_path/'scene.xml';ET.ElementTree(root).write(path,encoding='unicode')
    return MicroPlant(path,dynamics_profile=ROOT/'configs/dynamics_150g.json')

def test_physical_motor_order_and_isolation(px4_plant):
    p=px4_plant
    p.step_normalized([.1,.2,.3,.4])
    np.testing.assert_allclose(p.data.ctrl[p.motors],p.maximum*[.1,.2,.3,.4])
    np.testing.assert_array_equal(p.data.ctrl[4:],0)
    for bad in [[-.01,0,0,0],[float('nan'),0,0,0],[1,2,3]]:
        with pytest.raises(ValueError):p.step_normalized(bad)

def test_sensor_frame_and_message(px4_plant):
    plant=px4_plant
    sensor=SensorProducer(plant)
    msg=sensor.hil_sensor();odom=sensor.odometry();distance=sensor.distance_sensor()
    assert msg.get_type()=='HIL_SENSOR' and odom.get_type()=='ODOMETRY'
    assert distance.get_type()=='DISTANCE_SENSOR'
    assert distance.orientation==mavlink.MAV_SENSOR_ROTATION_PITCH_270
    assert distance.min_distance==1 and distance.max_distance==400 and distance.current_distance==2
    assert odom.frame_id==mavlink.MAV_FRAME_LOCAL_NED
    assert odom.child_frame_id==mavlink.MAV_FRAME_BODY_FRD
    np.testing.assert_allclose([msg.xacc,msg.yacc,msg.zacc],plant.state()['imu']['accel_body'],atol=.02)
    np.testing.assert_allclose([odom.q[0],odom.q[1],odom.q[2],odom.q[3]],plant.state()['quaternion'])


def test_distance_sensor_follows_scene_ray_and_has_explicit_no_hit(px4_plant):
    plant=px4_plant
    sensor=SensorProducer(plant)
    # The initial vehicle origin is 2 cm above the fixture dock floor.
    assert sensor.distance_sensor().current_distance==2
    assert set(plant.model.geom_group[plant.model.geom_bodyid==plant.body])=={5}

    # Move the same aircraft upward without changing its assigned slot origin;
    # a slot-relative height implementation would still report 2 cm.
    plant.data.qpos[plant.qa+2]+=.05
    mujoco.mj_forward(plant.model,plant.data)
    assert sensor.distance_sensor().current_distance==7

    # Outside the fixture dock there is no collision surface below the ray.
    # Parked vehicles are group-filtered and therefore cannot become a false
    # range return; no-hit is represented by the configured 4 m maximum.
    plant.data.qpos[plant.qa:plant.qa+3]+=[.5,0.,0.]
    mujoco.mj_forward(plant.model,plant.data)
    assert sensor.distance_sensor().current_distance==400


def test_pressure_group_and_integer_parameters(px4_plant):
    import struct
    from types import SimpleNamespace
    from rm27.autopilot.px4.runtime import Runtime
    msg=SensorProducer(px4_plant).hil_sensor()
    assert msg.fields_updated & 0x1a00 == 0x1a00
    assert msg.abs_pressure>900 and msg.temperature==20
    raw=struct.unpack('<f',struct.pack('<i',101))[0]
    assert Runtime.decode_param(SimpleNamespace(param_type=mavlink.MAV_PARAM_TYPE_INT32,param_value=raw))==101


def test_landing_contact_chatter_and_slot_gate():
    from rm27.autopilot.px4.flight import LandingMonitor
    monitor=LandingMonitor()
    monitor.check(1.,np.array([0,0,.02]),[{'force_N':1}])
    monitor.check(1.1,np.array([0,0,.0201]),[])
    monitor.check(1.4,np.array([0,0,.02]),[{'force_N':1}])
    with pytest.raises(RuntimeError,match='grace'):
        monitor.check(2.,np.array([0,0,.02]),[])
    with pytest.raises(RuntimeError,match='slot'):
        monitor.check(1.5,np.array([.008,0,.02]),[{'force_N':1}])


def test_land_requests_autonomous_landing_without_disarm():
    from rm27.autopilot.px4.runtime import Runtime
    runtime=Runtime.__new__(Runtime)
    calls=[]
    runtime.command=lambda code,*args:calls.append((code,args))
    runtime.mode('LAND')
    assert calls[0][0]==mavlink.MAV_CMD_NAV_LAND
    assert np.isnan(calls[0][1][3:]).all()


def test_px4_restores_user_mode_after_auto_disarm():
    from rm27.autopilot.px4.flight import confirm_auto_disarm
    telemetry={'armed':False,'mode':'OFFBOARD','landed_state':mavlink.MAV_LANDED_STATE_ON_GROUND,
               'landed_time':10.,'command_ack':{mavlink.MAV_CMD_NAV_LAND:{'result':mavlink.MAV_RESULT_ACCEPTED}}}
    confirm_auto_disarm(telemetry,10.1,True)
    for invalid in [dict(telemetry,armed=True),dict(telemetry,landed_state=2),dict(telemetry,landed_time=9.),dict(telemetry,command_ack={})]:
        with pytest.raises(RuntimeError):confirm_auto_disarm(invalid,10.1,True)


def test_px4_policy_adapter_keeps_force_and_frame_contract():
    from types import SimpleNamespace
    from rm27.autopilot.px4.control import ForceMapping, PX4Adapter, PX4PolicyAdapter
    from rm27.experiments.aggressive_contact.teacher_interface import RateThrust
    class Policy:
        def infer(self, observation, timestamp):
            assert observation=='registered-estimate' and timestamp==2.
            return RateThrust(np.array([1.,2.,3.]),2.)
    class Mav:
        def set_attitude_target_send(self, **fields):self.fields=fields
    runtime=SimpleNamespace(mav=Mav())
    adapter=PX4PolicyAdapter(Policy(),PX4Adapter(ForceMapping(4.)))
    action=adapter.send(runtime,'registered-estimate',2.)
    np.testing.assert_allclose(action.body_rates_flu_rad_s,[1.,2.,3.])
    np.testing.assert_allclose([runtime.mav.fields['body_roll_rate'],runtime.mav.fields['body_pitch_rate'],runtime.mav.fields['body_yaw_rate']],[1.,-2.,-3.])
    assert runtime.mav.fields['thrust']==.5


def test_route_deadline_is_explicitly_configurable(tmp_path):
    from rm27.experiments.route_experiment import RouteExperiment
    mission=json.loads((ROOT/'configs/px4_corridor_mission.json').read_text())
    assert mission['physics_deadline_s']==260.


def test_odometry_adapter_requires_qualified_metric_contract():
    from rm27.autopilot.px4.odometry import OdometryAdapter, Registration
    from rm27.perception.vision.localization import LocalizationEstimate, TimestampEvidence, TrackingState, ScaleState
    stamp=lambda value: TimestampEvidence(value,'ns','vio','PUBLISH','VERIFIED')
    covariance=np.eye(6).tolist()
    quality={'flight_qualified':True,'pose_covariance':covariance,'velocity_covariance':covariance}
    def estimate(source,epoch=0,**changes):
        values=dict(source_time=stamp(source),publish_time=stamp(source+1_000_000),parent_frame='map',child_frame='body',
                    translation=(1.,2.,3.),orientation_wxyz=(1.,0.,0.,0.),tracking_state=TrackingState.TRACKING,
                    valid=True,initialized=True,relocalizing=False,localization_epoch=epoch,scale_state=ScaleState.METRIC,
                    translation_unit='m',velocity_unit='m/s',velocity=(.1,.2,.3),velocity_frame='map',quality=quality)
        values.update(changes);return LocalizationEstimate(**values)
    identity=tuple(tuple(float(v) for v in row) for row in np.eye(3))
    registration=Registration('map','body','vio',identity,(0.,0.,0.),identity)
    adapter=OdometryAdapter(registration)
    first=adapter.packet(estimate(1_000_000_000),1_020_000_000)
    assert first.get_type()=='ODOMETRY' and first.frame_id==mavlink.MAV_FRAME_LOCAL_NED
    assert first.reset_counter==0
    second=adapter.packet(estimate(1_040_000_000,epoch=1),1_060_000_000)
    assert second.reset_counter==1
    with pytest.raises(ValueError,match='qualification'):
        adapter.packet(estimate(1_080_000_000,epoch=1,quality={}),1_100_000_000)
