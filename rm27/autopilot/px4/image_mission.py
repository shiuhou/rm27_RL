"""Current-flight image/VIO target approach, causal recovery and AprilTag return.

Development only. A separate physical scorer decides whether actual contact,
recovery, hover and resting gates pass. No hardware endpoint is available.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from .flight import run
from .live_flight import VioRuntime, ROOT
from .image_route import ImageRoute
from ...perception.sitl.armor_pose import ArmorPoseDetector


def independent_recovery(run, inferred, normal):
    from .recovery import evaluate_rows
    path=Path(run)/'physical_audit.jsonl'
    first=None
    for line in path.open():
        row=json.loads(line)
        if any('rm27_base_armor' in c['geoms'] for c in row.get('contacts',[])):
            first=dict(time_s=row['timestamp'],position_world_m=row['truth']['position_nwu'],
                armor_normal_speed_m_s=-float(np.asarray(row['truth']['velocity_nwu'])@normal))
            break
    if first is None:return dict(passed=False,failures=['physical_target_contact_missing'])
    with path.open() as stream:
        score=evaluate_rows((json.loads(line) for line in stream),first)
    delay=None if inferred is None else inferred['time_s']-first['time_s']
    score.update(impact_detection_delay_s=delay,first_impact_source='independent 1 ms physical audit')
    if delay is None or not -.001<=delay<=.1:
        score['passed']=False;score['failures'].append('impact_detection_outside_100ms_gate')
    return score


def execute(scene, audit_path, output, *, configuration=None, camera_configuration=None, instance=40, speedup=2., view=False, flight_runner=None):
    scene=Path(scene).resolve();output=Path(output).resolve()
    camera_path=Path(camera_configuration) if camera_configuration else ROOT/'configs/px4_sim_camera_mission_dev.json'
    configuration=Path(configuration) if configuration else ROOT/'configs/px4_image_mission_dev.json'
    settings=json.loads(configuration.read_text());audit=json.loads(Path(audit_path).read_text())
    observer_interval=settings.get('imu_observer_interval_us',20_000)
    if type(observer_interval) is not int or observer_interval not in (4000,8000,10000,20000):
        raise ValueError('explicit supported host-SITL IMU observer interval required')
    files=[Path(__file__),Path(__file__).with_name('image_route.py'),
        ROOT/'rm27/perception/sitl/armor_pose.py',camera_path,configuration,Path(audit_path),
        ROOT/'configs/px4'/f'{settings["estimation_profile"]}.json']
    termination_settings=None
    if settings.get('return_termination_configuration'):
        path=ROOT/settings['return_termination_configuration']
        termination_settings=json.loads(path.read_text())
        files.extend([path,Path(__file__).with_name('return_termination.py')])
    input_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    if hashlib.sha256(scene.read_bytes()).hexdigest()!=audit.get('scene_sha256'):
        raise ValueError('reviewed corridor scene hash mismatch')
    for path,expected in audit['asset_sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=expected:
            raise ValueError('reviewed corridor asset hash changed: '+path)
    camera=json.loads(camera_path.read_text())
    runtime_class=VioRuntime
    if settings.get('use_tag_height_registration'):
        from .tag_height import TagHeightRuntime
        runtime_class=TagHeightRuntime
        path=Path(__file__).with_name('tag_height.py')
        input_hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    # Preserve exact input bytes even for development files not yet tracked
    # by Git; the tracked diff alone cannot reproduce those inputs.
    snapshot={path:Path(path).read_bytes() for path in input_hashes}
    route=ImageRoute(audit,settings,camera)
    detector=ArmorPoseDetector(*settings['armor_rectangle_dimensions_m'],fovy_deg=camera['fovy_deg'])

    class ImageRuntime(runtime_class):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.profile=settings['estimation_profile']
            self.vio_camera_config=camera_path;self.next_pose=None;self.pose_stream=None
            self.audit_stream=None;self.physical_first_time=None
            # Configure the PX4 startup stream. Sending SET_MESSAGE_INTERVAL
            # before mavlink starts silently loses the request.
            self.telemetry_rates['HIGHRES_IMU']=1e6/observer_interval
            self.imu_times=[]
            self.cached_return_tag=None

        def __enter__(self):
            rt=super().__enter__()
            self.pose_stream=(self.output/'armor_poses.jsonl').open('x',buffering=1)
            self.audit_stream=(self.output/'physical_audit.jsonl').open('x',buffering=1)
            return rt

        def receive(self):
            messages=super().receive()
            for message in messages:
                if message.get_type()=='HIGHRES_IMU':
                    route.observe_imu(message);self.imu_times.append(int(message.time_usec))
            return messages

        def advance(self, controls):
            # The task never receives this stream. Capture the first physical
            # impact and all recovery substeps before sensor publication can
            # delay the actor's contact inference.
            for _ in range(self.substeps):
                self.plant.step_normalized(controls)
                now=float(self.plant.data.time);contacts=self.plant.contacts()
                non_dock=[c for c in contacts if 'rm27_dock_collision_bottom' not in c['geoms']]
                if non_dock and self.physical_first_time is None:self.physical_first_time=now
                window=self.physical_first_time is not None and now<=self.physical_first_time+3.1
                if self.audit_stream and (non_dock or window):
                    self.audit_stream.write(json.dumps(dict(timestamp=now,truth=self.plant.truth(),contacts=contacts),allow_nan=False)+'\n')

        def step(self,timeout=5.):
            row=super().step(timeout)
            now=float(self.plant.data.time);frame=self.vio.last_frame
            if self.return_observation and self.return_observation.get('valid'):
                self.cached_return_tag=self.return_observation.copy()
            pose=None
            if frame is not None and self.next_pose!=frame.timestamp_us:
                self.next_pose=frame.timestamp_us
                pose=detector.detect(frame,now_us=round(now*1e6))
                self.pose_stream.write(json.dumps(asdict(pose),allow_nan=False)+'\n')
            route.observe(pose,self.telemetry,now)
            return row

        def __exit__(self,kind,exc,traceback):
            if kind is not None and termination_settings:
                from .return_termination import CAMERA_FAILURES,handle_camera_failure
                if str(exc) in CAMERA_FAILURES:
                    handle_camera_failure(self,str(exc),self.cached_return_tag,termination_settings)
            return super().__exit__(kind,exc,traceback)

        def close(self):
            if self.imu_times:
                delta=np.diff(self.imu_times);positive=delta[delta>0]
                evidence=dict(requested_interval_us=observer_interval,messages=len(self.imu_times),
                    observed_median_interval_us=float(np.median(positive)) if len(positive) else None,
                    observed_maximum_interval_us=int(positive.max()) if len(positive) else None,
                    non_increasing_timestamps=int(np.sum(delta<=0)),
                    scope='Measured PX4 simulation publication, not raw hardware IMU or M3C UART qualification')
                (self.output/'observer_rate.json').write_text(json.dumps(evidence,indent=2)+'\n')
            if self.pose_stream:self.pose_stream.close();self.pose_stream=None
            if self.audit_stream:self.audit_stream.close();self.audit_stream=None
            super().close()

    runner=flight_runner if flight_runner is not None else run
    code=runner(scene,output,instance,dynamics=ROOT/'configs/dynamics_150g.json',profile='vio',
        view=view,speedup=speedup,route=route,runtime_factory=ImageRuntime,wall_deadline_s=600.)
    source_directory=output/'image_inputs';source_directory.mkdir(exist_ok=False)
    source_paths={}
    for path,contents in snapshot.items():
        saved=source_directory/(hashlib.sha256(contents).hexdigest()+'-'+Path(path).name)
        saved.write_bytes(contents);source_paths[path]=str(saved.relative_to(output))
    (output/'image_mission_inputs.json').write_text(json.dumps(dict(
        mission=settings,camera=camera,audit=str(audit_path),
        input_sha256=input_hashes,input_snapshot_paths=source_paths,
        run_type='headless closed-loop SITL' if not view else 'live-view closed-loop SITL',
        scope='complete-mission development; physical acceptance and camera hardware qualification remain separate'),indent=2)+'\n')
    summary=json.loads((output/'summary.json').read_text())
    summary.update(gate_version=settings['gate_version'],model_mass_kg=.15,
        scope='current-flight image/VIO complete mission; simulation assumptions, no hardware qualification')
    summary['image_route']=route.result()
    termination_path=output/'termination_summary.json'
    if termination_path.exists():
        summary['fault_termination']=json.loads(termination_path.read_text())
        summary['passed']=False
    if (output/'physical_audit.jsonl').exists():
        score=independent_recovery(output,route.impact,route.armor_normal)
        (output/'recovery_score.json').write_text(json.dumps(score,indent=2)+'\n')
        summary['independent_recovery_score']=score
        if code==0:
            summary['passed']=summary.get('passed') is True and score.get('passed') is True
            if not summary['passed']:summary['failure']='Independent physical contact/recovery gate failed'
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return 0 if summary['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path);p.add_argument('--camera-config',type=Path)
    p.add_argument('--instance',type=int,default=40)
    p.add_argument('--speedup',type=float,default=2.);p.add_argument('--view',action='store_true')
    a=p.parse_args();return execute(a.scene,a.audit,a.output,configuration=a.config,camera_configuration=a.camera_config,
        instance=a.instance,speedup=a.speedup,view=a.view)


if __name__=='__main__':raise SystemExit(main())
