"""Export rendered images and IMU from a recorded current PX4/MuJoCo flight.

This is recorded-input perception evaluation, not a live VIO-controlled flight.
Estimator input contains RGB/gray images and noisy body-FLU IMU only. Physics
truth is kept in the original flight recording for independent scoring.
"""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
from rm27.telemetry.recording import open_recording

import cv2
import mujoco
import numpy as np

from .mujoco_camera import MujocoCamera
from .image_target_detector import DetectorConfig, ImageTargetDetector

ROOT=Path(__file__).resolve().parents[3]


def digest(path):
    h=hashlib.sha256()
    with open_recording(path,'rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def intrinsics(width,height,fovy):
    f=(height/2)/math.tan(math.radians(fovy)/2)
    return [f,f,(width-1)/2,(height-1)/2]


def camera_to_imu(config):
    forward=np.asarray(config['forward_body'],dtype=float);up=np.asarray(config['up_body'],dtype=float)
    forward/=np.linalg.norm(forward);up/=np.linalg.norm(up)
    # OpenCV optical axes: right, down, forward. FLU body axes: forward,left,up.
    right=np.cross(forward,up)
    matrix=np.eye(4);matrix[:3,:3]=np.column_stack([right,-up,forward])
    matrix[:3,3]=config['offset_body']
    if not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-6):
        raise ValueError('camera axes must be orthogonal')
    return matrix


def export(run, output, *, camera_config=ROOT/'configs/px4_sim_camera.json', duration=None):
    run,output=Path(run).resolve(),Path(output).resolve()
    metadata=json.loads((run/'metadata.json').read_text())
    scene=Path(metadata['scene'])
    if digest(scene)!=metadata['scene_sha256']:raise ValueError('recorded scene changed')
    config=json.loads(Path(camera_config).read_text())
    if duration is not None and (not np.isfinite(duration) or duration<=0):raise ValueError('duration must be positive')
    output.mkdir(parents=True,exist_ok=False)
    sensors=output/'sensors';images=sensors/'cam0/data';images.mkdir(parents=True)
    (sensors/'imu0').mkdir()
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model)
    rng=np.random.default_rng(config['seed'])
    kwargs={key:config[key] for key in ['width','height','fovy_deg','body_name','offset_body','forward_body','up_body']}
    detector=ImageTargetDetector(DetectorConfig(fovy_deg=config['fovy_deg']))
    next_image=0.;period=1./config['camera_hz'];camera_count=imu_count=safe_count=0
    last_time=-1.;image_hashes={};start_time=None
    with MujocoCamera(model,**kwargs) as camera,open_recording(run/'frames.jsonl') as stream, (sensors/'cam0/data.csv').open('w') as camfile, (sensors/'imu0/data.csv').open('w') as imufile, (output/'targets.jsonl').open('w') as targetfile:
        camwriter,imuwriter=csv.writer(camfile),csv.writer(imufile)
        camwriter.writerow(['#timestamp [ns]','filename'])
        imuwriter.writerow(['#timestamp [ns]','w_x [rad/s]','w_y [rad/s]','w_z [rad/s]','a_x [m/s^2]','a_y [m/s^2]','a_z [m/s^2]'])
        for line in stream:
            row=json.loads(line);t=float(row['timestamp'])
            if start_time is None:start_time=t
            if duration is not None and t-start_time>duration:break
            if t<=last_time:raise ValueError('recording time is nonmonotonic')
            last_time=t;ns=round(t*1e9)
            state=row['state']['imu']
            gyro=np.asarray(state['gyro'])*[1,-1,-1]+rng.normal(0,config['gyroscope_noise_std_rad_s'],3)
            accel=np.asarray(state['accel_body'])*[1,-1,-1]+rng.normal(0,config['accelerometer_noise_std_m_s2'],3)
            imuwriter.writerow([ns,*gyro,*accel]);imu_count+=1
            if t+1e-9<next_image:continue
            next_image+=period
            if next_image<t:next_image=t+period
            data.qpos[:]=row['all_qpos'];data.qvel[:]=row['all_qvel']
            frame=camera.capture(data,round(t*1e6))
            estimate=detector.detect(frame,now_us=frame.timestamp_us)
            safe_count+=estimate.safe_for_control
            targetfile.write(json.dumps(asdict(estimate),allow_nan=False)+'\n')
            gray=cv2.cvtColor(frame.image_rgb,cv2.COLOR_RGB2GRAY)
            name=str(ns)+'.png';cv2.imwrite(str(images/name),gray)
            image_hashes[name]=digest(images/name)
            camwriter.writerow([ns,name]);camera_count+=1
            if camera_count%200==0:print(json.dumps(dict(camera_frames=camera_count,sim_time_s=t)),flush=True)
            if camera_count==1 or estimate.safe_for_control and not (output/'target_visible.png').exists():
                cv2.imwrite(str(output/('initial.png' if camera_count==1 else 'target_visible.png')),cv2.cvtColor(frame.image_rgb,cv2.COLOR_RGB2BGR))
    if camera_count==0 or imu_count<2:raise ValueError('no usable sensor data')
    report=dict(passed=True,run_type='recorded-input image and IMU export',
                flight_control='original flight controller; these images did not control the flight',
                original_run=str(run),original_metadata_sha256=digest(run/'metadata.json'),
                original_recording_sha256=digest(run/'frames.jsonl'),scene_sha256=digest(scene),
                configuration=config,calibration=dict(model='pinhole',intrinsics=intrinsics(config['width'],config['height'],config['fovy_deg']),
                    distortion=[0.,0.,0.,0.],T_imu_cam=camera_to_imu(config).tolist()),
                source_time=dict(clock_domain='mujoco_simulation',unit='ns',semantic='PHYSICS_SAMPLE_TIME',evidence_status='VERIFIED'),
                camera_frames=camera_count,imu_samples=imu_count,safe_target_frames=safe_count,
                imu_sampling_hz=(imu_count-1)/(last_time-start_time),
                imu_noise_model=dict(accelerometer_sample_std_m_s2=config['accelerometer_noise_std_m_s2'],
                    gyroscope_sample_std_rad_s=config['gyroscope_noise_std_rad_s'],
                    bias_random_walk=0.,scope='exporter noise applied to plant sensor state; HIL_SENSOR noise is not in this recording field'),
                start_time_s=start_time,end_time_s=last_time,truth_in_estimator_input=False,
                image_hashes=image_hashes,imu_csv_sha256=digest(sensors/'imu0/data.csv'),
                camera_csv_sha256=digest(sensors/'cam0/data.csv'))
    (output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='image_hashes'},indent=2))
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'configs/px4_sim_camera.json');p.add_argument('--duration',type=float)
    a=p.parse_args();export(a.run,a.output,camera_config=a.config,duration=a.duration)

if __name__=='__main__':main()
