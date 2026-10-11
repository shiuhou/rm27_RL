"""Run pinned OpenVINS on current MuJoCo flight image/IMU input, no truth mount."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import uuid

from .dataset import digest

ROOT=Path(__file__).resolve().parents[3]
DEFAULT_INSTALL=Path('/home/shiuhou/Projects/rm27-vio-20261007/adapter-install03.json')


def yaml_file(path, data):
    # JSON-compatible inline lists are accepted by OpenCV's YAML parser.
    text='%YAML:1.0\n'
    for name,values in data.items():
        text+=name+':\n'
        for key,value in values.items():
            if isinstance(value,list) and value and isinstance(value[0],list):
                text+='  '+key+':\n'+''.join('    - '+json.dumps(row)+'\n' for row in value)
            else:text+='  '+key+': '+json.dumps(value)+'\n'
    Path(path).write_text(text)


def prepare_config(source,install,config,*,stationary_init=False,settings=None):
    upstream=Path(install['source'])/'config/euroc_mav'
    estimator=(upstream/'estimator_config.yaml').read_text()
    overrides=dict(use_stereo='false',max_cameras='1',calib_cam_extrinsics='false',calib_cam_intrinsics='false',
                   calib_cam_timeoffset='false',track_frequency=str(source['configuration']['camera_hz']),num_opencv_threads='1')
    settings=settings or {}
    allowed={'num_pts','fast_threshold','grid_x','grid_y','min_px_dist','max_clones','max_slam','max_slam_in_update','up_msckf_sigma_px','up_slam_sigma_px'}
    if set(settings.get('estimator',{}))-allowed:raise ValueError('unsupported estimator setting')
    overrides.update({k:str(v) for k,v in settings.get('estimator',{}).items()})
    if stationary_init:
        # Static initialization before a gentle takeoff avoids requiring a
        # high-acceleration "jerk" which this flight never commands.
        overrides.update(try_zupt='true',zupt_only_at_beginning='true')
    for key,value in overrides.items():estimator=re.sub(r'^'+key+r':.*$',key+': '+value,estimator,flags=re.M)
    (config/'estimator_config.yaml').write_text(estimator)
    imu=(upstream/'kalibr_imu_chain.yaml').read_text()
    if settings.get('use_simulated_imu_noise'):
        import math
        c=source['configuration'];hz=source.get('imu_sampling_hz') or (source['imu_samples']-1)/(source['end_time_s']-source['start_time_s'])
        # Export reads plant state, before HIL_SENSOR transport noise. Convert
        # the explicitly added exporter sample noise to continuous densities.
        values=dict(accelerometer_noise_density=c['accelerometer_noise_std_m_s2']*settings.get('accelerometer_density_margin',1.)/math.sqrt(hz),
            gyroscope_noise_density=c['gyroscope_noise_std_rad_s']*settings.get('gyroscope_density_margin',1.)/math.sqrt(hz),
            accelerometer_random_walk=1e-8,gyroscope_random_walk=1e-8,update_rate=hz)
        for key,value in values.items():imu=re.sub(r'^(\s*'+key+r':).*$',lambda m:m[1]+' '+str(value),imu,flags=re.M)
    (config/'kalibr_imu_chain.yaml').write_text(imu)
    calibration=source['calibration'];c=source['configuration']
    yaml_file(config/'kalibr_imucam_chain.yaml',dict(cam0=dict(T_imu_cam=calibration['T_imu_cam'],cam_overlaps=[],camera_model='pinhole',
        distortion_coeffs=calibration['distortion'],distortion_model='radtan',intrinsics=calibration['intrinsics'],
        resolution=[c['width'],c['height']],rostopic='/cam0/image_raw')))


def run(dataset,output,installation=DEFAULT_INSTALL,rate=.5,stationary_init=False,settings=None):
    if not __import__('math').isfinite(rate) or rate<=0:raise ValueError('playback rate must be finite and positive')
    dataset,output=Path(dataset).resolve(),Path(output).resolve()
    source=json.loads((dataset/'summary.json').read_text());install=json.loads(Path(installation).read_text())
    if source.get('passed') is not True or source.get('truth_in_estimator_input') is not False:raise ValueError('invalid sensor input manifest')
    if install['commit']!='69488123ed9362dd44b6f28e7f4680abbff1442b':raise ValueError('OpenVINS pin mismatch')
    for path,hashvalue in install['files_sha256'].items():
        if digest(path)!=hashvalue:raise ValueError('installed OpenVINS input changed: '+path)
    binary=Path(install['workspace'])/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    if digest(binary)!=install['binary_sha256']:raise ValueError('OpenVINS binary changed')
    for name,hashvalue in source['image_hashes'].items():
        if digest(dataset/'sensors/cam0/data'/name)!=hashvalue:raise ValueError('image input changed: '+name)
    for name,key in [('cam0','camera_csv_sha256'),('imu0','imu_csv_sha256')]:
        if digest(dataset/'sensors'/name/'data.csv')!=source[key]:raise ValueError('sensor CSV changed')
    output.mkdir(parents=True,exist_ok=False);config=output/'config';config.mkdir()
    settings=json.loads(Path(settings).read_text()) if settings is not None else {}
    prepare_config(source,install,config,stationary_init=stationary_init,settings=settings)
    container='rm27-sim-vio-'+uuid.uuid4().hex
    command=['docker','run','--rm','--name',container,'--network','none','--cpus','4','--memory','12g',
        '--user',f'{os.getuid()}:{os.getgid()}',
        '-e','ROS_DOMAIN_ID=119','-e','ROS_LOCALHOST_ONLY=1','-e','OPENBLAS_NUM_THREADS=1',
        '-v',f'{ROOT/"tools/openvins/simulated_flight_ros2.py"}:/player.py:ro','-v',f'{install["workspace"]}:/ws:ro',
        '-v',f'{dataset/"sensors"}:/data:ro','-v',f'{output}:/out',install['image'],
        'bash','-c','source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 /player.py --rate "$1"','rm27',str(rate)]
    launch=dict(command=command,installation=install,dataset_summary_sha256=digest(dataset/'summary.json'),
                configuration_sha256={p.name:digest(p) for p in config.iterdir()},run_type='recorded-input native VIO',
                flight_qualified=False,truth_mounted=False,stationary_init=stationary_init)
    launch['development_settings']=settings
    launch['runner_sha256']=digest(__file__)
    launch['player_sha256']=digest(ROOT/'tools/openvins/simulated_flight_ros2.py')
    (output/'launch.json').write_text(json.dumps(launch,indent=2)+'\n')
    try:
        with (output/'container.log').open('w') as log:
            status=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=(source['end_time_s']-source['start_time_s'])/rate+120)
    except BaseException:
        subprocess.run(['docker','stop','--time','5',container],capture_output=True,timeout=15)
        raise
    result=json.loads((output/'runtime_summary.json').read_text()) if (output/'runtime_summary.json').exists() else dict(passed=False,failure='missing runtime summary')
    result.update(passed=result.get('passed') is True and status.returncode==0,run_type='recorded-input native VIO',
                  flight_qualified=False,truth_mounted=False,dataset_summary_sha256=launch['dataset_summary_sha256'])
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--installation',type=Path,default=DEFAULT_INSTALL);p.add_argument('--rate',type=float,default=.5)
    p.add_argument('--stationary-init',action='store_true',help='Allow static initialization via an initial ZUPT stage')
    p.add_argument('--settings',type=Path,help='Explicit development estimator settings JSON')
    a=p.parse_args();r=run(a.dataset,a.output,a.installation,a.rate,a.stationary_init,a.settings);return 0 if r['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
