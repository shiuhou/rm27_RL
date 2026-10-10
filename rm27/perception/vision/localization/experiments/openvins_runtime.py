"""Isolated EuRoC/OpenVINS runtime adapter, retaining incomplete v2 evidence.

ROS transport dependencies are confined to the observer and bag reader. Legacy
monocular datasets and LocalizationEstimate semantics are deliberately unchanged.
"""
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile
import numpy as np

from ..schema import TimestampEvidence
from .common import require, checked_file, read_json, write_json, number, file_hash
from .openvins import PIN, pose_observation
from .trajectory import rotation_wxyz


class StreamOrder:
    def __init__(self):
        self.previous = {}

    def push(self, stream, ns):
        require(type(ns) is int and ns >= 0, 'TIMESTAMP_INVALID', 'integer nonnegative ns')
        require(stream not in self.previous or ns > self.previous[stream],
                'STREAM_ORDER_INVALID', stream)
        self.previous[stream] = ns


def wire_pose(position, quaternion_xyzw, state_ns):
    x,y,z,w = quaternion_xyzw
    q = [number(v,'quaternion') for v in [w,x,y,z]]
    rotation_wxyz(q)
    require(len(position)==3, 'POSE_INVALID', 'three position values')
    return dict(parent_frame='backend_world', child_frame='imu',
                translation=[number(v,'position') for v in position], orientation_wxyz=q,
                scale_state='METRIC', translation_unit='m',
                source_time=TimestampEvidence(state_ns,'ns','EuRoC:V1_01_easy:sensor_time',
                    'ESTIMATOR_IMU_STATE_TIME_DOUBLE_DERIVED','CODE_SUPPORTED').to_dict())


def world_velocity(quaternion_xyzw, local_velocity):
    x,y,z,w = quaternion_xyzw
    r = rotation_wxyz([w,x,y,z])
    require(len(local_velocity)==3, 'VELOCITY_INVALID', 'three velocity values')
    return (r @ np.asarray([number(v,'velocity') for v in local_velocity])).tolist()


def state_row(line):
    row = line.split()
    require(len(row)>=19, 'STATE_ROW_INVALID', 'missing fields')
    values = [number(float(x),'state row') for x in row]
    return dict(state_ns=int(Decimal(row[0])*10**9),
                image_ns=int((Decimal(row[0])-Decimal(row[17]))*10**9),
                position=values[5:8], quaternion_wxyz=[values[4],*values[1:4]],
                velocity_world=values[8:11])


def metric_consistency(candidate, baseline):
    # Fixed before this run, to allow bounded native scheduling variation.
    limits = [('ate','rmse_m',.01), ('rpe_1s','translation_rmse_m',.01),
              ('rpe_1s','rotation_rmse_deg',.1),
              ('scale_diagnostic','path_length_ratio_est_over_ref',.01)]
    checks = {f'{a}.{b}':dict(delta=abs(candidate[a][b]-baseline[a][b]), tolerance=t)
              for a,b,t in limits}
    return {'pass':all(v['delta']<=v['tolerance'] for v in checks.values()), 'checks':checks}


def coverage_consistency(candidate, baseline):
    checks={k:candidate[k]==baseline[k] for k in ['raw_pose_count','evaluated_pose_count','gt_csv_sha256']}
    checks['duration']=abs(candidate['evaluated_duration_s']-baseline['evaluated_duration_s']) <= .005
    checks['camera_coverage']=all(candidate['association'][k]==baseline['association'][k]
                                  for k in ['first_camera_index','last_camera_index'])
    checks['fixed_SE3']=all(r['alignment']['type']=='SE3' and r['alignment']['scale']==1.0 for r in [candidate,baseline])
    return {'pass':all(checks.values()),'checks':checks}


def check_output_row(observed, recorded, odometry=False):
    keys=['state_ns','position','quaternion_xyzw']+(['velocity_local'] if odometry else [])
    require(all(observed[k]==recorded[k] for k in keys),'OUTPUT_TRANSPORT_MISMATCH','live observer vs independent bag')


def reconcile_outputs(raw, observed):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(raw/'online-output'),storage_id=''),rosbag2_py.ConverterOptions('',''))
    counts={'pose':0,'odom':0}
    while reader.has_next():
        topic,data,_=reader.read_next()
        require(topic in ['/poseimu','/odomimu'],'OUTPUT_TOPIC_INVALID',topic)
        name='pose' if topic=='/poseimu' else 'odom'
        m=deserialize_message(data,PoseWithCovarianceStamped if name=='pose' else Odometry)
        p,q=m.pose.pose.position,m.pose.pose.orientation
        row=dict(state_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                 position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w])
        if name=='odom':
            v=m.twist.twist.linear
            row['velocity_local']=[v.x,v.y,v.z]
        require(counts[name]<len(observed[name]),'OUTPUT_COUNT_MISMATCH',name)
        check_output_row(observed[name][counts[name]],row,name=='odom')
        counts[name]+=1
    require(all(counts[k]==len(observed[k])>0 for k in counts),'OUTPUT_COUNT_MISMATCH','pose and propagated output required')
    return counts


class EurocVioDataset:
    """ASL + verified ROS2 transport; independent of optional-IMU mono loader."""
    def __init__(self, archive, bag):
        self.archive, self.bag = Path(archive), Path(bag)
        verification = read_json(self.bag/'source_verification.json')
        require(verification.get('source_sha256')=='a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa',
                'VIO_SEQUENCE_UNSUPPORTED','this qualified adapter currently supports frozen V1_01_easy only')
        require(verification.get('readback_exact') is True and verification.get('truth_included') is False,
                'VIO_TRANSPORT_UNVERIFIED', 'complete transport verification required')
        checked_file(self.archive, verification['source_sha256'])
        for name,digest in verification['artifacts_sha256'].items():
            checked_file(self.bag/name,digest)
        self.provenance = verification
        with zipfile.ZipFile(self.archive) as z:
            self.camera = [l.split(',') for l in z.read('mav0/cam0/data.csv').decode().splitlines() if l and not l.startswith('#')]
            self.imu = [l.split(',') for l in z.read('mav0/imu0/data.csv').decode().splitlines() if l and not l.startswith('#')]
        self.camera_ns = [int(r[0]) for r in self.camera]
        self.imu_ns = [int(r[0]) for r in self.imu]
        order = StreamOrder()
        for stream, stamps in [('cam0',self.camera_ns),('imu0',self.imu_ns)]:
            for ns in stamps:
                order.push(stream,ns)
            require(len(stamps)==verification['verified_counts'][stream], 'VIO_INPUT_COUNT', stream)
        require(np.isfinite(np.asarray([r[1:7] for r in self.imu],float)).all(), 'IMU_NONFINITE','ASL raw SI samples')


def lines(path):
    with Path(path).open() as f:
        return [json.loads(line) for line in f if line.strip()]


def normalize(root, dataset, install, run_id):
    from tools.openvins.native_accuracy import associate
    import cv2
    raw = root/'raw'
    observed = {name:lines(raw/(name+'.jsonl')) for name in ['cam0','imu0','pose','odom']}
    order = StreamOrder()
    for name, records in observed.items():
        for row in records:
            order.push(name,row['state_ns'])
    reconciled=reconcile_outputs(raw,observed)
    require([r['state_ns'] for r in observed['cam0']]==dataset.camera_ns,'CAMERA_DELIVERY_MISMATCH','observer input sequence')
    require([r['state_ns'] for r in observed['imu0']]==dataset.imu_ns,'IMU_DELIVERY_MISMATCH','observer input sequence')
    with zipfile.ZipFile(dataset.archive) as z:
        for row, source in zip(observed['cam0'],dataset.camera):
            pixels = cv2.imdecode(np.frombuffer(z.read('mav0/cam0/data/'+source[1]),np.uint8),cv2.IMREAD_UNCHANGED)
            require(row['encoding']=='mono8' and row['width']==752 and row['height']==480
                    and row['step']==752 and hashlib.sha256(pixels.tobytes()).hexdigest()==row['pixels_sha256'],
                    'CAMERA_PIXELS_MISMATCH','runtime observer versus ASL')
    for row, source in zip(observed['imu0'],dataset.imu):
        require(row['gyro']+row['accel']==list(map(float,source[1:7])), 'IMU_VALUES_MISMATCH','runtime observer versus ASL SI values')
    states = [state_row(l) for l in (raw/'state_estimate.txt').read_text().splitlines() if l and not l.startswith('#')]
    poses = observed['pose']
    require(len(states)==len(poses)>0,'POSE_COUNT_MISMATCH','native text and live adapter')
    require(associate([s['state_ns'] for s in states],[p['state_ns'] for p in poses],6000)==list(range(len(poses))),
            'STATE_ASSOCIATION_MISMATCH','one-to-one state timestamps')
    camera_ids = associate(dataset.camera_ns,[s['image_ns'] for s in states],6000)
    records = []
    for i,(s,p,camera_id) in enumerate(zip(states,poses,camera_ids)):
        require(p['frame_id']=='global','FRAME_INVALID','pose parent')
        require(np.max(abs(np.array(s['position'])-p['position'])) <= 5.01e-7,'POSITION_MISMATCH','text vs live')
        q=p['quaternion_xyzw']
        require(np.max(abs(np.array(s['quaternion_wxyz'])-[q[3],*q[:3]]))<=5.01e-7,'QUATERNION_MISMATCH','text vs live')
        r=pose_observation(source_frame_id=camera_id,image_ns=dataset.camera_ns[camera_id],
            state_ns=p['state_ns'],recorder_ns=p['observed_system_ns'],position=p['position'],
            quaternion_xyzw=q,run_id=run_id,epoch=0,binary_sha256=install['binary_sha256'],
            raw_reference=dict(file='raw/pose.jsonl',topic='/poseimu',ordinal=i))
        r['observation_timestamp']=TimestampEvidence(p['observed_monotonic_ns'],'ns',run_id+':observer_monotonic',
            'ADAPTER_CALLBACK_RECEIPT','OBSERVED').to_dict()
        r['localization_fields'].update(velocity=s['velocity_world'],velocity_frame='backend_world',velocity_unit='m/s')
        r['raw_output'].update(velocity_source=f'raw/state_estimate.txt:row{i}',velocity_precision_decimal_places=6,
            initialization_evidence='CODE_SUPPORTED: pinned visualize returns before publication when uninitialized',
            imu_data_coverage=dict(first_ns=dataset.imu_ns[0],last_ns=dataset.imu_ns[-1],count=len(dataset.imu_ns),
                scope='adapter subscriber only; estimator callback ingestion not instrumented'))
        records.append(r)
    (root/'online').mkdir()
    write_json(root/'online/trajectory.json',dict(schema_version=1,trajectory_variant='online',records=records,
        normalized_after_run=True,source='live observer plus matched online state export; not optimized',
        contract='LocalizationEstimate v2 partial projections',contract_complete=False))
    propagated=[]
    for p in observed['odom']:
        require(p['frame_id']=='global' and p['child_frame_id']=='imu','FRAME_INVALID','odom convention')
        fields=wire_pose(p['position'],p['quaternion_xyzw'],p['state_ns'])
        fields.update(velocity=world_velocity(p['quaternion_xyzw'],p['velocity_local']),velocity_frame='backend_world',velocity_unit='m/s')
        propagated.append(dict(localization_fields=fields,trajectory_variant='online',stream='imu_propagated',
            contract_complete=False,tracking_state='UNKNOWN',publish_timestamp=None,localization_epoch=0,
            observed_monotonic_ns=p['observed_monotonic_ns']))
    write_json(root/'online/propagated.json',dict(trajectory_variant='online',records=propagated))
    def rates(rows):
        return dict(count=len(rows),sensor_time_hz=(len(rows)-1)*1e9/(rows[-1]['state_ns']-rows[0]['state_ns']),
            observer_wall_hz=(len(rows)-1)*1e9/(rows[-1]['observed_monotonic_ns']-rows[0]['observed_monotonic_ns'])) if len(rows)>1 else dict(count=len(rows))
    evidence=dict(input_observation='all image bytes and raw IMU SI values/times/order equal ASL',
        output_transport_exact=reconciled,
        timestamp_association='one-to-one within 6us text precision; camera index mapping retained',
        streams={k:rates(v) for k,v in observed.items()},final_optimized_trajectory=None,
        validity_policy='UNKNOWN backend tracking -> false validity, incomplete v2; not LOST or invalid numeric pose',
        velocity_policy='visual text v_IinG in m/s; propagated IMU local velocity rotated by Hamilton ItoG',
        source_to_publish_latency_s=None,estimator_imu_ingestion_count=None)
    write_json(root/'online/evidence.json',evidence)
    return records,evidence


def evaluate_adapter(root, dataset, records, baseline):
    from tools.openvins.native_accuracy import evaluate, interpolate, prepare_reference_quaternions, metrics
    # Outside the estimator container, after shutdown. Ground truth never enters it.
    evaluate(root/'raw',dataset.archive,root/'native-evaluation')
    with zipfile.ZipFile(dataset.archive) as z:
        gt=[l.split(',') for l in z.read('mav0/state_groundtruth_estimate0/data.csv').decode().splitlines() if l and not l.startswith('#')]
    ns=[r['backend_timestamp']['raw_value'] for r in records]
    q,normalization=prepare_reference_quaternions([[float(v) for v in row[4:8]] for row in gt])
    ref=interpolate([int(r[0]) for r in gt],[[float(v) for v in r[1:4]] for r in gt],q,ns,10000000)
    ids=ref['indices']
    result=metrics([ns[i] for i in ids],[records[i]['localization_fields']['translation'] for i in ids],
        [records[i]['localization_fields']['orientation_wxyz'] for i in ids],ref['positions'],ref['quaternions'])
    native=read_json(root/'native-evaluation/metrics.json')
    for field in ['ate','rpe_adjacent','rpe_1s','scale_diagnostic','alignment']:
        require(result[field]==native[field],'ADAPTER_METRICS_DIFFER','same-run raw versus adapter')
    baseline_report=read_json(baseline)
    coverage=coverage_consistency(native,baseline_report)
    require(coverage['pass'],'NATIVE_COVERAGE_DRIFT','same full-sequence coverage and reference required')
    comparison=metric_consistency(result,baseline_report)
    require(comparison['pass'],'NATIVE_BASELINE_DRIFT','predeclared repeat-run metric tolerances exceeded')
    result.update(evaluated_pose_count=len(ids),excluded_outside_gt=len(records)-len(ids),
        source_sha256=file_hash(root/'online/trajectory.json'),baseline_sha256=file_hash(baseline),
        same_run_metrics_exact=True,native_run02_consistency=comparison,reference_normalization=normalization,
        baseline_coverage_consistency=coverage,
        validity_scope='numerical pose evaluation only; unknown tracking excluded from operational validity claims',
        trajectory_variant='online',scale_correction=False)
    write_json(root/'adapter-evaluation.json',result)
    return result


def run(config):
    result=dict(status='REFUSED',execution_started=False,stage_result='PARTIAL')
    root=None
    try:
        require(config.get('purpose')=='VIO-P_PUBLIC_BASELINE','VIO_REAL_RUN_REQUIRED','explicit public baseline only')
        require(set(config)=={'schema_version','purpose','run_id','backend','archive','bag','installation','baseline','output'},
                'RUN_CONFIG_UNSUPPORTED','dedicated VIO config keys')
        require(config['schema_version']==1 and config['backend']=='OpenVINS','RUN_SCHEMA_UNSUPPORTED','OpenVINS v1')
        dataset=EurocVioDataset(config['archive'],config['bag'])
        install=read_json(config['installation'])
        require(install['commit']==PIN,'BACKEND_VERSION_UNSUPPORTED',PIN)
        for path,digest in install['files_sha256'].items():
            checked_file(path,digest)
        checked_file(Path(install['workspace'])/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf',install['binary_sha256'])
        require(install['image'].startswith('sha256:') and len(install['image'])==71,'IMAGE_UNPINNED','immutable image ID')
        # Assign only after exclusive creation: a refused repeat must never
        # overwrite an existing run.json from the exception handler.
        root=create_output(config['output'])
        repo=Path(__file__).resolve().parents[5]
        container='rm27-vio-'+__import__('uuid').uuid4().hex
        command=['docker','run','--rm','--name',container,'--network','none','--cpus','4','--memory','12g',
            '--user',f'{__import__("os").getuid()}:{__import__("os").getgid()}',
            '-v',f'{repo}:/repo:ro','-v',f'{install["source"]}:/upstream:ro',
            '-v',f'{install["workspace"]}:/ws:ro','-v',f'{dataset.bag}:/data:ro',
            '-v',f'{root/"raw"}:/out',install['image'],'bash','/repo/tools/openvins/run_adapter_ros2.sh']
        result.update(status='EXECUTING',execution_started=True,config=config,command=command,
            installation=install,dataset=dataset.provenance,final_optimized_trajectory=None)
        write_json(root/'run.json',result)
        with (root/'docker.log').open('w') as log:
            try:
                completed=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=900)
            except BaseException:
                # Docker client death alone does not stop a daemon-owned job.
                subprocess.run(['docker','stop','--time','15',container],stdout=log,stderr=subprocess.STDOUT,timeout=30)
                raise
        result['exit_code']=completed.returncode
        require(completed.returncode==0,'BACKEND_EXECUTION_FAILED','see docker/raw logs')
        records,evidence=normalize(root,dataset,install,config['run_id'])
        evaluation=evaluate_adapter(root,dataset,records,config['baseline'])
        for path,digest in install['files_sha256'].items():
            checked_file(path,digest)
        checked_file(dataset.archive,dataset.provenance['source_sha256'])
        for name,digest in dataset.provenance['artifacts_sha256'].items():
            checked_file(dataset.bag/name,digest)
        result.update(status='EXECUTION_COMPLETED',stage_result='ADAPTER_RUNTIME_AND_METRIC_PASS',
            online_trajectory='online/trajectory.json',evaluation='adapter-evaluation.json',evidence=evidence,
            output_sha256={str(p.relative_to(root)):file_hash(p) for p in root.rglob('*') if p.is_file() and p.name!='run.json'})
    except (ValueError,KeyError,OSError,subprocess.SubprocessError) as exc:
        result.update(status='FAILED' if result['execution_started'] else 'REFUSED',
            failure_code=getattr(exc,'code','VIO_RUNTIME_ERROR'),failure_reason=str(exc))
    if root is not None and root.is_dir():
        write_json(root/'run.json',result)
    return result


def create_output(path):
    root=Path(path).resolve()
    root.mkdir(parents=True,exist_ok=False)
    (root/'raw').mkdir()
    return root
