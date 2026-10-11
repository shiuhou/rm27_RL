"""ROS2 playback of current-flight images/IMU, isolated inside one container.

No physics truth is mounted or read. Output state and receipt clocks stay raw;
receiving a ROS pose alone does not establish full flight eligibility.
"""
import argparse
import csv
import json
from pathlib import Path
import signal
import subprocess
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def stamp(message, ns, frame):
    message.header.stamp.sec=ns//10**9
    message.header.stamp.nanosec=ns%10**9
    message.header.frame_id=frame


def main():
    p=argparse.ArgumentParser();p.add_argument('--rate',type=float,default=.5);a=p.parse_args()
    if a.rate<=0:raise ValueError('playback rate must be positive')
    native=None;log=None;node=None
    result=dict(passed=False,scope='recorded sensor input, native OpenVINS; no flight control qualification')
    counts=dict(camera=0,imu=0,pose=0,odom=0,native_state=0)
    try:
        rclpy.init();node=Node('rm27_simulated_flight_player')
        cam_pub=node.create_publisher(Image,'/cam0/image_raw',QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE))
        imu_pub=node.create_publisher(Imu,'/imu0',QoSProfile(depth=10000,reliability=ReliabilityPolicy.RELIABLE))
        outputs={name:open('/out/'+name+'.jsonl','x',buffering=1) for name in ['pose','odom','native_state']}
        def receive(name,message):
            p,q=message.pose.pose.position,message.pose.pose.orientation
            row=dict(source_time_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                     receipt_monotonic_ns=time.monotonic_ns(),position=[p.x,p.y,p.z],
                     quaternion_xyzw=[q.x,q.y,q.z,q.w],pose_covariance=list(message.pose.covariance),
                     frame_id=message.header.frame_id,contract_complete=False,
                     reason='native tracking/reset and publication evidence still required')
            if name=='odom':
                v=message.twist.twist.linear
                row.update(velocity_local=[v.x,v.y,v.z],child_frame_id=message.child_frame_id)
            outputs[name].write(json.dumps(row,allow_nan=False)+'\n');counts[name]+=1
        subscriptions=[node.create_subscription(cls,topic,lambda msg,n=name:receive(n,msg),1000)
            for cls,topic,name in [(PoseWithCovarianceStamped,'/poseimu','pose'),(Odometry,'/odomimu','odom')]]
        def native_state(message):
            row=json.loads(message.data);row['receipt_monotonic_ns']=time.monotonic_ns()
            outputs['native_state'].write(json.dumps(row,allow_nan=False)+'\n');counts['native_state']+=1
        subscriptions.append(node.create_subscription(String,'/rm27_native_state',native_state,1000))
        log=open('/out/native.log','x')
        command=['/ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf','--ros-args',
                 '-p','config_path:=/out/config/estimator_config.yaml',
                 '-p','save_total_state:=true','-p','filepath_est:=/out/state_estimate.txt',
                 '-p','filepath_std:=/out/state_deviation.txt','-p','verbosity:=INFO']
        native=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+30
        while not (cam_pub.get_subscription_count()>0 and imu_pub.get_subscription_count()>0):
            if native.poll() is not None:raise RuntimeError('OpenVINS exited before sensor subscription')
            if time.monotonic()>deadline:raise RuntimeError('sensor subscription discovery timed out')
            rclpy.spin_once(node,timeout_sec=.1)
        def data(path):
            with open(path) as stream:return [r for r in csv.reader(stream) if r and not r[0].startswith('#')]
        cams=data('/data/cam0/data.csv');imus=data('/data/imu0/data.csv')
        events=sorted([(int(r[0]),'imu',r) for r in imus]+[(int(r[0]),'camera',r) for r in cams],key=lambda r:(r[0],r[1]=='camera'))
        zero=events[0][0];start=time.monotonic();last_progress=start
        for ns,kind,row in events:
            while True:
                delay=start+(ns-zero)/1e9/a.rate-time.monotonic()
                if delay<=0:break
                rclpy.spin_once(node,timeout_sec=min(.002,delay))
            if native.poll() is not None:raise RuntimeError('OpenVINS exited during input playback')
            if kind=='imu':
                message=Imu();stamp(message,ns,'body_flu')
                message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z=map(float,row[1:4])
                message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z=map(float,row[4:7])
                imu_pub.publish(message)
            else:
                image=cv2.imread('/data/cam0/data/'+row[1],cv2.IMREAD_GRAYSCALE)
                if image is None:raise ValueError('missing input image: '+row[1])
                message=Image();stamp(message,ns,'camera_optical')
                message.height,message.width=image.shape;message.step=message.width;message.encoding='mono8';message.data=image.tobytes()
                cam_pub.publish(message)
            counts[kind]+=1
            rclpy.spin_once(node,timeout_sec=0)
            if time.monotonic()-last_progress>20:
                print(json.dumps(dict(sim_time_s=ns/1e9,counts=counts)),flush=True);last_progress=time.monotonic()
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        native.send_signal(signal.SIGINT)
        deadline=time.monotonic()+20
        while native.poll() is None:
            if time.monotonic()>deadline:raise TimeoutError('OpenVINS clean shutdown timed out')
            rclpy.spin_once(node,timeout_sec=.02)
        result.update(native_exit_code=native.returncode,
            passed=native.returncode==0 and counts['pose']>0 and counts['odom']>0 and counts['camera']==len(cams) and counts['imu']==len(imus))
    except Exception as exc:
        result['failure']=str(exc)
    finally:
        if native and native.poll() is None:
            native.terminate()
            try:native.wait(timeout=5)
            except subprocess.TimeoutExpired:native.kill();native.wait(timeout=5)
        if log:log.close()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        result['counts']=counts
        Path('/out/runtime_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result.get('passed') else 1

if __name__=='__main__':raise SystemExit(main())
