"""Read-only ROS2 live observer. No truth access, control messages or VIO math."""
import hashlib
import json
from pathlib import Path
import signal
import sys
import time
sys.path.insert(0,'/repo')
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rm27.perception.vision.localization.experiments.openvins_runtime import wire_pose, world_velocity


def main():
    rclpy.init()
    node=Node('rm27_openvins_observer')
    files={name:open('/out/'+name+'.jsonl','x',buffering=1) for name in ['cam0','imu0','pose','odom']}
    counts={name:0 for name in files}
    stopping=False
    def stop(*args):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    def receive(name,m):
        stamp=m.header.stamp
        row=dict(state_ns=stamp.sec*10**9+stamp.nanosec,
                 observed_monotonic_ns=time.monotonic_ns(),observed_system_ns=time.time_ns(),frame_id=m.header.frame_id)
        if name=='cam0':
            row.update(width=m.width,height=m.height,step=m.step,encoding=m.encoding,
                       pixels_sha256=hashlib.sha256(bytes(m.data)).hexdigest())
        elif name=='imu0':
            row.update(gyro=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
                       accel=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
        else:
            p,q=m.pose.pose.position,m.pose.pose.orientation
            row.update(position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],raw_covariance=list(m.pose.covariance))
            row['localization_fields']=wire_pose(row['position'],row['quaternion_xyzw'],row['state_ns'])
            row['contract_complete']=False
            if name=='odom':
                v=m.twist.twist.linear
                row.update(velocity_local=[v.x,v.y,v.z],child_frame_id=m.child_frame_id)
                row['localization_fields'].update(velocity=world_velocity(row['quaternion_xyzw'],row['velocity_local']),
                                                  velocity_frame='backend_world',velocity_unit='m/s')
        files[name].write(json.dumps(row,allow_nan=False)+'\n')
        counts[name]+=1
    # Read-only observer buffer only; upstream estimator's QoS is unchanged.
    imu_qos=QoSProfile(depth=10000,reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions=[node.create_subscription(kind,topic,lambda m,n=name:receive(n,m),qos)
        for name,kind,topic,qos in [('cam0',Image,'/cam0/image_raw',100),('imu0',Imu,'/imu0',imu_qos),
                                  ('pose',PoseWithCovarianceStamped,'/poseimu',100),('odom',Odometry,'/odomimu',1000)]]
    Path('/out/observer.ready').write_text('subscriptions created\n')
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        for f in files.values():
            f.close()
        Path('/out/observer-counts.json').write_text(json.dumps(counts,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
