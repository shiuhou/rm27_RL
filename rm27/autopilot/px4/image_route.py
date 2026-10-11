"""Surveyed corridor plus image-driven armor approach and causal impact detection.

Task inputs are timestamped RGB poses, PX4 telemetry and HIGHRES_IMU. This
controller never reads a simulator body, contact flag or target identity.
"""
import math
import json

import numpy as np
from scipy.spatial.transform import Rotation

from ...perception.sitl.dataset import camera_to_imu
from ...experiments.route_experiment import path_distance

BASIS=np.diag([1.,-1.,-1.])


class ImageRoute:
    def __init__(self, audit, mission, camera_configuration):
        if audit.get('passed') is not True or not audit.get('surface_route_clear'):
            raise ValueError('reviewed clear corridor required')
        self.audit,self.mission=audit,mission
        self.points=np.asarray(audit['waypoints_world_m'],dtype=float)
        self.origin=self.points[0]-[0,0,.6]
        self.armor_prior=np.asarray(mission['surveyed_armor_center_m'],dtype=float)
        self.armor_normal=np.asarray(mission['surveyed_armor_normal'],dtype=float)
        self.armor_normal/=np.linalg.norm(self.armor_normal)
        self.camera_to_body=camera_to_imu(camera_configuration)
        self.armor_center=self.armor_prior.copy()
        self.reference=self.points[0].copy();self.reference_velocity=np.zeros(3)
        self.state='OUTBOUND';self.index=1;self.last_time=None;self.visited=[];self.impact=None
        self.physics_deadline=mission['physics_deadline_s']
        self.maximum_speed_m_s=mission['maximum_flight_speed_m_s']
        self.maximum_tilt_deg=mission['maximum_tilt_deg']
        self.horizontal_command_limit_m_s=mission['horizontal_command_limit_m_s']
        self.vertical_command_limit_m_s=mission['vertical_command_limit_m_s']
        self.pose=None;self.pose_time=None;self.last_image=None;self.navigation=None
        self.navigation_time=None;self.imu=None;self.imu_time=None;self.stable_since=None
        self.commit_time=None;self.commit_position=None;self.last_impact_imu=None
        self.aborted=False;self.abort_reason=None;self.contact_impulses=[]
        self.contact_point=None
        if 'contact_point_body_flu_m' in mission:
            self.contact_point=np.asarray(mission['contact_point_body_flu_m'],float)
            if self.contact_point.shape!=(3,) or not np.isfinite(self.contact_point).all():
                raise ValueError('configured finite body-frame contact point required')
        self.contact_bias=np.zeros(3);self.imu_rates=None

    def _event(self, name, time, position):
        row=dict(state=name,time_s=float(time),position=np.asarray(position).tolist())
        self.visited.append(row);print(json.dumps(dict(image_route_event=row)),flush=True)

    def observe(self, pose, telemetry, now):
        self.navigation=telemetry.copy();self.navigation_time=now
        if pose is None or not pose.valid or self.last_image==pose.source_time_us:return
        source=pose.source_time_us/1e6
        if not 0<=now-source<=.2:return
        if max(now-telemetry.get(k,-100) for k in ('local_time','attitude_time','estimator_time'))>.2:return
        if not telemetry.get('estimator_flags',0)&8:return
        q=np.asarray(telemetry['quaternion'],float)
        if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.02:return
        rotation=BASIS@Rotation.from_quat(q[[1,2,3,0]]).as_matrix()@BASIS
        rates=BASIS@np.asarray(telemetry['rates'])
        rotation=rotation@Rotation.from_rotvec(rates*(source-telemetry['attitude_time'])).as_matrix()
        position=self.origin+BASIS@(np.asarray(telemetry['local_position'])+
            np.asarray(telemetry['local_velocity'])*(source-telemetry['local_time']))
        relative=self.camera_to_body[:3,3]+self.camera_to_body[:3,:3]@np.asarray(pose.translation_camera_m)
        normal=rotation@self.camera_to_body[:3,:3]@np.asarray(pose.normal_camera)
        measured=position+rotation@relative
        if np.linalg.norm(measured-self.armor_prior)>.2 or normal@self.armor_normal<.8:return
        self.last_image=pose.source_time_us
        if pose.position_std_m>.02:return
        self.pose=measured;self.pose_time=source
        if self.state in ('ACQUIRE','VISUAL_APPROACH'):
            self.armor_center=measured.copy()

    def observe_imu(self, message):
        values=np.asarray([message.xacc,message.yacc,message.zacc],dtype=float)
        if np.isfinite(values).all():
            self.imu=values;self.imu_time=message.time_usec/1e6
            rates=np.asarray([getattr(message,k,float('nan')) for k in ('xgyro','ygyro','zgyro')])
            self.imu_rates=rates if np.isfinite(rates).all() and getattr(message,'fields_updated',63)&56==56 else None

    def fresh_image(self, now):
        return self.pose_time is not None and 0<=now-self.pose_time<=.2

    def _move(self, goal, rate, dt):
        delta=np.asarray(goal)-self.reference;distance=float(np.linalg.norm(delta))
        if distance<1e-9:self.reference_velocity[:]=0;return self.reference.copy()
        acceleration=.35 if abs(delta[2])>np.linalg.norm(delta[:2]) else .5
        rate=min(rate,math.sqrt(2*acceleration*distance))
        self.reference_velocity=delta/distance*rate
        self.reference+=delta*min(1.,rate*dt/distance)
        if rate*dt>=distance:self.reference_velocity[:]=0
        return self.reference.copy()

    def check(self, position):
        path=np.vstack([self.points,self.armor_center])
        if path_distance(position,path)>.3:
            raise RuntimeError('Image mission left reviewed corridor tube')

    def contact_allowed(self, contact):
        # Called by the independent physical safety audit, not the task actor.
        return self.mission['armor_geom'] in contact['geoms'] and self.state in ('COMMIT','RECOVER','RETREAT')

    def _impact(self, now, position, velocity):
        if self.imu is None or self.imu_time==self.last_impact_imu or not 0<=now-self.imu_time<=.1:return False
        self.last_impact_imu=self.imu_time
        tel=self.navigation
        q=np.asarray(tel['quaternion']);rotation=BASIS@Rotation.from_quat(q[[1,2,3,0]]).as_matrix()@BASIS
        acceleration=rotation@(BASIS@self.imu)+[0,0,-9.81]
        outward=float(acceleration@self.armor_normal)
        # Gyro and acceleration belong to the same fresh HIGHRES_IMU sample.
        # ATTITUDE_QUATERNION can arrive one control tick later; consuming the
        # impulse while looking at its previous rates added a 20 ms delay.
        rate=float(np.linalg.norm(self.imu_rates if self.imu_rates is not None else tel['rates']))
        rate_fresh=self.imu_rates is not None or 0<=now-tel.get('attitude_time',-100)<=.1
        # A low-speed impact may last less than one 20 ms IMU publication
        # interval. Requiring two acceleration peaks misses that first hit.
        # Admission happens only in the bounded image-committed contact phase.
        strong=outward>=self.mission['impact_acceleration_threshold_m_s2']
        rate_spike=rate_fresh and rate>=self.mission['impact_rate_threshold_rad_s']
        if not (strong or rate_spike):return False
        self.contact_impulses.append((self.imu_time,outward,rate))
        self.impact=dict(time_s=now,position_world_m=position.tolist(),velocity_world_m_s=velocity.tolist(),
            armor_normal_speed_m_s=-float(velocity@self.armor_normal),source='fresh IMU impulse or rate spike within image contact commitment',
            imu_samples=self.contact_impulses[-1:])
        self._event('inferred_armor_contact',now,position)
        return True

    def target(self, time, position, speed, contacts=(), velocity=None):
        if contacts:raise ValueError('Simulator contacts are forbidden task inputs')
        position=np.asarray(position);velocity=np.asarray(velocity)
        if self.navigation is None or time-self.navigation.get('estimator_time',-100)>.3 or not self.navigation.get('estimator_flags',0)&8:
            raise RuntimeError('Image mission navigation unavailable')
        dt=0. if self.last_time is None else time-self.last_time;self.last_time=time
        if self.state=='OUTBOUND':
            goal=self.points[self.index]
            if np.linalg.norm(position-goal)<.12 and speed<.10:
                self._event('outbound_'+str(self.index),time,position)
                self.index+=1;self.reference=position.copy()
                if self.index==len(self.points):
                    self.state='ACQUIRE';self.stable_since=time
                else:goal=self.points[self.index]
            return self._move(goal,.35,dt)
        if self.state=='ACQUIRE':
            if self.fresh_image(time):
                self.state='VISUAL_APPROACH';self.reference=position.copy();self.stable_since=None
                self._event('image_target_acquired',time,position)
            elif time-self.stable_since>5:
                self.aborted=True;self.abort_reason='armor acquisition timeout';self.state='RETREAT'
            return self._move(self.points[-1],.06,dt)
        if self.state=='VISUAL_APPROACH':
            if not self.fresh_image(time):
                self.stable_since=time if self.stable_since is None else self.stable_since
                self.reference=position.copy();self.reference_velocity[:]=0
                if time-self.stable_since>1:
                    self.aborted=True;self.abort_reason='armor image lost before commitment';self.state='RETREAT'
                return self.reference.copy()
            self.stable_since=None
            offset=np.zeros(3)
            if self.contact_point is not None:
                q=np.asarray(self.navigation['quaternion'])
                rotation=BASIS@Rotation.from_quat(q[[1,2,3,0]]).as_matrix()@BASIS
                offset=rotation@self.contact_point
            displacement=position+offset-self.armor_center
            distance=float(displacement@self.armor_normal)
            tangent=displacement-distance*self.armor_normal
            attitude=self.navigation
            rates=np.linalg.norm(attitude['rates'])
            staging=self.mission['tip_staging_distance_m'] if self.contact_point is not None else .12
            tangent_gate=.008 if self.contact_point is not None else .01
            speed_gate=.12 if self.contact_point is not None else .15
            if distance<staging+.005 and np.linalg.norm(tangent)<tangent_gate and speed<speed_gate and rates<.3:
                self.state='COMMIT';self.commit_time=time;self.commit_position=self.armor_center.copy()
                self.contact_bias=-offset
                self.reference=position.copy()
                self._event('image_guard_tip_commitment' if self.contact_point is not None else 'image_contact_commitment',time,position)
            return self._move(self.armor_center-offset+self.armor_normal*staging,.03,dt)
        if self.state=='COMMIT':
            if self._impact(time,position,velocity):
                self.state='RECOVER';self.stable_since=None
                self.reference=position+self.armor_normal*.025
            elif time-self.commit_time>self.mission['contact_commitment_lease_s']:
                self.aborted=True;self.abort_reason='bounded contact lease exhausted';self.state='RETREAT'
                self.reference=position.copy()
            else:
                stop_distance=-.002 if self.contact_point is not None else .02
                return self._move(self.commit_position+self.contact_bias+self.armor_normal*stop_distance,.1,dt)
        if self.state=='RECOVER':
            tel=self.navigation;q=np.asarray(tel['quaternion'])
            rotation=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
            rates=np.asarray(tel['rates'])
            stable=rotation[2,2]>=math.cos(math.radians(10)) and np.linalg.norm(rates)<=math.radians(30) and speed<=.3
            self.stable_since=(time if self.stable_since is None else self.stable_since) if stable else None
            if self.stable_since is not None and time-self.stable_since>=1:
                self.state='RETREAT';self._event('estimated_recovery_confirmed',time,position)
            if time-self.impact['time_s']>4:
                self.aborted=True;self.abort_reason='recovery timeout';self.state='RETREAT'
            return self._move(self.armor_center+self.contact_bias+self.armor_normal*.6,.12,dt)
        if self.state=='RETREAT':
            if np.linalg.norm(position-self.points[-1])<.12 and speed<.1:
                self.state='INBOUND';self.index=len(self.points)-2;self.reference=position.copy()
            else:return self._move(self.points[-1],.12,dt)
        goal=self.points[self.index]
        if np.linalg.norm(position-goal)<(.03 if self.index==0 else .12) and speed<.08:
            self._event('inbound_'+str(self.index),time,position)
            if self.index==0:return None
            self.index-=1;goal=self.points[self.index];self.reference=position.copy()
        return self._move(goal,.35,dt)

    def result(self):
        return dict(passed=self.impact is not None and not self.aborted,visited=self.visited,impact=self.impact,
            contact_point_body_flu_m=None if self.contact_point is None else self.contact_point.tolist(),
            committed_contact_bias_world_m=self.contact_bias.tolist(),
            aborted=self.aborted,abort_reason=self.abort_reason,source='RGB metric pose plus PX4 telemetry and IMU',
            scope='complete-mission development, independently scored; no real hardware qualification')
