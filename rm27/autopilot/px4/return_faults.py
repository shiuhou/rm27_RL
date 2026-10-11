"""Inject faults into current-flight return images before AprilTag detection.

Fault scheduling sees only the mission phase and simulation clock. Detection,
guidance, EKF2 and PX4 LAND remain the normal current-flight software paths.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from .flight import run
from .live_flight import VioRuntime, ROOT


class ImageFault:
    def __init__(self, configuration):
        self.configuration=dict(configuration)
        self.start=None
        self.previous=None
        self.held=None
        self.records=[]
        if self.configuration['kind'] not in ('missing','stale','wrong_id'):
            raise ValueError('explicit missing/stale/wrong_id image fault required')
        if not 0<=self.configuration['delay_s'] or not 0<self.configuration['duration_s']<=5:
            raise ValueError('bounded positive image fault duration required')
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.dictionary=dictionary
        self.detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters())

    def transform(self, frame, phase, now):
        c=self.configuration
        if self.start is None and phase==c['phase']:
            self.start=now+c['delay_s']
        active=self.start is not None and self.start<=now<self.start+c['duration_s']
        if not active:
            self.previous=frame
            return frame
        changed=frame
        replacements=0
        if c['kind']=='missing':
            changed=replace(frame,image_rgb=np.zeros_like(frame.image_rgb))
        elif c['kind']=='stale':
            if self.held is None:self.held=self.previous
            if self.held is None:raise RuntimeError('stale-image fault lacks a prior acquired frame')
            changed=self.held  # Preserve acquisition time; never stamp an old image as current.
        else:
            gray=cv2.cvtColor(frame.image_rgb,cv2.COLOR_RGB2GRAY)
            corners,ids,_=self.detector.detectMarkers(gray)
            if ids is not None:
                pixels=frame.image_rgb.copy()
                for points,identity in zip(corners,np.asarray(ids).ravel()):
                    if int(identity)!=c['assigned_tag_id']:continue
                    marker=cv2.aruco.generateImageMarker(self.dictionary,c['replacement_tag_id'],256)
                    transform=cv2.getPerspectiveTransform(np.array([[0,0],[255,0],[255,255],[0,255]],np.float32),
                        np.asarray(points,np.float32).reshape(4,2))
                    warped=cv2.warpPerspective(marker,transform,(frame.width,frame.height),flags=cv2.INTER_NEAREST)
                    mask=np.zeros(gray.shape,np.uint8)
                    cv2.fillConvexPoly(mask,np.rint(points.reshape(4,2)).astype(np.int32),255)
                    pixels[mask>0]=warped[mask>0,None]
                    replacements+=1
                changed=replace(frame,image_rgb=pixels)
        self.records.append(dict(received_time_s=now,source_time_us=changed.timestamp_us,phase=phase,
            kind=c['kind'],replacement_count=replacements,
            input_pixels_sha256=hashlib.sha256(frame.image_rgb.tobytes()).hexdigest(),
            fault_pixels_sha256=hashlib.sha256(changed.image_rgb.tobytes()).hexdigest()))
        return changed


class FaultCamera:
    def __init__(self, camera, runtime, fault):
        self.camera,self.runtime,self.fault=camera,runtime,fault

    def capture(self, data, timestamp_us):
        frame=self.camera.capture(data,timestamp_us)
        return self.fault.transform(frame,self.runtime.return_phase,float(data.time))

    def close(self):self.camera.close()


def assess(summary, observations, injections):
    """Require normal LAND after re-identification, retaining mission success separately."""
    events=summary.get('events',[])
    reacquires=[e for e in events if e.get('event')=='REACQUIRE']
    failsafes=[e for e in events if e.get('event')=='STATUSTEXT' and 'failsafe' in e.get('text','').lower()]
    fresh_after=bool(injections and any(o.get('valid') and o.get('source_time_s',0)>
        injections[-1]['received_time_s'] for o in observations))
    landing=summary.get('landing_commitment',{})
    not_committed_during_fault=bool(injections and landing.get('timestamp_s',0)>injections[-1]['received_time_s'])
    wrong_injected=bool(injections and (injections[0]['kind']!='wrong_id' or
        any(i['replacement_count'] for i in injections)))
    return dict(passed=bool(summary.get('passed') is True and reacquires and fresh_after and
        not_committed_during_fault and wrong_injected and not failsafes),
        task_success=summary.get('passed') is True,fault_injected=bool(injections),
        reacquired=bool(reacquires),fresh_assigned_image_after_fault=fresh_after,
        no_landing_commitment_during_fault=not_committed_during_fault,
        actual_wrong_tag_injection=wrong_injected,unexpected_failsafes=failsafes,
        scope='temporary image fault handling; no permanent-loss or navigation-loss acceptance')


def execute(scene, output, configuration, *, instance=37, speedup=2.):
    output=Path(output).resolve()
    fault=ImageFault(configuration)

    class FaultRuntime(VioRuntime):
        def __enter__(self):
            rt=super().__enter__()
            if self.return_camera is None:raise RuntimeError('dedicated return camera missing')
            self.return_camera.camera=FaultCamera(self.return_camera.camera,self,fault)
            return rt

    code=run(scene,output,instance,dynamics=ROOT/'configs/dynamics_150g.json',profile='vio',
        speedup=speedup,runtime_factory=FaultRuntime)
    (output/'fault_configuration.json').write_text(json.dumps(configuration,indent=2)+'\n')
    (output/'fault_injections.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in fault.records))
    summary=json.loads((output/'summary.json').read_text())
    path=output/'return_camera/observations.jsonl'
    observations=[json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    assessment=assess(summary,observations,fault.records)
    (output/'fault_summary.json').write_text(json.dumps(assessment,indent=2)+'\n')
    return 0 if code==0 and assessment['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--kind',choices=('missing','stale','wrong_id'),required=True)
    p.add_argument('--phase',default='DESCEND');p.add_argument('--delay',type=float,default=.5)
    p.add_argument('--duration',type=float,default=1.)
    p.add_argument('--instance',type=int,default=37);p.add_argument('--speedup',type=float,default=2.)
    a=p.parse_args()
    return execute(a.scene,a.output,dict(kind=a.kind,phase=a.phase,delay_s=a.delay,duration_s=a.duration,
        assigned_tag_id=21,replacement_tag_id=22),instance=a.instance,speedup=a.speedup)


if __name__=='__main__':raise SystemExit(main())
