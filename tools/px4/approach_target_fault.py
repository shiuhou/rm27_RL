"""Alter actual target pixels during visual approach, before contact commitment."""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from rm27.autopilot.px4 import image_mission
from rm27.autopilot.px4.return_fault_batch import digest,frozen_inputs
from rm27.experiments import image_conditions,vio_initial_admission
from rm27.experiments.fast_bearing import detect
from rm27.perception.sitl.image_target_detector import DetectorConfig,ImageTargetDetector,_mask
from .ground_rest import prepare
from .initial_vio_admission import StableInitialAdmission


def alter_target(frame,fovy,kind):
    detector=ImageTargetDetector(DetectorConfig(fovy_deg=fovy))
    bearing=detect(detector,frame,now_us=frame.timestamp_us)
    if not bearing.safe_for_control:
        return frame,0
    focal=(frame.height/2)/math.tan(math.radians(fovy)/2)
    center=((frame.width-1)/2+focal*math.tan(bearing.yaw_rad),
        (frame.height-1)/2-focal*math.tan(bearing.pitch_rad))
    blue=_mask(cv2.cvtColor(frame.image_rgb,cv2.COLOR_RGB2HSV),detector.config.blue_hue)
    contours,_=cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    selected=[c for c in contours if cv2.pointPolygonTest(c,center,False)>=0]
    if len(selected)!=1:
        return frame,0
    region=np.zeros_like(blue)
    cv2.drawContours(region,selected,-1,255,cv2.FILLED)
    selected=(region!=0)&(blue!=0)
    pixels=frame.image_rgb.copy()
    pixels[selected]=[30,30,30] if kind=='occlusion' else [220,20,20]
    return replace(frame,image_rgb=pixels),int(np.count_nonzero(selected))


class ApproachCamera:
    def __init__(self,camera,route,configuration,fovy):
        self.camera,self.route,self.configuration,self.fovy=camera,route,configuration,fovy
        self.start=None;self.records=[]

    def capture(self,data,timestamp_us):
        frame=self.camera.capture(data,timestamp_us)
        now=float(data.time)
        if self.start is None and self.route.state=='VISUAL_APPROACH':
            self.start=now+self.configuration['delay_s']
        if (frame is not None and self.start is not None and
                self.start<=now<self.start+self.configuration['duration_s']):
            changed,count=alter_target(frame,self.fovy,self.configuration['kind'])
            self.records.append(dict(received_time_s=now,source_time_us=frame.timestamp_us,
                route_state=self.route.state,changed_pixels=count,
                original_sha256=hashlib.sha256(frame.image_rgb.tobytes()).hexdigest(),
                delivered_sha256=hashlib.sha256(changed.image_rgb.tobytes()).hexdigest()))
            return changed
        return frame

    def close(self):
        self.camera.close()


def run(args):
    if not .0<=args.delay<=.5 or not (.2<=args.duration<=.8 or 1.5<=args.duration<=3.):
        raise ValueError('Explicit bounded pre-commitment image fault required')
    settings=dict(kind=args.kind,delay_s=args.delay,duration_s=args.duration)
    if json.loads(args.conditions.read_text()).get('initial_vio_admission') is not True:
        raise ValueError('Stable first VIO admission required')
    hashes=frozen_inputs(args.scene,args.config)
    paths=[args.audit,args.camera_config,args.conditions,Path(__file__),
        *[Path(__file__).with_name(n+'.py') for n in ['ground_rest','causal_flight','initial_vio_admission']]]
    hashes.update({str(p.resolve()):digest(p) for p in paths})
    manifest=dict(input_sha256=hashes,fault=settings,
        sources={str(p):p.read_text() for p in paths if p.suffix=='.py'},
        scope='Pixel-defined target occlusion or wrong team colour during VISUAL_APPROACH; background and acquisition time preserved. No simulator target pose or detector-output forgery.')
    with args.output.with_name(args.output.name+'-approach-fault-manifest.json').open('x') as stream:
        stream.write(json.dumps(manifest,indent=2)+'\n')
    selected_run,_=prepare(args.output,retry_alignment=True,causal_inputs=True)
    original_run=image_mission.run;original_guard=vio_initial_admission.initial_consistent
    camera_holder=[];trace=[]
    fovy=json.loads(args.camera_config.read_text())['fovy_deg']
    def fault_run(*positional,**keywords):
        route=keywords['route'];base=keywords['runtime_factory']
        class FaultRuntime(base):
            def __enter__(self):
                result=super().__enter__()
                camera=ApproachCamera(self.vio.camera,route,settings,fovy)
                self.vio.camera=camera;camera_holder.append(camera)
                self.previous_state=None
                return result
            def step(self,timeout=5.):
                row=super().step(timeout)
                if route.state!=self.previous_state:
                    trace.append(dict(time_s=float(self.plant.data.time),state=route.state,aborted=route.aborted))
                    self.previous_state=route.state
                return row
        keywords['runtime_factory']=FaultRuntime
        return selected_run(*positional,**keywords)
    image_mission.run=fault_run
    vio_initial_admission.initial_consistent=StableInitialAdmission()
    try:
        image_conditions.execute(args.scene,args.audit,args.config,args.camera_config,args.conditions,
            args.output,instance=args.instance)
    finally:
        image_mission.run=original_run;vio_initial_admission.initial_consistent=original_guard
    records=camera_holder[0].records if camera_holder else []
    summary=json.loads((args.output/'summary.json').read_text())
    route=summary.get('image_route',{})
    changed=[r for r in records if r['changed_pixels']>0 and r['original_sha256']!=r['delivered_sha256']]
    abort=route.get('aborted') is True and route.get('abort_reason')=='armor image lost before commitment'
    checks=dict(injected_before_commitment=bool(changed) and changed[0]['route_state']=='VISUAL_APPROACH',
        expected_task=(summary.get('passed') is True if args.duration<1 else summary.get('passed') is False and abort),
        landed_and_disarmed=summary.get('auto_disarm',{}).get('confirmed') is True,
        original_physical_rest=summary.get('independent_resting_score',{}).get('passed') is True,
        independent_physical_monitor=summary.get('independent_physical_monitor',{}).get('passed') is True)
    if args.duration>=1:
        aborted=[t for t in trace if t['aborted'] and t['state']=='RETREAT']
        checks['no_contact_commitment']=route.get('impact') is None and not any(t['state']=='COMMIT' for t in trace)
        checks['bounded_abort']=bool(aborted and changed and 0<=aborted[0]['time_s']-changed[0]['received_time_s']<=1.35)
    result=dict(passed=all(checks.values()),task_success=summary.get('passed') is True,
        checks=checks,fault=settings,scope=manifest['scope'],
        run_type='headless current-flight image/VIO closed-loop SITL target-fault development',
        route_state_trace=trace,changed_frames=len(changed))
    (args.output/'approach_target_injections.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (args.output/'approach_target_fault_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path)
    for flag in ['audit','config','camera-config','conditions','output']:
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--kind',choices=['occlusion','wrong_colour'],required=True)
    parser.add_argument('--delay',type=float,default=.2)
    parser.add_argument('--duration',type=float,required=True)
    parser.add_argument('--instance',type=int,default=70)
    raise SystemExit(run(parser.parse_args()))
