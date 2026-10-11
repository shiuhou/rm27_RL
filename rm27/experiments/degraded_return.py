"""Permanent return-camera loss with a continued PX4 LAND development trial.

The original mission fails. An explicit bounded landing policy continues the
same SITL physics until landed/automatic disarm, before closing processes.
Its result is fault termination, never full task success. No hardware endpoint.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np

from ..autopilot.px4.flight import run
from ..autopilot.px4.return_termination import landing_admission,continue_landing
from ..autopilot.px4.tag_height import TagHeightRuntime
from ..autopilot.px4.live_flight import ROOT




class PermanentBlackout:
    def __init__(self,camera,runtime,settings):
        self.camera,self.runtime,self.settings=camera,runtime,settings
        self.start=None;self.records=[]

    def capture(self,data,timestamp_us):
        frame=self.camera.capture(data,timestamp_us);now=float(data.time)
        if self.start is None and self.runtime.return_phase==self.settings['fault_phase']:
            self.start=now+self.settings['fault_delay_s']
        if self.start is None or now<self.start:return frame
        changed=replace(frame,image_rgb=np.zeros_like(frame.image_rgb))
        self.records.append(dict(timestamp_s=now,source_time_us=changed.timestamp_us,
            input_pixels_sha256=hashlib.sha256(frame.image_rgb.tobytes()).hexdigest(),
            fault_pixels_sha256=hashlib.sha256(changed.image_rgb.tobytes()).hexdigest()))
        return changed

    def close(self):self.camera.close()


def execute(scene,configuration,output,*,instance=46,speedup=2.):
    settings=json.loads(Path(configuration).read_text());output=Path(output).resolve()
    terminal={};fault=None

    class DegradedRuntime(TagHeightRuntime):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            if settings.get('camera_configuration'):
                self.vio_camera_config=Path(settings['camera_configuration']).resolve()

        def __enter__(self):
            nonlocal fault
            result=super().__enter__();self.cached_tag=None
            fault=PermanentBlackout(self.return_camera.camera,self,settings)
            self.return_camera.camera=fault
            return result

        def step(self,timeout=5.):
            row=super().step(timeout)
            if self.return_observation and self.return_observation.get('valid'):
                self.cached_tag=self.return_observation.copy()
            return row

        def __exit__(self,kind,exc,traceback):
            # Restrict this development intervention to the declared permanent
            # camera timeout. Other errors remain unqualified failures.
            reason=str(exc)
            eligible=(kind is not None and fault and fault.records and
                reason in ('AprilTag reacquisition timed out','AprilTag reacquisition attempts exhausted') and
                landing_admission(self.telemetry,self.cached_tag,float(self.plant.data.time),settings))
            terminal.update(passed=False,task_success=False,admitted=bool(eligible),trigger=reason,
                strategy='bounded normal LAND with fresh valid navigation and previously aligned assigned tag')
            try:
                if eligible:self.terminate()
            except Exception as error:
                terminal.update(passed=False,failure=f'{type(error).__name__}: {error}')
            finally:
                (self.output/'termination_summary.json').write_text(json.dumps(terminal,indent=2)+'\n')
                super().__exit__(kind,exc,traceback)

        def terminate(self):
            terminal.update(continue_landing(self,settings))

    run(scene,output,instance,dynamics=ROOT/'configs/dynamics_150g.json',profile='vio',
        speedup=speedup,runtime_factory=DegradedRuntime)
    summary=json.loads((output/'summary.json').read_text())
    terminal.update(original_mission_passed=summary.get('passed') is True,
        fault_frames=len(fault.records) if fault else 0,gate_version=settings['gate_version'],
        run_type='headless closed-loop SITL; same physics continued through termination')
    (output/'termination_summary.json').write_text(json.dumps(terminal,indent=2)+'\n')
    (output/'fault_configuration.json').write_text(json.dumps(settings,indent=2)+'\n')
    (output/'fault_injections.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in fault.records) if fault else '')
    (output/'termination_runner.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(terminal,indent=2));return 0 if terminal.get('passed') else 1


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--instance',type=int,default=46);p.add_argument('--speedup',type=float,default=2.)
    a=p.parse_args();return execute(a.scene,a.config,a.output,instance=a.instance,speedup=a.speedup)


if __name__=='__main__':raise SystemExit(main())
