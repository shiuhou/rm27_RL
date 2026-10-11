"""Exercise current-flight downward-camera faults after target contact/recovery."""
import argparse
import hashlib
import json
from pathlib import Path

from ..autopilot.px4 import image_mission
from ..autopilot.px4.return_faults import ImageFault,FaultCamera,assess
from .degraded_return import PermanentBlackout


def execute(scene,audit,mission,camera,fault_configuration,output,*,instance=48):
    output=Path(output).resolve();settings=json.loads(Path(fault_configuration).read_text())
    source=Path(__file__).read_bytes();fault=None;original=image_mission.run

    def run_fault(*args,**kwargs):
        base=kwargs['runtime_factory']
        class FaultRuntime(base):
            def __enter__(self):
                nonlocal fault
                result=super().__enter__()
                if settings['kind']=='permanent_missing':
                    fault=PermanentBlackout(self.return_camera.camera,self,dict(
                        fault_phase=settings['phase'],fault_delay_s=settings['delay_s']))
                    self.return_camera.camera=fault
                else:
                    fault=ImageFault(settings)
                    self.return_camera.camera=FaultCamera(self.return_camera.camera,self,fault)
                return result
        kwargs['runtime_factory']=FaultRuntime
        return original(*args,**kwargs)

    image_mission.run=run_fault
    try:
        image_mission.execute(scene,audit,output,configuration=mission,
            camera_configuration=camera,instance=instance)
    finally:image_mission.run=original
    summary=json.loads((output/'summary.json').read_text())
    records=fault.records if fault else []
    if settings['kind']=='permanent_missing':
        termination=summary.get('fault_termination',{})
        checks=dict(fault_injected=bool(records),task_failed=summary.get('passed') is False,
            termination=termination.get('passed') is True and termination.get('task_success') is False,
            target_route=summary.get('image_route',{}).get('passed') is True,
            physical_recovery=summary.get('independent_recovery_score',{}).get('passed') is True,
            physical_hover=summary.get('independent_hover_score',{}).get('passed') is True)
        result=dict(passed=all(checks.values()),checks=checks,
            task_success=False,safe_termination=termination,
            scope='Complete-route permanent return-camera failure termination; not mission success')
    else:
        path=output/'return_camera/observations.jsonl'
        observations=[json.loads(line) for line in path.open()] if path.exists() else []
        result=assess(summary,observations,records)
    result.update(configuration=settings,runner_sha256=hashlib.sha256(source).hexdigest(),
        run_type='headless closed-loop SITL complete image/VIO route with camera fault')
    (output/'full_fault_runner.py').write_bytes(source)
    (output/'fault_configuration.json').write_text(json.dumps(settings,indent=2)+'\n')
    (output/'fault_injections.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (output/'fault_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--camera-config',type=Path,required=True);p.add_argument('--fault-config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--instance',type=int,default=48)
    a=p.parse_args();return execute(a.scene,a.audit,a.config,a.camera_config,a.fault_config,a.output,instance=a.instance)


if __name__=='__main__':raise SystemExit(main())
