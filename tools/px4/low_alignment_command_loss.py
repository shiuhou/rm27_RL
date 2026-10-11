"""Command loss after low-altitude assigned-tag alignment, with no LAND injection."""
import argparse
import json
from pathlib import Path

from rm27.autopilot.px4 import image_mission
from rm27.autopilot.px4.return_fault_batch import digest,frozen_inputs
from rm27.experiments import image_conditions,vio_initial_admission
from . import full_return_command_loss as original
from .ground_rest import prepare
from .initial_vio_admission import StableInitialAdmission


def eligible(tag,telemetry,phase,now):
    position=telemetry.get('local_position',[])
    return bool(phase=='ALIGN' and len(position)==3 and .10<=-position[2]<=.14 and
        original.eligible(tag,telemetry,phase,now))


def run(args):
    settings=json.loads(args.conditions.read_text())
    if settings.get('initial_vio_admission') is not True:
        raise ValueError('Explicit stable native admission required')
    files=[args.audit,args.config,args.camera_config,args.conditions,Path(__file__),Path(original.__file__),
        *[Path(__file__).with_name(n+'.py') for n in ['ground_rest','causal_flight','initial_vio_admission']]]
    hashes=frozen_inputs(args.scene,args.config);hashes.update({str(p.resolve()):digest(p) for p in files})
    manifest=dict(input_sha256=hashes,sources={str(p):p.read_text() for p in files if p.suffix=='.py'},
        scope='Complete image/contact/recovery route followed by command loss only in ALIGN at10–14 cm estimated height, fresh assigned image within3 mm and slow navigation for0.25 s. No commands after interruption; onboard Offboard-loss LAND required. Earlier return and arbitrary-route loss remain unqualified.')
    with args.output.with_name(args.output.name+'-low-command-loss-manifest.json').open('x') as stream:
        stream.write(json.dumps(manifest,indent=2)+'\n')
    selected,_=prepare(args.output,retry_alignment=True,causal_inputs=True)
    original_run=image_mission.run;original_guard=vio_initial_admission.initial_consistent
    terminal={};injection={};command_records=[]
    def fault_run(*positional,**keywords):
        base=keywords['runtime_factory']
        class LostRuntime(base):
            def __init__(self,*p,**k):
                super().__init__(*p,**k)
                self.loss=None;self.stable=None;self.blocked=0;self.loss_events=[]
            def receive(self):
                messages=super().receive()
                if self.loss is not None:
                    for message in messages:
                        if message.get_type() in ('COMMAND_ACK','STATUSTEXT'):
                            self.loss_events.append(dict(timestamp_s=float(self.plant.data.time),message=message.to_dict()))
                return messages
            def step(self,timeout=5.):
                row=super().step(timeout);now=float(self.plant.data.time)
                if self.loss is None:
                    if eligible(self.return_observation,self.telemetry,self.return_phase,now):
                        self.stable=now if self.stable is None else self.stable
                        if now-self.stable>=.25:
                            self.loss=now
                            injection.update(source='fresh assigned image and PX4 telemetry, no truth',
                                onset_s=now,phase=self.return_phase,tag=self.return_observation,
                                telemetry=self.telemetry.copy())
                            (self.output/'command_loss_injection.json').write_text(json.dumps(injection,indent=2)+'\n')
                    else:self.stable=None
                return row
            def position(self,*p,**k):
                if self.loss is not None:
                    self.blocked+=1
                    raise RuntimeError('Upstream command transport unavailable')
                return super().position(*p,**k)
            def mode(self,*p,**k):
                if self.loss is not None:
                    self.blocked+=1
                    raise RuntimeError('Upstream command transport unavailable')
                return super().mode(*p,**k)
            def command(self,*p,**k):
                command_records.append(dict(time_s=float(self.plant.data.time),after_interruption=self.loss is not None,command=p[0] if p else None))
                if self.loss is not None:
                    self.blocked+=1
                    raise RuntimeError('Upstream command transport unavailable')
                return super().command(*p,**k)
            def __exit__(self,kind,error,traceback):
                if self.loss is not None and str(error)=='Upstream command transport unavailable':
                    try:terminal.update(original.monitor(self))
                    except Exception as failure:terminal.update(passed=False,task_success=False,failure=f'{type(failure).__name__}: {failure}')
                    terminal.update(trigger=str(error),blocked_commands=self.blocked,onset_s=self.loss,
                        gate_version='PX4_LOW_ALIGNMENT_COMMAND_LOSS_DEVELOPMENT_V1')
                    (self.output/'termination_summary.json').write_text(json.dumps(terminal,indent=2)+'\n')
                return super().__exit__(kind,error,traceback)
        keywords['runtime_factory']=LostRuntime
        return selected(*positional,**keywords)
    image_mission.run=fault_run;vio_initial_admission.initial_consistent=StableInitialAdmission()
    try:
        image_conditions.execute(args.scene,args.audit,args.config,args.camera_config,args.conditions,args.output,instance=args.instance)
    finally:
        image_mission.run=original_run;vio_initial_admission.initial_consistent=original_guard
    summary=json.loads((args.output/'summary.json').read_text())
    checks=dict(injected=bool(injection),failed_task=summary['passed'] is False,
        original_route=summary.get('image_route',{}).get('passed') is True,
        original_recovery=summary.get('independent_recovery_score',{}).get('passed') is True,
        physical_termination=terminal.get('passed') is True,
        no_external_commands_after_interruption=not any(r['after_interruption'] for r in command_records))
    result=dict(passed=all(checks.values()),checks=checks,task_success=False,termination=terminal,
        scope=manifest['scope'],run_type='headless closed-loop SITL low-alignment upstream command-loss development')
    (args.output/'upstream_command_calls.json').write_text(json.dumps(command_records,indent=2)+'\n')
    (args.output/'command_loss_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('scene',type=Path)
    for flag in ['audit','config','camera-config','conditions','output']:parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--instance',type=int,default=76)
    raise SystemExit(run(parser.parse_args()))
