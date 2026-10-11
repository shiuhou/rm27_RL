"""Declared native-coordinate reset during known-dock hover; no pose passthrough."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

from rm27.autopilot.px4 import image_mission
from rm27.autopilot.px4.runtime import Runtime
from rm27.autopilot.px4.return_termination import continue_landing
from rm27.autopilot.px4.return_fault_batch import digest,frozen_inputs
from rm27.experiments import image_conditions,vio_initial_admission
from .ground_rest import prepare
from .initial_vio_admission import StableInitialAdmission
from .native_failure_termination import admission,range_fallback
from .registered_epoch import RegisteredEpochContract


def run(args):
    if not .3<=args.delay<=2.:
        raise ValueError('Explicit dock-hover localization reset schedule required')
    if json.loads(args.conditions.read_text()).get('initial_vio_admission') is not True:
        raise ValueError('Stable native first registration required')
    files=[args.audit,args.config,args.camera_config,args.conditions,Path(__file__),
        *[Path(__file__).with_name(n+'.py') for n in ['registered_epoch','ground_rest','causal_flight',
            'initial_vio_admission','native_failure_termination']]]
    hashes=frozen_inputs(args.scene,args.config);hashes.update({str(p.resolve()):digest(p) for p in files})
    manifest=dict(input_sha256=hashes,sources={str(p):p.read_text() for p in files if p.suffix=='.py'},
        delay_s=args.delay,declared_native_coordinate_jump_m=[.5,0,0],
        scope='Synthetic native publication epoch/origin reset at contract boundary; current flight still supplies native images and IMU. Known initial dock hover only, no real native restart claim.')
    with args.output.with_name(args.output.name+'-epoch-manifest.json').open('x') as stream:
        stream.write(json.dumps(manifest,indent=2)+'\n')
    selected,_=prepare(args.output,retry_alignment=True,causal_inputs=True)
    original=image_mission.run;original_guard=vio_initial_admission.initial_consistent
    records=[];termination={};onset={}
    def fault_run(*positional,**keywords):
        base=keywords['runtime_factory']
        class EpochRuntime(base):
            def __enter__(self):
                result=super().__enter__()
                self.epoch_failed=False;self.schedule=None
                original_contract=self.vio.contract
                runtime=self
                class InjectedContract:
                    def estimate(self,row):
                        now=float(runtime.plant.data.time)
                        if runtime.schedule is not None and now>=runtime.schedule and not onset:
                            modified=copy.deepcopy(row)
                            modified['epoch']+=1;modified['position'][0]+=.5
                            onset.update(timestamp_s=now,phase=runtime.return_phase,
                                original_sha256=hashlib.sha256(json.dumps(row,sort_keys=True).encode()).hexdigest(),
                                delivered_sha256=hashlib.sha256(json.dumps(modified,sort_keys=True).encode()).hexdigest(),
                                original_epoch=row['epoch'],delivered_epoch=modified['epoch'],
                                original_native_row=row,delivered_native_row=modified)
                            return original_contract.estimate(modified)
                        return original_contract.estimate(row)
                self.boundary=RegisteredEpochContract(InjectedContract(),self.vio,records.append)
                self.vio.contract=self.boundary
                return result

            def step(self,timeout=5.):
                if self.epoch_failed:return Runtime.step(self,timeout)
                if self.return_phase=='HOVER' and self.schedule is None:
                    self.schedule=float(self.plant.data.time)+args.delay
                row=super().step(timeout)
                if self.boundary.frame_changed:
                    raise RuntimeError('Registered localization epoch changed')
                return row

            def __exit__(self,kind,error,traceback):
                if str(error)=='Registered localization epoch changed':
                    now=float(self.plant.data.time)
                    termination.update(passed=False,task_success=False,trigger=str(error),
                        admitted=admission(self.telemetry,self.return_phase,now),
                        admission_telemetry=self.telemetry.copy(),onset_s=now)
                    try:
                        if termination['admitted']:
                            self.epoch_failed=True
                            # The changed frame cannot produce further admissible
                            # navigation. End its IPC session explicitly before
                            # continuing PX4/physics on independent flow/range.
                            self.vio.close()
                            fallback=range_fallback(self)
                            termination.update(continue_landing(self,dict(termination_wall_deadline_s=90,
                                termination_physics_deadline_s=35,safe_rest_horizontal_m=.025)))
                            termination['fallback_parameters']=fallback
                            termination['passed'] &= fallback['early_contacts_safe']
                    except Exception as failure:
                        termination.update(passed=False,failure=f'{type(failure).__name__}: {failure}')
                    termination.update(gate_version='PX4_REGISTERED_EPOCH_DOCK_TERMINATION_DEVELOPMENT_V1',
                        scope='Known dock-hover frame-change termination. New epoch is blocked; no continued mission or arbitrary-route qualification.')
                    (self.output/'termination_summary.json').write_text(json.dumps(termination,indent=2)+'\n')
                return super().__exit__(kind,error,traceback)
        keywords['runtime_factory']=EpochRuntime
        return selected(*positional,**keywords)
    image_mission.run=fault_run;vio_initial_admission.initial_consistent=StableInitialAdmission()
    try:
        image_conditions.execute(args.scene,args.audit,args.config,args.camera_config,args.conditions,
            args.output,instance=args.instance)
    finally:
        image_mission.run=original;vio_initial_admission.initial_consistent=original_guard
    summary=json.loads((args.output/'summary.json').read_text())
    checks=dict(injected=bool(onset),epoch_rejected=bool(records),task_failed=summary['passed'] is False,
        physical_termination=termination.get('passed') is True,
        independent_physical_monitor=summary.get('independent_physical_monitor',{}).get('passed') is True)
    result=dict(passed=all(checks.values()),checks=checks,task_success=False,
        injection=onset,epoch_events=records,termination=termination,scope=manifest['scope'],
        run_type='headless native image/IMU closed-loop SITL; declared contract epoch reset and continued physical termination')
    (args.output/'epoch_boundary_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('scene',type=Path)
    for flag in ['audit','config','camera-config','conditions','output']:parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--delay',type=float,default=.6);parser.add_argument('--instance',type=int,default=74)
    raise SystemExit(run(parser.parse_args()))
