"""Export the student observation/action contract and verify episode partitions."""
import hashlib,json,shutil
from pathlib import Path
import numpy as np
from ..evidence import write_json,snapshot
from ..model import load_config,build_model,scenario
from .dataset import settings,designs

SCHEMA=dict(version='rm27.student.v1',observation_dimension=32,
    features=['time_over_planned_duration','planned_duration_over_6','mode_onehot[3]','ego_position_minus_committed_target[3]',
              'world_velocity[3]','body_rotation_first_two_columns[6]','body_rate[3]','estimated_motor_force[4]',
              'estimated_specific_thrust_ratio','previous_proposed_rate_thrust[4]','estimated_contact_latch',
              'time_since_estimated_contact','ego_observation_age'],
    action=['body_roll_rate_FLU_rad_s','body_pitch_rate_FLU_rad_s','body_yaw_rate_FLU_rad_s','collective_force_N'],
    action_limit=dict(rate_rad_s=20.,force='[0,4*nominal_max_rotor_force]'),
    forbidden_actor_inputs=['physical_draw','actual_motor_state','simulator_contact','teacher_phase','future_state','teacher_reference'],
    supervised_only=['teacher_rate_thrust','teacher_reference'],
    observations='future estimator types; present truth or synthetic-noise ego samples and committed static target; no validated onboard perception',
    history='lags [0,2,6] correspond to 0/10/30 ms at 200 Hz; pad only within episode',
    hybrid='learned time-conditioned reference + retained geometric/actuator feedback + causal impact detector, not a pure neural actor',
    contact_detection='20 ms signed normal velocity-innovation window, >0.12 m/s near nominal target; no perfect contact truth',
    deployment='physical N requires calibration to AP normalized throttle; no MAVLink sender or firmware validation')


def run(dataset,checkpoint,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);snapshot(output,load_config())
    rows=json.loads((Path(dataset)/'index.json').read_text());seen={};errors=[];sources=[]
    allowed=settings()
    for row in rows:
        group=row['scenario_group'];split=row['split']
        if group in seen and seen[group]!=split:errors.append('scenario split overlap '+group)
        seen[group]=split
        seed=int(group.split('/')[0])
        if seed not in allowed[split+'_seeds']:errors.append('undeclared seed '+group)
        p=Path(row['path']);a=np.load(p/'demo.npz')
        if not np.all(np.diff(a['time_s'])>0):errors.append('clock '+str(p))
        sources.append(dict(path=str(p),sha256=hashlib.sha256((p/'demo.npz').read_bytes()).hexdigest()))
    for train_seed in allowed['train_seeds']:
        if train_seed in allowed['validation_seeds']+allowed['holdout_small_seeds']+allowed['holdout_stress_seeds']:errors.append('seed overlap')
    shutil.copyfile(checkpoint,output/'policy.npz')
    cfg=load_config();_,_,info,_=build_model(scenario(cfg),cfg)
    parameters={};nominal=None
    for row in rows:
        if row['split']=='train' and row['teacher'] not in parameters:
            meta=json.loads((Path(row['path'])/'metadata.json').read_text())
            parameters[row['teacher']]=meta['outcome']['parameters']
            if row['teacher']=='balanced':nominal=meta['task_config']
    write_json(output/'nominal_config.json',nominal)
    write_json(output/'task_parameters.json',parameters)
    write_json(output/'schema.json',SCHEMA);write_json(output/'source_hashes.json',sources)
    write_json(output/'nominal_vehicle.json',dict(status='ENGINEERING ASSUMPTIONS, not measured calibration',vehicle_info=info))
    write_json(output/'task_modes.json',designs());write_json(output/'splits.json',allowed)
    result=dict(passed=not errors,errors=errors,episodes=len(rows),scenario_groups=len(seen),
                checkpoint_sha256=hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest(),
                checkpoint_bytes=Path(checkpoint).stat().st_size,scope='dataset hashes and seed partition; original failed teacher artifacts and DAgger trajectories retained separately')
    write_json(output/'summary.json',result);return result
