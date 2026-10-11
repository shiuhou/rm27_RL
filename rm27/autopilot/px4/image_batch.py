"""Frozen complete image/VIO missions, with independent recovery and EKF2 evidence."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from .return_fault_batch import frozen_inputs, ROOT, digest
from ...telemetry.ulog import analyze


def assess(summary, vio, log, *, surfaces=None, require_surfaces=False,
           observer=None, required_observer_interval_us=None):
    flags=log.get('fusion',{})
    fused=any(all(instance.get(k,{}).get('ever') is True for k in
        ('cs_ev_pos','cs_ev_vel','cs_ev_hgt')) for instance in flags.values())
    failsafes=[e for e in summary.get('events',[]) if e.get('event')=='STATUSTEXT'
        and 'failsafe' in e.get('text','').lower()]
    checks=dict(mission=summary.get('passed') is True,
        physical_hover=summary.get('independent_hover_score',{}).get('passed') is True,
        physical_rest=summary.get('independent_resting_score',{}).get('passed') is True,
        physical_recovery=summary.get('independent_recovery_score',{}).get('passed') is True,
        image_target=summary.get('image_route',{}).get('passed') is True,
        assigned_tag=summary.get('landing_commitment',{}).get('assigned_tag_id')==21,
        auto_disarm=summary.get('auto_disarm',{}).get('confirmed') is True,
        native_vio=vio.get('passed') is True,
        ulog_integrity=log.get('passed') is True,actual_ekf2_ev_fusion=fused,
        no_unexpected_failsafe=not failsafes)
    if require_surfaces:
        checks['recorded_local_surfaces']=(surfaces or {}).get('passed') is True
    if required_observer_interval_us is not None:
        observer=observer or {}
        median=observer.get('observed_median_interval_us')
        checks['measured_imu_observer']=(observer.get('requested_interval_us')==required_observer_interval_us and
            median is not None and 0<median<=required_observer_interval_us and
            observer.get('non_increasing_timestamps')==0)
    return dict(passed=all(checks.values()),checks=checks,unexpected_failsafes=failsafes)


def trial(job):
    command,destination,hashes,surface_configuration,options=job
    changed=[p for p,h in hashes.items() if not Path(p).exists() or digest(p)!=h]
    if changed:return dict(passed=False,failure='Frozen inputs changed: '+str(changed))
    with destination.with_suffix('.log').open('x') as stream:
        process=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    def read(path):
        return json.loads(path.read_text()) if path.exists() else dict(passed=False,failure=str(path)+' missing')
    summary=read(destination/'summary.json');vio=read(destination/'vio/summary.json')
    logs=list((destination/'log').rglob('*.ulg'))
    log=analyze(logs[0],destination/'ulog_analysis') if len(logs)==1 else dict(passed=False,failure='Expected exactly one ULog')
    surfaces=None
    if surface_configuration:
        # Inspect the closed recording, independently of actor inputs and of
        # the physical colliders (many field meshes are visual-only).
        try:
            from ...experiments.target_region_audit import audit_recording
            surfaces=audit_recording(destination,surface_configuration,destination/'local_surface_audit')
        except Exception as exc:
            surfaces=dict(passed=False,failure=f'{type(exc).__name__}: {exc}')
            (destination/'local_surface_audit_failure.json').write_text(json.dumps(surfaces,indent=2)+'\n')
    observer=read(destination/'observer_rate.json')
    qualification=assess(summary,vio,log,surfaces=surfaces,require_surfaces=bool(surface_configuration),
        observer=observer,required_observer_interval_us=options.get('required_observer_interval_us'))
    archives=[]
    if options.get('compress_recordings') and (destination/'frames.jsonl').exists():
        from ...telemetry.recording import compress_recording
        archives.append(compress_recording(destination/'frames.jsonl'))
    return dict(passed=process.returncode==0 and qualification['passed'],command=command,
        returncode=process.returncode,path=str(destination),summary=summary,
        qualification=qualification,vio_summary=vio,ulog_summary_path=str(destination/'ulog_analysis/summary.json'),
        local_surface_audit_path=str(destination/'local_surface_audit/summary.json') if surfaces else None,
        observer_rate=observer,recording_archives=archives)


def report(attempts,spec):
    errors={}
    for name,score,key in [('resting_xy_m','independent_resting_score','horizontal_error_m'),
        ('resting_height_m','independent_resting_score','height_error_m'),
        ('hover_horizontal_m','independent_hover_score','horizontal_maximum_m'),
        ('hover_vertical_m','independent_hover_score','vertical_maximum_m'),
        ('impact_detection_delay_s','independent_recovery_score','impact_detection_delay_s')]:
        values=[a.get('summary',{}).get(score,{}).get(key) for a in attempts]
        values=[v for v in values if v is not None]
        errors[name]=dict(maximum=max(values),p95=float(np.percentile(values,95))) if values else None
    failures=Counter(a.get('failure') or a.get('summary',{}).get('failure') or
        ','.join(k for k,v in a.get('qualification',{}).get('checks',{}).items() if not v)
        for a in attempts if not a['passed'])
    return dict(passed=len(attempts)==len(spec['cases']) and all(a['passed'] for a in attempts),
        passed_count=sum(a['passed'] for a in attempts),total_attempts=len(attempts),required_count=len(spec['cases']),
        autopilot='px4',model_mass_kg=.15,gate_version=spec['gate_version'],
        estimation_source='current-flight native OpenVINS plus numerical flow/range/compass/IMU',
        run_type='headless closed-loop SITL independent launches',failure_categories=dict(failures),
        physical_errors=errors,scope=spec['scope'],attempts=attempts)


def run(scene,audit,configuration,output,*,workers=3,instance=50):
    if not 1<=workers<=3 or not 0<=instance<=99-workers:raise ValueError('1..3 distinct instances required')
    scene,audit,configuration=map(lambda p:Path(p).resolve(),(scene,audit,configuration))
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    spec=json.loads(configuration.read_text())
    mission=(ROOT/spec['mission_configuration']).resolve();camera=(ROOT/spec['camera_configuration']).resolve()
    profile=json.loads(mission.read_text())['estimation_profile']
    hashes=frozen_inputs(scene,configuration)
    for p in (audit,mission,camera,ROOT/'configs/px4'/f'{profile}.json'):
        hashes[str(p)]=digest(p)
    mission_settings=json.loads(mission.read_text())
    if mission_settings.get('return_termination_configuration'):
        p=ROOT/mission_settings['return_termination_configuration'];hashes[str(p)]=digest(p)
    p=ROOT/'rm27/telemetry/recording.py';hashes[str(p)]=digest(p)
    surface_configuration=None
    if spec.get('surface_configuration'):
        surface_configuration=(ROOT/spec['surface_configuration']).resolve()
        for p in (surface_configuration,ROOT/'rm27/experiments/target_region_audit.py'):
            hashes[str(p)]=digest(p)
    manifest=dict(configuration=spec,input_sha256=hashes,scene=str(scene),audit=str(audit),
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        seed_policy='declared camera-noise seed; separate process, parameters, logs and native VIO session per attempt',
        command=sys.argv,workers=workers,first_instance=instance)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=ROOT))
    attempts=[]
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for offset in range(0,len(spec['cases']),workers):
            group=spec['cases'][offset:offset+workers];jobs=[]
            for i,c in enumerate(group):
                dest=output/c['name'];camera_config=json.loads(camera.read_text())
                camera_config['seed']=c['camera_seed']
                camera_path=output/(c['name']+'-camera.json')
                camera_path.write_text(json.dumps(camera_config,indent=2)+'\n')
                cmd=[sys.executable,'-m','rm27.autopilot.px4.image_mission',str(scene),
                    '--audit',str(audit),'--config',str(mission),'--camera-config',str(camera_path),
                    '--output',str(dest),'--instance',str(instance+i),'--speedup',str(spec['speedup'])]
                jobs.append((cmd,dest,dict(hashes,**{str(camera_path):digest(camera_path)}),surface_configuration,spec))
            for case,result in zip(group,pool.map(trial,jobs)):
                row=dict(name=case['name'],camera_seed=case['camera_seed'],**result);attempts.append(row)
                summary=report(attempts,spec)
                (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
                print(json.dumps(dict(name=case['name'],passed=row['passed'],
                    failure=row.get('failure') or row.get('summary',{}).get('failure'),
                    checks=row.get('qualification',{}).get('checks'))),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=3)
    p.add_argument('--instance',type=int,default=50)
    a=p.parse_args();return 0 if run(a.scene,a.audit,a.config,a.output,workers=a.workers,instance=a.instance)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
