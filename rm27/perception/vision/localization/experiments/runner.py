"""Fail-closed offline experiment runner. No real run is possible without calibration."""
import argparse
from datetime import datetime, timezone
from pathlib import Path

from .backends import BackendAdapter, camera_config
from .benchmark import execute_process, timing_stats
from .calibration import validated_calibration
from .common import (ExperimentError, require, read_json, write_json, object_hash, file_hash,
                     environment, fixture_mode, checked_file)
from .dataset import CanonicalDataset
from .evaluation import behavior_metrics
from .trajectory import parse_tum
from .transforms import prepare_images


def run(config, *, _fixture_token=None):
    if config.get('backend') == 'OpenVINS':
        from .openvins_runtime import run as run_vio
        return run_vio(config)
    require(type(config.get('schema_version')) is int and config['schema_version']==1,'RUN_SCHEMA_UNSUPPORTED','version 1')
    require(set(config)<= {'schema_version','purpose','run_id','backend','dataset','installation','output','subset',
                          'input_hz','transforms','timeout_s','native_mask','online'},'RUN_CONFIG_UNSUPPORTED','unknown keys')
    require(isinstance(config.get('run_id'),str) and config['run_id'],'RUN_CONFIG_INVALID','run_id')
    require(type(config.get('online',False)) is bool, 'RUN_CONFIG_INVALID', 'online must be boolean')
    testing=fixture_mode(_fixture_token,config)
    root=Path(config['output']).resolve()
    root.mkdir(parents=True,exist_ok=False)
    result=dict(schema_version=1,run_id=config['run_id'], backend=config['backend'],
                purpose='TEST_FIXTURE_ONLY' if testing else 'OFFLINE_MONOCULAR_RESEARCH',
                input_mode='canonical_image_sequence', config=config,config_sha256=object_hash(config),
                host_environment=environment(),started_utc=datetime.now(timezone.utc).isoformat(),
                status='PREPARING', execution_started=False,
                artifacts={'raw':'raw/','normalized':'normalized/','benchmark':'benchmark.json','evaluation':'evaluation.json'},
                online_trajectory='UNAVAILABLE',final_optimized_trajectory='UNAVAILABLE', stage_result='NOT_QUALIFIED')
    write_json(root/'run.json',result)
    try:
        dataset=CanonicalDataset(config['dataset'])
        # Calibration always precedes backend preparation/execution.
        calibration,calibration_provenance=validated_calibration(dataset,_fixture_token=_fixture_token)
        if dataset.manifest.get('dataset_kind') == 'TUM_RGBD_PUBLIC':
            result['purpose'] = 'VSL-3P_PUBLIC_DATASET'
        frames=dataset.select(config.get('subset','all_decoded_sampled'),config.get('input_hz'))
        selection=[f.record() for f in frames]
        identity=dict(dataset_id=dataset.manifest['dataset_id'],dataset_manifest_sha256=dataset.manifest_hash,
                      calibration_id=calibration['calibration_id'],calibration_sha256=calibration_provenance['sha256'],
                      selected_source_frame_ids=[f.source_frame_id for f in frames],
                      timestamp_evidence=[f.timestamp_evidence for f in frames],
                      selected_image_hashes=[f.image_sha256 for f in frames],transforms=config.get('transforms',{}))
        result.update(dataset_id=dataset.manifest['dataset_id'],calibration_id=calibration['calibration_id'],
                      frame_selection=selection, comparison_identity=identity,comparison_sha256=object_hash(identity),
                      dataset_provenance=dataset.provenance,calibration_provenance=calibration_provenance)
        write_json(root/'run.json',result)
        adapter=BackendAdapter(config['backend'],config.get('installation'))
        install=adapter.describe_version(_fixture_token=_fixture_token)
        # Check this before image preparation so the reported refusal reflects
        # the unsupported backend feature rather than a later filesystem error.
        require(config.get('native_mask') is None, 'BACKEND_MASK_UNSUPPORTED',
                'the pinned TUM CLI paths do not expose native feature masks')
        if config.get('online',False):
            require(install.get('online_observation',{}).get('format') == 'rm27_online_csv_v1',
                    'ONLINE_OBSERVATION_UNAVAILABLE', 'requires an explicitly instrumented installation')
        images,derived,transform_provenance=prepare_images(frames,calibration,root/'inputs',config.get('transforms',{}))
        nominal_fps=config.get('input_hz') or dataset.manifest['observed_geometry']['nominal_fps']
        if not config.get('input_hz'):
            # Metadata setting inferred from selected IDs, never used to replace PTS.
            steps=[b.source_frame_id-a.source_frame_id for a,b in zip(frames,frames[1:])]
            from math import gcd
            from functools import reduce
            nominal_fps=nominal_fps/reduce(gcd,steps)
        text=camera_config(config['backend'],derived,calibration_provenance,nominal_fps,config.get('transforms',{}).get('grayscale',False))
        command,raw=adapter.prepare(root,frames,images,text,install,config.get('native_mask'))
        result.update(dataset_id=dataset.manifest['dataset_id'],calibration_id=calibration['calibration_id'],
                      backend_version=install, dataset_provenance=dataset.provenance,calibration_provenance=calibration_provenance,
                      frame_selection=selection,nominal_backend_fps=nominal_fps, timing_policy='original source timestamps; no exposure claim',
                      comparison_identity=identity,comparison_sha256=object_hash(identity),
                      backend_config_sha256=file_hash(root/'backend.yaml'),command=command,status='EXECUTING',execution_started=True)
        write_json(root/'run.json',result)
        benchmark=execute_process(command,raw,config.get('timeout_s',300))
        benchmark.update(backend=config['backend'],dataset_id=dataset.manifest['dataset_id'],
                         calibration_id=calibration['calibration_id'],nominal_input_hz=nominal_fps,
                         selected_source_frame_ids=[f.source_frame_id for f in frames],purpose=result['purpose'])
        result['benchmark']=benchmark
        write_json(root/'benchmark.json',benchmark)
        # Retain partial runtime traces even if a backend subsequently fails.
        if config.get('online',False):
            from .online import parse_online, online_metrics
            online_path=raw/'online.csv'
            if online_path.is_file():
                observed=parse_online(online_path,frames,config['backend'],install,config['run_id'])
                (root/'online').mkdir()
                write_json(root/'online/trajectory.json',dict(schema_version=1,purpose=result['purpose'],
                    trajectory_variant='online',records=observed,coordinate_continuity='UNVERIFIED',
                    raw_source='raw/online.csv',raw_sha256=file_hash(online_path),
                    limits='runtime public API snapshots; no final-pose substitution; map/reset/relocalization unavailable'))
                metrics=online_metrics(frames,observed)
                write_json(root/'online/evaluation.json',metrics)
                result.update(online_trajectory='online/trajectory.json',online_evaluation='online/evaluation.json')
                result['artifacts']['online']='online/'
                write_json(root/'run.json',result)
                if benchmark['failure_reason'] is None:
                    require(len(observed)==len(frames),'ONLINE_TRACE_INCOMPLETE','successful run requires one observation per input')
            elif benchmark['failure_reason'] is None:
                require(False,'ONLINE_OUTPUT_MISSING','instrumented binary did not emit online.csv')
        require(benchmark['failure_reason'] is None,'BACKEND_EXECUTION_FAILED',str(benchmark['failure_reason']))
        outputs=adapter.collect_outputs(raw)
        normalized=parse_tum(outputs['final_trajectory'],frames,direction=outputs['pose_direction'],
                             variant='final_optimized',scope=outputs['trajectory_scope'],
                             timestamp_significant_digits=outputs['timestamp_significant_digits'])
        (root/'normalized').mkdir()
        if outputs['processing_times'] is not None:
            values=[float(line) for line in outputs['processing_times'].read_text().splitlines() if line.strip()]
            require(len(values)==len(frames),'BACKEND_TIMING_COUNT_MISMATCH','track_times rows must match input')
            benchmark.update(processed_frame_count=len(values),per_frame_processing_s=values,
                             processing_time_statistics=timing_stats(values))
            write_json(root/'benchmark.json',benchmark)
            per_frame=dict(zip([f.source_frame_id for f in frames],values))
            for record in normalized:
                record['processing_time_s']=per_frame[record['source_frame_id']]
        write_json(root/'normalized/final_optimized.json',{'schema_version':1,'purpose':result['purpose'],
                   'trajectory_variant':'final_optimized','records':normalized, 'coordinate_continuity':'UNVERIFIED',
                   'limits':'stock exports omit tracking/reset/map state; no online estimate synthesized'})
        evaluation=behavior_metrics(frames,normalized,benchmark)
        evaluation.update(trajectory_variant='final_optimized',trajectory_scope=outputs['trajectory_scope'],
                          online_metrics=None, resource_use={k:benchmark.get(k) for k in
                          ('peak_sampled_rss_bytes','sampled_rss_growth_bytes','resource_scope')},
                          qualitative_intervals=dataset.intervals)
        write_json(root/'evaluation.json',evaluation)
        # Detect input mutation during external processing; don't rewrite raw output.
        checked_file(dataset.path,dataset.manifest_hash)
        checked_file(dataset.provenance['source_video'],dataset.provenance['source_sha256'])
        for path,digest in dataset.provenance['dependencies'].items():
            checked_file(path,digest)
        checked_file(calibration_provenance['path'],calibration_provenance['sha256'])
        for frame,image in zip(frames,images):
            checked_file(frame.image_path,frame.image_sha256)
        for item in transform_provenance['frames']:
            checked_file(item['image'],item['image_sha256'])
        result.update(status='EXECUTION_COMPLETED',final_optimized_trajectory='normalized/final_optimized.json',
                      evaluation=evaluation,raw_output_sha256={p.name:file_hash(p) for p in sorted(raw.iterdir()) if p.is_file()})
    except (ExperimentError, OSError, ValueError, KeyError, TypeError) as exc:
        result.update(status='FAILED' if result['execution_started'] else 'REFUSED',
                      failure_code=exc.code if isinstance(exc,ExperimentError) else 'INPUT_OR_OUTPUT_INVALID',failure_reason=str(exc))
    result['ended_utc']=datetime.now(timezone.utc).isoformat()
    write_json(root/'run.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    args=parser.parse_args()
    result=run(read_json(args.config))
    print(result['status']+': '+result.get('failure_reason','offline process completed; not a stage PASS'))
    return 0 if result['status']=='EXECUTION_COMPLETED' else 1


if __name__=='__main__':
    raise SystemExit(main())
