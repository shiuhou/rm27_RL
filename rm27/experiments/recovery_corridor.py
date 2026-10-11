"""Continuous clearance of a declared expanding guard-tip recovery corridor."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .recovery_region import RegionAudit

RUNNER_BYTES=Path(__file__).read_bytes()
DEPENDENCY_BYTES=Path(__file__).with_name('recovery_region.py').read_bytes()
SURFACE_BYTES=Path(__file__).with_name('target_region_audit.py').read_bytes()


def contains_pose(regions,pose):
    pose=np.asarray(pose,float)
    return any(np.all((pose>=np.asarray(box)[:,0])&(pose<=np.asarray(box)[:,1])) for box in regions)


def prior_corridor_region(path,settings,bounds,scene_hash,assets):
    """Reuse a verified region only with identical geometric inputs/methods."""
    path=Path(path);prior=json.loads(path.read_text())
    config=prior.get('configuration',{})
    geometry_keys=('armor_geom','armor_site','contact_point_body_flu_m',
        'allowed_surface_dimensions_m','backing_surface_tolerance_m','penetration_tolerance_m','capsule_segments')
    if (prior.get('passed') is not True or prior.get('scene_sha256')!=scene_hash or
        prior.get('asset_sha256')!=assets or any(config.get(k)!=settings.get(k) for k in geometry_keys)):
        return None
    saved=path.parent/'region_dependency.py'
    if hashlib.sha256(saved.read_bytes()).hexdigest()!=prior.get('dependency_sha256'):
        return None
    if saved.read_bytes()!=DEPENDENCY_BYTES:return None
    saved=path.parent/'surface_dependency.py';source=saved.read_bytes()
    if hashlib.sha256(source).hexdigest()!=prior.get('surface_dependency_sha256'):return None
    def functions(contents):
        code=contents.decode()
        return {node.name:ast.get_source_segment(code,node) for node in ast.parse(code).body
            if isinstance(node,ast.FunctionDef)}
    old,current=functions(source),functions(SURFACE_BYTES)
    names=('capsule_probes','segment_triangle_distance','local_meshes','outside_patch_triangles')
    if any(old.get(k)!=current.get(k) or k not in old for k in names):return None
    for result in prior['regions']:
        old_bounds=np.asarray(config['pose_regions'][result['region']],float)
        if (result.get('passed') is True and old_bounds.shape==(6,2) and
            np.all(bounds[:,0]>=old_bounds[:,0]) and np.all(bounds[:,1]<=old_bounds[:,1])):
            return dict(prior_certificate=str(path),prior_region=result['region'],
                prior_certificate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                method='subset of verified continuous corridor region with unchanged geometry functions')
    return None


def run(scene,configuration,output):
    scene,configuration,output=map(lambda p:Path(p).resolve(),(scene,configuration,output))
    settings=json.loads(configuration.read_text());output.mkdir(parents=True,exist_ok=False)
    assets={str((scene.parent/asset.get('file')).resolve()):
        hashlib.sha256((scene.parent/asset.get('file')).read_bytes()).hexdigest()
        for asset in ET.parse(scene).getroot().findall('.//asset/*') if asset.get('file')}
    auditor=RegionAudit(scene,settings);radius=max(auditor.radii.values())
    results=[];total=0
    with (output/'cells.jsonl').open('x',buffering=1) as stream:
        for index,box in enumerate(settings['pose_regions']):
            bounds=np.asarray(box,float);checked=certified=0;failure=None
            if bounds.shape!=(6,2) or not np.isfinite(bounds).all() or np.any(bounds[:,1]<bounds[:,0]):
                raise ValueError('six ordered pose axes required for every region')
            if settings.get('base_corridor_certificate'):
                reused=prior_corridor_region(settings['base_corridor_certificate'],settings,bounds,
                    hashlib.sha256(scene.read_bytes()).hexdigest(),assets)
                if reused:
                    results.append(dict(passed=True,region=index,**reused));continue
            # A complete prior continuous certificate may cover this box.
            base=settings.get('base_certificate')
            if base:
                path=Path(base);prior=json.loads(path.read_text()) if path.exists() else {}
                prior_bounds=np.asarray(prior.get('configuration',{}).get('pose_bounds',[]))
                matching=all(prior.get('configuration',{}).get(k)==settings.get(k) for k in (
                    'armor_geom','armor_site','contact_point_body_flu_m','allowed_surface_dimensions_m',
                    'backing_surface_tolerance_m','penetration_tolerance_m'))
                if (prior.get('passed') is True and matching and prior_bounds.shape==(6,2) and
                    prior.get('runner_sha256')==hashlib.sha256(DEPENDENCY_BYTES).hexdigest() and
                    prior.get('surface_runner_sha256')==hashlib.sha256(SURFACE_BYTES).hexdigest() and
                    prior.get('scene_sha256')==hashlib.sha256(scene.read_bytes()).hexdigest() and
                    np.all(bounds[:,0]>=prior_bounds[:,0]) and np.all(bounds[:,1]<=prior_bounds[:,1])):
                    results.append(dict(passed=True,region=index,prior_certificate=str(path),
                        prior_certificate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                        method='subset of already certified continuous pose region'))
                    continue
            stack=[(bounds[:,0],bounds[:,1],0)]
            while stack and checked<settings['maximum_cells_per_region']:
                low,high,depth=stack.pop();result=auditor.assess(low,high);checked+=1;total+=1
                stream.write(json.dumps(dict(region=index,low=low.tolist(),high=high.tolist(),depth=depth,**result))+'\n')
                if result['status']=='certified':certified+=1;continue
                if result['status']=='collision':failure=result;break
                if depth>=settings['maximum_depth']:
                    failure=dict(status='unresolved',reason='refinement depth exhausted',cell=result);break
                extent=high-low;axis=int(np.argmax(np.r_[extent[:3],np.deg2rad(extent[3:])*radius]))
                middle=(low[axis]+high[axis])/2;first_high=high.copy();first_high[axis]=middle
                last_low=low.copy();last_low[axis]=middle
                stack.append((last_low,high,depth+1));stack.append((low,first_high,depth+1))
                if checked%2000==0:print(json.dumps(dict(region=index,checked=checked,certified=certified)),flush=True)
            if stack and failure is None:failure=dict(status='unresolved',reason='cell budget exhausted')
            row=dict(passed=failure is None and not stack,region=index,checked_cells=checked,
                certified_cells=certified,failure=failure);results.append(row)
            print(json.dumps(row),flush=True)
    unchanged=all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==expected for path,expected in assets.items())
    result=dict(passed=all(r['passed'] for r in results) and unchanged,
        gate_version=settings['gate_version'],configuration=settings,regions=results,
        s1_complete=False,run_type='continuous bounded piecewise recovery-corridor clearance',
        assets_unchanged=unchanged,asset_sha256=assets,
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        configuration_sha256=hashlib.sha256(configuration.read_bytes()).hexdigest(),
        runner_sha256=hashlib.sha256(RUNNER_BYTES).hexdigest(),
        dependency_sha256=hashlib.sha256(DEPENDENCY_BYTES).hexdigest(),
        surface_dependency_sha256=hashlib.sha256(SURFACE_BYTES).hexdigest(),
        checked_cells=total,
        limitations='Configured corridor only. Full contact classification, force convergence and membership of every native physical recovery step remain separate. Visual surfaces are not a solid CAD model.')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    for name,data in [('runner.py',RUNNER_BYTES),('region_dependency.py',DEPENDENCY_BYTES),('surface_dependency.py',SURFACE_BYTES)]:
        (output/name).write_bytes(data)
    print(json.dumps({k:v for k,v in result.items() if k!='asset_sha256'},indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();return 0 if run(a.scene,a.config,a.output)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
