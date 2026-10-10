"""Frozen teacher numerical convergence checks, no parameter selection."""
import copy
import json
from pathlib import Path

from .evidence import snapshot,write_json,save_failure
from .teacher import prepare
from .maneuver import episode


def run(cfg,source,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);snapshot(output,cfg)
    methods=json.loads((Path(source)/'selection.json').read_text())['methods']
    prepared,template=prepare(cfg,output/'plant');rows=[]
    for name in ('aggressive','balanced','conservative'):
        for seed,scale in ((None,1.),(72004,2.)):
            for dt in (.001,.0005,.00025):
                c=copy.deepcopy(prepared[0]);c['physics_dt_s']=dt
                path=output/name/f'{seed}-{dt:g}'
                try:r=episode(c,methods[name],path,seed,scale,(c,prepared[1],prepared[2]),compact=True,template=template,finish_on_recovery=True)
                except Exception as exc:r=save_failure(path,exc)
                rows.append(dict(method=name,seed=seed,dt_s=dt,passed=r['passed'],failures=r['failure_modes'],
                  recovered_s=r.get('time_to_confirmed_recovery_s'),margin_m=(r.get('first_contact') or {}).get('region_margin_m'),
                  max_post_rate_rad_s=r.get('max_post_body_rate_rad_s')))
    result=dict(passed=all(r['passed'] for r in rows),scope='same frozen designs and validation draws at 1/.5/.25 ms, 200 Hz control',rows=rows)
    write_json(output/'summary.json',result);return result
