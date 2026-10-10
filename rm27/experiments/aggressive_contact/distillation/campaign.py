"""Frozen final-test manifest. Selection is based only on validation."""
import hashlib,json
from pathlib import Path
from ..evidence import snapshot,write_json
from ..model import load_config
from .dataset import settings
from .evaluation import run


def confirm(output,bc,hybrid):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);snapshot(output,load_config())
    methods={
      'teacher':dict(checkpoint=None,profile='perfect'),
      'adaptive_nominal':dict(checkpoint=None,profile='perfect',baseline=True),
      'bc':dict(checkpoint=str(bc),profile='perfect_filtered'),
      'hybrid_state':dict(checkpoint=str(hybrid),profile='perfect_filtered',hybrid=True),
      'hybrid_noisy':dict(checkpoint=str(hybrid),profile='noise_filtered',hybrid=True),
      'hybrid_noisy_delay':dict(checkpoint=str(hybrid),profile='noise_delay_filtered',hybrid=True)}
    manifest=dict(methods=methods,settings=settings(),reason='hybrid selected by validation; MLP retained as baseline; combined observation delay is a predeclared stress condition',
                  checkpoints={str(p):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in (bc,hybrid)},action_rate_limit_rad_s=20.)
    write_json(output/'frozen_selection.json',manifest)
    results={}
    for split in ('holdout_small','holdout_stress'):
        results[split]={}
        for name,kwargs in methods.items():
            results[split][name]=run(output=output/split/name,split=split,**kwargs)
    result=dict(passed=all(r['passed'] for methods in results.values() for r in methods.values()),
                scope='pipeline integrity only; no training or selection uses these holdout outcomes',results=results)
    write_json(output/'summary.json',result);return result
