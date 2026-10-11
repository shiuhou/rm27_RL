"""Exact recorded motor replay with the declared preflight physical fixture."""
import argparse
import json
from pathlib import Path

from rm27.autopilot.px4 import replay
from .physical_fixture import fixture_type


def run(recording,output):
    metadata=json.loads((Path(recording)/'metadata.json').read_text())
    settings=metadata.get('physical_fixture',{}).get('settings')
    if settings is None:raise ValueError('A recorded physical fixture is required')
    original=replay.MicroPlant
    replay.MicroPlant=fixture_type(original,settings)
    try:return replay.replay(recording,output)
    finally:replay.MicroPlant=original


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('recording',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=run(args.recording,args.output);print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
