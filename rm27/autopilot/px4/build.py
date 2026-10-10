"""Reproducible PX4 SITL build: python -m rm27.autopilot.px4.build --help."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from . import COMMIT, VERSION
ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SOURCE = ROOT / 'artifacts/deps/PX4-Autopilot'

def git(source, *args):
    return subprocess.check_output(['git', '-C', str(source), *args], text=True).strip()

def verify(source):
    source = Path(source).resolve()
    if git(source, 'rev-parse', 'HEAD') != COMMIT or git(source, 'rev-parse', VERSION+'^{commit}') != COMMIT:
        raise ValueError('PX4 source/tag does not match pinned v1.17.0 commit')
    if git(source, 'diff', '--ignore-submodules=all', 'HEAD', '--'):
        raise ValueError('PX4 source has tracked modifications')
    return source

def build(source, output, jobs=4):
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    source = Path(source).resolve()
    env = dict(os.environ, PATH=str(Path(sys.executable).parent)+os.pathsep+os.environ['PATH'])
    commands=[]
    def run(cmd):
        commands.append([str(s) for s in cmd])
        with (output/'build.log').open('a') as log:
            subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    summary={'passed':False,'version':VERSION,'commit':COMMIT,'commands':commands}
    try:
        if not source.exists():
            run(['git','clone','--branch',VERSION,'--depth','1','https://github.com/PX4/PX4-Autopilot.git',str(source)])
        verify(source)
        # Avoid recursively fetching FlightGear models and unrelated hardware SDKs.
        # The SITL target only needs these generated-message and compression trees.
        run(['git','-C',str(source),'submodule','update','--init','--depth','1','--',
             'src/modules/mavlink/mavlink','src/lib/heatshrink/heatshrink',
             'src/lib/events/libevents','src/lib/cdrstream/cyclonedds',
             'src/lib/cdrstream/rosidl','src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client',
             'src/drivers/gps/devices'])
        run(['git','-C',str(source/'src/modules/mavlink/mavlink'),'submodule',
             'update','--init','--depth','1','--','pymavlink'])
        run(['make','-C',str(source),f'-j{jobs}','px4_sitl_default'])
        binary=source/'build/px4_sitl_default/bin/px4'
        summary.update(passed=True,binary=str(binary),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),submodules=git(source,'submodule','status','--recursive'))
    except Exception as exc:
        summary['failure']=str(exc)
        raise
    finally:
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=DEFAULT_SOURCE)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--jobs',type=int,default=4)
    a=p.parse_args(); print(json.dumps(build(a.source,a.output,a.jobs),indent=2))
if __name__=='__main__':main()
