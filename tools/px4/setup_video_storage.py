"""Install the pinned lossless camera encoder into an isolated local directory."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys


EXPECTED_BINARY='e7e7fb30477f717e6f55f9180a70386c62677ef8a4d4d1a5d948f4098aa3eb99'


def run(output,runtime):
    output,runtime=Path(output).resolve(),Path(runtime).resolve()
    if platform.system()!='Linux' or platform.machine()!='x86_64':
        raise ValueError('This validated encoder build is Linux x86_64')
    output.mkdir(parents=True,exist_ok=False)
    if runtime.exists():raise FileExistsError('Use a new isolated runtime directory')
    downloads=output/'downloads';downloads.mkdir()
    commands=[[sys.executable,'-m','pip','download','--no-deps','--only-binary=:all:',
        '--dest',str(downloads),'imageio-ffmpeg==0.6.0']]
    with (output/'installation.log').open('x') as log:
        subprocess.run(commands[0],stdout=log,stderr=subprocess.STDOUT,check=True)
        wheels=list(downloads.glob('*.whl'))
        if len(wheels)!=1:raise ValueError('Exactly one pinned imageio-ffmpeg wheel required')
        commands.append([sys.executable,'-m','pip','install','--no-deps','--target',str(runtime),str(wheels[0])])
        subprocess.run(commands[1],stdout=log,stderr=subprocess.STDOUT,check=True)
    binary=runtime/'imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
    def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
    if digest(binary)!=EXPECTED_BINARY:raise ValueError('Pinned camera encoder binary differs')
    version=subprocess.check_output([str(binary),'-version'],text=True).splitlines()[0]
    if not version.startswith('ffmpeg version 7.0.2-static '):raise ValueError('Pinned encoder version differs')
    result=dict(passed=True,binary=str(binary),sha256=digest(binary),version='7.0.2-static',
        dependency='imageio-ffmpeg==0.6.0',wheel_sha256=digest(wheels[0]),commands=commands,
        source_sha256=digest(Path(__file__)),
        run_type='isolated pinned local generated-media dependency; not flight evidence')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--runtime',type=Path,required=True)
    args=parser.parse_args()
    run(args.output,args.runtime)
