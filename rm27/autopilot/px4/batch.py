"""Independent PX4 flight starts with a complete attempt manifest."""
import argparse,json,subprocess,sys
from pathlib import Path

def batch(scene,output,dynamics,count=10,instance=0):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    rows=[]
    for n in range(count):
        dest=output/f'flight-{n+1:02d}'
        command=[sys.executable,'-m','rm27','micro-flight',str(scene),'--autopilot','px4','--dynamics',str(dynamics),'--output',str(dest),'--instance',str(instance)]
        with (output/f'flight-{n+1:02d}.log').open('w') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        path=dest/'summary.json';summary=json.loads(path.read_text()) if path.exists() else {'passed':False,'failure':'summary missing'}
        rows.append({'attempt':n+1,'command':command,'returncode':result.returncode,'summary':summary,'path':str(path)})
        manifest={'passed':len(rows)==count and all(r['summary'].get('passed') is True and r['returncode']==0 for r in rows),'attempts':rows,'passed_count':sum(r['summary'].get('passed') is True for r in rows),'required_count':count,'run_type':'closed-loop SITL independent launches'}
        (output/'summary.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print(json.dumps({'attempt':n+1,'passed':summary.get('passed'),'failure':summary.get('failure')}),flush=True)
    return manifest

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path);p.add_argument('--dynamics',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--count',type=int,default=10);p.add_argument('--instance',type=int,default=0)
    a=p.parse_args();r=batch(a.scene,a.output,a.dynamics,a.count,a.instance);return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
