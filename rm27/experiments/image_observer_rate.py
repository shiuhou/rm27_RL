"""Development full image mission with a faster host-SITL HIGHRES_IMU observer.

The ordinary mission is reused. This changes only its requested MAVLink sensor
publication interval, not physics truth access, recovery scoring or hardware.
"""
import argparse
import hashlib
import json
from pathlib import Path

from ..autopilot.px4 import image_mission
from ..autopilot.px4.runtime import mavlink

RUNNER_BYTES=Path(__file__).read_bytes()


def execute(scene,audit,configuration,camera,output,*,interval_us=4000,instance=46):
    if interval_us not in (4000,8000,10000,20000):raise ValueError('explicit host-SITL observer interval required')
    original=image_mission.run

    def run_with_observer(*args,**kwargs):
        base=kwargs['runtime_factory']
        class ObserverRuntime(base):
            def __enter__(self):
                result=super().__enter__()
                self.command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,105,interval_us)
                (self.output/'observer_request.json').write_text(json.dumps(dict(
                    interval_us=interval_us,publication='PX4 HIGHRES_IMU',
                    scope='host UDP SITL development only; not M3C UART capacity or raw hardware IMU qualification'),indent=2)+'\n')
                return result
        kwargs['runtime_factory']=ObserverRuntime
        return original(*args,**kwargs)

    image_mission.run=run_with_observer
    try:
        code=image_mission.execute(scene,audit,output,configuration=configuration,
            camera_configuration=camera,instance=instance)
    finally:image_mission.run=original
    output=Path(output)
    times=[]
    for line in (output/'mavlink.jsonl').open():
        row=json.loads(line);message=row['message']
        if message.get('mavpackettype')=='HIGHRES_IMU':times.append(message['time_usec'])
    import numpy as np
    delta=np.diff(times);positive=delta[delta>0]
    evidence=dict(requested_interval_us=interval_us,messages=len(times),
        observed_median_interval_us=float(np.median(positive)) if len(positive) else None,
        observed_maximum_interval_us=int(positive.max()) if len(positive) else None,
        non_increasing_timestamps=int(np.sum(delta<=0)),runner_sha256=hashlib.sha256(RUNNER_BYTES).hexdigest(),
        scope='Measured PX4 simulation publication rate; hardware UART and raw IMU remain separate')
    (output/'observer_rate.json').write_text(json.dumps(evidence,indent=2)+'\n')
    (output/'observer_runner.py').write_bytes(RUNNER_BYTES)
    return code


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--camera-config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--interval-us',type=int,default=4000);p.add_argument('--instance',type=int,default=46)
    a=p.parse_args();return execute(a.scene,a.audit,a.config,a.camera_config,a.output,
        interval_us=a.interval_us,instance=a.instance)


if __name__=='__main__':raise SystemExit(main())
