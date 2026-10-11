"""Development flight with current-flight native VIO and numerical flow aiding."""
import argparse
from pathlib import Path

from .runtime import Runtime
from ...perception.sitl.live import LiveVio, ROOT


class VioRuntime(Runtime):
    def __init__(self, plant, output, instance=0, *, profile='vio'):
        if profile != 'vio':
            raise ValueError('VioRuntime requires the explicit development profile')
        super().__init__(plant,output,instance,profile='combined')
        self.profile='vio'
        self.external_vision=False  # Synthetic SensorProducer.odometry is never sent.
        self.sensor.minimum_range_m=.08
        self.vio=None

    def __enter__(self):
        self.vio=LiveVio(self.plant,self.output)
        self.vio.__enter__()
        return super().__enter__()

    def step(self, timeout=5.):
        self.vio.observe_telemetry(self.telemetry,float(self.plant.data.time))
        self.vio.sensors()
        for packet in self.vio.packets(float(self.plant.data.time)):
            self.hil.send(packet)
        return super().step(timeout)

    def close(self):
        super().close()
        if self.vio:self.vio.close()


def main():
    from .flight import run
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--dynamics',type=Path,default=ROOT/'configs/dynamics_150g.json')
    p.add_argument('--instance',type=int,default=27);p.add_argument('--speedup',type=float,default=.5)
    p.add_argument('--view',action='store_true')
    a=p.parse_args()
        return run(a.scene,a.output,a.instance,dynamics=a.dynamics,view=a.view,
               speedup=a.speedup,profile='vio',runtime_factory=VioRuntime)


if __name__=='__main__':
    raise SystemExit(main())
