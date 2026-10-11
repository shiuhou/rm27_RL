"""Development flight with current-flight native VIO and numerical flow aiding."""
import argparse
import json
from pathlib import Path
import mujoco

from .runtime import Runtime
from ...perception.sitl.live import LiveVio, ROOT
from ...perception.sitl.return_camera import DockReturnCamera


class VioRuntime(Runtime):
    def __init__(self, plant, output, instance=0, *, profile='vio'):
        if profile != 'vio':
            raise ValueError('VioRuntime requires the explicit development profile')
        super().__init__(plant,output,instance,profile='combined')
        self.profile='vio'
        self.external_vision=False  # Synthetic SensorProducer.odometry is never sent.
        self.sensor.minimum_range_m=.08
        self.vio=None
        self.return_camera=None
        self.return_phase='INITIALIZE'
        self.return_observation=None
        self.vio_camera_config=None
        self.vio_player=None
        self.vio_player_mounts=()

    def __enter__(self):
        self.vio=LiveVio(self.plant,self.output,camera_config=self.vio_camera_config,
            player=self.vio_player,player_mounts=self.vio_player_mounts)
        self.vio.__enter__()
        try:
            runtime=super().__enter__()
            config=json.loads((ROOT/'configs/px4_sim_down_camera.json').read_text())
            if mujoco.mj_name2id(self.plant.model,mujoco.mjtObj.mjOBJ_CAMERA,config['camera_name'])>=0:
                self.return_camera=DockReturnCamera(self.plant.model,self.output,config=config)
            return runtime
        except BaseException:
            self.close()
            raise

    def step(self, timeout=5.):
        self.vio.observe_telemetry(self.telemetry,float(self.plant.data.time))
        self.vio.sensors()
        for packet in self.vio.packets(float(self.plant.data.time)):
            self.hil.send(packet)
        result=super().step(timeout)
        if self.return_camera:
            self.return_observation=self.return_camera.sense(self.plant.data,self.telemetry,self.return_phase)
        return result

    def close(self):
        super().close()
        if self.return_camera:
            self.return_camera.close();self.return_camera=None
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
