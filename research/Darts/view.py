"""Interactive MuJoCo playback of the analytic trajectory, without SITL."""

import time
import mujoco

from .experiment import load_run,pose,camera

CAMERAS = ['overview','follow','target','launch','side','fpv']


def view(output,speed=.25,mode='overview',fovy=60.):
    import mujoco.viewer
    if not 0 < speed <= 10:
        raise ValueError('Playback speed must be in (0,10]')
    m,d,path,report=load_run(output,fovy)
    state=dict(paused=False,reset=False,mode=mode)

    def key(code):
        if code==32:
            state['paused']=not state['paused']
        elif code in (ord('R'),ord('r')):
            state['reset']=True
        elif ord('1') <= code <= ord('6'):
            state['mode']=CAMERAS[code-ord('1')]

    print('1 overview | 2 follow | 3 target | 4 launch | 5 side | 6 first person | Space pause | R replay')
    print('Analytic path playback; freezes at the target, then automatically repeats. Close window to exit.')
    if not report['passed']:
        print('WARNING: this run failed geometric validation; see summary.json.')
    cycle=0.;previous=time.monotonic();last_mode=None
    with mujoco.viewer.launch_passive(m,d,key_callback=key) as viewer:
        viewer.opt.geomgroup[3]=0
        while viewer.is_running():
            now=time.monotonic();elapsed=now-previous;previous=now
            if state['reset']:
                cycle=0.;state['reset']=False
            if not state['paused']:
                cycle=(cycle+elapsed*speed)%(path.duration+2*speed)
            with viewer.lock():
                pose(m,d,path,cycle)
                if last_mode != state['mode']:
                    camera(viewer.cam,state['mode'],path,report,cycle,m)
                    last_mode=state['mode']
                elif state['mode']=='follow':
                    viewer.cam.lookat[:]=path.state(min(cycle,path.duration))[0]
                elif state['mode']=='fpv':
                    camera(viewer.cam,'fpv',path,report,cycle,m)
                viewer.opt.geomgroup[4]=0
                viewer.opt.sitegroup[:]=0 if state['mode']=='fpv' else 1
            viewer.sync()
            time.sleep(1/90)
