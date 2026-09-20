"""Render images, scientific trajectory plots and an optional MP4 preview."""

from pathlib import Path
import json
import mujoco
import numpy as np
from PIL import Image,ImageDraw

from .experiment import load_run,pose,camera


def plot(path,report,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t,p,v=path.samples(.002)
    distance=np.linalg.norm(p[:,:2]-p[0,:2],axis=1)
    fig,ax=plt.subplots(2,1,figsize=(11,8),constrained_layout=True)
    ax[0].plot(p[:,0],p[:,1],color='darkorange',label='Dart front-reference path')
    for point,label,color in [(path.start,'Red launch','red'),(p[-1],'Blue upper target','blue'),
                               (report['dock_m'],'Dock','green')]:
        ax[0].scatter(point[0],point[1],color=color,s=45)
        ax[0].annotate(label,(point[0],point[1]),xytext=(5,8),textcoords='offset points')
    ax[0].set(xlabel='Field X (m)',ylabel='Field Y (m)',xlim=(-15,15),ylim=(-9,9),
              title='Plan view | native field coordinates; flight time selected, launch velocity solved')
    ax[0].set_aspect('equal',adjustable='box');ax[0].grid(alpha=.3)
    ax[1].plot(distance,p[:,2],color='darkorange')
    for k in np.linspace(0,len(t)-1,6,dtype=int):
        ax[1].scatter(distance[k],p[k,2],color='black',s=12)
        ax[1].annotate(f'{t[k]:.2f} s',(distance[k],p[k,2]),xytext=(0,8),textcoords='offset points')
    ax[1].set(xlabel='Horizontal distance from launch (m)',ylabel='Field Z (m)',
              ylim=(0,max(p[:,2])+0.5),title='Gravity-only parabola | no aerodynamic or impact dynamics')
    ax[1].grid(alpha=.3)
    fig.savefig(output/'trajectory.png',dpi=160)
    plt.close(fig)


def render(run,output,video=False,speed=.25,mode='follow'):
    if not 0 < speed <= 10:
        raise ValueError('Playback speed must be in (0,10]')
    output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    m,d,path,report=load_run(run)
    option=mujoco.MjvOption();option.geomgroup[3]=0;option.geomgroup[4]=0
    cam=mujoco.MjvCamera()
    with mujoco.Renderer(m,height=720,width=1280) as renderer:
        def frame(t,mode,annotate=True):
            pose(m,d,path,t);camera(cam,mode,path,report,t)
            renderer.update_scene(d,camera=cam,scene_option=option)
            im=Image.fromarray(renderer.render())
            if annotate:
                draw=ImageDraw.Draw(im)
                _,v=path.state(min(t,path.duration))
                phase='TARGET PLANE HIT (kinematic stop)' if t>=path.duration else 'FLIGHT'
                draw.rectangle((0,0,1280,65),fill=(20,23,29))
                draw.text((15,10),f'DART DEMO | {mode} | {phase} | t={min(t,path.duration):.3f} s | speed={np.linalg.norm(v):.2f} m/s',fill='white')
                draw.text((15,34),'CAD at actual scale | no drag / impact dynamics / interception',fill=(255,210,110))
                if not report['passed']:
                    draw.text((15,53),'GEOMETRIC VALIDATION FAILED',fill='red')
            return im

        for name,t,mode_at in [('overview',path.duration/2,'overview'),('target',path.duration,'target'),
                              ('launch',0,'launch'),('dart',path.duration/2,'follow'),('side',path.duration/2,'side')]:
            frame(t,mode_at).save(output/f'{name}.png')
        frames_written=0
        if video:
            import cv2
            fps=30
            writer=cv2.VideoWriter(str(output/'flight.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1280,720))
            if not writer.isOpened():
                raise RuntimeError('OpenCV MP4 encoder is unavailable')
            try:
                # One second at launch, slow flight, then two seconds at target.
                n=int(np.ceil((1+path.duration/speed+2)*fps))
                for i in range(n):
                    simulation_time=np.clip((i/fps-1)*speed,0,path.duration)
                    im=frame(simulation_time,mode)
                    # Stable whole-field context while the main camera follows the dart.
                    if mode=='follow':
                        small=frame(simulation_time,'overview',False).resize((384,216))
                        im.paste(small,(886,494))
                    writer.write(np.asarray(im)[:,:,::-1].copy())
                    frames_written+=1
            finally:
                writer.release()
    plot(path,report,output)
    (output/'render.json').write_text(json.dumps(dict(source_run=str(Path(run).resolve()),
        mode='offscreen MuJoCo rendering of analytic states',video_frames=frames_written,
        video_fps=30 if video else None,playback_speed=speed),indent=2)+'\n')
    return output
