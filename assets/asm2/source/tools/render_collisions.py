from pathlib import Path
import mujoco
from PIL import Image,ImageDraw,ImageFont
ROOT=(Path(__file__).resolve().parents[1] / 'mujoco')
m=mujoco.MjModel.from_xml_path(str(ROOT/'model.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
m.vis.headlight.ambient[:]=[.6,.6,.6];m.vis.headlight.diffuse[:]=[.7,.7,.7]
m.vis.global_.offwidth=1100;m.vis.global_.offheight=1000
for i in range(m.ngeom):
 name=m.geom(i).name
 if name.startswith('contact_front'):m.geom_rgba[i]=[0,.7,1,1]
 elif name.startswith('contact_rear'):m.geom_rgba[i]=[.2,1,.3,1]
 elif name.startswith('collision_'):m.geom_rgba[i]=[.65,.65,.7,1]
opt=mujoco.MjvOption();opt.geomgroup[:]=0;opt.geomgroup[3]=1;opt.sitegroup[:]=0
cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,0];cam.distance=.28;cam.azimuth=90
canvas=Image.new('RGB',(2200,1060),'white')
with mujoco.Renderer(m,height=1000,width=1100,max_geom=m.ngeom+100) as renderer:
 for i,elevation in enumerate([90,25]):
  cam.elevation=elevation;renderer.update_scene(d,camera=cam,scene_option=opt)
  frame=Image.fromarray(renderer.render())
  if i==0: frame=frame.rotate(90)
  canvas.paste(frame,(i*1100,40))
draw=ImageDraw.Draw(canvas)
font=ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",20)
draw.text((20,10),'Underside collision geometry | blue: front contact | green: rear contact',fill='black',font=font)
draw.text((1120,10),'Oblique collision view | 142 g | balanced battery',fill='black',font=font)
canvas.save(ROOT/'../preview/collision_preview.png')
