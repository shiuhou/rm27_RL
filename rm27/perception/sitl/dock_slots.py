"""Replace simulated green slot pads with four AprilTags and a down camera.

Only the generated scene is modified. The source CAD and field stay untouched.
Tags are visual-only; collision remains the existing dock bottom and dividers.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

from .dataset import digest
from .dock_tags import DockTagDetector
from .mujoco_camera import MujocoCamera
from .return_camera import slot_relative_position
from ...autopilot.bridge.frames import attitude_to_ap

ROOT=Path(__file__).resolve().parents[3]
DEFAULT_CONFIG=ROOT/'configs/px4_sim_down_camera.json'


def build(scene,output,config_path=DEFAULT_CONFIG):
    scene=Path(scene).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    config=json.loads(Path(config_path).read_text())
    ids={int(slot):int(tag) for slot,tag in config['tag_id_by_slot'].items()}
    if set(ids)!={1,2,3,4} or len(set(ids.values()))!=4:raise ValueError('four unique slot IDs required')
    size=float(config['tag_size_m'])
    if not .02<=size<=.09:raise ValueError('tag panel must fit in the 110 mm visual pad')
    root=ET.parse(scene).getroot();asset=root.find('asset')
    dock=next((b for b in root.findall('.//body') if b.get('name')=='rm27_dock'),None)
    drone=next((b for b in root.findall('.//body') if b.get('name')=='drone_1'),None)
    if dock is None or drone is None:raise ValueError('dock or vehicle missing')
    for slot,tag in ids.items():
        site=next((s for s in dock.findall('site') if s.get('name')==f'slot_{slot}'),None)
        if site is None:raise ValueError(f'slot_{slot} site missing')
        if site.get('rgba')!='.1 .8 .3 .6':raise ValueError('unexpected original green slot visual')
        site.set('rgba','0 0 0 0')
        marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),tag,240)
        padded=np.full((288,288),255,np.uint8);padded[24:264,24:264]=marker
        path=output/f'slot_{slot}_tag_{tag}.png'
        if not cv2.imwrite(str(path),padded):raise RuntimeError('tag image write failed')
        texture=f'rm27_slot_{slot}_texture';material=f'rm27_slot_{slot}_material'
        ET.SubElement(asset,'texture',name=texture,type='2d',file=str(path))
        ET.SubElement(asset,'material',name=material,texture=texture,texuniform='false',texrepeat='1 1',specular='0',shininess='0',emission='1')
        position=np.fromstring(site.get('pos'),sep=' ');position[2]+=.001
        half=size*.6
        ET.SubElement(dock,'geom',name=f'rm27_slot_{slot}_tag',type='box',pos=' '.join(map(str,position)),
            size=f'{half} {half} .0005',material=material,contype='0',conaffinity='0',group='0')
    if any(c.get('name')==config['camera_name'] for c in drone.findall('camera')):raise ValueError('downward camera already exists')
    ET.SubElement(drone,'camera',name=config['camera_name'],pos=' '.join(map(str,config['position_body_flu_m'])),
        fovy=str(config['fovy_deg']))
    destination=output/'scene.xml';ET.ElementTree(root).write(destination,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(destination));data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    qa=model.jnt_qposadr[model.body_jntadr[model.body('drone_1').id]]
    slot_world={slot:data.site_xpos[model.site(f'slot_{slot}').id].copy() for slot in ids}
    # Visibility is checked after takeoff, when the aircraft has cleared the rim.
    data.qpos[qa:qa+3]=slot_world[config['assigned_slot']]+[0,0,.6]
    mujoco.mj_forward(model,data)
    detector=DockTagDetector({tag:size for tag in ids.values()},fovy_deg=config['fovy_deg'])
    visibility=[]
    airborne_position=data.qpos[qa:qa+3].copy()
    with MujocoCamera(model,width=config['width'],height=config['height'],fovy_deg=config['fovy_deg'],
                      camera_name=config['camera_name']) as camera:
        frame=camera.capture(data,0)
        cv2.imwrite(str(output/'airborne_downward.png'),cv2.cvtColor(frame.image_rgb,cv2.COLOR_RGB2BGR))
        observations=detector.detect(frame,now_us=0)
        for height in [.6,.4,.3,.2,.12]:
            data.qpos[qa:qa+3]=slot_world[config['assigned_slot']]+[.004,.002,height]
            mujoco.mj_forward(model,data)
            sample=camera.capture(data,0)
            selected=next((o for o in detector.detect(sample,now_us=0)
                           if o.tag_id==ids[config['assigned_slot']] and o.safe_for_control),None)
            record=dict(height_above_slot_m=height,assigned_tag_visible=selected is not None)
            if selected is not None:
                attitude=attitude_to_ap(data.qpos[qa+3:qa+7])
                estimated=slot_relative_position(selected,attitude,config['position_body_flu_m'])
                record['independent_xy_error_m']=float(np.linalg.norm(estimated-[.004,.002]))
            visibility.append(record)
    seen={o.tag_id for o in observations if o.safe_for_control}
    summary=dict(passed=seen==set(ids.values()) and all(v['independent_xy_error_m']<.01
        for v in visibility if v['assigned_tag_visible']),run_type='airborne downward-camera slot-tag visibility audit',
        scene=str(destination),source_scene_sha256=digest(scene),scene_sha256=digest(destination),
        config_sha256=digest(config_path),tag_id_by_slot=ids,tag_size_m=size,
        camera=config,airborne_camera_body_position_world_m=airborne_position.tolist(),
        observed_ids=sorted(seen),detections=[dict(tag_id=o.tag_id,safe_for_control=o.safe_for_control,
            reason=o.reason,reprojection_rmse_px=o.reprojection_rmse_px) for o in observations],
        approach_visibility=visibility,
        mass_model_note='Down camera is massless in this simulation; user estimates 2-3 g, and total real takeoff mass has not been updated',
        flight_qualified=False)
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='camera'},indent=2));return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    a=p.parse_args();r=build(a.scene,a.output,a.config);return 0 if r['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
