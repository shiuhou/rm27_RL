"""Lossless video storage with verified exact original PNG reconstruction.

Only closed generated PX4 cameras are supported. Original pixel and PNG byte
hashes must survive the full roundtrip before any PNG is removed. Restore a
camera before using tools which expect its original individual image paths.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import uuid

import cv2
import numpy as np

from rm27.autopilot.px4.return_fault_batch import ROOT,digest


def binary():
    report=json.loads((ROOT/'artifacts/px4-lossless-video-runtime-01/summary.json').read_text())
    path=Path(report['binary'])
    if report.get('passed') is not True or digest(path)!=report['sha256']:
        raise ValueError('Verified lossless video runtime required')
    return path


def png_bytes(image):
    accepted,data=cv2.imencode('.png',image)
    if not accepted:raise ValueError('Original PNG encoder failed')
    return data.tobytes()


def decode(video,records):
    capture=cv2.VideoCapture(str(video))
    try:
        for row in records:
            success,bgr=capture.read()
            if not success:raise ValueError('Missing lossless video frame')
            image=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY) if row['channels']==1 else bgr
            if list(image.shape)!=row['shape']:
                raise ValueError('Decoded image dimensions changed')
            encoded=png_bytes(image)
            if hashlib.sha256(encoded).hexdigest()!=row['sha256']:
                raise ValueError('Original PNG bytes did not survive video roundtrip: '+row['filename'])
            yield row,encoded
        if capture.read()[0]:raise ValueError('Unexpected extra video frame')
    finally:capture.release()


def archive(camera):
    camera=Path(camera).resolve();images=camera/'images'
    # Simulator filenames are acquisition timestamps. Temporal order also
    # keeps the lossless video useful for inspection and efficient to store.
    # Primary frames use timestamp.png; return frames use sequence-timestamp.png.
    paths=sorted(images.glob('*.png'),key=lambda p:tuple(int(n) for n in p.stem.split('-')))
    if not paths:raise ValueError('Original generated PNG images required')
    target=camera/'images.lossless.mkv';manifest=camera/'images.archive.json'
    if target.exists() or manifest.exists():raise FileExistsError('Camera archive already exists')
    ffmpeg=binary();first=cv2.imread(str(paths[0]),cv2.IMREAD_UNCHANGED)
    if first is None:raise ValueError('Cannot decode original PNG')
    height,width=first.shape[:2]
    monochrome=first.ndim==2
    temporary=target.with_name(target.name+'.tmp-'+uuid.uuid4().hex)
    command=[str(ffmpeg),'-nostdin','-hide_banner','-loglevel','error','-f','rawvideo',
        '-pix_fmt','gray' if monochrome else 'bgr24','-video_size',f'{width}x{height}','-framerate','20','-i','pipe:0',
        '-c:v','libx264' if monochrome else 'libx264rgb','-pix_fmt','gray' if monochrome else 'bgr24',
        '-color_range','pc','-qp','0','-preset','medium','-threads','2','-f','matroska',str(temporary)]
    records=[];stats={};process=None
    try:
        with (camera/'images.archive-encoder.log').open('x') as error_log:
            process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=error_log)
            for path in paths:
                original=path.read_bytes();image=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
                if image is None or image.dtype!=np.uint8 or image.shape[:2]!=(height,width):
                    raise ValueError('Fixed8-bit image dimensions required')
                channels=1 if image.ndim==2 else image.shape[2]
                if channels not in (1,3) or png_bytes(image)!=original:
                    raise ValueError('Exact original PNG encoder reconstruction required: '+str(path))
                stat=path.stat();stats[path]=(stat.st_ino,stat.st_size,stat.st_mtime_ns)
                records.append(dict(filename=path.name,shape=list(image.shape),channels=channels,
                    sha256=hashlib.sha256(original).hexdigest(),original_bytes=len(original)))
                if (channels==1)!=monochrome:raise ValueError('Uniform camera channel count required')
                process.stdin.write(image.tobytes())
            process.stdin.close()
            if process.wait()!=0:raise ValueError('Lossless video encoder failed')
        # Verify decoding and reconstruction of every original file.
        count=sum(1 for _ in decode(temporary,records))
        for path,row in zip(paths,records):
            stat=path.stat()
            if (stat.st_ino,stat.st_size,stat.st_mtime_ns)!=stats[path] or digest(path)!=row['sha256']:
                raise ValueError('Original camera image changed during archive')
        original_size=sum(r['original_bytes'] for r in records)
        if temporary.stat().st_size>=original_size:
            return dict(passed=True,storage_changed=False,images=count,original_bytes=original_size,
                candidate_bytes=temporary.stat().st_size,saved_bytes=0,
                scope='Every original PNG verified; video is larger, so original PNG files remain')
        os.link(temporary,target)
        record=dict(passed=count==len(records),run_type='verified lossless generated camera storage; not flight evidence',
            images=count,records=records,storage_path=target.name,video_sha256=digest(target),
            storage_changed=True,original_bytes=original_size,compressed_bytes=target.stat().st_size,
            command=command,ffmpeg_sha256=digest(ffmpeg),opencv_version=cv2.__version__,
            runner_sha256=digest(__file__),scope='Exact original PNG bytes verified for every frame. Auxiliary image readers require restoration.')
        with manifest.open('x') as stream:stream.write(json.dumps(record,indent=2)+'\n')
        for path in paths:path.unlink()
        return record
    finally:
        if process and process.poll() is None:
            process.terminate();process.wait(timeout=10)
        temporary.unlink(missing_ok=True)


def restore(camera):
    camera=Path(camera).resolve();manifest=json.loads((camera/'images.archive.json').read_text())
    video=camera/manifest['storage_path']
    if digest(video)!=manifest['video_sha256']:raise ValueError('Camera video bytes changed')
    restored=0
    for row,encoded in decode(video,manifest['records']):
        name=row['filename']
        if Path(name).name!=name:raise ValueError('Original image filename must stay in its directory')
        path=camera/'images'/name
        if path.exists():
            if digest(path)!=row['sha256']:raise ValueError('Existing PNG differs from original')
        else:
            temporary=path.with_name(path.name+'.restore-'+uuid.uuid4().hex)
            try:
                with temporary.open('xb') as stream:stream.write(encoded);stream.flush();os.fsync(stream.fileno())
                os.link(temporary,path);restored+=1
            finally:temporary.unlink(missing_ok=True)
    return dict(passed=True,restored_images=restored,images=len(manifest['records']),
        run_type='verified exact original PNG restoration; not a new flight')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('camera',type=Path)
    parser.add_argument('--restore',action='store_true');parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run=args.camera.parent
    summary=json.loads((run/'summary.json').read_text());metadata=json.loads((run/'metadata.json').read_text())
    if args.camera.name not in ('vio','return_camera') or summary.get('autopilot')!='px4' or 'scene_sha256' not in metadata:
        raise ValueError('Closed generated PX4 camera required')
    args.output.mkdir(parents=True,exist_ok=False)
    result=restore(args.camera) if args.restore else archive(args.camera)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='records'},indent=2))
