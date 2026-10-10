"""Offline model-compatibility experiment using the frozen calibration split.

This is a diagnostic extension of artifacts/m3c-calibration-20261007/fit_calibration.py,
not a replacement calibration gate. No frame selection, holdout tuning or admission.
"""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def metrics(values):
    e=np.asarray(values)
    return dict(rms_px=float(np.sqrt(np.mean(e**2))),p95_px=float(np.percentile(e,95)),
                max_px=float(e.max()),point_count=len(e))


def fit_radtan4(records,board,size):
    records=[r for r in records if r['accepted']]
    if not records or any(r['split'] not in ('fit','validation') for r in records):
        raise ValueError('frozen fit/validation split required')
    fit=[r for r in records if r['split']=='fit']
    validation=[r for r in records if r['split']=='validation']
    if len(fit)<3 or not validation:
        raise ValueError('fit and held-out observations required')
    if len({r['image'] for r in records})!=len(records):
        raise ValueError('duplicate view identity')
    objects=[board[np.asarray(r['corner_ids'],dtype=int)] for r in fit]
    points=[np.asarray(r['corners_px'],np.float32).reshape(-1,1,2) for r in fit]
    # k3 is fixed to zero DURING fitting; never truncate an already fitted model.
    _,k,d,rvecs,tvecs,std,_,_=cv2.calibrateCameraExtended(objects,points,size,None,None,
        flags=cv2.CALIB_FIX_K3,
        criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_COUNT,200,1e-10))
    if d.size!=5 or d.ravel()[4]!=0:
        raise ValueError('unexpected OpenCV four-parameter fit representation')
    views=[];errors={'fit':[],'validation':[]}
    for row in records:
        obj=board[np.asarray(row['corner_ids'],dtype=int)]
        pts=np.asarray(row['corners_px'],np.float64).reshape(-1,1,2)
        if row['split']=='fit':
            i=fit.index(row);rv,tv=rvecs[i],tvecs[i]
        else:
            ok,rv,tv=cv2.solvePnP(obj,pts,k,d,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok: raise ValueError('held-out board pose failed')
        proj=cv2.projectPoints(obj,rv,tv,k,d)[0]
        e=np.linalg.norm((pts-proj).reshape(-1,2),axis=1)
        errors[row['split']].extend(e.tolist())
        views.append(dict(image=row['image'],split=row['split'],rvec=rv.ravel().tolist(),
            tvec_m=tv.ravel().tolist(),**metrics(e)))
    return dict(model='opencv_pinhole_brown_conrady_4',image_width=size[0],image_height=size[1],
        fx=float(k[0,0]),fy=float(k[1,1]),cx=float(k[0,2]),cy=float(k[1,2]),camera_matrix=k.tolist(),
        distortion_coefficients=d.ravel()[:4].tolist(),distortion_order=['k1','k2','p1','p2'],
        intrinsic_std=std.ravel().tolist(),fit_views=len(fit),validation_views=len(validation),
        fit_metrics=metrics(errors['fit']),validation_metrics=metrics(errors['validation']),views=views,
        status='DIAGNOSTIC_CANDIDATE',synchronized_vio_qualified=False)


def audit(root,output):
    root,output=Path(root),Path(output)
    if output.exists(): raise FileExistsError(output)
    paths=[root/'calibration_result.json',root/'fit_protocol.json',root/'selected-01/capture_manifest.json']
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    old,protocol,manifest=[json.loads(p.read_text(encoding='utf-8')) for p in paths]
    records=[r for r in manifest['records'] if r['accepted']]
    for row in records:
        path=root/'selected-01'/row['image']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['image_sha256']:
            raise ValueError('saved calibration image changed')
    board=np.zeros((49,3),np.float32)
    board[:,:2]=np.mgrid[0:7,0:7].T.reshape(-1,2)*manifest['target']['center_distance_m']
    result=fit_radtan4(records,board,(old['image_width'],old['image_height']))
    # Compare two mathematical models on real observed board rays (not residuals).
    delta=[]
    for row,view in zip(records,old['views']):
        if row['image']!=view['image']: raise ValueError('original view association mismatch')
        obj=board[np.asarray(row['corner_ids'],dtype=int)]
        k=np.asarray(old['camera_matrix']);d=np.asarray(old['distortion_coefficients'])
        rv,tv=np.asarray(view['rvec']),np.asarray(view['tvec_m'])
        full=cv2.projectPoints(obj,rv,tv,k,d)[0]
        truncated=cv2.projectPoints(obj,rv,tv,k,d[:4])[0]
        delta.extend(np.linalg.norm((full-truncated).reshape(-1,2),axis=1))
    result['silent_k3_truncation_pixel_displacement_on_saved_board_rays']=metrics(delta)
    k=np.asarray(result['camera_matrix']);d=np.asarray(result['distortion_coefficients'])
    x,y=np.meshgrid(np.linspace(-k[0,2]/k[0,0],(old['image_width']-k[0,2])/k[0,0],41),
        np.linspace(-k[1,2]/k[1,1],(old['image_height']-k[1,2])/k[1,1],25))
    q=np.stack([x.ravel(),y.ravel(),np.ones(x.size)],axis=1)
    def distort(v): return cv2.projectPoints(v,np.zeros(3),np.zeros(3),np.eye(3),d)[0].reshape(-1,2)
    eps=1e-5;qx=q.copy();qy=q.copy();qx[:,0]+=eps;qy[:,1]+=eps
    base=distort(q);dx=(distort(qx)-base)/eps;dy=(distort(qy)-base)/eps
    det=dx[:,0]*dy[:,1]-dx[:,1]*dy[:,0]
    g=protocol['gates'];fm=result['fit_metrics'];vm=result['validation_metrics'];std=result['intrinsic_std']
    result['same_original_geometric_checks']=dict(fit_rms=fm['rms_px']<=g['fit_rms_px_max'],
        validation_rms=vm['rms_px']<=g['validation_rms_px_max'],validation_p95=vm['p95_px']<=g['validation_point_p95_px_max'],
        validation_view_rms=max(v['rms_px'] for v in result['views'] if v['split']=='validation')<=g['validation_view_rms_px_max'],
        point_max=max(fm['max_px'],vm['max_px'])<=g['point_error_max_px'],
        focal_uncertainty=max(std[0]/k[0,0],std[1]/k[1,1])<=g['relative_focal_std_max'],
        principal_point_uncertainty=max(std[2:4])<=g['principal_point_std_px_max'],
        positive_distortion_jacobian=bool(np.all(det>0)),
        principal_point_in_image=bool(0<k[0,2]<old['image_width'] and 0<k[1,2]<old['image_height']))
    result['same_original_geometric_checks']={key:bool(v) for key,v in result['same_original_geometric_checks'].items()}
    result.update(opencv_version=cv2.__version__,inputs_sha256=hashes,distortion_jacobian_min=float(det.min()),
        limitations='Diagnostic fixed-model refit; original holdout was previously evaluated, not fresh blind validation. '
                    'No outlier removal, changed gate or real VIO qualification. Provenance/timing/extrinsics remain unresolved.')
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in hashes.items()):
        raise ValueError('source changed during audit')
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x',encoding='utf-8') as f: json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k not in ('views','intrinsic_std')},indent=2))


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();audit(args.root,args.output)
