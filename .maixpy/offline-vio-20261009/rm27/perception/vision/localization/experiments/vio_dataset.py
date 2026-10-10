"""Pure offline VIO timestamp/calibration interface checks, not an admission gate.

These helpers never fit unknown clocks, generate IMU samples, or qualify hardware.
Existing LocalizationEstimate, calibration admission, adapter and evaluator remain
authoritative. Evidence hashes here are syntax-checked references, not file reads.
"""
from bisect import bisect_left, bisect_right
from decimal import Decimal, InvalidOperation, ROUND_HALF_EVEN
import math
import re


def need(ok, message):
    if not ok:
        raise ValueError(message)


def nonnegative_int(value):
    return type(value) is int and value >= 0


def digest(value):
    return isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value) is not None


def map_timestamp(raw, mapping, *, source_epoch, source_domain, source_unit):
    m=mapping
    need(nonnegative_int(raw),'integer raw timestamp required')
    need(m.get('evidence_status')=='VERIFIED' and digest(m.get('evidence_sha256')),
         'explicit evidenced map required; UNKNOWN cannot become zero offset')
    need((source_epoch,source_domain,source_unit)==(m['source_epoch'],m['source_domain'],m['source_unit']),
         'clock domain/unit/epoch mismatch')
    for k in ('source_epoch','source_domain','target_epoch','target_domain'):
        need(isinstance(m[k],str) and m[k] not in ('','UNKNOWN'),'known epochs/domains required')
    need(m['source_unit'] in ('ns','us','ms','s'),'unsupported source unit')
    need(nonnegative_int(m['source_anchor']) and nonnegative_int(m['target_anchor_ns']), 'integer anchors required')
    limits=m['valid_source_range']
    need(isinstance(limits,list) and len(limits)==2 and all(nonnegative_int(x) for x in limits)
         and limits[0]<=m['source_anchor']<=limits[1] and limits[0]<=raw<=limits[1],
         'no extrapolation outside qualified map range')
    need(nonnegative_int(m['uncertainty_ns']),'explicit nonnegative uncertainty required')
    try:
        scale=Decimal(m['scale_ns_per_tick'])
    except (InvalidOperation,TypeError,ValueError):
        raise ValueError('invalid clock scale')
    need(isinstance(m['scale_ns_per_tick'],str) and scale.is_finite() and scale>0,'positive decimal-string scale required')
    # Subtract integer anchors before multiplication; do not float-convert epochs.
    from decimal import localcontext
    with localcontext() as ctx:
        ctx.prec=max(50,len(str(raw))+len(str(m['target_anchor_ns']))+len(str(scale))+10)
        exact=Decimal(m['target_anchor_ns'])+Decimal(raw-m['source_anchor'])*scale
        rounded=int(exact.to_integral_value(rounding=ROUND_HALF_EVEN))
    need(rounded>=0,'mapped time precedes target epoch')
    return dict(raw_value=raw,source_unit=source_unit,source_domain=source_domain,
        source_epoch=source_epoch,mapped_ns=rounded,target_domain=m['target_domain'],
        target_epoch=m['target_epoch'],uncertainty_ns=m['uncertainty_ns']+1,
        evidence_sha256=m['evidence_sha256'],
        limits='map evidence claim only; no exposure-semantic conversion; +1ns conservative rounding bound')


def imu_windows(camera_ns, imu_ns, *, max_gap_ns):
    """Inclusive IMU index brackets for first frame / successive frame intervals.

Inputs MUST already share an explicitly verified clock epoch and time semantic.
The first frame requires an IMU strictly before it. Later windows share their
boundary samples. Returned indices preserve original samples; no extrapolation.
"""
    need(nonnegative_int(max_gap_ns) and max_gap_ns>0,'positive integer max gap required')
    for values in (camera_ns,imu_ns):
        need(len(values)>0 and all(nonnegative_int(x) for x in values)
             and all(b>a for a,b in zip(values,values[1:])), 'strictly increasing integer timestamps required')
    result=[]
    for i,t in enumerate(camera_ns):
        left=(bisect_left(imu_ns,t)-1) if i==0 else bisect_right(imu_ns,camera_ns[i-1])-1
        right=bisect_left(imu_ns,t)
        need(left>=0 and right<len(imu_ns),'unbracketed camera interval; no extrapolation')
        need(all(imu_ns[j+1]-imu_ns[j]<=max_gap_ns for j in range(left,right)),
             'IMU gap crosses camera integration interval')
        result.append((left,right))
    return result


def validate_calibration_interface(c):
    import numpy as np
    need(type(c.get('schema_version')) is int and c['schema_version']==1,'calibration interface version')
    for field in ('intrinsics_sha256','extrinsics_sha256','timing_sha256','noise_sha256'):
        need(digest(c.get(field)),'explicit artifact digest required: '+field)
    for field in ('extrinsic_status','clock_mapping_status','noise_status'):
        need(c.get(field)=='VERIFIED','unqualified calibration: '+field)
    need(c.get('translation_unit')=='m' and c.get('transform_convention')=='camera_to_imu',
         'T_imu_camera maps camera coordinates into IMU, metres')
    matrix=c.get('T_imu_camera')
    need(isinstance(matrix,list) and len(matrix)==4 and all(isinstance(row,list) and len(row)==4 for row in matrix),
         '4x4 rigid transform required')
    need(all(type(x) in (int,float) and math.isfinite(x) for row in matrix for x in row),'finite matrix required')
    t=np.asarray(matrix,float);r=t[:3,:3]
    need(np.allclose(t[3],[0,0,0,1],atol=1e-9,rtol=0) and
         np.allclose(r.T@r,np.eye(3),atol=1e-6,rtol=0) and abs(np.linalg.det(r)-1)<=1e-6,
         'proper rigid transform required; no reflection or scale')
    need(c.get('exposure_timestamp_semantic')=='MID_EXPOSURE','receipt/encoded PTS is not exposure midpoint')
    need(c.get('shutter_model') in ('GLOBAL','ROLLING'),'qualified shutter model required')
    if c['shutter_model']=='ROLLING':
        need(type(c.get('line_delay_ns')) in (int,float) and math.isfinite(c['line_delay_ns'])
             and c['line_delay_ns']>0 and c.get('readout_direction') in ('TOP_TO_BOTTOM','BOTTOM_TO_TOP'),
             'rolling shutter requires evidenced row delay and direction')
    noise=c.get('noise_parameters',{})
    need(c.get('noise_units') == {'gyro_noise_density':'rad/s/sqrt(Hz)',
        'gyro_random_walk':'rad/s^2/sqrt(Hz)', 'accel_noise_density':'m/s^2/sqrt(Hz)',
        'accel_random_walk':'m/s^3/sqrt(Hz)'}, 'continuous-time SI noise units required')
    for field in ('gyro_noise_density','gyro_random_walk','accel_noise_density','accel_random_walk'):
        need(type(noise.get(field)) in (int,float) and math.isfinite(noise[field]) and noise[field]>0,
             'positive calibrated noise parameter required: '+field)
    return dict(interface_valid=True,hardware_qualified=False,
        limits='artifact references are syntax checked, not evidence revalidation; existing calibration/admission gates still apply')


def openvins_radtan4(c):
    """Pack a declared compatible camera model; NOT calibration qualification."""
    need(c.get('model') == 'opencv_pinhole_brown_conrady_4' and
         c.get('distortion_order') == ['k1','k2','p1','p2'],
         'OpenVINS radtan requires explicit four-coefficient model; never truncate k3')
    d=c.get('distortion_coefficients')
    need(isinstance(d,list) and len(d)==4,'four ordered coefficients required')
    values=[c.get(k) for k in ('fx','fy','cx','cy')]+d
    need(all(type(x) in (float,int) and math.isfinite(x) for x in values)
         and values[0]>0 and values[1]>0, 'finite camera parameters and positive focal lengths required')
    return values


def bracket_mapped_streams(camera,gyro,accel,*,max_gap_ns,max_uncertainty_ns):
    """Check common nominal-time brackets, preserving separate sensor streams.

    Map/exposure evidence must be verified independently by the admission layer.
    Uncertainty is bounded/reported, not modeled as an exact sample instant.
    """
    need(nonnegative_int(max_uncertainty_ns),'explicit uncertainty budget required')
    need(bool(camera) and bool(gyro) and bool(accel),'three nonempty streams required')
    domain=(camera[0].get('target_domain'),camera[0].get('target_epoch'))
    need(all(isinstance(x,str) and x not in ('','UNKNOWN') for x in domain),'known common clock required')
    for stream,semantic in ((camera,'MID_EXPOSURE'),(gyro,'ACQUISITION'),(accel,'ACQUISITION')):
        for row in stream:
            need((row.get('target_domain'),row.get('target_epoch'))==domain,'common clock/epoch mismatch')
            need(row.get('semantic')==semantic and row.get('semantic_status')=='VERIFIED',
                 'explicit event semantics required; clock map is insufficient')
            need(digest(row.get('evidence_sha256')),'clock evidence reference required')
            need(nonnegative_int(row.get('uncertainty_ns')) and row['uncertainty_ns']<=max_uncertainty_ns,
                 'timestamp uncertainty exceeds declared budget')
    times=lambda stream:[row['mapped_ns'] for row in stream]
    return dict(gyro=imu_windows(times(camera),times(gyro),max_gap_ns=max_gap_ns),
        accel=imu_windows(times(camera),times(accel),max_gap_ns=max_gap_ns),
        target_domain=domain[0],target_epoch=domain[1],hardware_qualified=False,
        max_timestamp_uncertainty_ns=max(row['uncertainty_ns'] for s in (camera,gyro,accel) for row in s),
        limits='nominal mapped-time brackets only; no resampling or uncertainty compensation; '
               'evidence claims need independent admission validation')
