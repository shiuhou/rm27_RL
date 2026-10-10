"""Native OpenVINS /poseimu evaluation; rigid alignment ONLY, no estimator input.

Original EuRoC orientation reference has known V1_01_easy accuracy limitations.
All quaternion inputs Hamilton wxyz; small CSV rounding errors normalized explicitly.
"""
import bisect
import hashlib
import json
from decimal import Decimal
from pathlib import Path
import zipfile
import numpy as np


def check(condition, message):
    if not condition:
        raise ValueError(message)


def ordered(ns):
    check(len(ns) > 0 and all(isinstance(t, (int, np.integer)) for t in ns), 'integer ns required')
    check(all(b > a for a, b in zip(ns, ns[1:])), 'timestamps must be strictly increasing')


def associate(source, query, tolerance):
    ordered(source)
    ordered(query)
    indices = []
    for t in query:
        at = bisect.bisect_left(source, t)
        candidates = [i for i in (at-1, at) if 0 <= i < len(source)]
        i = min(candidates, key=lambda i: abs(source[i]-t))
        check(abs(source[i]-t) <= tolerance, 'association outside tolerance')
        check(not indices or i > indices[-1], 'association must be unique and ordered')
        indices.append(i)
    return indices


def unit_quaternions(q):
    q = np.asarray(q, dtype=float)
    check(q.ndim == 2 and q.shape[1] == 4 and np.isfinite(q).all(), 'invalid quaternion')
    norms = np.linalg.norm(q, axis=1)
    check(np.all(abs(norms-1) <= 2e-6), 'quaternion norm outside CSV rounding tolerance')
    return q / norms[:, None]


def rotations(q):
    out = []
    for w, x, y, z in unit_quaternions(q):
        out.append([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return np.asarray(out)


def prepare_reference_quaternions(q):
    """Explicit reference-only normalization; does not alter source archive.

    Actual original EuRoC norms deviate by up to 20.526 ppm, beyond decimal
    rounding alone. Normalize direction for SO(3); reject >0.1% or nonfinite.
    This is not an orientation-truth correction or a positional scale fit.
    """
    q = np.asarray(q, float)
    check(q.ndim == 2 and q.shape[1] == 4 and np.isfinite(q).all(), 'invalid reference quaternion')
    norms = np.linalg.norm(q, axis=1)
    check(np.all(abs(norms-1) <= .001), 'reference norm deviation exceeds explicit 0.1% limit')
    return q/norms[:,None], dict(policy='explicit unit normalization of reference only; source unchanged',
                               max_abs_norm_deviation=float(max(abs(norms-1))),
                               count_beyond_2ppm=int(np.sum(abs(norms-1)>2e-6)))


def interpolate(ns, positions, quaternions, query, max_gap_ns):
    ordered(ns)
    ordered(query)
    p, q = np.asarray(positions, float), unit_quaternions(quaternions)
    check(p.shape == (len(ns), 3) and len(q) == len(ns) and np.isfinite(p).all(), 'invalid reference')
    indices, ps, qs, brackets = [], [], [], []
    for index, t in enumerate(query):
        if t < ns[0] or t > ns[-1]:
            continue  # never extrapolate
        right = bisect.bisect_left(ns, t)
        left = right if ns[right] == t else right-1
        gap = ns[right]-ns[left]
        check(gap <= max_gap_ns, 'reference interpolation gap too large')
        alpha = (t-ns[left])/gap if gap else 0.
        a, b = q[left].copy(), q[right].copy()
        dot = float(a @ b)
        if dot < 0:
            b, dot = -b, -dot
        dot = np.clip(dot, -1, 1)
        if dot > .9995:
            quat = (1-alpha)*a+alpha*b
        else:
            theta = np.arccos(dot)
            quat = (np.sin((1-alpha)*theta)*a + np.sin(alpha*theta)*b)/np.sin(theta)
        quat /= np.linalg.norm(quat)
        indices.append(index)
        ps.append((1-alpha)*p[left]+alpha*p[right])
        qs.append(quat)
        brackets.append([left, right, int(t-ns[left]), int(ns[right]-t)])
    check(len(indices) > 0, 'no reference overlap')
    return dict(indices=indices, positions=np.asarray(ps), quaternions=np.asarray(qs), brackets=brackets)


def metrics(ns, estimated_p, estimated_q, reference_p, reference_q):
    ordered(ns)
    x, y = np.asarray(estimated_p, float), np.asarray(reference_p, float)
    check(x.shape == y.shape == (len(ns), 3) and np.isfinite(x).all() and np.isfinite(y).all(), 'invalid positions')
    er, gr = rotations(estimated_q), rotations(reference_q)
    check(len(er) == len(gr) == len(ns), 'pose count mismatch')
    xc, yc = x-x.mean(0), y-y.mean(0)
    check(np.linalg.matrix_rank(xc) >= 2 and np.linalg.matrix_rank(yc) >= 2, 'degenerate alignment')
    u, singular, vt = np.linalg.svd(yc.T @ xc)
    d = np.diag([1., 1., np.linalg.det(u @ vt)])
    r = u @ d @ vt
    t = y.mean(0)-r @ x.mean(0)
    aligned = x @ r.T+t
    ate = np.linalg.norm(aligned-y, axis=1)

    def rpe(pairs):
        translations, angles = [], []
        for i, j in pairs:
            # inv(delta_gt) @ delta_est, T_world_imu convention.
            de_r, dg_r = er[i].T @ er[j], gr[i].T @ gr[j]
            de_t, dg_t = er[i].T @ (x[j]-x[i]), gr[i].T @ (y[j]-y[i])
            translations.append(np.linalg.norm(dg_r.T @ (de_t-dg_t)))
            angles.append(np.degrees(np.arccos(np.clip((np.trace(dg_r.T @ de_r)-1)/2, -1, 1))))
        return dict(pair_count=len(pairs), pairs=pairs,
                    translation_rmse_m=float(np.sqrt(np.mean(np.square(translations)))) if pairs else None,
                    rotation_rmse_deg=float(np.sqrt(np.mean(np.square(angles)))) if pairs else None,
                    actual_dt_s_range=[min((ns[j]-ns[i])/1e9 for i,j in pairs), max((ns[j]-ns[i])/1e9 for i,j in pairs)] if pairs else None)
    pairs = []
    for i, stamp in enumerate(ns):
        at = bisect.bisect_left(ns, stamp+10**9)
        candidates = [j for j in (at-1, at) if i < j < len(ns)]
        if candidates:
            j = min(candidates, key=lambda j: abs(ns[j]-stamp-10**9))
            if abs(ns[j]-stamp-10**9) <= 25000000:
                pairs.append([i,j])
    path_x = np.linalg.norm(np.diff(x, axis=0), axis=1).sum()
    path_y = np.linalg.norm(np.diff(y, axis=0), axis=1).sum()
    return dict(alignment=dict(type='SE3', scale=1.0, rotation=r.tolist(), translation=t.tolist(), singular_values=singular.tolist()),
                ate=dict(rmse_m=float(np.sqrt(np.mean(ate**2))), median_m=float(np.median(ate)),
                         p95_m=float(np.percentile(ate,95)), max_m=float(max(ate))),
                rpe_adjacent=rpe([[i,i+1] for i in range(len(ns)-1)]), rpe_1s=rpe(pairs),
                scale_diagnostic=dict(path_length_ratio_est_over_ref=float(path_x/path_y),
                                      path_length_bias_percent=float(100*(path_x/path_y-1)),
                                      centered_extent_ratio_est_over_ref=float(np.linalg.norm(xc)/np.linalg.norm(yc)),
                                      estimated_path_m=float(path_x), reference_path_m=float(path_y),
                                      correction_applied=False,
                                      note='Shape/noise-sensitive diagnostics, not a fitted Sim3 correction'))


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def evaluate(run, archive, out):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseWithCovarianceStamped
    run, archive, out = Path(run), Path(archive), Path(out)
    check(all((run/(name+'.exit')).read_text().strip() == '0' for name in ['native','player','recorder']), 'native run not clean')
    rows = [line.split() for line in (run/'state_estimate.txt').read_text().splitlines() if line and not line.startswith('#')]
    text_ns = [int(Decimal(row[0])*10**9) for row in rows]
    image_ns = [int((Decimal(row[0])-Decimal(row[17]))*10**9) for row in rows]
    with zipfile.ZipFile(archive) as z:
        camera = [int(l.split(',')[0]) for l in z.read('mav0/cam0/data.csv').decode().splitlines() if l and not l.startswith('#')]
        gt_bytes = z.read('mav0/state_groundtruth_estimate0/data.csv')
        gt = [l.split(',') for l in gt_bytes.decode().splitlines() if l and not l.startswith('#')]
    gt_ns = [int(row[0]) for row in gt]
    gt_p = [[float(v) for v in row[1:4]] for row in gt]
    gt_q = [[float(v) for v in row[4:8]] for row in gt]
    camera_ids = associate(camera, image_ns, 6000)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run/'online-output'), storage_id=''), rosbag2_py.ConverterOptions('',''))
    ns, positions, quaternions = [], [], []
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if topic != '/poseimu':
            continue
        m = deserialize_message(data, PoseWithCovarianceStamped)
        check(m.header.frame_id == 'global', 'unexpected pose parent frame')
        ns.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        positions.append([p.x,p.y,p.z])
        quaternions.append([q.w,q.x,q.y,q.z])
    check(len(ns) == len(rows), 'raw/text count mismatch')
    check(associate(text_ns, ns, 6000) == list(range(len(ns))), 'raw/text not one-to-one')
    check(np.max(abs(np.asarray(positions)-np.asarray([[float(v) for v in row[5:8]] for row in rows]))) <= 5.01e-7, 'raw/text position mismatch')
    check(np.max(abs(np.asarray(quaternions)-np.asarray([[float(row[4]), *map(float,row[1:4])] for row in rows]))) <= 5.01e-7, 'raw/text quaternion mismatch')
    gt_q, reference_normalization = prepare_reference_quaternions(gt_q)
    ref = interpolate(gt_ns, gt_p, gt_q, ns, 10000000)
    ids = ref['indices']
    selected_ns = [ns[i] for i in ids]
    result = metrics(selected_ns, np.asarray(positions)[ids], np.asarray(quaternions)[ids], ref['positions'], ref['quaternions'])
    associations = [dict(raw_pose_index=i, camera_index=camera_ids[i], camera_ns=camera[camera_ids[i]], state_ns=ns[i],
                         gt_bracket=bracket) for i,bracket in zip(ids,ref['brackets'])]
    result.update(schema_version=1, trajectory_variant='online', variant='pinned OpenVINS + explicit ROS2 lifecycle compatibility patch',
                  reference='original EuRoC V1_01_easy state_groundtruth_estimate0; known orientation accuracy caveat',
                  time_policy='IMU state header time; linear position + shortest-arc quaternion SLERP; no time-shift fitting; no extrapolation; <=10ms reference bracket',
                  rpe_policy='adjacent and 1s nearest within 25ms; indices refer to associations.json',
                  quaternion_policy='JPL GtoI xyzw equals Hamilton ItoG xyzw; reorder only, no conjugation; explicit reference-only unit normalization recorded separately',
                  reference_quaternion_normalization=reference_normalization,
                  raw_pose_count=len(ns), evaluated_pose_count=len(ids), excluded_outside_gt=len(ns)-len(ids),
                  evaluated_duration_s=(selected_ns[-1]-selected_ns[0])/1e9,
                  association=dict(max_image_residual_ns=max(abs(camera[j]-t) for j,t in zip(camera_ids,image_ns)),
                                   max_raw_text_time_residual_ns=max(abs(a-b) for a,b in zip(ns,text_ns)),
                                   unique_camera_indices=True, first_camera_index=camera_ids[0], last_camera_index=camera_ids[-1]),
                  inputs_sha256={str(p):digest(p) for p in [archive,run/'state_estimate.txt',run/'online-output/online-output_0.mcap',Path(__file__)]},
                  gt_csv_sha256=hashlib.sha256(gt_bytes).hexdigest(),
                  scope='Numerical public-baseline evaluation, not a flight accuracy qualification; no hardware; reset events not invented')
    out.mkdir(parents=True, exist_ok=False)
    (out/'associations.json').write_text(json.dumps(associations, indent=2)+'\n')
    result['associations_sha256'] = digest(out/'associations.json')
    (out/'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('inputs_sha256','rpe_adjacent','rpe_1s')}, indent=2))
    print(json.dumps({k:{a:b for a,b in result[k].items() if a != 'pairs'} for k in ['rpe_adjacent','rpe_1s']}, indent=2))


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', required=True)
    p.add_argument('--archive', required=True)
    p.add_argument('--out', required=True)
    args = p.parse_args()
    evaluate(args.run, args.archive, args.out)
