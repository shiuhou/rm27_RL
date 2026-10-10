"""Raw-IMU evidence validation and offline statistics; no synchronization or VIO."""
from decimal import Decimal

from ..schema import TimestampEvidence
from .common import require, number


def validate_imu_sample(sample):
    require(sample.get('schema_version') == 1 and type(sample.get('schema_version')) is int,
            'IMU_SCHEMA_INVALID','version 1')
    require(type(sample.get('sequence')) is int and sample['sequence']>=0,'IMU_SEQUENCE_INVALID','sequence')
    TimestampEvidence.from_dict(sample['timestamp'])
    require(sample.get('measurement_kind')=='RAW_IMU','IMU_KIND_INVALID','attitude telemetry is not raw gyro/accelerometer data')
    require(sample.get('gyro_unit')=='rad/s' and sample.get('accel_unit')=='m/s^2', 'IMU_UNITS_INVALID','explicit SI units required')
    require(isinstance(sample.get('device_id'),str) and sample['device_id'] not in ('','UNKNOWN'), 'IMU_DEVICE_MISSING','identity')
    for key in ('gyro_xyz','accel_xyz'):
        vector=sample.get(key)
        require(isinstance(vector,list) and len(vector)==3,'IMU_VECTOR_INVALID',key)
        for x in vector:
            number(x,key)
    if sample.get('saturation') is not None:
        require(type(sample['saturation']) is bool,'IMU_SATURATION_INVALID','boolean or absent/null')
    return sample


def validate_imu_sequence(samples):
    """One device/sequence space/clock. Gaps are not proof of physical drops."""
    require(isinstance(samples, list) and len(samples) >= 2,
            'IMU_SEQUENCE_TOO_SHORT', 'at least two samples')
    for sample in samples:
        validate_imu_sample(sample)
    first = samples[0]
    stamp = first['timestamp']
    factors = {'ns': Decimal('1e-9'), 'us': Decimal('1e-6'),
               'ms': Decimal('1e-3'), 's': Decimal(1)}
    require(stamp['unit'] in factors, 'IMU_TIME_UNIT_UNSUPPORTED', stamp['unit'])
    intervals, gaps = [], []
    for previous, current in zip(samples, samples[1:]):
        require(current['device_id'] == first['device_id'], 'IMU_DEVICE_CHANGED', 'split recordings by device')
        require(all(current['timestamp'][key] == stamp[key]
                    for key in ('unit', 'clock_domain', 'semantic', 'evidence_status')),
                'IMU_CLOCK_CHANGED', 'split recordings at clock or timestamp evidence changes')
        delta_seq = current['sequence'] - previous['sequence']
        require(delta_seq > 0, 'IMU_SEQUENCE_ORDER_INVALID', 'duplicate/reordered/reset sequence')
        delta = current['timestamp']['raw_value'] - previous['timestamp']['raw_value']
        require(delta > 0, 'IMU_TIME_ORDER_INVALID', 'duplicate/reordered timestamp')
        intervals.append(float(Decimal(delta) * factors[stamp['unit']]))
        if delta_seq > 1:
            gaps.append(dict(after_sequence=previous['sequence'], before_sequence=current['sequence'],
                             unrepresented_sequence_values=delta_seq-1))
    qualified = (stamp['clock_domain'] != 'UNKNOWN' and stamp['semantic'] != 'UNKNOWN'
                 and stamp['evidence_status'] == 'VERIFIED')
    import numpy as np
    median = float(np.median(intervals))
    return dict(schema_version=1, sample_count=len(samples), device_id=first['device_id'],
                first_timestamp=stamp, last_timestamp=samples[-1]['timestamp'],
                clock_qualified=qualified, observed_rate_hz=(len(samples)-1)/sum(intervals),
                duration_s=sum(intervals), median_interval_s=median,
                min_interval_s=min(intervals), max_interval_s=max(intervals),
                max_relative_interval_deviation=max(abs(dt/median-1) for dt in intervals),
                sequence_gaps=gaps, missing_sample_count=None,
                saturation_true_count=sum(s.get('saturation') is True for s in samples),
                saturation_unknown_count=sum(s.get('saturation') is None for s in samples),
                limits='cadence of recorded timestamps only; no verified exposure synchronization, hardware drops or sensor output rate')


def analyze_static_imu(samples, *, stationary_confirmed=False):
    """Overlapping Allan deviation of rate samples; does not fit noise densities.

    Stationarity is operator evidence, not inferred from small sample variance.
    Acceleration mean includes gravity; it is NOT an accelerometer bias estimate.
    """
    import numpy as np
    summary = validate_imu_sequence(samples)
    require(stationary_confirmed is True, 'IMU_STATIC_UNCONFIRMED', 'operator must confirm static capture')
    require(summary['clock_qualified'], 'IMU_CLOCK_UNQUALIFIED', 'known, VERIFIED timestamp domain and semantic')
    require(samples[0]['timestamp']['semantic'] == 'ACQUISITION', 'IMU_ACQUISITION_TIME_REQUIRED',
            'Allan tau requires verified acquisition timestamps, not receipt/publish time')
    require(len(samples) >= 32 and not summary['sequence_gaps']
            and not summary['saturation_true_count']
            and summary['max_relative_interval_deviation'] <= .01,
            'IMU_NOISE_INPUT_UNQUALIFIED', '>=32 samples, contiguous, no observed saturation, <=1% cadence jitter')
    values = np.array([s['gyro_xyz'] + s['accel_xyz'] for s in samples], dtype=float)
    cumulative = np.vstack([np.zeros(6), np.cumsum(values, axis=0)])
    allan = []
    cluster = 1
    while 2*cluster <= len(samples)//2:
        averages = (cumulative[cluster:] - cumulative[:-cluster])/cluster
        differences = averages[cluster:] - averages[:-cluster]
        deviation = np.sqrt(.5*np.mean(differences**2, axis=0))
        allan.append(dict(cluster_samples=cluster, tau_s=cluster*summary['median_interval_s'],
                          pair_count=len(differences), gyro_rad_s=deviation[:3].tolist(),
                          accel_m_s2=deviation[3:].tolist()))
        cluster *= 2
    return dict(schema_version=1, sequence=summary, stationarity='OPERATOR_CONFIRMED_NOT_INDEPENDENTLY_VERIFIED',
                gyro_mean_rad_s=values[:, :3].mean(axis=0).tolist(),
                accel_mean_m_s2=values[:, 3:].mean(axis=0).tolist(),
                gyro_sample_std_rad_s=values[:, :3].std(axis=0, ddof=1).tolist(),
                accel_sample_std_m_s2=values[:, 3:].std(axis=0, ddof=1).tolist(),
                overlapping_allan_deviation=allan, noise_parameters=None,
                limits='descriptive statistics only; gravity remains in accel mean; no calibrated noise densities, bias random walks or static detection; unknown saturation remains unknown')


def main():
    import argparse
    import json
    from pathlib import Path
    from .common import file_hash
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', required=True, type=Path, help='raw IMU JSONL')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--stationary-confirmed', action='store_true', help='enable static statistics; operator assertion')
    args = parser.parse_args()
    before = file_hash(args.samples)
    samples = [json.loads(line) for line in args.samples.read_text().splitlines() if line.strip()]
    report = (analyze_static_imu(samples, stationary_confirmed=True) if args.stationary_confirmed
              else validate_imu_sequence(samples))
    require(file_hash(args.samples) == before, 'IMU_INPUT_CHANGED', 'source changed during analysis')
    report.update(source_file=str(args.samples.resolve()), source_sha256=before)
    # Do not overwrite an earlier report or a source capture.
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
