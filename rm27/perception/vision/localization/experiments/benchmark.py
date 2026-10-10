"""Host external-process measurement, reusable on Linux M3C; no device deployment."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

from .common import require, number, environment, write_json


def timing_stats(seconds):
    for value in seconds:
        number(value, 'processing seconds')
        require(value >= 0, 'TIMING_INVALID', 'negative processing time')
    return {'count':len(seconds), **{name:float(np.percentile(seconds, q)) if seconds else None
                                   for name,q in [('p50_s',50),('p95_s',95),('p99_s',99)]}}


def sample_process(pid):
    """Linux /proc observations, null elsewhere; no architecture-specific units."""
    result = dict(rss_bytes=None, available_system_memory_bytes=None, process_cpu_seconds=None,
                  temperature_c=None, throttling=None, queue_depth=None, map_points=None)
    try:
        status = Path(f'/proc/{pid}/status').read_text().splitlines()
        rss = next(line.split()[1] for line in status if line.startswith('VmRSS:'))
        result['rss_bytes'] = int(rss)*1024
        mem = Path('/proc/meminfo').read_text().splitlines()
        result['available_system_memory_bytes'] = int(next(line.split()[1] for line in mem if line.startswith('MemAvailable:')))*1024
        stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        result['process_cpu_seconds'] = (int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK')
    except (OSError, ValueError, StopIteration, IndexError):
        pass
    return result


def execute_process(command, cwd, timeout_s=300., sample_period_s=.1):
    number(timeout_s,'timeout_s',True)
    number(sample_period_s,'sample_period_s',True)
    cwd = Path(cwd)
    require(cwd.is_dir(), 'BENCHMARK_INVALID', 'working directory must exist')
    start = time.monotonic()
    samples, timed_out = [], False
    with (cwd/'stdout.log').open('w') as stdout, (cwd/'stderr.log').open('w') as stderr:
        try:
            process = subprocess.Popen(command, cwd=cwd, stdout=stdout, stderr=stderr,
                                       start_new_session=os.name == 'posix')
        except OSError as exc:
            return {'schema_version':1,'execution_completed':False,'exit_code':None,
                    'failure_reason':'PROCESS_START_FAILED','detail':str(exc), 'wall_clock_s':time.monotonic()-start,
                    'samples':[], 'measurement_platform':'HOST', 'environment':environment()}
        while True:
            elapsed = time.monotonic()-start
            sample = dict(elapsed_s=elapsed, **sample_process(process.pid))
            if samples and sample['process_cpu_seconds'] is not None and samples[-1]['process_cpu_seconds'] is not None:
                dt = elapsed-samples[-1]['elapsed_s']
                sample['cpu_percent_one_core'] = 100*(sample['process_cpu_seconds']-samples[-1]['process_cpu_seconds'])/dt if dt else None
            else:
                sample['cpu_percent_one_core'] = None
            samples.append(sample)
            if process.poll() is not None:
                break
            if elapsed >= timeout_s:
                timed_out = True
                if os.name == 'posix':
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                else:
                    process.kill()
                process.wait()
                break
            time.sleep(min(sample_period_s, max(.001, timeout_s-elapsed)))
    rss = [s['rss_bytes'] for s in samples if s['rss_bytes'] is not None]
    return dict(schema_version=1, execution_completed=True, exit_code=process.returncode,
                failure_reason='TIMEOUT' if timed_out else 'PROCESS_NONZERO_EXIT' if process.returncode else None,
                timeout=timed_out, crash_signal=-process.returncode if process.returncode < 0 else None,
                oom='UNKNOWN', wall_clock_s=time.monotonic()-start, samples=samples,
                peak_sampled_rss_bytes=max(rss) if rss else None,
                sampled_rss_growth_bytes=rss[-1]-rss[0] if len(rss)>1 else None,
                processed_frame_count=None, skipped_frames=None, per_frame_processing_s=None,
                processing_time_statistics=timing_stats([]), source_to_output_age_s=None,
                temperature_c=None, throttling=None, queue_depth=None, map_growth=None,
                measurement_platform='HOST', environment=environment(),
                resource_scope='launched process only; samples may miss peaks; not children or whole system CPU')



def source_to_output_age(source, output):
    """Seconds only for independently verified same-domain timestamp evidence.

    Result concerns the named source event (e.g. receive); it is not automatically
    exposure age. Cross-clock mappings and device synchronization are future work.
    """
    from ..schema import TimestampEvidence
    a,b=TimestampEvidence.from_dict(source),TimestampEvidence.from_dict(output)
    factors={'ns':1e-9,'us':1e-6,'ms':1e-3,'s':1.}
    if (a.evidence_status!='VERIFIED' or b.evidence_status!='VERIFIED'
            or a.clock_domain=='UNKNOWN' or a.clock_domain!=b.clock_domain
            or a.unit not in factors or b.unit not in factors
            or a.semantic=='UNKNOWN' or b.semantic=='UNKNOWN'):
        return None
    from decimal import Decimal
    delta=Decimal(b.raw_value)*Decimal(str(factors[b.unit]))-Decimal(a.raw_value)*Decimal(str(factors[a.unit]))
    require(delta>=0,'CLOCK_ORDER_INVALID','output precedes source event')
    result=float(delta)
    number(result,'source-to-output age')
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--timeout-s', type=float, default=30)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args=parser.parse_args()
    require(args.command, 'BENCHMARK_INVALID','supply -- executable arguments')
    command=args.command[1:] if args.command[0]=='--' else args.command
    args.out.mkdir(parents=True,exist_ok=False)
    result=execute_process(command,args.out,args.timeout_s)
    result.update(command=command, purpose='HOST_PROCESS_MEASUREMENT_ONLY', vsl_4='NOT_STARTED')
    write_json(args.out/'benchmark.json',result)
    print(result['failure_reason'] or 'HOST_PROCESS_COMPLETED')
    return 0 if result['failure_reason'] is None else 1


if __name__=='__main__':
    raise SystemExit(main())
