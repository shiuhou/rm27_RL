"""Real no-data native shutdown regression, not a synthetic VIO test.

Run inside the isolated research image after sourcing /ws/install/setup.bash.
No sensor publishers, dataset or RM27 adapter are involved.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--count', type=int, default=5)
args = parser.parse_args()
if args.count < 1:
    raise ValueError('count must be positive')
args.out.mkdir(parents=True, exist_ok=False)
rclpy.init()
observer = rclpy.create_node('rm27_shutdown_test_observer')
results = []
try:
    for mode in ('true', 'false'):
        for index in range(args.count):
            logfile = args.out / f'pub-{mode}-{index}.log'
            command = ['/ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf', '--ros-args',
                       '-p', 'config_path:=/upstream/config/euroc_mav/estimator_config.yaml',
                       '-p', 'use_stereo:=false', '-p', 'max_cameras:=1', '-p', 'verbosity:=INFO',
                       '-p', f'multi_threading_pubs:={mode}']
            with logfile.open('x') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                ready = False
                timed_out = False
                try:
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline and process.poll() is None:
                        if observer.count_subscribers('/imu0') == 1 and observer.count_subscribers('/cam0/image_raw') == 1:
                            ready = True
                            break
                        time.sleep(0.1)
                    if ready:
                        time.sleep(0.5)  # permit an idle visualization loop before shutdown
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        code = process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        timed_out = True
                        process.kill()
                        code = process.wait()
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait()
            text = logfile.read_text()
            errors = [token for token in ('SEVERE WARNING', 'terminate called', 'Error in destruction',
                      'Failed to delete', 'Segmentation fault') if token in text]
            results.append(dict(publisher_thread=mode, iteration=index, ready=ready,
                                returncode=code, timeout=timed_out, teardown_errors=errors,
                                passed=ready and code == 0 and not timed_out and not errors,
                                command=command, log=logfile.name))
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and (observer.count_subscribers('/imu0') or observer.count_subscribers('/cam0/image_raw')):
                time.sleep(0.1)
finally:
    observer.destroy_node()
    rclpy.shutdown()
report = {'schema_version': 1, 'sensor_samples_sent': 0, 'results': results,
          'passed': all(item['passed'] for item in results)}
(args.out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
