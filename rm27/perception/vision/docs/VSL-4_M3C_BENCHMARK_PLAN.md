# Future VSL-4 benchmark contract and host harness

Implementation: **IMPLEMENTED** external-process host measurement and version-1
JSON result format. Validation: labeled host stubs exercise successful exit,
nonzero exit and timeout, serialization, clock-aware age and timing statistics.
**VSL-4 remains NOT_STARTED; no M3C deployment or measurement occurred.**
Physical blockers: validated VSL-2 calibration, backend build for the actual
M3C environment, current-configuration VSL-1B timing/sequence evidence and an
identified device with available resource sensors. Next gate: qualify one
backend/dataset/calibration on the host, then explicitly authorize a matched
M3C bench run. Host results do not establish M3C throughput or memory fitness.

## Implemented measurement

`experiments.benchmark.execute_process` starts an external process with an
argument array, saves stdout/stderr, samples resources and enforces a timeout.
On POSIX, timeout kills the launched process group. `runner` binds results to
backend, dataset, calibration, input selection and nominal rate. The generic
benchmark CLI measures only the provided command; it does not invent these
identities when the command is not a localization experiment.

The format records:

| Field | Meaning / availability |
|---|---|
| `schema_version` | 1 |
| `measurement_platform`, `environment` | HOST now; observed OS/architecture/interpreter/module hashes |
| backend/dataset/calibration/rate/frame IDs | Attached by the offline runner; no guessed identity in generic process measurements |
| `processed_frame_count`, `skipped_frames` | Null unless observed; submitted frames alone do not prove processed frames |
| `per_frame_processing_s`, `processing_time_statistics` | Observed backend call durations; count, p50/p95/p99; null quantiles when absent |
| `wall_clock_s` | Host monotonic process-launch-to-termination interval |
| `samples[].rss_bytes` | Sampled launched-process RSS via Linux `/proc`; null if unavailable |
| `samples[].available_system_memory_bytes` | Linux MemAvailable; whole-system observation, not process RSS |
| `samples[].process_cpu_seconds`, `cpu_percent_one_core` | Process CPU time and sampled derivative, 100% = one core; may exceed 100% |
| `peak_sampled_rss_bytes`, `sampled_rss_growth_bytes` | Sampled, may miss peaks; first-to-last growth is not proof of a leak |
| `temperature_c`, `throttling` | Null until actual device sensor/counter integration exists |
| `queue_depth`, `map_growth` | Null until exposed by an observed backend/device interface |
| `source_to_output_age_s` | Null in present stock backend runs; no fabricated timing relation |
| `exit_code`, `crash_signal`, `timeout`, `failure_reason` | Observable process outcomes |
| `oom` | UNKNOWN: a SIGKILL alone does not establish OOM |

Samples use native `/proc` units and system clock-tick rate, with no x86-specific
result schema. On other systems the unavailable resource fields stay null.
Child-process RSS/CPU is not aggregated; system-wide CPU or hardware peak memory
is not claimed. A short process may end before a useful RSS sample. Long-run
samples can reveal trends, but map size/growth requires future explicit telemetry.

`source_to_output_age` only computes a duration for independently VERIFIED,
known compatible units and the same known clock domain, with named events.
It refuses negative ordering. A receive-to-publish age is not exposure-to-output
latency. Cross-device clock mapping is not implemented. Raw VIN PTS remains
UNKNOWN until separately established.

## Host-only example

This command was exercised as a software measurement, not a SLAM backend:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.benchmark --out artifacts/new-host-process-benchmark --timeout-s 2 -- /home/shiuhou/venvs/mujoco/bin/python -c 'import time; print("TEST_FIXTURE_ONLY"); time.sleep(0.2)'
```

Use a new directory. It produces `benchmark.json`, `stdout.log`, `stderr.log`.
The offline runner's fixture tests also replay selected image metadata through
both mock backend command interfaces and attach the same measurement format.
No stub estimates motion from images; its predetermined poses only test software.

## Planned M3C protocol, not executed

Freeze module/lens/focus/crop/build, calibration, exact input images/PTS and
backend settings. Record actual memory capacity/availability and software
versions rather than inferring from a product label. Run fixed workloads at
several supported input rates; record submitted/processed/skipped frame IDs,
queue policy/depth, per-frame processing times and clock-qualified age. Preserve
warmup and long-run resource/map traces, thermal data and evidence of throttling
if observable. Record crash, timeout and kernel OOM evidence separately.

Keep paced replay versus unpaced batch execution visible. The pinned ORB example
sleeps and runs a viewer; stella's adapter uses unpaced headless execution.
Their wall-clock runtimes are not directly comparable performance instruments.
Before fair VSL-4 throughput comparison, review equivalent pacing/viewer settings
or a minimal measurement driver without changing estimator algorithms.

NPU/CUDA availability is not assumed to accelerate geometric SLAM. Establish
actual measurements before claiming a board can sustain any input rate. Nothing
in this harness marks VSL-4 PASSED, HARDWARE-TESTED or FLIGHT-TESTED.
