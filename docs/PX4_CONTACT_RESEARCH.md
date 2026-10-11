# PX4 Phase 4/5 contact research

Updated 2026-10-11. The existing aggressive-contact research now has an explicit
PX4 v1.17.0 backend. It uses the unchanged 180 g comparison aircraft, contact
geometry, reference designs and physical contact/recovery evaluator. These
runs start in an explicitly restrained airborne fixture; they do not qualify
takeoff, image target identification, return or landing.

## Software boundary

`flight_control.sitl` accepts `--autopilot {ardupilot,px4}` and
`--policy {student,teacher}`. The PX4 backend supports attitude plus force and
rate plus force. The inherited AP default remains available for historical
reproduction. PX4 uses reviewed `configs/px4/response.json`; it rejects AP
parameter files and privileged truth/correction actor options.

Synthetic external navigation retains its acquisition timestamps and body
velocity frame through MAVLink 2 ODOMETRY. PX4 EKF2 telemetry drives the actor.
Teacher contact detection uses the existing causal velocity-innovation observer.
The original physical evaluator reads recorded motor inputs separately and
checks their reconstructed state against the original flight before scoring.
The replay supports both 2 ms AP and 4 ms PX4 command holds. A retained AP
flight replay (`px4-phase4-ap-replay-01`) has maximum state error zero; this is
recorded-input compatibility evidence, not a new paired flight.

PX4 research commands and the causal observer run at 250 Hz. The inherited
Student timing is overridden explicitly in `configs/px4_contact_policy.json`.
Teacher features use contact time relative to airborne release, avoiding a
negative contact-age feature caused by the initialization clock. New Teacher
runs save causal observations and rate/force labels to `teacher_data.npz` for
future PX4-specific fitting. No trained PX4 Student is claimed yet.

The source scene is verified by hash and rebuilt for each run. Reviewed asset
root relocation is recorded in `configs/px4_research_inputs.json`; no stale
worktree absolute paths are inherited into the generated scene. Fourteen small
required files from the original Phase 3 bundle were restored into ignored
local artifacts, preserving hashes in `px4-phase4-inputs-01/source.json`.
Their original workspace remains read-only.

## Observed results

All named directories below are under `artifacts/`. Check their actual
`summary.json`; failed attempts remain present.

| Result | Evidence | Scope |
|---|---|---|
| Student attitude hover passed; 3.23 mm position estimate RMSE | `px4-phase4-hover-03` | Airborne research hover |
| Teacher attitude hover passed; 3.20 mm position estimate RMSE | `px4-phase4-teacher-hover-02` | Airborne research hover |
| Teacher conservative rate contact passed once; recovery 0.924 s; replay error zero | `px4-phase4-teacher-rate-01` | Development candidate only |
| Inherited Student development: 0/6 tasks passed, one verified safe abort | `px4-phase5-development-01` | Delay, dropout, bias and declared uncertainty |
| Frozen Teacher holdout v1: 1/14 tasks passed; one safe abort; nominal 0/8 | `px4-phase4-teacher-holdout-01` | Research acceptance failed |

The Teacher holdout retains all fourteen attempts. Correct bumper/armor first
contact in the nominal runs was followed by undesignated contact; delay/dropout
cases also expose edge contact and recovery/envelope failures. Thus the one
development success does not establish a reliable PX4 contact strategy. The
reserved v1 outcomes are retained as a failed acceptance set; subsequent policy
selection requires a separate development set and a new reserved gate version.

The old Student development batch reports `passed: true` only for execution
integrity, with that scope explicit in its saved summary. It is not task
acceptance. New matrix summaries separately report execution status, task
success count, safe abort count and failure categories. Specifications with
`acceptance: task_success` require every declared task/safe-abort outcome and
cannot pass merely because the processes exited. Source, configuration and
binary hashes are frozen before the first attempt and checked between groups.

Earlier initialization, Student/Teacher contact and recovery failures are
retained: `px4-phase4-hover-01/-02`, `px4-phase4-teacher-hover-01`,
`px4-phase4-student-contact-01`, `px4-phase4-student-rate-01/-02`,
`px4-phase4-student-conservative-01`, `px4-phase4-teacher-contact-01`, and
`px4-phase4-student-conservative-rate-01`.

## Run and inspect

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
/home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.experiments.aggressive_contact.flight_control.sitl \
  --autopilot px4 --policy teacher --mode maneuver --observation telemetry \
  --action-layer rate --skill-mode conservative --duration 10 --instance 45 \
  --parameters configs/px4/response.json --output artifacts/px4-teacher-new

/home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.experiments.aggressive_contact.flight_control.batch \
  --config configs/px4_phase4_teacher_holdout_v1.json --workers 2 \
  --output artifacts/px4-teacher-holdout-new
```

Use a fresh output directory each time. The second command reproduces the
failed reserved v1 configuration; it is not a supported claim of task success.
True AP/PX4 paired current flights, thrust-response selection, PX4 Student
retraining and the complete 150 g mission remain unfinished. Hardware is not
connected or modified.
