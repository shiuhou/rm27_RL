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
PX4-specific fitting. A conservative fitted hybrid reference now exists; its
closed-loop research acceptance remains failed, as described below.

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
Current paired development flights and a fresh fitted hybrid reference are
documented below. Their task gates fail, so they do not qualify an aggressive
contact strategy. Complete 150 g mission acceptance remains separate.
Hardware is not connected or modified.

Development version 2 keeps the original geometry and gates, and records any
Teacher design changes per run in `teacher_design.json`. Recentering the
terminal aim passes two development cases (plain center and modified retreat),
while the two slower candidates fail; see `px4-phase4-teacher-development-02`.
This is a new development set, not another selection pass over reserved v1.
An AP Teacher backend was added for the same rebuilt comparison inputs and
causal actor; its first rate-interface development flight
`px4-phase4-ap-teacher-development-01` fails its envelope and misses the target.
Motor replay matches exactly. These are development findings; no matched
holdout comparison or replacement accepted contact strategy is claimed.

## Fresh PX4 hybrid reference

`px4-teacher-reference-data-01` contains twelve independently seeded center-aim
Teacher episodes, all physically successful: eight train and four validation.
The v1 failed holdout is excluded. `distillation.px4_reference_training` fits
position/velocity/acceleration using train episodes only and retains geometric
feedback, the causal observer and source timestamps. It supports conservative
mode only; the bundle rejects other modes. This is a fitted hybrid reference,
not a direct action network.

Fit 01 failed its 2 mm validation gate because feature time included measured
observation age. Fit 02 removes that age and follows the Teacher's observation
clock; validation position RMSE is 0.409/0.377/0.440 mm. One fresh development
Student flight passes, with recovery 0.817 s and exact motor replay.

The separately frozen `px4-hybrid-reference-holdout-01` then fails: **9/14 task
successes**, one verified safe abort, nominal **7/8**. Delayed/dropout estimates
expose edge, undesignated and secondary contact and recovery failures. All
attempts are retained; neither the fitting score nor the development flight
qualifies the strategy. Further repair requires new development data and a new
reserved gate. The complete 150 g image mission uses the separate low-speed
image commitment/recovery controller, not this 180 g research reference.

The subsequent `px4-ap-teacher-pair-development-01` executes twelve current
paired flights: three independent equal sensor seeds for each backend and
rate/attitude interface, with the same 180 g geometry, centre-aim Teacher and
physical scorer. PX4 rate succeeds 3/3; AP rate, AP attitude and PX4 attitude
each succeed 0/3. The batch fails its all-task gate and retains misses, edge/
undesignated contact and recovery failures. Replay and launch inputs remain
available per attempt. This is development comparison evidence; it does not
qualify AP/PX4 equivalence, the fitted Student or a complete mission.

```bash
/home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.experiments.aggressive_contact.distillation.px4_reference_training \
  artifacts/px4-teacher-reference-data-01 --output artifacts/px4-reference-fit-new

/home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.experiments.aggressive_contact.flight_control.batch \
  --config configs/px4_hybrid_reference_holdout_v2.json --workers 2 \
  --output artifacts/px4-hybrid-holdout-new
```
