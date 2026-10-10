# RM27 aggressive contact and recovery laboratory

This executable research slice uses the current root 180 g rotor-level MuJoCo
vehicle generator, a continuous trajectory, geometric feedback, a physical
bottom bumper, a bounded inclined armor patch, forbidden rim collision, and
event-triggered separation/recovery. It is independent of the frozen G1 and
ArduPilot flight gates. See the [implementation report](../../../docs/AGGRESSIVE_CONTACT_REPORT.md)
for measured results and limitations.

## Whole-maneuver optimization

The [whole-maneuver report](../../../docs/WHOLE_MANEUVER_REPORT.md) removes the
fixed slow terminal handoff. Interior p/v/a/j derivatives, segment times,
contact state and recovery offsets are optimized for time from airborne dock
start to confirmed recovery. An optional constant-thrust-direction arc prepares
attitude before contact. The fast and early-pose designs expose a measured tradeoff.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver --parameters configs/contact_maneuver_early_pose.json --view --output "artifacts/maneuver-view-$(date +%Y%m%d-%H%M%S)"
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver-campaign --output artifacts/maneuver-my-campaign
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver-pose-study --input artifacts/maneuver-my-campaign --output artifacts/maneuver-my-pose
```

Use `configs/contact_maneuver_fast.json` for the faster design. A maneuver demo
ends after the same one-second recovery hold; campaigns keep a longer observation
window. It does not include takeoff, return or landing. Logs/model/trace are in
`OUTPUT/flight`, design and whole-task metrics in `OUTPUT/summary.json`.
`view`, `replay`, `render-maneuver` and `maneuver-audit` accept `--input OUTPUT/flight`.
Compact campaign recordings reference a shared hashed model plus a physical-array
overlay; retain the parent campaign directory when moving trial evidence.

## Existing RMUC field mission and viewer (v2)

The [v2 report](../../../docs/AGGRESSIVE_CONTACT_V2_REPORT.md) extends the same
plant/controller into the existing full RMUC field. One vehicle starts airborne
above the actual dock, flies continuously to the current blue-base armor,
contacts with the bottom bumper, separates, recovers, and returns near the dock.
The nominal mission takes 32 simulated seconds. It does not take off, land or
disarm. All state feedback remains MuJoCo truth; no SITL/hardware claim follows.

Run from the repository root. Paste each command as **one complete line**;
pressing Enter immediately after Python's `-m` causes a shell error.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact arena --view --output "artifacts/contact-arena-$(date +%Y%m%d-%H%M%S)"
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/contact-arena-selected-v2-20260923 --speed 0.5 --loop
```

`arena --view` executes new closed-loop physics, while `view` displays recorded
states. Space pauses; V switches follow/overview. UI interaction modifies a
private viewer model/data copy, never the control plant. Remove `--view` for a
headless mission; `--seed 501 --scale 2` enables a reproducible physical draw.
`arena` defaults to **adaptive_nominal** (nominal trajectory plus actuator observer
and thrust adaptation). For the scenario-selected trajectory add
`--parameters artifacts/contact-v2-confirmatory-20260924/selected_parameters.json`.
Both variants have completed the full nominal mission. Viewer API isolation is
tested and EGL rendering executed; a desktop GUI session has not been certified.

The compositor requires local existing assets, pinned in `configs/contact_arena.json`:
`artifacts/dock-final/scene.xml`, field mesh/texture files and dock exports. They
are ignored raw/generated assets, not guaranteed by a fresh Git clone. It checks
scene hashes and field revision, remaps the explicit historical field prefix,
and verifies retained geometry poses/sizes/collision masks. Missing assets fail
with traceback; there is no substitute toy field. `--scene isolated` uses the
existing isolated dock/base scene instead. Base tower visual geometry remains
visual-only; the added forbidden rim is a labeled research assumption.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact robust-campaign --candidates 48 --trials 64 --output artifacts/contact-v2-my-campaign
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact validate-v2 --input artifacts/contact-v2-my-campaign --output artifacts/contact-v2-my-numerical
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact causal-study --trials 32 --output artifacts/contact-v2-my-causal
MPLCONFIGDIR=/tmp/rm27-mpl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact plot --input artifacts/contact-v2-my-campaign --output artifacts/contact-v2-my-plots
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact render --input artifacts/contact-arena-selected-v2-20260923 --output artifacts/contact-my-render
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact arena-audit --input artifacts/contact-arena-selected-v2-20260923 --output artifacts/contact-my-route-audit
```

The v2 campaign freezes its selected parameters before fresh paired evaluation:
five methods × two uncertainty levels × two rectangle orientations. V1 had the
90 mm dimension along the slope; the existing arena has it across world Y.
`target.in_plane_rotation_deg` separates rectangle geometry from the desired
vehicle thrust direction. Default zero preserves v1 exactly; the arena uses 90°.
Compare methods **within** a geometry split. `selected_config.json` includes
the selected controller and geometry: use it with `episode --config` when
running v2 locally; a parameters file alone does not enable the v2 controller.

The actuator observer integrates past commands with an assumed 40 ms response;
feedback compensates actuator lag without reading actual motor states. A bounded
effective thrust/mass estimator uses ego velocity increments and freezes on first
contact. Differentiated truth velocity is an acknowledged deployment limitation.
Scenario selection adds a signed contact-region margin objective and worst-quartile
loss. This is a small model-based adaptive controller and parameter search,
not RL or a certified recovery value function. All original v1 command defaults
below are preserved.

## Local contact experiment (v1 defaults)

From the repository root, using the installed interpreter:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact episode --output artifacts/contact-my-demo
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact replay --input artifacts/contact-my-demo --output artifacts/contact-my-replay
MPLCONFIGDIR=/tmp/rm27-mpl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact plot --input artifacts/contact-my-demo --output artifacts/contact-my-plots
```

Use a **new directory** for every command. `summary.json` contains `passed` and
failure modes; an unsuccessful single episode exits with status 1. Outputs are
local generated evidence under ignored `artifacts/`. The demo starts airborne;
it does not take off from a dock or perform return/landing.

The selected advanced method can be trained and compared against nominal and
contact-only selection with the same plant, candidate pool and trial draws:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact campaign --candidates 48 --trials 64 --output artifacts/contact-my-campaign
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact episode --parameters artifacts/contact-my-campaign/selected_parameters.json --output artifacts/contact-my-selected-demo
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact validate --input artifacts/contact-my-campaign --output artifacts/contact-my-validation
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact recovery-map --output artifacts/contact-my-recovery-map
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact rate-ablation --output artifacts/contact-my-rate-ablation
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_aggressive_contact.py -q
```

`plot` also accepts a campaign or recovery-map directory. `replay` accepts an
episode directory (including `campaign/demo` or any individual training/test
trial). It advances physics using recorded motor commands and compares the
states; it does not re-run feedback and is not a separate reliability trial.

Dependencies are the existing integration environment plus SciPy, recorded in
[`requirements-contact-research.txt`](../../../requirements-contact-research.txt).
There is no neural-network framework, GPU requirement, hardware access or
network operation. A fresh environment can install that requirements file.

## Model and action contract

- `model.py` reuses `rm27.integration.drone.add`: same nominal mass, inertia
  estimate, rail/arm collision, motor layout, thrust bounds, yaw torque and
  first-order motor lag. It adds one spherical bottom bumper. It does not
  replace MuJoCo with a point-mass approximation.
- The target is the current `configs/base_target.json` patch, translated into
  local coordinates: 90 × 55 mm, normal approximately `(0.4617, 0, 0.887)`.
  V1's in-plane axes differ from the arena; use the explicit v2 orientation above.
  The extra rim is an assumed forbidden contact surface. No claim about the
  collision geometry of the complete field or `base2.STEP` follows.
- All unmeasured contact, control and randomization parameters are labeled
  NOMINAL/ASSUMPTION in `configs/aggressive_contact.json`. MuJoCo `solref` and
  friction are solver-model parameters, not measured stiffness/restitution.
- World and body +Z are up. `VehicleEstimate.rotation` maps body to world;
  translational velocity is world-frame, angular velocity is body-frame.
  `TargetState` supplies position/normal in the same local world frame, capture
  time and covariance. The current adapter is a static imperfect target estimate
  plus perfect ego-state truth; covariance is recorded, not fused.
- `Skill.command(estimate, contact_bit)` returns four commanded rotor forces
  in N, **before** motor lag/uncertain gain. Motor order matches the root model.
  There are no state resets, applied external forces or teleports after initial
  conditions. Evaluator truth is separate from the estimate/action interface.
- Target is acquired once at startup and committed for this short maneuver.
  Stale initial estimates abort. No continuous visual replanning, camera FoV,
  estimator dynamics or visual recovery is implemented. The contact bit is
  privileged simulator contact, latched until the next 200 Hz controller tick.

The flight phases are FREE_FLIGHT → PRE_CONTACT → CONTACT → SEPARATION →
RECOVERY → NORMAL_FLIGHT. Miss/stale-input branches enter ABORT. NORMAL_FLIGHT
currently holds the recovery anchor; ABORT is a simulated return-to-start hold
request, not a hardware-certified safe exit.

## Baseline and advanced method

A degree-7 polynomial satisfies initial and terminal position, velocity,
acceleration and jerk. For the given endpoint derivatives and fixed duration,
this is the minimum integrated squared-snap interpolant. Duration and endpoints
are selected outside that interpolation. Terminal acceleration is
`alpha * target_normal - gravity`; thrust-derived attitude aligns body +Z with
the surface normal. The body-frame bumper offset determines the COM endpoint.
Execution uses position/velocity feedback, SO(3) attitude error, desired rate
feedforward, rotor allocation/clipping and the existing motor lag.

The terminal reference penetrates the virtual plane by a configurable small
amount to ensure physical encounter. **Actual impact happens before the nominal
terminal time**, so actual contact speed/attitude must be measured; they are not
assumed equal to the endpoint constraints. This also exposes the difficulty of
optimizing a hybrid event rather than an unconstrained endpoint.

Three methods share the controller/recovery law:

1. `nominal`: one interpretable fixed parameter set.
2. `contact_only`: scenario search minimizing contact-part/edge error, contact
   position/attitude/rate/tangent speed and maneuver duration.
3. `recovery_aware`: the same loss and candidate pool, plus full-task failure,
   secondary-contact, recovery-time, post-contact rate and altitude-loss terms.

Latin-hypercube candidate search optimizes duration, endpoint normal/tangent
speed, terminal thrust and penetration. Selection minimizes mean training loss
plus 0.25 times worst training loss. It uses eight training scenarios. A selected
parameter file is written **before** held-out evaluation. This is empirical
scenario optimization of a feedback skill, not PPO, ILC, global optimization or
a learned viability certificate. A finite search can fail to find the best
method; the report retains negative comparisons.

`rate-ablation` tests a plausible simpler change: suppressing the finite-difference
desired angular-rate feedforward after the contact-triggered anchor reset. The
optional controller keys `recovery_rate_blank_s` (default 0) and
`recovery_rate_feedforward` (default true) implement this experiment. The study
uses only development seeds and is not used to select the primary methods.
Reducing peak rate alone is insufficient: the measured recovery can become
slower and fail the same deadline.

## Evaluation and evidence

Joint success requires designated first contact on the front face within the
bounded valid patch, configured speed/attitude limits, separation, no later
forbidden/secondary contact, and recovery. Recovery must enter tilt ≤10°,
rate ≤30°/s and speed ≤0.3 m/s within two seconds, hold for one second, lose
≤0.3 m altitude, and remain in the configured local volume. Long actuator
saturation, excessive rate, ground contact and simulator warnings also fail.
These are experiment gates, not referee rules or structural limits.

All simultaneous first contact pairs are evaluated. Contact-point velocity
includes `R @ (omega_body × r_body)`; impulse integrates signed compressive
armor-normal forces using the existing contact measurement utility. Other
parts/forbidden rim remain physical colliders. A secondary encounter requires
a gap longer than the configured 30 ms debouncing interval; shorter contact
chatter belongs to the original encounter.

Every episode records `trajectory.npz`, `summary.json`, `physical.json`,
`parameters.json`, `config.json`, generated XML and the actual compiled
`model.mjb`. Trace column names include state, motor commands/states, reference
position/velocity/acceleration/attitude/rate, force, contact and saturation.
JSON includes transitions, first-contact state, impulses, errors and failures.
Campaigns retain every candidate/trial, source snapshots, hashes, versions,
seeds and append-only progress. Runner exceptions produce `traceback.log` and
a failing summary. Existing output directories are never overwritten.

Campaign `passed` means execution completed without runner errors and the
selected nominal demo passed; randomized success counts are separate.
Recovery-map `passed` means its measurement campaign completed; green/red
states are in individual rows. Validation `passed` means timestep checks passed,
not that all sensitivity challenges succeeded. Read these scopes with each
summary. Referee scoring remains `null`.

## Toward ArduPilot and perception

The current root SITL adapter is a fixed 500 Hz, four-body scene with a tested
position/velocity guidance path. This research instead needs a 200 Hz geometric
controller and 1 kHz contact physics. Reusing the vehicle generator isolates
the research without rewriting the flight bridge. **No claim that ArduPilot is
broken or that this motor policy is directly deployable is made.**

The next integration step is a separate attitude/rate-and-thrust SITL adapter
with explicit NWU/FLU ↔ NED/FRD transforms, timestamp semantics, motor ordering
and thrust calibration. Its ArduPilot inner loop differs from this controller;
re-run the same evaluator before transferring any result. Replacing
`VehicleEstimate`/`TargetState` with timestamped real estimates and replacing
the privileged contact bit with a sensed/estimated event are separate tasks.

No exact AUW/inertia/thrust/contact calibration, camera pipeline, complete
takeoff/landing mission, damage scoring, structural survival, regulatory compliance
or hardware result is established by this package.

## Whole-maneuver teacher (current research)

See [the teacher report](../../../docs/WHOLE_MANEUVER_TEACHER_REPORT.md) for the
frozen 240-trial comparison. The global minimum-snap planner already exists;
`teacher.py` searches its time/alignment/terminal-thrust parameters, evaluates
contact AND recovery, and freezes three operating points before holdout.
`adaptive_nominal` and `robust_v2` retain their strong adaptive controller.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-return --parameters configs/contact_teacher_balanced.json --output artifacts/teacher-demo-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/teacher-demo-new --speed 1 --loop
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-campaign --output artifacts/teacher-campaign-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-export --input artifacts/whole-teacher-01 --output artifacts/teacher-dataset-new
```

Additional commands: `recoverability-study`, `recoverability-audit`,
`teacher-interfaces`, `teacher-return-campaign`, `teacher-validate`, `teacher-plot`.
The latter five take `--input CAMPAIGN` except `recoverability-audit`, which takes
its model-study directory. All take a new `--output` directory. Recoverability
fitting uses the historical artifacts explicitly listed in `recoverability.py`;
`configs/contact_teacher.json` pins the trained model path. These inputs and the
existing local field assets must be available.

Dataset `rm27.teacher.v1.1` has an explicit actor allowlist and privileged-only
supervision, events, physical draw, motor states and future outcomes. It keeps
failed trials and immutable source hashes. Holdout demos remain evaluation-only.
Ego observations use truth in place of an onboard estimator. Body-rate +
collective-force projection is tested with a MuJoCo rate-loop proxy and MAVLink
serialization, **not ArduPilot firmware or hardware**. A calibrated N-to-throttle
map, Guided thrust semantics and angle-boost handling are still required.

## Policy distillation

The [distillation package](distillation/README.md) adds actual BC/history,
DAgger/corrective training and a working hybrid student. See the
[distillation report](../../../docs/WHOLE_MANEUVER_DISTILLATION_REPORT.md) for the
384-episode frozen comparison and observation/action limitations. The pure
neural action students did not succeed in this study; their failures are retained.
