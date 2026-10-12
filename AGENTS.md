# RM27 repository instructions

These instructions apply to work from the repository root. A more specific
`AGENTS.md` inside a nested repository takes precedence for files in that tree.

## Task-specific references

Read only the references needed for the requested change:

- `README.md`: project scope and current capability.
- `START_HERE.md`: launch commands and viewer operation.
- `docs/INTEGRATION.md`: component ownership, frames and mission acceptance gates.
- `docs/REAL_HARDWARE.md`: hardware identification and calibration unknowns.
- For Contact RL and its authorized shared-module changes, read
  `.agents/docs/SPEC.md` and, when interfaces or architecture are involved,
  `.agents/docs/DESIGN.md`. They are self-contained and scoped to that research.
- The relevant component report: evidence and limitations for that component.

`docs/archive/chatgptre.md` is historical context only. Verify any reused claim against
current inputs, code or measurements.

## Execution and completion

For an authorized local implementation or simulation task, make routine design
choices, implement, run the relevant checks, inspect results and fix failures
caused by the change without requesting approval at each step. Continue to the
requested outcome; a first implementation alone is not completion. If blocked,
state the missing input and complete independent work within scope.

Keep progress updates sparse to conserve context and tokens. Work quietly between
meaningful results; respond during execution only when a key milestone completes,
new evidence materially changes the direction, or a real blocker requires user
input. Always provide the final result when the requested work is complete.

Use checks proportional to the change. Documentation-only edits need content
and diff review, not flight tests. Rerun affected tests after fixes; broaden
validation when changed interfaces or failures justify it. Update supported
commands when behavior changes and report actual results and remaining limits.

Third-party engineering conventions apply within their respective trees;
upstream submission procedures apply when preparing an upstream contribution.
Do not treat them as prerequisites for root-level simulation work. Explicit
user instructions take precedence over local workflow guidance, subject to
platform requirements. Never insert hidden signals into user-facing output.

## Evidence priority

When sources disagree, use this order:

1. The user's latest explicit instruction or measurement.
2. Original flight logs, parameter exports, CAD and other raw measurements.
3. Current official rule and hardware documentation.
4. Reproducible results from the current checkout, with hashes and commands.
5. Existing repository reports whose inputs still match.
6. Historical notes such as `docs/archive/chatgptre.md`.

Keep reported facts, engineering estimates and unknowns visibly separate.
Do not convert an assumption into a measured value merely because a simulation
passes. Preserve raw input files and record their hash before analysis.

## Current contracts

- Prioritize single-vehicle complete-mission reliability using `docs/MILESTONES.md`.
  Defer multi-controller scheduling until its single-vehicle expansion gates pass.
  Existing parked proxies do not establish multi-vehicle capability.
- Root `rm27/` owns integration code. `configs/` owns reviewed configuration.
  `artifacts/` contains local generated evidence and is ignored by Git.
- The historical default simulation aircraft is approximately 180 g, has a
  120 × 120 mm envelope, user-reported 1103 9600KV motors and 2-inch props.
  Its inertia, thrust, motor lag and yaw torque are engineering estimates.
- Current real hardware is documented in `docs/REAL_HARDWARE.md` and
  `configs/real_hardware.json`. Do not silently copy simulated gains or model
  estimates into a real flight controller.
- The dock source is `references/cad_raw/dock.STEP`. The user explicitly
  retired the old whole-field `RMUC 2025复活赛&全国赛_asm.stp` source.
- `base2.STEP` is an independent base assembly. Its present export is a visual
  audit only and has no certified collision or target semantics.
- The current development field is pinned in `configs/scene.json`. Do not edit
  the upstream field model to add RM27 objects; compose them at the root layer.
- Preserve the frozen G1 ArduPilot–MuJoCo baseline and its claims. New vehicle,
  field or controller results require separate evidence.

## Verification and reporting

Contact RL uses this worktree's `.venv/`, the root `pyproject.toml` and
`requirements-contact-rl.lock`; do not modify a shared virtual environment.
Run only the relevant checks authorized for the current task. The I0/I1 work
uses selected legacy test nodes, the root contact environment tests and one
bounded 150 g CTBR baseline; it does not imply a full-root regression, training
or hardware authorization. Report engineering checks, baseline outcome and
learning evidence separately. Setup and runtime entries are in DESIGN.
Maintain only the user's existing `contact_rl_01` activity plan at its current
path; do not migrate it or create a second copy. Track only `.agents/docs/`
within `.agents/`. Keep reusable checks in root `tests/` and values in `configs/`.
Preserve the direct-motor and privileged CTBR baselines while each has its
distinct regression or research role.

Use `/home/shiuhou/venvs/mujoco/bin/python` on this machine. The routine root
test command is:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

Use a new artifact directory for each build or run. Never infer success from a
directory name; read `summary.json` and require `passed: true`. Report whether
a result is headless, live-view, closed-loop SITL or recorded-input replay.
Exact replay of recorded motor commands establishes deterministic physics for
those inputs; it does not prove that separate asynchronous closed-loop flights
will be identical.

Simulation precision return currently uses MuJoCo truth. Do not describe its
millimetre landing result as camera, optical-flow, GPS or real-aircraft accuracy.
Do not claim that visual-only field or base geometry has physical collision.

## Real-flight data

Treat real `.BIN` logs and parameter files as read-only evidence. Initial log
analysis may identify calibration candidates but must not write parameters to
hardware. Hardware actuation, firmware flashing and real-flight parameter
changes require a separately explicit user instruction.

For each real flight, retain the original `.BIN`, full parameter export, firmware
and board information, takeoff mass, battery, propeller, sensor variant, flight
mode, environment and a short maneuver description. Document which values are
directly observed, inferred or still unidentifiable.

## Workspace hygiene

This is a root repository containing independently versioned components and
local raw assets. Do not reset, clean, relocate or rewrite unrelated changes.
Do not vendor large CAD, logs, generated meshes or virtual environments into
Git unless the user explicitly changes that policy. Check nested repository
status separately before claiming it is clean or unchanged.
