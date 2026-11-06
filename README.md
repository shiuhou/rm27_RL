# RM27 Agile Flight RL

The main route is now **PerchRL-style Two-Stage PPO**. First learn the complete
continuous flight maneuver and controlled physical underside contact from a
stable-hover takeover. Learning-efficiency comparisons and collision recovery
remain separate research steps.

Stage 1 uses reliable simulated state and the existing PPO/CTBR stack. Traditional
systems own hangar departure, navigation and search; the policy owns acceleration,
deceleration and attitude changes after hover takeover. The platform is fixed.
There is no teacher-action imitation, prescribed flight trajectory or attitude
schedule. Stage 2 will fine-tune the learned policy with processed Tracking,
VIO/PX4 estimates, sensor rates, noise, latency and short observation loss. It
does not ask the policy to recognize RGB images.

The new engineering task is `fixed_surface_contact_v1`, with observation schema
`fixed_surface_state_v1`. Its 26 state inputs exclude motor internals, contact
labels and evaluator phases. A 12-dimensional hover perturbation covers position,
velocity, attitude and angular rate; independent evaluation uses 19 fixed starts.
Success requires actual 1 ms MuJoCo solver contact by the designated underside
fixture on the registered surface, under separate geometry and safety checks.
Old I4 arrival 0.20 m before the plate is a different task.

Task parameters and reward weights in [task_contact_geometry.json](configs/stage1/task_contact_geometry.json) are
`PROPOSED_UNCONFIRMED`, with `training_allowed=false`. The current foundation
permits only a bounded engineering smoke with one 64-transition PPO rollout,
one epoch and at most 60 seconds of learning. This does not deliver a trained
flight policy. Evidence belongs to `artifacts/agile_flight/stage1_foundation_01/`;
completed checks and costs must be read from its actual outputs, not inferred
from the presence of an implementation or command.

The foundation interface checks completed: 26 initial new tests, one added zero-step
replay test and 67 related regression tests passed, with one 64-transition rollout
and exact checkpoint/episode replay.
The policy achieved **0/19**, all flight-volume exits. The physical contact fixture
also **failed**: a registered plate and an unregistered near-coplanar CAD shell
contacted simultaneously. Both failures and the original geometry are preserved.
That historical rejection remains recorded in `smoke_completed/summary.json`.
The contact follow-up traced `35364` to the existing corrected `visual_115` skin/rim
proxy and `36293` to the confirmed `visual_117` front armor shell. The current task
adds exactly its eight front partitions through a hash-checked, task-owned
[registration](configs/stage1/contact_surface_registration.json); the physical model,
90×100 mm footprint, reward and scorer gates are unchanged. The old `task.json`
and foundation PPO configuration retain their original classification and checkpoint semantics.

Contact engineering checks now accept the real 6 ms center event, reject a clear-start
outside-region first contact at 61 ms and a wrong-aircraft-part contact at 79 ms,
and retain initial-overlap, side/back-face and simultaneous-illegal-contact failures.
Five recorded episodes re-simulate exactly. Evidence is in
`artifacts/agile_flight/stage1_contact_geometry_01/`; this follow-up performed zero
policy updates. Contact geometry is ready for engineering acceptance.
The user has now approved one simulation-only Stage 1-A seed 0 pilot using
`configs/stage1/ppo_stage1a_seed0.json` and `task_stage1a_seed0.json`. Numerical
settings and reward are inherited unchanged; this approval is not hardware calibration.
The run is bounded by 131,072 training transitions / 1,800 s learning and a separate
shared 600 s assessment budget, with the original 19 starts before and after training.
The single approved run is now complete and stopped: 131,072 transitions in
1,501.227 s, 256 finite PPO updates. Formal success is **0/19 before and 0/19 after**;
all final episodes terminate on excessive body rate. Controlled contact and agile
motion are not learned. That original run stopped at its authorized boundary. The
[evidence viewer](artifacts/agile_flight/stage1a_seed0_01/analysis/index.html)
contains every update curve, all 19 before/after outcomes, checkpoints and five
full-duration recorded-state replays. Assessment including two exact command
re-simulations used 131.082 s. The complete application summary was saved; the
outer tool session returned 143 for an unverified reason, so clean process exit
is not claimed. Saved final checkpoint and replay integrity checks passed.
A subsequent [offline diagnosis](artifacts/agile_flight/stage1a_seed0_diagnosis_01/index.html)
reconstructs the saved actor-to-motor chain without new physics steps or learning.
It distinguishes inverse-mixer clipping from lagged rotor force and identifies
collective/torque distortion during the failure. Gamma-consistent deterministic
returns worsened despite improved raw reward, so early-failure reward exploitation
is not established. A subsequent [bounded action-scale diagnostic](artifacts/agile_flight/stage1a_action_scale_01/index.html)
completed 16 windows without training. Matched physical CTBR trajectories are
identical. With the frozen final policy, explicit rate_scale_beta=0.1 removes
mixer clipping and reduces peak rate from 20.002 to 1.349 rad/s, but the aircraft
still exits the flight volume at 0.788 s. The 1-second no-safety-failure gate
therefore FAILS; task learning is not demonstrated. Default mapping, task bounds,
reward and Physics remain unchanged. The later separately approved comparison is recorded below.
The original training exit 143 remains UNKNOWN; this diagnostic exited 0.

The separately approved [β=0.1 seed0 comparison](artifacts/agile_flight/stage1a_beta01_seed0_01/analysis/index.html)
is now complete: original initial actor/critic/optimizer/normalizer restored exactly;
only the rate mapping changes, with the same physical-contact benchmark and 19 starts.
131,072 controls / 1,523.502 s learning, 192.438 s assessment, process exit 0.
Formal success remains **0/19 before and after**, all final failures are flight-volume exits.
Mean final mixer clipping falls from 96.04% to 44.54%; discounted return improves
relative to the old final, but worsens slightly relative to this run's own initial.
The final policy has prolonged zero-thrust intervals; contact and coordinated flight
remain unlearned. All 256 update checkpoints and 460 training trajectories are retained,
including four rejected physical contacts and one incomplete final fragment.
No further training or task changes are authorized. Original results remain unchanged.

E0/E1-A BPTT work and G2/X9/X18 PPO/SAC results remain historical method evidence.
E1-B is still an old-I4 experiment, not the new Stage 1. Collision recovery first
uses a separate control/estimator-readmission baseline; eventual policy-to-recovery
handoff must preserve continuous physical state. Stage 1 need not learn full recovery.

## PDF season scope and historical interface batch (2026-10-30)

Research source: `RM27 无人机组开发文档 (2).pdf`, SHA-256
`d4526f85248623ada0a7c64e120184c8b6838dae215593ea034ee6e8825c437e`. Current goal: repeated fixed-base underside contact,
separation, attitude/estimation recovery and target reacquisition in one sortie.
RK3576 is the new platform direction; its full hardware/data chain is not qualified.
Return-to-dock and precision landing below belong to historical baselines, not the
new season's mandatory deliverables. Traditional/RL allocation remains a research
choice in that historical batch; the current Stage 1 allocation is stated above.
Detailed hardware tasks remain separately scheduled by the user.

Component responsibility: Autonomous agile-flight/contact policy research and fair traditional-control comparison. The historical cross-repository interface batch only exposed observation/action conversion and fixed-seed untrained actor inference; its evidence does not establish learned First Contact capability.

That cross-repo interface batch is synthetic/offline only, with no
learning, firmware execution, hardware operations or Git publication. The root
repository coordinates the single activity plan and derived report. Component
contracts remain self-contained; the shared engineering method is unchanged.

Integration source when in the parent checkout: [../../docs/INTEGRATION.md](../../docs/INTEGRATION.md). The local `interface_worker.py` is a JSONL test endpoint; it does not qualify the underlying estimator/controller.

## Local cross-process interface contract

Run `python -u -m interface_worker` from this repository's root using its isolated
Python environment. Input/output is one JSON object per line; stderr is diagnostic.
Envelope: `version: 1`, `source: coordinator`, increasing integer `seq`,
`input_kind: synthetic`, `kind`, and object `payload`. `kind: hello`, payload `{}`
returns actual loaded source paths and dependency versions. Replies echo kind/seq,
identify this component, and contain `ok` with payload or typed error. Other input
provenance labels are intentionally refused by this synthetic acceptance endpoint.
The application schema below is owned by this repo; JSONL is not the final vehicle bus.

`kind: infer` accepts observation_schema=contact_world_v1, native VehicleEstimate
state, target_position/target_normal in world NWU, previous_action CTBR,
remaining_s and seed. It uses the existing 26D observation formula, a fixed-seed
untrained SB3 ActorCriticPolicy and NormalizedCTBR action conversion. Reply contains
observation, normalized/action arrays, maximum_collective_N, policy identity and
learning_steps=0. The first inference initializes the actor; subsequent calls use
that same actor. Restart the process to choose another seed. No learn/reset/step or
reward change. Environment: historical MuJoCo 3.12.0, Torch 2.14.1+cpu, SB3 2.9.0.
Physics comes from the independently installed simulator package.


Independent native MuJoCo contact-and-recovery research: a 150 g engineering
profile, CTBR control, shared contact evaluator and a Gymnasium environment.
The CPU PPO workflow supports checkpoint save/reload and independent evaluation.
A privileged CTBR baseline previously recovered successfully; the I3 65,536-step
policy returned TARGET_MISS. This is not evidence of reliable learned recovery.

I4 adds a separate state-based, no-contact terminal-arrival task. Its actor
autonomously commands the same CTBR chain; arrival is scored independently of
contact recovery. Original Contact defaults and historical checkpoints remain
available. New task/schema metadata prevents loading a Contact policy as Arrival.

## Install (Linux amd64, Python 3.12)

```bash
git clone https://github.com/shiuhou/rm_uav_mujoco.git
git clone https://github.com/shiuhou/rm27_RL.git
cd rm27_RL
python3.12 -m venv .venv
uv pip sync --python .venv/bin/python --require-hashes requirements-contact-rl.lock
uv pip install --python .venv/bin/python --no-deps --no-build-isolation -e ../rm_uav_mujoco/packages/physics -e .
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl --help
```

No parent RM27 clone is required. Clone both component repositories alongside each
other, using revisions containing the separation change. The single hash lock supplies
third-party dependencies; Physics comes from the simulator checkout and RL from this checkout.
Physics can also be built and installed as a wheel without RL, Gymnasium or Torch.
Pin the simulator checkout SHA or Physics wheel SHA-256 when consuming it elsewhere; version
0.1.0 alone does not identify source. RL uses editable installation with these
configuration files; a standalone RL wheel is not the supported distribution.

The new Stage 1 task additionally needs the explicitly registered extracted scene,
target mapping and Ver2 inputs listed in `configs/stage1/task_contact_geometry.json`. These local
assets and the compiled private scene are not bundled in Git clones. Existing
I0–I4 install examples do not establish that Stage 1 assets or Docker mounts exist.

## Stage 1 local engineering entry points

The source scene remains unchanged. Preparation verifies its frozen hashes and
creates a private fixed-platform model with the existing assumed underside sphere.
Normal environment resets reuse that verified model. Commands below describe the
interfaces; they do not authorize another smoke or a formal training run.
The current task reuses the foundation MJB without recompilation. Its optional
registration is applied only after the frozen physical cache is validated. The
`ppo_foundation.json` smoke below is the historical configuration, not a seed 0
training configuration for the corrected registration. To check contact behavior
without policy optimization using the prepared cache:

```bash
env -u PYTHONPATH RM27_STAGE1_TEST_MODEL=artifacts/agile_flight/stage1_foundation_01/scene/scene.mjb PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_stage1_task.py -q -k 'contact or registration or overlap or face'
```

```bash
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-prepare --config configs/stage1/task.json --output artifacts/agile_flight/stage1_foundation_01/scene
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-smoke --config configs/stage1/ppo_foundation.json --output artifacts/agile_flight/stage1_foundation_01/smoke
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl evaluate --checkpoint <checkpoint-dir> --task fixed_surface_contact_v1 --output <new-evaluation-dir>
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-replay <episode-dir>
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-replay <episode-dir> --view
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-replay <episode-dir> --render <new-image-dir>
```

Episode outputs retain the full scene and motor states, observations, commands,
rewards, the terminal contact/failure record and telemetry plots. Default replay
re-simulates recorded commands and checks state/reward/termination consistency;
`--view` and `--render` display saved states. Neither playback nor a successful
optimizer smoke demonstrates autonomous contact learning. Formal Stage 1 work
must deliver a learned policy, complete replay, independent success rate and failures.
See [SPEC §10](.agents/docs/SPEC.md) and [DESIGN §14](.agents/docs/DESIGN.md) for
the PerchRL adaptation, protected historical semantics and remaining integration work.

## Docker and results

```bash
bash docker/run.sh build rm27-contact-rl:separated-cpu
bash docker/run.sh run rm27-contact-rl:separated-cpu /path/to/results new-baseline --check
bash docker/run.sh evaluate rm27-contact-rl:separated-cpu /path/to/results new-evaluation /path/to/checkpoints/final
# Existing bounded CPU training entry (not part of separation verification):
bash docker/run.sh train rm27-contact-rl:separated-cpu /path/to/results new-training
# I4: one bounded CPU run, initial/final policy and traditional comparison,
# nine fixed initial states each; 524,288 transitions or 15 learning minutes.
bash docker/run.sh train rm27-contact-rl:separated-cpu /path/to/results new-arrival-training --config configs/agile_flight/ppo_terminal_arrival.json
```

One headless CPU image contains both packages and configurations. Dependencies
live in `/opt/venv`; optional `--source` mounts RL at `/workspace` and Physics at `/opt/physics`, both read-only.
Builds use a separate Docker build context for Physics. Set `RM27_PHYSICS_ROOT` to
override its default sibling path. Rebuild old images before using this layout.
Results are writable separately; checkpoint input is mounted read-only. Each run
needs a new identifier. Evaluation exits 2 for a recorded task failure; that is
not necessarily an installation failure. Old experiment/checkpoint artifacts
are not stored in Git and must be supplied explicitly for historical comparison.

`src/rm27_contact_rl` owns RL; the external simulator repository owns
`rm_uav_mujoco/packages/physics` physical/control and historical Contact/Arrival evaluation implementation.
The new Stage 1 task scorer stays in RL; `configs`, `tests` and `docker` contain their respective inputs.
[SPEC](.agents/docs/SPEC.md) is the adopted engineering and research contract;
[DESIGN](.agents/docs/DESIGN.md) describes interfaces, setup and evidence limits.
Generated environments, wheels and results are ignored.

The local I4 activity plan is `.agents/plans/plan_agile_terminal_arrival_01.md`.
It is ignored and not distributed through Git; SPEC and DESIGN fully describe
the task contract without requiring this local file. A newer source checkout
must be built into the image before using its new task configuration.

History and `research/contact-rl` remain intact. Original RM27 consumer adaptation
(S3) is pending; the old system still uses its existing implementation. The review
branch does not automatically become the GitHub default. No hardware calibration,
GPU scaling or simulation-to-real success is claimed.


## Shared real-camera test videos

[Shared video catalog](data/shared_vision/README.md) provides four user-supplied
real drone-camera MP4 fixtures, hashes, reported capture settings, decoding checks
and replay instructions. `data/shared_vision/` is a relative link to the shared
RM27 dataset, with one physical copy; videos are local data, not included in a
standalone clone. Keep derived test outputs separate from these inputs. See the
catalog for exposure-mapping and synchronized-IMU/timestamp limitations.
These videos can support offline decoding, preprocessing or visual-frontend
regressions for future Stage 2 work; they are not Stage 1 policy inputs or full
VIO/real-flight acceptance evidence.

## Shared M3C and MaixCAM2 board references

[Shared board catalog](data/shared_m3c_board/README.md) provides the user-supplied
M3C/MaixCAM2 schematics and 3D/mechanical files, with a SHA-256 manifest.
Use `data/shared_m3c_board/raw/` from this checkout; all local aliases share one
physical collection. Match the document to the actual board revision. Raw files
are local references supplied separately from Git clones; see the catalog for paths.

### Independent noncontact flight pilot

`state_flight_control_probe_v1`: hover and four 0.20 m position shifts; separate identity from fixed-surface contact. Contract: SPEC §13; implementation and pinned upstream review: DESIGN §15. Configurations: `configs/flight_probe/`. Paired bounded runner: `python -m rm27_contact_rl.flight_probe_runner`; requires passing engineering evidence. Existing contact/I4 results remain unchanged. Do not launch another pilot without a new approved budget.

Completed seed0 pilot: A0/23, B0/23; both65,536 controls, no stable-flight success. The PD feasibility baseline passed5/5. Curves, full evaluation tables, checkpoints and recorded MuJoCo playback are in `artifacts/agile_flight/state_flight_probe_ab_seed0_01/index.html`. Use `stage1-replay <episode-directory> --view` for recorded flight playback. This result does not supersede contact0/19 or authorize further training.
