# RM27 Agile Flight RL

Independent native MuJoCo contact-and-recovery research: a 150 g engineering
profile, CTBR control, shared contact evaluator and a Gymnasium environment.
The CPU PPO workflow supports checkpoint save/reload and independent evaluation.
A privileged CTBR baseline previously recovered successfully; the I3 65,536-step
policy returned TARGET_MISS. This is not evidence of reliable learned recovery.

I4 adds a separate state-based, no-contact terminal-arrival task. Its actor
autonomously commands the same CTBR chain; arrival is scored independently of
contact recovery. Original Contact defaults and historical checkpoints remain
available. New task/schema metadata prevents loading a Contact policy as Arrival.

## Version and ownership boundary

The install commands below target this published `main`: it still includes
`packages/physics/`, and both first-party packages are installed from this checkout.
The development workspace has a later extraction of shared Physics to
[rm27_sim](https://github.com/shiuhou/rm27_sim), locally named
`components/rm_uav_mujoco/`. Do not apply that branch's external-package install
instructions to this version or remove its bundled Physics through a docs edit.
RL environments, rewards, policies and training Docker remain owned here; shared
simulation scenes, complete missions and viewers are developed in Sim.

The published task described here is no-contact I4 Arrival. Later development
research and the proposed Ver2 first-contact task do not make this checkout a
completed learned contact/return system. Follow this revision's SPEC and DESIGN;
existing training commands are entry points, not permission to reopen a completed
budget. The ignored local activity plan is not available from a fresh clone.

## Install (Linux amd64, Python 3.12)

```bash
git clone --branch main https://github.com/shiuhou/rm27_RL.git
cd rm27_RL
python3.12 -m venv .venv
uv pip sync --python .venv/bin/python --require-hashes requirements-contact-rl.lock
uv pip install --python .venv/bin/python --no-deps --no-build-isolation -e ./packages/physics -e .
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl --help
```

No parent RM27 clone or submodule is required. The single hash lock supplies
third-party dependencies; the two first-party packages come from this checkout.
Physics can also be built and installed as a wheel without RL, Gymnasium or Torch.
Pin a full checkout SHA or wheel SHA-256 when consuming it elsewhere; version
0.1.0 alone does not identify source. RL uses editable installation with these
configuration files; a standalone RL wheel is not the supported distribution.

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
live in `/opt/venv`; optional `--source` mounts the checkout at `/workspace`.
Results are writable separately; checkpoint input is mounted read-only. Each run
needs a new identifier. Evaluation exits 2 for a recorded task failure; that is
not necessarily an installation failure. Old experiment/checkpoint artifacts
are not stored in Git and must be supplied explicitly for historical comparison.

`src/rm27_contact_rl` owns RL; `packages/physics` owns physical/control/evaluation
implementation; `configs`, `tests` and `docker` contain their respective inputs.
[SPEC](.agents/docs/SPEC.md) is the adopted engineering and research contract;
[DESIGN](.agents/docs/DESIGN.md) describes interfaces, setup and evidence limits.
Generated environments, wheels and results are ignored.

The local I4 activity plan is `.agents/plans/plan_agile_terminal_arrival_01.md`.
It is ignored and not distributed through Git; SPEC and DESIGN fully describe
the task contract without requiring this local file. A newer source checkout
must be built into the image before using its new task configuration.

History and `research/contact-rl` remain intact. S3 consumer adaptation is not
claimed by this published baseline; later local integration has separate evidence.
No hardware calibration, GPU scaling or simulation-to-real success is claimed.
