# RM27 Agile Flight RL

This workspace contains the ASM2 simulation, control, and evaluation code and the
earlier SimpleFlight research implementation.

- [Research specification](.agents/docs/SPEC.md)
- [Design and simulation commands](.agents/docs/DESIGN.md)
- [Milestones and verification status](.agents/docs/ROADMAP.md)
- [Engineering workflow](AGENTS.md)
- [ASM2 model release and verification](artifacts/asm2_model_release/README.md)
- [Isaac runtime build and GPU checks](.agents/docs/DESIGN.md#34-isaac-runtime-on-the-rtx-4090)

The root repository covers project source, launchers, configurations, tests,
and shared documentation. A fresh checkout needs the simulation assets in
`assets/`, the pinned upstream checkouts, and any run-specific inputs referenced
by the configuration. Keep these local inputs, release archives, checkpoints,
and experiment outputs in their documented locations. Local plans and run
records live under `.agents/`.

Current research status is recorded in the [roadmap](.agents/docs/ROADMAP.md).
R8, historical R9 demonstration-regularized PPO, and R9.1 are closed No-Go
results. N1/R9-A has completed offline consumer checks; formal fixed-handover
evaluation remains blocked (three catalogued scenarios, zero eligible or
evaluated). New R9-B learning is unselected and not authorized.

The shared-handover consumer and its tests require the separately acquired G0
release subset from
[`shiuhou/rm27_drones`](https://github.com/shiuhou/rm27_drones/tree/9ea80518259173c15882ae8a3ab6059eb2643336),
revision `9ea80518259173c15882ae8a3ab6059eb2643336`, at
`upstream/rm27_g0/9ea8051/`, and the original receiver receipt at
`.agents/runs/n1-r9-a/acquisition.json` (SHA-256
`0fd9e74b8b99edd15da1b5986ea4fcfbb498fc1e09962e52ed1203cd90eb1641`).
These inputs are not included in this repository. Provision the pinned inputs
before running `tests/test_asm2_shared_handover.py`; a fresh clone alone is
insufficient. Keep receipt identity checks intact. See
[DESIGN §6.5](.agents/docs/DESIGN.md#65-n1r9-a-shared-scenario-consumer--fixed-handover-protocol)
for the supported offline scope and remaining dependencies.

The upstream checkouts are separate Git repositories. Use the following
revisions when restoring this workspace.

| Directory | Revision |
| --- | --- |
| `upstream/SimpleFlight` | `f5ae8fc3689edc7c982e07cbfd587eeeab72f279` |
| `upstream/rm27_RL` | `ed4ebc550800baf032348f67af5699b44e7953f2` |
| `upstream/rm27_sim` | `6ec552b4a836c601295b7657c78c4bd3af977600` |
| `upstream/rm_mjcf` | `d9439a97cee80baba9d13a1de9882f2d8095fca5` |
