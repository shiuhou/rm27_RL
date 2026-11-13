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

The upstream checkouts are separate Git repositories. Use the following
revisions when restoring this workspace.

| Directory | Revision |
| --- | --- |
| `upstream/SimpleFlight` | `f5ae8fc3689edc7c982e07cbfd587eeeab72f279` |
| `upstream/rm27_RL` | `ed4ebc550800baf032348f67af5699b44e7953f2` |
| `upstream/rm27_sim` | `6ec552b4a836c601295b7657c78c4bd3af977600` |
| `upstream/rm_mjcf` | `d9439a97cee80baba9d13a1de9882f2d8095fca5` |
