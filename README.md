# RM27 Agile Flight RL

Simulation research for the 142 g ASM2 aircraft. The project implements Direct-CTBR flight control, deterministic MuJoCo contact evaluation, an Isaac Lab contact proxy and PPO braking experiments.

Start with the [specification](.agents/docs/SPEC.md), [design](.agents/docs/DESIGN.md) and [roadmap](.agents/docs/ROADMAP.md). The [engineering rules](engineering.md) describe the workspace conventions.

Initialize the pinned upstream checkouts with:

```sh
git submodule update --init --recursive
```

The simulation launchers use hash-pinned local inputs under `.agents/runs/`. Restore the corresponding local artifacts at the paths documented in the design and `configs/` before running those workflows. These include the frozen scene and runtime receipts, experiment datasets and checkpoints. Experiment source scripts are tracked in `experiments/asm2-terminal/`. To use the workspace launchers, copy the relevant phase directory's scripts into its matching `.agents/runs/asm2-terminal/` directory alongside its pinned inputs.

The snapshot includes the derived aircraft simulation assets. The source STEP model and preview renders are supplied separately for workflows that regenerate assets from CAD.
