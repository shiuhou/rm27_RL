# Whole-maneuver imitation research

Read [the report](../../../../docs/WHOLE_MANEUVER_DISTILLATION_REPORT.md) before
interpreting a checkpoint. Pure action BC/history/DAgger failed closed-loop in
this study. The working student is a supervised B-spline reference with retained
geometric feedback, actuator estimation/adaptation, and causal contact detection.
It is explicitly a hybrid, not a successful end-to-end neural policy.

Use the repository MuJoCo Python. No new deep-learning dependency is needed:
NumPy, SciPy and the existing simulator suffice. For repeatable host CPU timing,
set `OPENBLAS_NUM_THREADS=1` before starting Python.

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation generate --output artifacts/distill-data-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation train --input artifacts/distill-data-01 --history --output artifacts/distill-history-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation spline --input artifacts/distill-data-01 --output artifacts/distill-spline-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation evaluate --policy artifacts/distill-spline-01/policy.npz --hybrid --profile noise_filtered --output artifacts/distill-eval-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation mission --policy artifacts/distill-spline-01/policy.npz --profile noise_filtered --output artifacts/distill-mission-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/distill-mission-03 --speed 1 --loop
```

`train --profile noise` regenerates causal features under synthetic noise;
`--extra TRAIN_ORACLE_RUN` aggregates privileged teacher labels at student states.
`evaluate --split train --oracle --mix 0.7` collects teacher-intervened training
trials. Never describe their success as autonomous student success.
`confirm --input BC_CHECKPOINT --policy HYBRID_CHECKPOINT` freezes the test manifest
before rollouts. Do not train/select on the resulting holdout trajectories.

The standalone `policy.StudentAPI` takes nominal config/task/calibration,
initial `VehicleEstimate`, committed `TargetState`, a loaded `MLP` or
`SplinePolicy`, and operating-point index. Call `infer(estimate, now=clock)` to
receive `RateThrust` (FLU radians/s and physical N). Its motor observer predicts
from previous commands; it never queries true motor states. Pass source
measurement timestamps separately from the current control clock.

The API does not instantiate `ManeuverSkill`; the simulation wrapper may hold
one for evaluator references or explicit oracle labeling. Hybrid tracking and
recovery retain the existing model-based `Skill.track`. Shared Python imports
still target the current RM27 runtime. No embedded build, M3C timing, calibrated
MAVLink throttle, firmware or hardware deployment is established.

The student schema and exact splits are exported under
`artifacts/distill-student-bundle-02/`. Training is grouped by entire physical
scenario, with all modes of a seed in one split. Failure data and raw physics
remain in their source directories. `passed: true` on a campaign means runner
integrity; inspect per-flight successes and the additional 8 mm margin gate.
