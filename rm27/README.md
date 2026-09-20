# RM27 integration package

This is the only current runtime package owned by the root project. Its target
internal structure is documented in [`docs/MIGRATION_PLAN.md`](../docs/MIGRATION_PLAN.md).
It builds
the composed MuJoCo scene and provides the single-vehicle flight, route,
contact, calibration, replay and log-analysis entry points.

The package consumes reviewed files in `configs/`, external field and vehicle
components under `rm27/simulation/` and `rm27/autopilot/`, and writes generated
evidence to `artifacts/`. Raw CAD inputs remain under `references/cad_raw/`.
It does not claim that a simulation result proves real-aircraft performance.

Use `python -m rm27 --help` for the supported commands. Keep new integration
behavior here and add a focused test under `tests/`.
