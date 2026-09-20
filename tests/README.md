# Root tests

These tests cover the root integration package: scene and dock composition,
vehicle dynamics, routes, contact metrics, calibration, log intake and flight
interfaces.

Run from the repository root with:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

Tests validate software contracts and simulation evidence. They do not certify
real hardware, physical collision, or flight safety.
