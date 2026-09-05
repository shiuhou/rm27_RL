# Astra terminal-feasibility laboratory

The initial question is whether perception timing and visibility constrain
competition-fixture contact more strongly than additional thrust. This package
compares three observation-only guidance baselines in a vectorized simulator.

```bash
# From the outer rm27_drones repository root; numpy and matplotlib required.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest astra/tests -q
python3 -m astra.campaign --output astra/results/my_development
python3 -m astra.campaign --split evaluation --full --output astra/results/my_evaluation
```

Each output directory must be new. Runs include source snapshots, complete
resolved configuration, unique trajectory identifiers, raw JSONL episodes,
Wilson intervals, paired bootstrap comparisons, and Markdown tables. The frozen
configuration is `configs/campaign.json`. Never tune on evaluation seed 27991.

## Meaning of the evidence

- Three-dimensional thrust-vector surrogate with attitude/motor lag, tilt/thrust
  limits, yaw response and linear drag. **Not** a six-DOF rotor-level vehicle.
- Geometric camera features: noisy calibrated bearing and spherical angular
  extent, bounded FOV, contour clipping, latency, dropped frames and occlusion.
  **Not** evidence of successful detection in real imagery.
- Ego position/velocity errors are a declared correlated error abstraction with
  drift, scale and yaw error. **Not** a working GPS-denied estimator.
- Known spherical fixture radius is a prior; size-error sweeps challenge it.
  Pixel bearing and angular extent must support any inferred target position.
- Airborne initial condition, target initially visible, 12-second horizon,
  no obstacles except evaluation floor/volume limits. No search, takeoff,
  full mission or multi-UAV success claim.
- Target paths are unbounded horizontally within the generous evaluator volume;
  this is a terminal-guidance test, not a confirmed competition field.
- Contact means swept geometric overlap (0.10 m fixture + 0.07 m vehicle radii).
  Impact survival and safe contact speed are not established. Failed trials
  remain in every denominator. Energy is unmeasured; effort is not watt-hours.

## Architecture and interfaces

`scenario` and `simulation` own physical truth. `ImageObservation` includes pixel
offsets, angular radius, validity, capture/delivery times and estimated camera
pose at exposure. `EgoEstimate` contains estimated local position/velocity and
camera attitude. `Guidance` receives only these types. It never receives the
scenario, target ID, future path, true coordinates, or physical performance draw.

Local coordinates use Z up, body X forward/Z up, and camera right/down/forward.
Camera extrinsics follow achieved vehicle attitude. A fixed forward camera tilts
down during forward acceleration. Range requires a fully visible sphere contour.

The same alpha-beta tracker feeds all methods:

1. `pursuit`: steer toward exposure-time target estimate; delayed baseline.
2. `compensated_pursuit`: extrapolate to now, then pursue current target estimate.
3. `lead`: extrapolate to now and select a constant-velocity interception point.

Lead versus compensated pursuit is the primary guidance comparison. Delayed
pursuit separates the value of timestamp compensation. Lead's constant-speed
intercept is an approximation, not proof of reachability. Actuator limits are
applied by the plant, without revealing actual performance to guidance. All
methods brake locally when observation age exceeds 0.5 s.

Nominal control requests 6 m/s with a fixed gain; parameters are shared across
trajectories, with no evaluation tuning. Guidance is 100 Hz, physics 200 Hz,
camera nominally 30 Hz. Sensor/control events occur on the next physics tick;
the additional timing quantization is <5 ms. Numerical convergence is checked
separately at exactly representable camera times.

## Reference assets

The corrected upstream [rm_uav_lab](https://github.com/shiuhou/rm_uav_lab) was
cloned to `/tmp/rm27_reference_lab`, pinned at
`b9e61bac550e3719a8d6a216bf1ff8a94a7dd03b`. Its 134 tests and 11-case M3
benchmark were reproduced locally with MuJoCo 3.12.0. No upstream code was
copied into Astra. Its observation interfaces and truth-routing tests informed
the boundary design. The clone is optional and can be recreated from the URL.

```bash
git clone https://github.com/shiuhou/rm_uav_lab /tmp/rm27_reference_lab
git -C /tmp/rm27_reference_lab checkout b9e61bac550e3719a8d6a216bf1ff8a94a7dd03b
```
