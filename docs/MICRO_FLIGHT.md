# Single micro-UAV dock flight — 2026-09-12

An optional live viewer is now available; see [live-view usage and validation](LIVE_VIEW.md).
The measurements below retain the original headless experiment provenance.

The current bounded experiment passed twice with unchanged runner and parameters:
`artifacts/micro-flight-alignment-1` and `artifacts/micro-flight-alignment-2`.
Real ArduPilot SITL controlled drone 1
through normal arming, takeoff, ten seconds of measured hover, a 0.12 m offset,
return, alignment above its assigned slot, physical touchdown, LAND mode and
automatic disarm. The other three vehicles remained physical bodies in the
same MuJoCo world with zero motor inputs.

This result covers the provisional vehicle and the tested dock sequence.
Base contact, four-autopilot scheduling and perception remain future work.
Simulation truth supplies precision guidance; this is not a hardware or
camera/GPS-only docking result.

## Model and baseline contract

Development field: `67c1327`; scene `artifacts/dock-final/scene.xml`, SHA-256
`768545c221a1b9337ba1dbc5ed49dc5fa56dec15eed875c3d73812efbbf646cb`.
Four 180 g, 120 × 120 mm provisional vehicles and the audited dock share one
500 Hz RK4 simulation. Drone 1's slot floor is at world
(0.651751442, 1.082427939, 1.74) m. Heights below are body-center heights above
that floor; the resting center is approximately 20 mm above it.

Each rotor uses a 50.8 mm propeller visual, assumed maximum thrust 1.103625 N
and a 40 ms motor filter. Estimated inertia and physical geometry are retained
through the flight trials. The user-reported 1103 9600KV motors establish the
intended hardware configuration, but battery voltage, thrust curves and measured
inertia remain unknown. These are simulation parameters, not hardware tuning.

`MicroPlant` selects the vehicle's joint and actuators by name. AP channels map
to motor indices `[2,0,3,1]`; only those motors receive commands. Every received
500 Hz servo frame advances all four physical vehicles once. ArduPilot runs at
250 Hz with EKF3, Quad-X and `ARMING_SKIPCHK=0`. Normal ARM and NAV_TAKEOFF
commands are used; LAND performs automatic disarming.

The existing bridge protocol and frame conversions are reused without editing
the frozen `mujoco_uav_ardupilot` repository. Its verified 1 kg G1' vehicle,
controller settings and recorded baseline remain unchanged. This micro flight
is a separate root-level experiment against the updated field, not a relabeling
of the original G1' result.

`configs/micro_hover.parm` appends micro-only settings to the original defaults.
Hover throttle is `(0.4−0.15)/(0.95−0.15)=0.3125`. Current roll and pitch rate
P/I/D gains are three times the initial inertia-scaled estimates, retaining
their I/P and D/P ratios. Yaw retains its initial estimate. This refinement
addresses small-command behavior seen with integer PWM outputs; its effect is
assessed through the actual flight gates, not assumed from the gain calculation.
Each run records the executable hash, source commit, runner hash, parameter-file
contents and actual parameter readback.

## Precision guidance

Takeoff uses a normal 0.6 m NAV_TAKEOFF command. Subsequent GUIDED commands are
sent at 20 Hz. The local ArduPilot parameter is named `GUID_OPTIONS`; value 16
disables its horizontal position stabilization while retaining velocity and
attitude control. The root runner supplies the horizontal position loop.

For horizontal true position error `e_xy`, all in metres in the local NWU frame:

```
i_xy = clip(i_xy + 0.1 * e_xy * dt, -0.015, +0.015)
desired_true_velocity_xy = clip(0.8 * e_xy + i_xy, -0.06, +0.06)
command_velocity_NED = estimated_velocity_NED - true_velocity_NED
                       + world_to_NED([desired_true_velocity_xy, 0])
```

Clamps apply separately to each horizontal component. The integral is reset
on entry to RETURN and updated only during RETURN, DESCEND, ALIGN and
FINAL_DESCEND. Its bounds are ±0.015 m/s per axis. The desired horizontal
velocity bound is ±0.06 m/s per axis, not a bound on the diagonal vector norm.

Altitude retains ArduPilot position control. The command position is the
estimated position plus the true position error transformed into NED; estimated
position is extrapolated from the latest LOCAL_POSITION_NED sample to the
current physics time. The same velocity-discrepancy compensation applies
vertically. Horizontal position fields are ignored by the selected Guided
option. Commanded yaw preserves the slot's initial orientation.

These operations change commands only. The bridge sends the physical state
and IMU values; no pose, sensor telemetry, motor force or contact is overwritten
to make the vehicle pass. ArduPilot generates the motor commands throughout.
LOCAL_POSITION_NED older than 0.5 s, heartbeat older than 2 s during flight,
or estimated/true velocity disagreement above 0.25 m/s fails the experiment.

SITL retains sensor imperfections, including default gyro drift, calibration
offsets and minimum sensor noise. Setting `SIM_*_RND` to zero does not remove
all of those effects. In the earlier position-only trial, estimated east
velocity retained approximately 0.024 m/s bias while delayed GPS velocity
closely matched physics truth. The full sensor-level cause has not been
isolated. The measured millimetre landing error therefore demonstrates this
simulation-truth guidance loop, not a deployed localization system.

## Sequence and physical gates

After ten seconds of bounded hover at 0.6 m, the position reference moves
0.12 m along local world +X at 0.04 m/s and returns at the same reference speed.
Return requires XY error below 4 mm, height error below 15 mm and total speed
below 0.01 m/s before descending.

Descent first stops at 0.12 m above the slot floor. ALIGN holds there until
XY error is below 3 mm, horizontal speed below 0.006 m/s and height error below
5 mm continuously for 0.75 s. FINAL_DESCEND then lowers the reference at
0.03 m/s. Below 0.1 m, XY error exceeding 6 mm aborts. This additional hold
prevents an instantaneous return check from being treated as evidence that
alignment will remain adequate near the rim.

Only contact with `rm27_dock_collision_bottom` is accepted. Touchdown speed
must remain below 0.15 m/s, contact force below 10 N per contact and penetration
below 5 mm. After touching, normal LAND mode must be observed while armed,
followed by automatic disarm. The body must remain within 7 mm XY of its slot
center and within 5 mm of the nominal 20 mm resting height. Completion also
requires one second of disarmed rest and each selected rotor's force below
1e−5 N.

The runner rejects nonfinite state, flight-envelope violations and any
missing/duplicate/reset/invalid bridge frames. Inactive vehicles must retain
zero motor input and force, XY displacement below 1 mm, initial-height change
below 2 mm and tilt below 1°. After the initial settling second, each inactive
velocity component must remain below 0.001 in its corresponding units.
These are measured gates for this sequence, not a proof of field-wide collision
coverage or repeatability under arbitrary disturbances.

## Measured current run

The table and measurements describe
[`micro-flight-alignment-1`](../artifacts/micro-flight-alignment-1/summary.json).

| Event | Physics time (s) |
|---|---:|
| Armed heartbeat / takeoff request | 43.048 |
| Takeoff command accepted | 43.056 |
| HOVER entered / OFFSET entered | 50.858 / 60.860 |
| 0.12 m offset reached | 66.862 |
| Returned over slot / initial descent | 76.750 |
| ALIGN entered at 0.12 m | 89.402 |
| Final descent after continuous alignment | 90.156 |
| Physical touchdown / LAND requested | 93.938 |
| Disarmed heartbeat / POST entered | 98.050 |
| Post-disarm observation complete | 99.052 |

Measured hover duration: 10.000 s; height 0.598891–0.604242 m; maximum speed
0.040047 m/s; maximum tilt 0.799585°. Maximum hover XY displacement was
47.880 mm, so hover is not reported as millimetre position hold. Peak takeoff
speed was 0.769952 m/s.

Touchdown speed was 0.029828 m/s and tilt 0.052729°. Three guard geoms first
contacted the intended dock bottom. Maximum penetration from touchdown through
post-disarm was 0.180109 mm; maximum force at a single contact was 0.147150 N.
The larger whole-run peaks, 0.768132 mm and 0.387987 N, occurred during initial
unpowered settling before arming.

Final body center relative to the slot floor:
(0.000910423, −0.001435893, 0.019984835) m; XY error **1.700194 mm**.
Final mode was LAND, armed false, with approximately 2.13e−14 N remaining at
each selected rotor. Other vehicles' maximum observed XY displacement was 0 m.
Per-frame logs include all four vehicles' positions, velocities and motor forces.

49,526 frames were exchanged with zero missing, duplicate, reset or invalid
frames. Physics advanced 99.052 s in 48.257 wall seconds, approximately 2.05×
including startup, logging and cleanup. The focused root suite has 13 passing
tests covering dock geometry, motor mapping, selected-vehicle frames and
inactive-vehicle isolation.

## Second flight and exact recorded-input replay

The [second flight](../artifacts/micro-flight-alignment-2/summary.json) also passed
all gates with the same scene, captured parameter files and runner hash.

| Measurement | Alignment 1 | Alignment 2 |
|---|---:|---:|
| Physics frames | 49,526 | 52,026 |
| Physics duration (s) | 99.052 | 104.052 |
| Wall duration including startup/cleanup (s) | 48.257 | 49.651 |
| Touchdown time (s) | 93.938 | 98.590 |
| Touchdown speed (m/s) | 0.029828 | 0.029939 |
| Final horizontal error (mm) | 1.700194 | 1.444141 |
| Final mode / armed | LAND / false | LAND / false |
| Inactive vehicles' maximum XY displacement (m) | 0 | 0 |

Both have zero missing, duplicate, reset or invalid frames. Both recorded-input
replays match every transmitted state, selected vehicle truth, all 28 qpos,
all 24 qvel and all 16 motor forces exactly, including the final full-scene
summary. Maximum absolute replay error is zero. Evidence:
[first replay](../artifacts/micro-flight-alignment-1/replay.json),
[second replay](../artifacts/micro-flight-alignment-2/replay.json).

Full-scene SHA-256 values (sorted-key compact JSON, newline per frame):

```
alignment-1: 0e97a22191d7ca5d356e4c442e18b9d0d3f467d7c7cb62a82300d6c50cc71b22
alignment-2: 30b958e1217be8d402301dc102b22f0fc95e86b7073f2a01478b2d6003817806
```

The two closed-loop flights are **not bit-identical**. Their first different
PWM and physical state occur at frame 28,802, physics time 57.606 s during
HOVER. Across 49,526 common frame times, maximum position separation is
0.162158 m at 81.668 s. This is a time-aligned comparison: different descent
start times contribute to the separation; it is not final landing error.
The second touchdown occurs 4.652 s later. The cause of command divergence
has not been isolated. Exact replay establishes deterministic physics for the
recorded inputs, not deterministic asynchronous SITL command delivery or
reliability across arbitrary repeated flights. See
[closed-loop comparison](../artifacts/micro-flight-alignment-2/closed-loop-comparison.json).

![Recorded flight measurements](../artifacts/micro-flight-alignment-1/flight.png)

![Six physical replay snapshots](../artifacts/micro-flight-alignment-1/replay-views.png)

## Retained development results

- `micro-flight-1`: SITL exited during startup; obsolete LAND parameter names
  were found during this iteration. The current local names are `LAND_SPD_MS`
  and `LAND_SPD_HIGH_MS`.
- `micro-flight-2` and `-3`: idle auto-disarm interrupted the normal arming/spool
  sequence. The current idle delay is ten seconds.
- `micro-flight-4`: takeoff, hover and offset passed; return failed its 4 mm
  gate with roughly 22 mm true Y error. Position-only correction was insufficient.
- `micro-flight-5`: the earlier position/velocity correction completed once,
  with 0.750222 mm final XY error. Its repeat, `micro-flight-final`, failed the
  6 mm near-rim gate. The name “final” is retained historical output, not a pass.
- `micro-flight-precision-1`: stronger outer position gain increased oscillation
  and failed return. `micro-flight-velocity-1` stopped on the obsolete
  `GUIDED_OPTIONS` parameter name; `-velocity-2` used the correct `GUID_OPTIONS`
  but still failed return with the initial rate gains.
- `micro-flight-rates-1`: increased roll/pitch gains improved small-command
  behavior but the sequence still failed near-rim alignment. The current
  integral correction and continuous ALIGN hold were subsequently added.
- `micro-flight-alignment-1` and `-2`: the current complete sequence passed
  twice as measured above. Raw logs and parameter snapshots remain in every
  original directory.

None of these changes modifies dock geometry, contact settings, UAV mass or
inertia, motor lag, the integer PWM protocol, or the physical acceptance gates
to turn a failed flight into a pass.

## Reproduction

From the project root, use a new output directory and an unused SITL instance:

```bash
cd /home/shiuhou/Projects/rm27_drones
/home/shiuhou/venvs/mujoco/bin/python -m rm27 micro-flight artifacts/dock-final/scene.xml --output artifacts/my-micro-flight --instance 31
/home/shiuhou/venvs/mujoco/bin/python -m rm27.plot_micro_flight artifacts/my-micro-flight
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.replay_micro_flight artifacts/my-micro-flight --render
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

Generate a dock scene first with `python -m rm27 build-dock --output ...` if
needed, then substitute its `scene.xml` path. Flight uses an isolated local SITL
subprocess. Plots and replay snapshots visualize recorded results. Full logs
are local ignored artifacts; parameter and executable metadata are retained
alongside each run for interpretation and reproduction.
