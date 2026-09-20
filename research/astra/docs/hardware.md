# Hardware evidence and first bench measurements

**An assembled prototype is available; its performance is not measured here yet.**
The user reports 1103 **9600KV** motors, MicoAir743v2-AIO-35A, MTF-02P and available
1.8-inch/2.0-inch propellers. This supersedes the earlier 9500KV starting assumption.
Battery, flying mass, installed firmware, proven flight modes and camera configuration
remain unreported. Use the [first real experiment protocol](real_experiments.md)
and [logging contract](real_logging.md) for the next evidence collection.

The campaign varies dimensionless thrust authority; it does not establish what
this prototype can produce. A 200 g vehicle needs 50 gf per motor just to hover,
before guard losses.
Nominal 2:1 thrust/weight requires 100 gf per motor; 3:1 requires 150 gf.

## Primary-source facts, accessed 2026-09-05

| Item | Evidence | Consequence |
|---|---|---|
| MicoAir743v2-AIO-35A | Manufacturer: 10 g, 2–6S, 5 V/2 A BEC, 35 A/channel with cooling; no onboard compass | Usable candidate, but rail and cooling margins must be measured; distinguish v2-AIO from other MicoAir743 boards |
| MTF-02P | Manufacturer: 1.5 g, 0.2 W at 5 V, 50 Hz, 42° flow FOV, specified 7 m/s at 1 m height, >60 lux flow requirement | Flow performance depends on altitude, texture and lighting; not an absolute-position sensor |
| MaixCAM | Sipeed documents capture buffering and roughly 30+ ms minimum capture latency in its tests | A claimed 20 ms total visual-control pipeline is not supported by those measurements; measure capture-to-command age on the actual variant |

Sources: [v2-AIO manufacturer](https://micoair.com/flightcontroller_micoair743v2_aio_35a/),
[MTF-02P manufacturer](https://micoair.com/optical_range_sensor_mtf-02p/),
[MaixPy camera timing](https://wiki.sipeed.com/maixpy/doc/en/vision/camera.html).
These are published specifications, not our bench verification.

## Provisional mass allocation, not a shopping list

| Assembly | Allocation, g | Provenance |
|---|---:|---|
| Frame | 15 | Design allowance |
| Full/substantial guards | 20 | Design allowance |
| Four motors | 16 | Design allowance; weigh exact 1103 variant |
| Four propellers | 4 | Design allowance |
| FC/ESC | 10 | Manufacturer |
| Battery | 40 | Allocation; exact pack undecided |
| Flow/ToF | 1.5 | Manufacturer, harness counted below |
| Camera + compute | 20 | Allocation; screenless assembly unweighed |
| Radio | 3 | Design allowance |
| Wiring/connectors/regulation | 8 | Design allowance |
| Mounts/isolation | 6 | Design allowance |
| Contingency | 20 | Reserved, not installed hardware |
| **Total including contingency** | **163.5** | **Allocation only** |

This leaves 36.5 g within a 200 g planning envelope. It proves only that an
allocation can be written, not that the chosen parts meet it. Replace every
allowance with an exact part/revision and scale measurement before selection.

Power model must include motor electrical power versus thrust **and voltage**,
compute/camera power, FC/sensors/radio, converter efficiency, transient current,
pack internal resistance versus SOC/temperature, and thermal limits. The 5 V BEC
rating is 10 W shared capacity, not 10 W free for compute. Do not rank packs by
advertised C-rating alone. Existing propsim data provenance/interpolation ideas
are reusable; its nominal-voltage sag calculation cannot establish how available
thrust changes during a mission. Do not copy that simplification into rankings.

## Measurement order

The [four-experiment protocol](real_experiments.md) replaces the earlier generic
measurement list: props-off field flow/ToF and camera acquisition/timing; guarded
1.8/2.0-inch comparison at the rated battery cell count; then paired low-energy
field flights with independent reference. It specifies durations, repetitions,
raw signals, metadata, abort conditions and held-out validation. The
[run template](../configs/real_trial_template.json) leaves unmeasured values null.

Whole-vehicle inertia, maximum thrust, high-speed drag, long-duration sensor drift,
impact survival and unguarded propulsion comparisons are deferred until evidence
shows they limit the next decision. Guarded hover power and field visibility have
higher immediate information value than another assumed high-fidelity airframe.

Hardware experiments require the physical rig and human operator. No actuation
or configuration of a real flight controller is provided by this package.

## Transfer architecture under consideration

ArduPilot owns stabilization, estimator health and local failsafes. A lightweight
companion owns timestamped target perception/tracking and terminal guidance.
Hangar compute may own scheduling and coarse target cues, subject to actual RM27
rules; terminal commands should not depend on a healthy radio link. Four sequential
sorties can use a small scheduler with one active vehicle lease, command expiry
and explicit vehicle states, without a swarm framework.

Astra acceleration/yaw requests need an explicit ArduPilot adapter and SITL
validation before this becomes a hardware interface. They are not MAVLink
messages and cannot be sent directly. State frame conversion, command masks,
timestamps, watchdog timeouts, health gating and achievable acceleration must be
verified against the selected firmware build. The current evidence does not
justify final airframe/prop/battery/compute selection or competition readiness.
