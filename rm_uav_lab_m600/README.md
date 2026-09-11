# RM UAV Lab

A development workspace for quadrotor UAV simulation and flight-control
experimentation. The repo combines two independent simulation stacks — a
minimal, fully transparent MuJoCo quadrotor baseline and a full ArduPilot
software-in-the-loop (SITL) checkout — plus supporting directories for flight
logs, parameter sets, notes, and tooling.

This directory is part of the `rm27_drones` umbrella repository. Large
third-party and independently versioned components are recorded as Git
submodules; `ardupilot/` remains its own checkout.

## Repository Layout

```text
rm_uav_lab/
├── ardupilot/          # ArduPilot source checkout (git repo, master branch)
│   ├── ArduCopter/     # Copter vehicle firmware code
│   ├── libraries/      # Shared ArduPilot libraries (controls, HAL, MAVLink, ...)
│   ├── Tools/          # SITL autotest framework, build scripts
│   ├── build/sitl/     # Prebuilt SITL binaries (arducopter)
│   └── AGENTS.md       # ArduPilot-specific AI contribution guidelines
├── mujoco_uav/         # Standalone MuJoCo quadrotor simulator (Python)
│   ├── models/quadrotor.xml
│   ├── fly.py / simulator.py / pygame_controller.py / launch.py
│   ├── tests/          # pytest suite (protocol, flight sequence, safety, UDP)
│   └── README.md       # Detailed sub-project documentation (Traditional Chinese)
├── logs/
│   ├── raw/            # Raw flight/simulation logs (placeholder, empty)
│   └── analysis/       # Processed logs and analysis output (placeholder, empty)
├── params/             # Parameter files (e.g. ArduPilot .parm sets) (placeholder)
├── notes/              # Design notes and experiment records (placeholder)
├── tools/              # Helper scripts shared across the lab (placeholder)
└── README.md           # This file
```

## Component 1: `mujoco_uav` — MuJoCo Quadrotor Baseline

A minimal, verifiable quadrotor simulation with no dependency on ROS,
ArduPilot, PX4, or any flight stack. The physics simulation and the pilot UI
run as **two separate OS processes with two separate windows**, communicating
over localhost UDP. See [mujoco_uav/README.md](mujoco_uav/README.md) for the
full documentation (written in Traditional Chinese).

### Architecture

```text
pygame_controller.py          simulator.py
(Pygame window, keyboard)     (MuJoCo physics + viewer)
        │                            ▲
        │  CommandPacket  UDP        │  TelemetryPacket  UDP
        │  127.0.0.1:14560 (~50 Hz)  │  127.0.0.1:14561 (~25 Hz)
        └────────────────────────────┘
```

- **Physics**: MuJoCo, 500 Hz RK4 integration (`timestep=0.002`). The model is
  a 1 kg X-quadrotor (`models/quadrotor.xml`) with four `motor` actuators
  whose ctrl values are direct per-rotor thrust in Newtons (0–8 N), including
  reaction yaw torque via the `gear` parameter.
- **Control**: cascaded position + SO(3) attitude controller with a quadrotor
  mixer in `fly.py`. World and body frames are right-handed, +Z up; body +X
  through the red nose, +Y left; MuJoCo quaternions are `[w, x, y, z]`.
- **Flight state machine** (`simulator.py`):
  `DISARMED → ARMED_IDLE → TAKING_OFF → HOVERING ↔ MANUAL → LANDING → LANDED → DISARMED`,
  plus `EMERGENCY_STOP` reachable from any powered state. Smoothstep takeoff
  to 1.2 m, controlled landing with touchdown detection and a 0.5 s motor
  ramp-down, and automatic safe disarm after landing.
- **IPC** (`ipc_protocol.py`): JSON packets with protocol version, monotonic
  sequence numbers, and timestamps. Malformed JSON, NaN/Inf, unknown actions,
  out-of-range commands, non-loopback sources, and stale sequences are all
  rejected. High-level normalized commands only — the UI can never send raw
  motor thrusts.
- **Safety**: 0.5 s command-loss timeout recaptures the current pose and
  hovers; in-flight DISARM is refused; Pygame focus loss zeroes axes and
  sends a neutral command immediately, with a full key-release required
  before control is re-armed; EMERGENCY STOP is the only in-flight action
  that cuts motors instantly.

### Quick Start

```bash
cd ~/rm_uav_lab/mujoco_uav
python -m venv .venv          # Python 3.10+
source .venv/bin/activate
python -m pip install -r requirements.txt   # mujoco, numpy, pygame-ce, pytest
python launch.py              # starts simulator + controller, cleans up both on exit
```

Click the Pygame window first so it shows `CONTROL FOCUS ACTIVE`, then fly:

| Key | Action |
|---|---|
| W / S, A / D | move forward/back/left/right relative to current yaw |
| R / F | climb / descend |
| Q / E | yaw left / right |
| T | smooth takeoff (auto-ARM) |
| L | controlled land |
| H / Space | hover / brake (captures current pose) |
| M / N | ARM / DISARM |
| Backspace | RESET (ground only) |
| Shift + Esc | EMERGENCY STOP |

On macOS use `python launch.py --sim-python mjpython` (or set
`MUJOCO_SIM_PYTHON=mjpython`) so the MuJoCo passive viewer gets the main
thread. The two processes can also be started independently
(`python simulator.py` / `python pygame_controller.py`); either side
tolerates the other being absent.

### Headless Operation and Tests

```bash
cd ~/rm_uav_lab/mujoco_uav
source .venv/bin/activate
python simulator.py --headless --scenario takeoff_hover --duration 8
python simulator.py --headless --scenario takeoff_land --duration 14
python -m pytest -q
```

The pytest suite covers packet encode/decode and validation, action
de-duplication, full takeoff→hover and takeoff→land→disarm sequences,
heading-relative commands, in-flight DISARM refusal, emergency zero-thrust,
command-timeout hover fallback, Pygame focus/key-stuck protection, real UDP
loopback round-trips, and controller/mixer regressions.

### Known Limitations

Ground-truth state only (no sensor noise or estimator), no motor lag, drag,
ground effect, or blade aerodynamics; UDP is simple single-machine control,
not MAVLink; the lost-link policy is hover-in-place. It is a readable,
extensible baseline — not a representation of real-flight safety.

## Component 2: `ardupilot` — ArduPilot SITL

`ardupilot/` is a clean git checkout of the ArduPilot master branch
(4.7.0-beta series as of the last fetch), intended for software-in-the-loop
work with the production flight stack. A SITL build of ArduCopter is already
present at `ardupilot/build/sitl/bin/arducopter`.

Typical SITL usage:

```bash
cd ~/rm_uav_lab/ardupilot
./waf configure --board sitl     # only needed for a fresh build
./waf copter                     # builds build/sitl/bin/arducopter
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

Notes:

- `ardupilot/AGENTS.md` contains ArduPilot-specific AI contribution
  guidelines (coding style, build system, testing, commit and PR rules).
  Read it before editing anything inside `ardupilot/`.
- ArduPilot is safety-critical software: never fabricate test results, and
  disclose AI assistance in contributions submitted upstream.
- Keep lab-local experiments out of the upstream-facing checkout where
  practical; treat `ardupilot/` as the reference implementation.

## Supporting Directories

- `logs/raw/` — unmodified logs from simulations or SITL flights.
- `logs/analysis/` — plotted/processed results derived from raw logs.
- `params/` — versioned parameter sets (e.g. ArduPilot `.parm` files).
- `notes/` — experiment records, design decisions, derivations.
- `tools/` — reusable helper scripts that span the lab.

All five are currently empty placeholders defining the intended layout.

## Environment Summary

| Item | Requirement |
|---|---|
| OS | Linux primary (this workspace), macOS supported by `mujoco_uav` |
| Python | 3.10+ (a ready venv exists at `mujoco_uav/.venv`) |
| Key Python deps | `mujoco>=3.2`, `numpy>=1.24`, `pygame-ce>=2.5.7`, `pytest>=8` |
| Display | OpenGL-capable display for the MuJoCo viewer and Pygame windows |
| ArduPilot build | standard ArduPilot SITL toolchain (waf, gcc); already built once |

## Suggested Reading Order

1. `mujoco_uav/models/quadrotor.xml` — airframe, inertials, actuators.
2. `mujoco_uav/fly.py` — position/SO(3) controller and mixer.
3. `mujoco_uav/ipc_protocol.py` — the UDP command/telemetry contract.
4. `mujoco_uav/simulator.py` — flight state machine and sim loop.
5. `mujoco_uav/pygame_controller.py` — pilot UI and focus protection.
6. `mujoco_uav/tests/` — executable specification of the above.
7. `ardupilot/` — production flight stack for comparison and future SITL work.
