# Live micro-flight viewer

The root `micro-flight` command and direct `rm27.autopilot.micro_flight` module accept
`--view` and `--speedup`. Viewing requests 1× speed by default; headless mode
retains the 5× default. The physics timestep remains 0.002 s and the controller
remains 250 Hz in both modes. Speed changes affect wall-clock pacing only.

```bash
cd /home/shiuhou/Projects/rm27_drones
source /home/shiuhou/venvs/mujoco/bin/activate
RM27_RUN="artifacts/micro-flight-$(date +%Y%m%d-%H%M%S)"
python -m rm27 micro-flight artifacts/dock-final/scene.xml --output "$RM27_RUN" --instance 31 --view
```

The camera starts near the dock. Mouse rotation, pan and zoom remain available.
An overlay displays the flight phase, physics time, body-center height above
the assigned slot floor, horizontal position error, mode and armed status.
Initialization normally takes about 43 physics seconds before arming.
Closing the window or pressing Ctrl+C in the terminal cancels the experiment;
normal completion closes the window automatically. The summary records the
result. This is a simulation observer, not a hardware control interface.

## State ownership and cleanup

The ArduPilot plant remains the only stepped simulation. The viewer receives
a complete `mj_copyData` snapshot of its state, using an independent model and
data object for rendering. That display object is never stepped. Viewer UI
perturbations, resets or option changes cannot write back to the flight plant.
All four UAVs, their motor states and physical contacts come from the live run.

Display synchronization is limited to at most 30 updates per wall second, with
the interval measured from the end of the previous synchronization. This
avoids a slow render turning every physics step into another render. Actual
frame rate and end-to-end simulation speed depend on the graphics hardware.

The runner owns and cleans up its SITL subprocess. It also waits for the GUI
threads created by `launch_passive` to finish after requesting viewer closure,
before allowing Python to exit. The wall deadline scales for slower requested
speeds; physical flight acceptance gates and control parameters are unchanged.

## Development evidence

Final live run and headless regression both passed the full takeoff, ten-second
hover, offset, return, touchdown, LAND and automatic-disarm gates. Both processes
exited normally with code 0. The live window was visually inspected during
flight, with its overlay and four physical vehicles visible.

| Measurement | Live (`micro-flight-live-final`) | Headless (`micro-flight-view-regression`) |
|---|---:|---:|
| Requested speed | 1× | 5× |
| Physics duration | 104.050 s | 104.052 s |
| Wall duration including cleanup | 110.125 s | 51.323 s |
| Physics frames | 52,025 | 52,026 |
| Final XY landing error | 3.994 mm | 3.688 mm |
| Inactive vehicles' maximum XY displacement | 0 m | 0 m |
| Missing / duplicate / reset / invalid frames | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |

The live run recorded 2,497 display updates before its final status update.
Actual end-to-end speed was approximately 0.945×. Both runs' recorded motor
inputs replay every full-scene state exactly, with zero error. These are
separate closed-loop flights; identical trajectories between modes are not
claimed. Evidence:

- [Live summary](../artifacts/micro-flight-live-final/summary.json) and
  [exact replay](../artifacts/micro-flight-live-final/replay.json).
- [Headless summary](../artifacts/micro-flight-view-regression/summary.json) and
  [exact replay](../artifacts/micro-flight-view-regression/replay.json).
- [Window-close test](../artifacts/micro-flight-live-cancel/close-check.json):
  sent a normal `WM_DELETE_WINDOW` event to the test's own viewer before arming;
  the runner recorded cancellation and exited with code 1, without a native
  crash. The owned SITL was cleaned up.

![Actual live window during alignment](../artifacts/micro-flight-live-final/live-view.png)

The first attempt, `micro-flight-live-1`, rendered too often and was interrupted
before arming. `micro-flight-live-2` completed the physical flight but Python
exited with a native segmentation fault during GUI teardown. Its flight summary
alone therefore does not establish a successful application run. Both outputs
are retained. Rate limiting and explicit GUI-thread cleanup address these
separate problems.

The root test suite now has 14 passing tests. The added observer test changes
display-side gravity, controls and position and verifies that the authoritative
plant is unaffected. It checks snapshot equality for all vehicles and motor
forces, and checks throttling after a simulated slow display update.

Original G1' and upstream field tracked files are unchanged. The micro vehicle
still uses the provisional 180 g model and simulation-truth precision guidance
documented in [MICRO_FLIGHT.md](MICRO_FLIGHT.md). Visual observation does not
validate camera-based localization or real hardware docking.
