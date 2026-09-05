# Current engineering risks

| Risk | Evidence | Next discriminating check |
|---|---|---|
| Target loss during attitude changes/occlusion | Lead 96.2→81.5% at 60° FOV; 61.6% with 800 ms occlusion | Calibrated camera/body geometry, acquisition/reacquisition campaign with clutter |
| Range bias from target size/blur | Rendered 120°/11 px blur: −19.1% median bias, 12/80 abstentions | Real target fixture images versus range/exposure/angular speed |
| Timing and acquisition assumptions | Latency trend reverses with common initial hold | Measure exposure timestamps and capture-to-command age; test track-confidence readiness |
| Unverified propulsion and energy | No exact motor curves; allocations are unmeasured | Guarded thrust/current/voltage sweeps and battery thermal/sag measurements |
| Contact survivability | p95 contact speed about 5.75 m/s; overlap score only | Competition-approved benign fixture and instrumented low-energy contact tests |
| Ego estimation/dynamics abstraction | No gyro vibration, roll/pitch error, rotor-level attitude saturation in Astra | Import calibrated uncertainty; integrate observation adapter with M3 and firmware SITL |
| Rules and permitted infrastructure | No confirmed matching 2027 rule set established | Resolve actual dimensions, targets, contact constraints, radio/external vision permissions |
| Full mission and sortie management absent | Current scenarios start airborne with target in view | End-to-end launch/search/abort and sequential sorties after sensing assumptions are validated |

An architecture decision is provisional until its dominant risk has relevant
evidence. Neither a simulator pass nor a vendor rating closes a hardware risk.
