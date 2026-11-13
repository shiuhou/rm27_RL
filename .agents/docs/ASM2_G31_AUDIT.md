# ASM2 Phase G3.1 — Contact geometry, dynamics and research reuse

Completed 2026-11-13, Asia/Shanghai. **Decision A: contact geometry needs further validation.** No training or controller development was performed. The frozen native plant, contact contract, geometry and checkpoint bytes were preserved. Phase 4 `checkpoint-186368.pt` remains the verified learned baseline and Fixed A the continuation reference. Goal Hybrid remains closed and separate.

The requested `/home/dsa/project/ceo_pj/RM27_Agile_Flight_RL_Project/` does not exist on this host. Work used the existing `/home/dsa-cli/ceo_pj/RM27_Agile_Flight_RL_Project/`. The root is `main`, with no commits or remote and pre-existing untracked project contents. No commits, pushes, hardware, PX4/RK3576 changes, new RL algorithm or physics edits occurred.

## 1. Evidence and scope

**CONFIRMED:** two pilot native rollouts reproduce the complete archived G2 row-51 nominal/stale states with maximum absolute state error **0.0**, including first-contact times and identities. A prospective manifest then froze **78** local sensitivity rollouts. Total **80/300**, with zero optimizer steps. One initial source-mesh path error happened before any integration; its manifest and error receipt are retained. There were no failed/hidden rollout retries. Pilot/model preparation and execution took 28.509 s; the main run took 126.639 s, excluding research, rendering and report preparation.

The pilot applies the archived CTBR commands through the unchanged live rate allocator and motor dynamics. It does **not** rerun the Goal Hybrid controller. Main runs start at each reproduced history's 200 ms state, with exact realized rotor thrust and prior rotor targets, and execute the remaining recorded commands. They test local conditional contact sensitivity, not closed-loop policy robustness, new reachable states, or a success-rate distribution. No held-out data was used for selection.

Runtime: MuJoCo 3.12.0, `mjSOL_NEWTON`, `mjINT_IMPLICITFAST`, 100 solver iterations, tolerance 1e-8, gravity 9.81 m/s², **0.1 ms physics / 5 ms nominal CTBR**, all **11,874 aircraft colliders**, all **1,822 frozen Field geoms** and **25 registered contact targets**. G2 model metadata matched exactly. Main timing cases only move the final 205 ms command switch; physics remains unchanged. The scorer retains every signed forbidden contact, including zero-load body points.

## 2. Original literature and implementation audit

Downloaded and read the original PDFs, not just abstracts. Inspected commit-pinned source snapshots, license files, build dependencies, relevant algorithms and nested repository availability. Third-party code was kept as research evidence and was **not installed, built, imported into ASM2 or executed**. Reported paper hardware performance is **REPORTED**, not independently reproduced here. Exact downloaded bytes and source commits are in the research manifest; public source permalinks below make the findings independent of local run files.

### Fast-Perching

Original: Ji et al., [Real-Time Trajectory Planning for Aerial Perching, IROS 2022](https://arxiv.org/abs/2203.01061), §§III–V, especially equations 13, 19–22. [Source commit](https://github.com/ZJU-FAST-Lab/Fast-Perching/tree/d4c0a7e66d9fa9939171b01b96fb4a4fb7cb577b).

- **Method:** flat-output polynomial trajectories with MINCO_S4; optimize intermediate points, duration, terminal tangential velocity and terminal thrust. Terminal normal speed is specified; tangential velocity is optimized with a quadratic penalty. Terminal acceleration aligns thrust with the surface normal, with a sine transform keeping terminal thrust within bounds. Terminal jerk is zero. Body-rate, thrust, velocity and collision constraints become sampled/integrated penalties, solved with L-BFGS. This is numerical constrained planning, not a certified full-state reachable set.
- **Verified code:** [`traj_opt_perching.cc`](https://github.com/ZJU-FAST-Lab/Fast-Perching/blob/d4c0a7e66d9fa9939171b01b96fb4a4fb7cb577b/src/traj_opt/src/traj_opt_perching.cc), `forward_thrust`, terminal `tailS`, `generate_traj`, `grad_cost_perching_collision`; `minco.hpp` contains `MINCO_S4`. The perching collision model uses a rotated circular bottom support against a plane with proximity activation; the code includes a **5 mm** geometry adjustment in the penalty. It does not distinguish ASM2 front/rear/body hull identities or resolve impulses.
- **Replanning:** warm-starts the remaining polynomial. The public [`planning_nodelet.cpp`](https://github.com/ZJU-FAST-Lab/Fast-Perching/blob/d4c0a7e66d9fa9939171b01b96fb4a4fb7cb577b/src/planning/src/planning_nodelet.cpp) demonstration restarts from the previous trajectory's position/velocity/acceleration/jerk; it is not a complete measured-state, motor-aware ASM2 feedback implementation.
- **Evidence:** paper reports a 0.36 kg magnet/adhesive perching vehicle, thrust-to-weight 1.7, static inclined/vertical and moving-platform experiments (0.6 m/s target, 10 Hz replanning). Reported solve time under 20 ms and warm-start around 2 ms are platform-specific. Its 0.3 m/s prescribed normal touchdown speed and attachment mechanism are not ASM2 qualification rules.
- **Reuse:** terminal-state parameterization, polynomial continuity and warm-start optimization are strong candidates. Add ASM2 motor state, allocation bounds and detailed contact checks rather than copying its disk-plane geometry or constants. C++14, Eigen, ROS1/catkin and visualization coupling need isolation. Root/planner **GPLv3**; some bundled solver headers (including L-BFGS/MINCO) carry MIT notices and visualization packages have separate notices. Inspect licenses at file scope before extracting code; the repository is not uniformly MIT.

### Moving inclined surface perching

Original: Liu et al., [Perching on Moving Inclined Surfaces using Uncertainty Tolerant Planner and Thrust Regulation](https://arxiv.org/abs/2212.10829), **v1**, §§III–V, Algorithms 1–2, Tables I–III.

- **Method:** candidate states use position, velocity and acceleration. A waypoint's forward reachable set is scored by coverage of a Gaussian terminal-state distribution, averaged over Gaussian tracking uncertainty. Local optimal dexterous waypoints are selected from feasible trajectories, avoiding unreachable candidates. The translation rule using `exp(-A T)` is derived with restricted constraints; the paper explicitly discusses geometric limitations. This is not a full nonlinear rotor-state reachability certificate for arbitrary obstacles.
- **Continuous transition:** update predicted intercept and waypoints, choose the forward waypoint, replan piecewise polynomial segments; a complementary correction handles infeasible replans by trading initial versus terminal error. Position/velocity boundary continuity is relevant to G2's otherwise abrupt attitude/acceleration switches.
- **Thrust regulation:** during terminal attitude tracking, solve for bounded thrust compensation to reduce terminal position/velocity error. Pontryagin's principle produces switching controls; a low-dimensional nonlinear optimization searches switching times, with attitude evolution approximations. It does not independently command incompatible translational and attitude accelerations, and does not identify motor lag.
- **Evidence:** Gazebo comparisons use **10 trials per session**, with planner OW **5/10** versus two-end TE **2/10**; adding thrust regulation gives **7/10**. Thus the often quoted “50%” corresponds to 2/10 versus 7/10, a **50 percentage-point** difference in that small comparison, not an ASM2 effect size. Outdoor rear-car-window demonstrations use approximately 70° inclination, a master hexacopter for perception and a slave quadrotor for execution. Ground-computer planning/state estimation operates at 30 Hz in the described setup. Their gripper allows much larger tangential/contact velocities than ASM2.
- **Availability/license:** no implementation link or software license was found in the original paper; an exact-title GitHub repository search returned zero matches. A guessed author-account query failed and supplies no evidence. **Public implementation not located**, not proven nonexistent. Reuse the published method through a fresh, small implementation if later authorized; do not infer that Fast-Perching or TrajPlanARPL implements this paper.

### Colliding Drone

Original: Bredenbeck et al., [A Tactile Feedback Approach to Path Recovery After High-Speed Impacts for Collision-Resilient Drones](https://arxiv.org/abs/2410.14249), downloaded **v3, 2025-10-31**, §§3–6. [Repository at pinned commit](https://github.com/BioMorphic-Intelligence-Lab/colliding-drone/tree/33dfe2aed4c6ca6c75cee6b59aa960faa74d0d03). The pinned umbrella commit is from 2024: do not assume it fully implements the later v3 manuscript.

- **Method:** an icosahedral tensegrity cage with **12 point contacts**, binary tactile sensing, impulse-based velocity/rate correction in a switching Kalman prediction, and a vector-field path representation modified by collision-location repulsion. Restitution and friction are estimated from unactuated impact data. Contact normals are approximated from cage beams; sliding friction and averaging contributions over active vertices are model assumptions. This is collision recovery, not gentle patch-qualified perching or arbitrary full-mesh simultaneous-contact mechanics.
- **Evidence:** paper reports Monte Carlo recovery up to 8 m/s and real recovery at **2.3–3.7 m/s**, including five illustrated high-speed collisions, with motion capture and a compliant cage. Those numbers do not validate ASM2 geometry, damage tolerance or contact impulses.
- **Actual source:** [`colliding-drone-py-sim`](https://github.com/BioMorphic-Intelligence-Lab/colliding-drone-py-sim/tree/f250c0c8dbe4a6810f1540bd9b15b8df0d00dbea) is labeled preliminary. `tensegrity_drone.py:get_contact_force` uses a normal spring/damper plane model; `controller.py:predict_delta_vel` has a fixed 45° rotation and filters active vertices by travel direction. It is not the complete general v3 contact estimator. [`collision-based-replanning`](https://github.com/BioMorphic-Intelligence-Lab/collision-based-replanning/tree/bffcb3ce2ed09a27fa19a28ac167215534cebae7) exposes the vector-field prototype and plots, without ASM2 feasibility enforcement.
- **Nested availability:** [`tensegrity_flightcontroller`](https://github.com/BioMorphic-Intelligence-Lab/tensegrity_flightcontroller/tree/5a07fa1918807882794ac5e59274adde441c5ffd) is mostly infrastructure/submodule references. Its pinned `muellerlab/GeneralCode` tree returned HTTP 404, so full controller reproducibility is **BLOCKED by unavailable source**. No firmware work was attempted. The pinned [`tensegrity-sysID/id.py`](https://github.com/BioMorphic-Intelligence-Lab/tensegrity-sysID/blob/f6ba64d4fc2f0adfb0c4e4d301fe8bedd8d6ccb5/id.py) is inspectable; it averages per-contact impulses and fits restitution/friction, defaulting angular-velocity fit weight to zero. Its point-velocity expression uses `cross(r_i, omega_prior)`, opposite to the usual `omega × r`; this must be frame/sign verified before reuse, not silently transplanted.
- **License/reuse:** no license file or explicit software grant was located in the inspected umbrella, Python studies, plotting, controller wrapper or contact-SysID snapshots. Public visibility and the paper's publication license do not supply code reuse permission. Reuse the methodology of separating pre/post contact state, measured impulse and collision identity; retain ASM2's native multi-hull contact model. Direct code reuse is not cleared.

### Data-driven system identification

Original: Eschmann et al., [Data-Driven System Identification of Quadrotors Subject to Motor Delays](https://arxiv.org/abs/2404.07837), **v2**, §III and §IV; [sysid.tools](https://sysid.tools/); [MIT-licensed source](https://github.com/arplaboratory/data-driven-system-identification/tree/2d267dd07b4262f579ee223d20b26a6dc9d17147).

- **Method:** mass and rotor locations are known inputs. Given a candidate common motor-speed time constant, apply an exact first-order filter to RPM-equivalent setpoints, fit polynomial thrust coefficients by linear least squares against accelerometer-derived force, and search the time constant by MAP (uniform bounded prior in the derivation). Fit roll/pitch inertia and yaw torque with selected excitation data. Low-excitation vertical-rotor yaw dynamics have an inertia/torque scaling ambiguity; a structural inertia-ratio assumption supplies yaw inertia.
- **Code verified:** [`sysid.ipynb`](https://github.com/arplaboratory/data-driven-system-identification/blob/2d267dd07b4262f579ee223d20b26a6dc9d17147/sysid.ipynb), `filter_ema`, `combine`, `estimate_motor_parameters`, time-constant sweep, `I_zz=(I_xx+I_yy)/2*inertia_ratio`, and torque regression; [`load_ulg.py`](https://github.com/arplaboratory/data-driven-system-identification/blob/2d267dd07b4262f579ee223d20b26a6dc9d17147/load_ulg.py) aligns topic data by sample timestamps. Website documents required acceleration, angular-velocity and motor-command signals and excitation-range limitations. The notebook contains Crazyflie and large-quadrotor examples; browser UI currently states PX4 ULog support.
- **Evidence:** hardware identification on Crazyflie 2.1 and a **3.35 kg** quadrotor; paper reports 73 s identification data and subsequent outdoor control demonstration. These are other aircraft's measurements. Included logs were inspected as repository contents, not treated as ASM2 data or uploaded anywhere.
- **Reuse:** strongest immediately reusable implementation once suitable existing ASM2 logs exist: local loader, delay sweep, thrust regression and residual/excitation analysis. Python NumPy/SciPy/pandas/pyulog notebook can be separated from web/WASM infrastructure. **Critical mismatch:** the paper filters motor speed then applies nonlinear thrust, whereas ASM2 filters force in newtons. Nonlinear mapping and filtering do not commute. A fitted RPM lag cannot simply replace ASM2's 40 ms thrust lag. Its diagonal-inertia fit also does not validate ASM2's full off-diagonal tensor. Report uncertainty, excitation and parameter correlation, not just a best fit.

### TrajPlanARPL

[GPLv3 source commit](https://github.com/arplaboratory/TrajPlanARPL/tree/0a5da582498e34377dd58550029d8fcc93c36e24). Also read its linked original [Robust Active Visual Perching with Quadrotors on Inclined Surfaces](https://arxiv.org/abs/2204.02458), rather than treating the README as experimental evidence.

- **Method/code:** minimum-snap polynomial QPs, waypoint derivative constraints, perching acceleration conditions, post-solve bound checks and iterative time extension; the paper adds FoV constraints and anticipation during replanning. [`ros_replanner_utils.cpp`](https://github.com/arplaboratory/TrajPlanARPL/blob/0a5da582498e34377dd58550029d8fcc93c36e24/ros_traj_gen_utils/src/ros_replanner_utils.cpp) predicts position/velocity from odometry plus old-trajectory increments, and transfers acceleration, jerk and snap into the new start state. It keeps the previous solution when replanning fails.
- **Non-portable assumptions:** [`TrajBase.cpp:calcPerchCond`](https://github.com/arplaboratory/TrajPlanARPL/blob/0a5da582498e34377dd58550029d8fcc93c36e24/traj_gen/src/traj_src/TrajBase.cpp) defaults to `[1,0,-3,0]` impact velocity, uses `4*sin(pitch)` terminal force/acceleration scaling, clamps target tilt past 90°, and adds fixed acceleration intervals. These constants are incompatible with ASM2's gentle contact contract. No full ASM2 hull, native first-contact or rotor-lag constraint is supplied.
- **Evidence:** original paper reports onboard 30 Hz active visual replanning, surfaces up to 90°, and experimental success comparisons of 80%→90% at 1.5 m and 70%→90% at 3.5 m. This is a different airframe, attachment, sensing and contact task; no ASM2 transfer claim.
- **Dependencies/license:** root LICENSE and README specify GPLv3 (README allows later versions), despite package manifests containing `TODO` license tags. C++17, ROS1, Eigen, OOQP, NLopt, BLAS, MA27/gfortran and custom messages are coupled in build files. The install script downloads/builds several external libraries and uses system installs; it was not run. MA27 and other dependencies need their own license review. Reuse continuity/anticipation formulations; do not port the whole dependency stack.

### Literature-to-ASM2 mapping

| Need | Reusable technique | Necessary ASM2 adaptation | Relative implementation cost |
|---|---|---|---|
| Adaptive terminal state and duration | Fast-Perching terminal transforms and MINCO; TrajPlan derivative constraints | Native legal patch/footprint, tangential-speed bounds, motor-state and allocation constraints | Medium; prototype several engineer-days, validation additional |
| Reachability waypoint selection | Liu local waypoint coverage over feasible trajectories | Include motor thrust/history and actual obstacle/contact geometry; no invented Gaussian tolerance | Medium/high; offline sampled graph first, no formal guarantee claimed |
| Thrust during attitude change | Liu bounded terminal thrust compensation | Respect rotating thrust axis, lag and shared rotor torque demand; no singular `1/cos(tilt)` compensation | Medium; after contact terminal set is credible |
| Full-body contact and impulses | ASM2 detailed native scorer; Colliding Drone pre/post momentum accounting | Preserve all forbidden hulls; measured compliance/restitution unavailable | Low for reporting, high for physical validation |
| Motor identification | Eschmann local delay/thrust regression and excitation checks | Known mass/hubs, correct timestamps/frames, unclipped logs; distinguish RPM and force lag | Low/medium software, currently blocked on raw ASM2 measurement data |
| Continuous replanning | MINCO warm start and TrajPlan anticipation/derivative continuity | Carry actual state and motor history, reserve feasible remainder and explicitly handle failed replans | Medium; no full G3 architecture in this phase |

These are engineering estimates, not measured development timings or authorization for implementation.

## 3. Contact geometry results

The pair is G2 `bank_row=51`, `dev_far`, `normal-center-attitude-attitude`, `original` arm. In the historical stale run, only outer feedback used a one-tick-old state; the inner allocator saw live state. The archived program changes force/attitude at 80 ms. This audit replays its two command tapes; it does not reopen actuator-memory diagnostics.

| Metric | Nominal | Stale |
|---|---:|---:|
| Native first signed sample | 206.1 ms | 206.5 ms |
| Native interval end | 206.2 ms | 206.6 ms |
| Result | QUALIFIED | FORBIDDEN_AIRFRAME_FIRST_TOUCH |
| First aircraft identities | `contact_front_006` | `contact_front_006`, `collision_0_550` |
| Signed manifold points | 8 | 12 (4 body) |
| Source body-minus-front plane support at common 205 ms | 11.887 µm | 13.331 µm |
| Same support at each first touch | 11.476 µm | 12.766 µm |
| Nearest actual native body-to-registered-target distance at first-touch pre-state | **+0.237 µm** | **−2.175 µm** |
| Nearest native front distance at that state | −11.208 µm | −14.909 µm |
| Maximum penetration in reported first manifold | 11.208 µm | 13.867 µm |
| Sum compressive first-interval normal impulse | 0.000430920 N·s | 0.000428349 N·s |
| Contact-point normal approach speed range | 0.150022–0.150108 m/s | 0.149800–0.150314 m/s |
| Contact-point tangential speed range | 0.158899–0.158910 m/s | 0.154916–0.154951 m/s |
| Body/target alignment | 2.895° | 2.798° |
| Rotor saturation controls | 0 | 0 |

**CONFIRMED:** the approximate 12–13 µm difference is reproduced, with its value depending on the comparison time. It is a support difference between source meshes against the infinite reference plane. It is **not** the forbidden body's clearance to the actual finite target hulls. At nominal first touch the source front remains +9.614 µm above the reference plane, yet its actual native target distance is −11.208 µm. The target hull shape/location and the actual contact manifold matter. Rear support is approximately 2.7–2.8 mm from the plane and no rear pair lies within the 1 mm query window.

Native pair-distance queries use separate `MjData`, so they cannot perturb integration. They examine all aircraft hulls against the 25 registered targets after conservative bounding-sphere pruning. `null` means no pair closer than 1 mm. The unchanged scorer independently handles all 1,822 static geoms, including forbidden targets. Pair-distance minima are separate native narrow-phase queries; they need not equal the finite set of reported manifold depths (the stale front values above show this). Neither is an independently certified CAD distance.

Per-point world normals, velocities, impulses and penetration are retained. Example nominal/stale front normals are `[-0.416384,0.002615,0.909185]` / `[-0.417918,0.002524,0.908481]`; other manifold normals remain individually recorded. All four stale body points have zero impulse but signed overlap, and therefore remain forbidden under the existing scorer. Similar impulse totals, better alignment, or a zero-force body contact cannot rescue that trial. At about 0.15 m/s approach, one frozen physics step corresponds to roughly **15 µm** displacement—comparable to the support lead. This is an explanation of possible sampling sensitivity, not proof that the native label is wrong or that continuous first-contact order is known.

### Frozen sensitivity grid

Each history has one zero-perturbation suffix, six target-normal translations (±5, ±10, ±20 µm), eight target-tangential translations (two axes × ±50/±100 µm), 18 body-axis attitude offsets (three axes × ±0.01/±0.05/±0.1°), and six final-switch shifts (±0.1/±0.2/±0.4 ms). Pose perturbations leave COM velocity, body rates and motor history unchanged; they are local uncertainty probes, not claimed reachable manoeuvres. The scale brackets the observed support lead, 0.112 mm state difference, 0.108° attitude difference and 0.4 ms contact-time difference. **Manufacturing, assembly, compliance and estimator-error distributions are UNKNOWN.**

| Local suffix family | Nominal qualified | Stale qualified |
|---|---:|---:|
| Unperturbed suffix | 1/1 | 0/1 |
| Translation | 13/14 | 6/14 |
| Attitude | 13/18 | 13/18 |
| Final command-switch timing | 6/6 | 0/6 |
| All local probes (descriptive only) | 33/39 | 19/39 |

All other outcomes were forbidden body contact. All sampled states pass the existing G2 flight-envelope check, all initial resets are clear, and no allocation saturates. The nominal failures include −20 µm target-normal shift, −0.05° body roll, +0.1° body pitch, and negative body yaw as small as **−0.01°**. Sign and magnitude effects are not monotonic: different offsets shift the sampled contact phase. Do not fit a manufacturing tolerance or infer a population success rate from these correlated, deterministic probes. Final-switch timing alone leaves each history's label unchanged; this does not contradict cumulative effects of 5 ms stale outer feedback.

![Source support, native clearance and first-touch kinematics](../runs/asm2-g31/contact-geometry.png)

![Local sensitivity grid](../runs/asm2-g31/contact-sensitivity.png)

## 4. Actuator provenance and uncertainty

Inspected the active `load_profile`, allocator, exact thrust filter and native actuator configuration; CAD mass metadata; legacy dynamics profile; inherited historical log-derived summaries. Searched project/upstream/artifact/run trees (including ignored files) for ULogs, SysID fits and the referenced parameter file. **No raw ASM2 flight ULog, ASM2 fitted SysID report, raw `2026_10_18_px4.ulg`, or raw `1018.param` was found in this checkout.** Therefore historical raw measurements could not be independently reprocessed; this is a data availability limit, not evidence that no logs exist elsewhere. Third-party example ULogs are unrelated aircraft.

| Parameter | Frozen ASM2 value/model | Provenance and confidence | Uncertainty / missing evidence |
|---|---|---|---|
| Mass | 0.142 kg | **REPORTED measured by user**; equals supplied package total | No scale record, battery/configuration repeatability or error bar |
| COM | Simulation origin; CAD COM `[-147.178441,482.488766,-1359.554944]` mm in source frame | **CAD_ESTIMATE**, package includes battery shift; part masses and uniform-density CAD volumes | No measured balance/COM uncertainty; absolute CAD coordinates are not aircraft dimensions |
| Full inertia | Diagonal `[1.11502578e-4,1.04557065e-4,1.87093388e-4]` kg·m²; `Ixy=1.03505127e-7`, `Ixz=-4.83224202e-6`, `Iyz=1.15618949e-7` | **CAD_ESTIMATE** from component mass distribution; tensor actually used | No pendulum/flight identification or uncertainty; do not replace with legacy box inertia |
| Rotor centers/axes | `(±0.0275,±0.041,0.00144473)` m, body +Z thrust | Centers/axis alignment CAD-derived; thrust direction assumption | Actual assembly alignment and force application unknown |
| Motor lag | 0.040 s, common exact first-order **thrust** filter | **PROVISIONAL inherited Ver2**, dynamics profile `status=engineering_estimate`, `evidence=[]` | Not measured on ASM2; rising/falling, voltage and loading dependence unknown. Prior 35/45 ms probes were sensitivity, not confidence bounds |
| Maximum rotor force | 1.103625 N each | **PROVISIONAL inherited Ver2** | No ASM2 thrust stand or flight fit; usable battery-dependent headroom unknown |
| Thrust coefficient/curve | No calibrated RPM/PWM-to-force coefficient in active plant; command already in N | Direct force actuator abstraction | `k_f` is **UNKNOWN**; the force cap is not a thrust coefficient |
| Reaction torque | Alternating ±0.018 m × rotor thrust | **PROVISIONAL inherited Ver2** | `k_tau/k_f`-type ratio, not independently calibrated RPM-squared torque coefficient; propeller asymmetry unknown |
| Rate loop | `17 s⁻¹*(rate_setpoint-rate)` angular acceleration, full inertia plus gyro compensation, inverse mixer and clipped rotor forces | **PROVISIONAL idealized simulation control**, not measured onboard transfer function | Sensor/filter/computation/transport delay and actual gains unknown. 20 rad/s native clamp differs from Phase 4's ±2 rad/s action map |
| Timing | Flight profile nominal 1 ms; contact plant explicitly 0.1 ms; command 5 ms | **CONFIRMED software settings** | Not measured hardware timing or jitter |

Historical `upstream/rm27_sim/configs/impact_recovery/px4_rate_1018.json` reports hover throttle 0.5105 from the October ULog and a legacy parameter snapshot; it is a separate approximate loop model and is **not** the current ASM2 rate controller. The corresponding IMU file reports raw accel/gyro rates about 1610/1997 Hz, clipping in normal flight, and assumed sensor/filter properties. These are **REPORTED historical summaries** for the older platform. Hover throttle and tuned gains do not identify ASM2 thrust curve, force lag or full inertia. No new hardware work or parameter fitting was attempted.

## 5. Reproduction, manifests and verification

Evidence lives in `.agents/runs/asm2-g31/`. [Verification receipt](../runs/asm2-g31/verification.json) · [Research/source manifest](../runs/asm2-g31/research-manifest.json) · [Pilot manifest](../runs/asm2-g31/pilot-manifest.json) · [Main manifest](../runs/asm2-g31/main-manifest.json) · [Native results](../runs/asm2-g31/main-results.json) · [Nominal 3D replay](../runs/asm2-g31/nominal-pilot-replay.npz) · [Stale 3D replay](../runs/asm2-g31/stale-pilot-replay.npz). Key artifacts:

- `pilot-manifest.json`, `main-manifest.json`: prospective immutable cases, model/runtime/source/checkpoint pins, perturbation rationale and budget.
- `pilot-ledger.jsonl`, `main-ledger.jsonl`: charged before each integration; 2 + 78 entries. `pilot-results.json`, `main-results.json`: complete native outcomes/manifolds; main traces retain every 0.1 ms pre-contact state from 200 ms.
- `contact-summary.json`, `first-contact-points.json`, `sensitivity.csv`: derived measurements. `contact-geometry.{png,pdf}` and `contact-sensitivity.{png,pdf}` are standalone figures.
- `nominal-pilot-replay.npz`, `stale-pilot-replay.npz`: representative full native command replays; `nominal-3d.png`, `stale-3d.png`, render logs. Both existing Isaac viewer self-tests passed with **14 visual meshes**. These are recorded native-state renderings, not new Isaac physics rollouts or policy inference. The micrometre margin is established numerically, not by screenshot inspection.
- `research-manifest.json`: downloaded PDF/source/license hashes, repository/submodule pins, retrieval failures and provenance. `verification.json`: final source-pin, replay, safety, budget and test receipts. Scripts are local diagnostics; there is no production controller dependency on the research snapshots.

From the project root, the executed native commands were:

```bash
export PYTHONPATH=src:. PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
.agents/runs/rm27-stage1/native-env/bin/python -m unittest discover -s tests -p test_asm2_contact_audit.py
.agents/runs/rm27-stage1/native-env/bin/python .agents/runs/asm2-g31/contact_audit.py freeze pilot
.agents/runs/rm27-stage1/native-env/bin/python .agents/runs/asm2-g31/contact_audit.py run pilot
.agents/runs/rm27-stage1/native-env/bin/python .agents/runs/asm2-g31/contact_audit.py freeze main
.agents/runs/rm27-stage1/native-env/bin/python .agents/runs/asm2-g31/contact_audit.py run main
./run_asm2_flight_viewer.sh .agents/runs/asm2-g31/nominal-pilot-replay.npz --self-test-output .agents/runs/asm2-g31/nominal-3d.png
./run_asm2_flight_viewer.sh .agents/runs/asm2-g31/stale-pilot-replay.npz --self-test-output .agents/runs/asm2-g31/stale-3d.png
```

These are provenance commands, **not an instruction to rerun completed experiments**. Existing-output guards intentionally refuse reruns/overwrites. For a separately authorized independent reproduction, use a separately provisioned copy of the diagnostic/output directory, preserve the published evidence and freeze that copy's source manifest. Relative project/root relocation changes absolute manifest paths and must be explicit. The external Goal Hybrid paths are read-only source inputs with pinned hashes.

Plotting uses an isolated `/tmp/rm27-g31-plot-env` with matplotlib 3.10.7 and NumPy 2.5.4; the pinned native runtime was not changed. `MPLCONFIGDIR=/tmp/rm27-g31-mpl /tmp/rm27-g31-plot-env/bin/python .agents/runs/asm2-g31/report.py` regenerates only derived figures/tables from completed results. Four focused tests cover perturbation frame semantics, preserved state components, final-switch scheduling and the cap. Previously completed training audits and actuator-memory diagnostics were not rerun.

## 6. Exactly one recommended next experiment

**A — independently validate continuous geometric first-contact order on the frozen G2 pair and the recorded label-flipping suffixes.** Use their already recorded pre-contact states, original detailed hulls and finite registered target triangles to bracket the first designated and forbidden intersections within each 0.1 ms interval with an independent swept-geometry calculation. Compare that bracket with the unchanged native manifold and source-mesh support, and explicitly account for rotation/interpolation error. Check CAD patch/body boundary semantics at `contact_front_006` / `collision_0_550`; do not alter that classification to obtain a pass. This is a diagnostic comparison, not a replacement scorer or authorization to change physics.

**Why:** the nominal forbidden-body clearance is only 0.237 µm, the stale label includes unloaded body points, and a −0.01° yaw perturbation flips qualification. The source-plane lead is smaller than approximately one tick of normal travel. Current evidence establishes discrete-model sensitivity but does not resolve continuous intersection ordering or physical surface tolerances. These uncertainties affect the terminal set that any reachability planner would use; optimizing against it immediately could select numerical contact opportunities.

**Cost:** approximately 1–2 engineer-days for an isolated swept convex/triangle check, event-bracketing tests and comparison against the existing traces; no PPO, hardware, new controller or full G3 architecture. Any new rollout budget would be separately specified, though the initial analysis can use saved traces. **Expected value:** determine whether a positive geometric separation interval exists and whether the current first-manifold decision is dominated by sampling or actual geometric co-contact. This is distinct from repeating the present perturbation grid. If trace resolution cannot establish the ordering, report an unresolved interval rather than a fabricated contact verdict.

Actuator calibration remains necessary for physical claims, but no raw ASM2 identification data is available here and the current contact ambiguity is directly measured. Proceeding to continuous rotation→braking→contact planning is not yet the next experiment. No additional experiment is launched by this recommendation.
