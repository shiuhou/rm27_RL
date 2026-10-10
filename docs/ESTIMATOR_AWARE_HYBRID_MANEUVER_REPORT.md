# RM27 Phase 5 — Estimator-aware hybrid maneuver

2026-09-23 · `astra/research` · root HEAD `da396bef63ce51476a71af4b9b70abdda03eed0b`.

## Main findings

The working architecture remains **learned continuous reference + model feedback**, with the existing actuator observer and causal recovery switch. No policy was retrained and no RL was introduced. This is not an additive neural motor residual.

This phase ran **254 full-field MuJoCo research episodes and 32 actual ArduPilot SITL flights** in the main studies. Development failures are preserved separately. The root suite passes **104 tests**. [All 286 main outcomes, including failures](../artifacts/phase5-results-02/trials.csv) and [aggregation/source hashes](../artifacts/phase5-results-02/summary.json) are saved.

The most useful findings are:

1. **Relative localization, persistent velocity bias and tilt bias matter together.** A 50 mm ego-only error misses; the same error shared by ego and target largely cancels. A persistent longitudinal velocity bias of only 0.05 m/s can move contact by 2–3 cm. “Position error > control error” is therefore too simple a description of this feedback system.
2. **A GPS-free actual EKF/control chain now runs.** Synthetic external-navigation position/velocity measurements feed real ArduPilot EKF3, while the actor reads only EKF telemetry. A new 12-flight sensor-seed holdout passes 9/12. This is a fusion/interface experiment, not a camera, optical-flow or VIO implementation.
3. **An uncertainty gate can avoid bad attempts, but does not repair localization.** On 24 paired MuJoCo holdout scenarios, the fixed policy completes 11; the guarded policy completes 6 and safely aborts 12. Six deliberately unreported-bias cases defeat all variants. Safe abort is never counted as contact success.
4. **Slowing down has limited value.** Conservative increases a small design-grid result from 14/21 to 15/21 versus balanced, but nominal time to confirmed recovery increases from 5.98 to 7.65 s. It does not solve a fixed localization offset.

These results establish a working research slice, not a certified uncertainty envelope. There is no hardware validation, online visual target tracking, takeoff/landing integration or full SITL dock-return mission in this phase.

## 1. Implemented interfaces and control

New code lives under `rm27/experiments/aggressive_contact/flight_control/`:

| File | Actual role |
|---|---|
| `synthetic.py` | Deterministic sensor producer: signed p/v/attitude bias, noise, delay, rate, dropout and velocity filtering. Only this simulation producer receives plant truth. |
| `quality.py` | `EstimateQuality`, covariance validation, relative-target registration, reset-time maneuver selection, one-time commitment guard and continuous braking reference. No simulator access. |
| `sensitivity.py` | Full RM field batch runner using the existing plant, policy API, reference and evaluator; saves every trajectory and sensor stream. |
| `diagnostics.py` | Explicit privileged, timestamp-aligned component substitutions; synthetic external-navigation MAVLink producer. Neither is hidden inside the actor. |
| `sitl.py` | Extended actual-firmware runner: synthetic/telemetry/truth inputs, study config, commitment gate, external-navigation injection, richer logs. |
| `batch.py` | Frozen local SITL matrices, isolated ports/processes/logs. |
| `error_budget.py` | Source-time/current-time vector decomposition and plots. |
| `summarize.py` | Machine-readable outcome table and sensitivity/commitment plots. |

`tests/test_contact_estimator_quality.py` adds seven contract tests. `evidence.py` snapshots Phase 5 configs/tests. `evaluation.py` now labels recorded-SITL replay correctly and marks reference diagnostics as a shadow evaluator. No vendor firmware or field-model source was edited. Existing unrelated root/nested changes were preserved.

The shared actor boundary remains:

```text
VehicleObservation(VehicleEstimate, velocity covariance)
TargetEstimate committed at reset
ActuatorEstimate from the causal nominal observer
EstimateQuality (declared covariance, persistent-bias bounds, freshness/confidence)
    → retained DeploymentPolicy
    → RateThrust / desired attitude reference
    → existing rate proxy OR actual AP attitude + thrust adapter
```

`RelativeTargetObservation` supplies future camera-derived position/normal in body FLU. It requires an ego sample aligned within 1 ms and rejects stale/confidence-invalid input. Registration is performed in the same estimated frame, preserving common-mode translation cancellation. Its covariance is **conditional relative uncertainty**, not an independent global target covariance. The guard can consume an explicitly supplied relative covariance instead of adding independent ego/target covariances. This adapter is unit-tested; no actual camera data was used.

The target is still fixed after reset. There is no moving-target tracker, continuous target update/replanning or tested target-update-frequency curve. The stale-target tests exercise declared quality metadata, not target motion. The existing actuator estimate remains a nominal force/ratio observer; true motor state is not supplied to the actor and motor-lag estimation was not added.

## 2. Estimator sensitivity map

The initial 91-episode design sweep freezes balanced reference, gains and nominal physical parameters, and changes one factor at a time. A further 63-episode grid compares all three operating points. Neither is a held-out success-rate estimate. Signed errors are in world NWU; attitude perturbations are body FLU.

![Sensitivity map](../artifacts/phase5-results-02/sensitivity.png)

| Isolated error | Observed balanced result on this plant |
|---|---|
| Position bias ±10 or ±20 mm, one axis at a time | All pass. At +20 mm X, first-contact margin is 9.2 mm. |
| Position X −25 / +25 mm | −25 mm passes with 9.25 mm margin; +25 mm fails despite 5.31 mm first-contact margin. |
| Position X/Y ±50 mm | All fail, mainly rim/wrong/undesignated contact. Z is asymmetric. |
| Velocity X ±0.03 m/s | Pass; margin 14.7 / 10.1 mm. |
| Velocity X ±0.05 m/s | Both fail. Contact error 23.3 / 26.9 mm. |
| Pitch bias ±0.5° | Pass with 18.3 / 13.2 mm margin. At +0.75°, margin is only 7.48 mm. |
| Pitch +1° | Fails; roll ±1° passes, ±3° fails. Yaw ±5° still passes in this fixed geometry. |
| 50 Hz estimates, 10 or 20 ms delay | Nominal proxy runs pass. |
| 50 Hz estimates, 50 ms delay | First contact is accurate (2.9 mm error), but recovery times out. |
| Velocity low-pass lag 20 / 50 / 100 ms | Pass / rim failure / target miss and tilt-limit failure. |

Why velocity bias matters: the retained feedback contains `Kp(p_ref-p_est) + Kd(v_ref-v_est)`. In a simple steady tracking approximation, position offset includes `−Kd/Kp × velocity_bias`. For horizontal gains 7/16, 0.05 m/s predicts 21.9 mm, consistent with the measured 23–27 mm contact error. This is an explanatory approximation, not a model of every transient/adaptation term.

Tilt bias affects the entire feedback trajectory, not merely the 42 mm bumper lever. Consequently, a small instantaneous geometric lever error cannot be used to dismiss attitude-estimation quality. The nonmonotonic negative-Z velocity result is retained; adaptation and contact dynamics prevent a universal scalar threshold.

Target translation bias produces almost the opposite of ego translation bias, as expected for relative tracking. Four common-mode shifts, including `[+100, −100, +100]` mm, preserve MuJoCo contact error at 2.67 mm. Two actual SITL common-mode checks also pass at approximately 7.37 mm, comparable with the new truth diagnostic's 7.40 mm. **A globally biased coordinate frame can work when ego and target share it; an accurate ego estimate paired with a misregistered target cannot.** This is a constructed invariance test, not measured visual performance.

Target-normal perturbations reveal an existing registration limitation: +3° and +5° fail with environment contact, while negative values pass. `TransformedReference` rotates the entire fixed-scene spline around the target. A 3° error can shift the distant departure reference by roughly 0.6 m. These failures therefore combine registration/field geometry with terminal-normal error; they must not be advertised as an intrinsic “normal accuracy must be <3°” requirement. No trajectory-registration redesign was made in this phase.

### A combined-error validation point

An independent, frozen 16-corner set uses:

- X position bias ±5 mm;
- X velocity bias ±0.01 m/s;
- pitch bias ±0.1°;
- target Z bias ±3 mm;
- 20 ms delay and 50 Hz samples;
- 1 mm position, 0.005 m/s velocity, 0.05° attitude white noise;
- new physical draws at 0.25× the existing assumed small-uncertainty distribution.

All **16/16 pass**, minimum first-contact margin **11.21 mm**. The signed four-dimensional corners, seeds and complete traces are in `phase5-requirements-holdout-01`. This is a tested candidate operating point, not a proof of the continuous volume between corners, all-axis simultaneous tolerance, real sensor suitability or hardware reliability.

## 3. Actual EKF failure: error budget and interventions

The Phase 4 GPS-backed EKF failed at 82.72 mm contact error. We retained its original logs and decomposed the position error vector as:

```text
estimate(t_source) − truth(t_now)
 = [estimate(t_source) − truth(t_source)]
 + [truth(t_source) − truth(t_now)]
```

Source-time truth is interpolated offline only. `time_boot_ms` is an estimate publication clock, not the raw sensor acquisition clock. Internal EKF prediction/delay remains part of the first term. The terms are correlated vectors; their norms cannot be added as independent causal percentages.

| Time after release | Source-time position error XYZ, mm | Source-age displacement XYZ, mm |
|---:|---|---|
| 1.0 s | −20.6, +7.4, +7.0 | +40.4, +1.0, −23.9 |
| 2.0 s | +16.6, +39.0, +2.9 | +130.4, +2.1, −4.7 |
| 3.0 s | +111.9, −9.5, −2.6 | +50.2, +15.1, +27.6 |
| 4.0 s | +34.7, −17.8, −20.0 | +50.0, +2.6, +15.1 |
| 4.466 s, just before contact | +58.5, −26.7, −26.8 | +17.9, −0.7, +13.7 |

Near contact, the direct bumper-lever contribution is only `[+0.10,+0.47,−0.05]` mm; the resulting current contact-point error in target tangent/tangent/normal coordinates is `[74.0,−27.0,23.7]` mm. Measured velocity error at source time is `[+0.057,−0.073,+0.007]` m/s, and attitude error is about 1.01°. The target map transform has zero injected error **by construction** in this diagnostic; real mounting/world registration remains unknown.

![Timing and estimator diagnostics](../artifacts/phase5-error-budget-01/error_budget.png)

Privileged interventions replace only selected telemetry components with truth at the **same publication timestamp**. They retain the telemetry cadence and age; none is a deployment solution.

| Actual SITL observation | Contact error | Margin | Joint result |
|---|---:|---:|---|
| Original GPS EKF (Phase 4) | 82.72 mm | −27.58 mm | FAIL |
| New unchanged GPS EKF repeat | 83.21 mm | negative | FAIL |
| Correct position only | 42.46 mm | 2.54 mm | FAIL: wrong/undesignated first contact |
| Correct velocity only | 67.39 mm | −32.98 mm | FAIL |
| Correct attitude/rate only | 77.43 mm | −25.34 mm | FAIL |
| Correct p/v/attitude/rate together, retain timestamp/age | **6.63 mm** | **24.20 mm** | **PASS** |

Position correction has the largest effect in this set, but neither it nor velocity correction alone restores valid contact. The all-component case shows that the existing timestamp propagation can work with the observed publication schedule. It does not identify each sensor bias or prove delay is harmless. These are single nominal diagnostic flights; nonlinear effects are not additive.

## 4. Uncertainty-aware commitment and its cost

`EstimateQuality` carries validated covariance matrices, explicitly declared persistent position/velocity/tilt bias bounds, target confidence and age. The empirical risk score projects relative position uncertainty into the contact tangents, includes bumper attitude uncertainty and age × velocity uncertainty, and adds persistent-bias allowances. The velocity and tilt coefficients are design-study heuristics, not certified worst-case bounds or calibrated chance constraints.

The guard checks once at **50% of approach duration**, before terminal commitment. It requires estimated error allowance ≤15 mm, ego age ≤35 ms, target confidence ≥0.8 and declared target age ≤250 ms. If rejected, it substitutes a 1.2 s quintic braking reference with continuous p/v/a, then holds. No contact detector truth is used. Once committed, it does not claim a new safe escape capability for later estimate loss; that remains unimplemented.

The first implementation switched immediately to a fixed hover point. Its development test reached **133.4° tilt**. That failure is preserved in `phase5-smoke-02`; its original `safe_abort` flag was too weak and must not be used. The corrected evaluator rejects flight-envelope violations. Continuous braking reduces the repeated development test maximum tilt to **33.0°**, with no target/environment contact. Actual SITL GPS-EKF and synthetic-bias abort tests both stop without contact, with maximum tilt about 33.4° and 33.0°.

The 24-case holdout uses six new paired physical/sensor draws per condition. Thresholds and braking duration were frozen before this holdout. “Reported bias” uses a known synthetic bound to test the decision rule; it does **not** demonstrate online uncertainty calibration.

| Condition, 6 cases each | Fixed: contact+recovery | Guard: contact+recovery / safe abort | Scheduled+guard: contact+recovery / safe abort |
|---|---:|---:|---:|
| Low declared uncertainty | 6 | 6 / 0 | 6 / 0 |
| Larger, correctly declared bias | 4 | 0 / 6 | 0 / 6 |
| 50 ms delay | 1 | 0 / 6 | 0 / 6 |
| Hidden 50 mm bias, overconfident metadata | 0 | 0 / 0 | 0 / 0 |
| **Total** | **11/24** | **6/24 + 12 safe aborts** | **6/24 + 12 safe aborts** |

![Commitment outcomes](../artifacts/phase5-results-02/commitment.png)

The guard improves the defined safe outcome count from 11 to 18, while reducing completed missions from 11 to 6. It unnecessarily aborts five attempts which the fixed controller happened to complete. This is an explicit conservatism/availability trade-off. Safe-abort checks include no physical contact, final speed/tilt/rate limits, altitude and flight-envelope limits; they do not certify an obstacle-free stopping corridor for arbitrary starts.

Scheduling selects conservative at reset for larger declared risk/age; it never abruptly switches between splines mid-flight. It adds no successful outcomes over the guard in this holdout, while mean recovery-confirmation time for the low-uncertainty group rises from **6.014 to 7.666 s**. We therefore **do not select scheduling as an improvement**. Accurate uncertainty and an early commitment decision are more useful here than indiscriminate slowing.

## 5. GPS-free actual ArduPilot experiment

`configs/contact_phase5/externalnav.parm` verifies GPS1_TYPE=0, SIM_GPS1_ENABLE=0, external-nav horizontal/vertical position and velocity sources, VISO_TYPE=1, and retains the IMU/compass attitude chain. Logs explicitly report **EKF3 is using external nav data**. A manually supplied global origin is registration metadata, not a GPS measurement.

`ExternalNavigationSensor` emits timestamped `VISION_POSITION_ESTIMATE` and `VISION_SPEED_ESTIMATE` at 50 Hz. Its measurements are synthesized from MuJoCo truth with configured errors. The actor never receives those raw values or true state; it receives AP EKF telemetry after fusion. This provides a working **synthetic VIO-equivalent producer → real non-GPS EKF → hybrid policy → real flight controller** chain. It is not optical flow/ToF or visual navigation.

The firmware binary remains the Phase 4 binary, SHA-256 `da030d364cf22c34403f7707d1a3d755263fd81dd59a9007b31e75538c0a9473`, reporting 4.8.0-dev (96f98857). Attitude + thrust and `ATC_RATE_FF_ENAB=0` are retained; no firmware rebuild or inner PID tuning was performed. Thrust mapping and all parameters remain simulation assumptions.

The first design sample passes at 17.19 mm contact error, 12.18 mm margin and 0.628 s recovery entry after impact. A separately frozen set of sensor seeds yields:

| Synthetic external-nav input | Joint successes | Failure detail |
|---|---:|---|
| 3 mm position / 0.02 m/s velocity noise, 10 ms source delay | 3/4 | One later undesignated contact, first-contact margin 2.2 mm |
| 10 mm / 0.05 m/s noise, 10 ms delay | 2/4 | Margins 3.4 mm and approximately zero; undesignated/rim contact |
| 3 mm / 0.02 m/s noise, 30 ms source delay | 4/4 | None in these four seeds |
| **Total** | **9/12** | All 12 recover; three fail the contact task |

The groups use independent sensor seeds; they are **not a paired proof that delay helps**. The 30 ms group deliberately retains VISO_DELAY_MS=10 and is a timing-mismatch test. Offboard clock correction, EKF prediction and the small sample count complicate interpretation. No parameters were changed after viewing this holdout.

Synthetic perfect-state delay tests further show why the old proxy is not enough: real AP passes the nominal balanced 0/20/50 ms cases, while the research rate proxy fails recovery at 50 ms. The conservative AP variants also pass these three nominal cases. This does not establish a 50 ms deployment allowance under noise, impact transients or packet loss.

All **32 main Phase 5 firmware flights** completed and received common contact evaluation. Recorded-motor replay verifies the evaluator against original states. Replay is not an independent firmware trial. The airborne initialization fixture, fixed target map, nominal mass/thrust/motor dynamics and truth-assisted synthetic sensor generation remain explicit limitations. No returned-to-dock or landing claim is made.

## 6. What a future estimator should deliver

The evidence supports requirements on **relative error and persistent bias**, not just nominal sensor resolution or total position RMSE:

- Use a common, verified ego/target frame. Carry frame registration and camera extrinsics; common-mode map errors can cancel, independent target/ego misregistration cannot.
- Start with the tested combined-error point above: roughly 5 mm longitudinal ego bias, 3 mm target-height bias, 0.01 m/s longitudinal velocity bias and 0.1° pitch bias with 20 ms/50 Hz estimates. These are simulation study inputs to reproduce, not measured hardware specifications or necessary limits for every axis.
- Preserve capture/source and receipt clocks, monotonic update identifiers, covariance/confidence and persistent-bias diagnostics. Repeated held estimates must not masquerade as fresh velocity. MAVLink publication timestamps alone are insufficient to identify sensor delay.
- Target at least 8 mm first-contact margin in validation, and keep checking later undesignated/secondary contacts and recovery. A valid first contact or a positive margin alone is insufficient.
- Do not infer confidence from a small white-noise covariance while omitting drift, extrinsic bias, velocity bias or delayed filtering. The hidden-bias holdout shows exactly why that fails.

For the real hardware, a reasonable staged path is optical-flow + ToF + IMU for local motion, AP non-GPS fusion, then a synchronized target-relative vision estimate. Flow/range alone do not identify the base-relative horizontal origin. Actual flow tracking limits, range behavior during tilt, sensor clocks, extrinsics and bias calibration still require measurements. The new synthetic external-nav source supplies an executable software boundary for that work; it does not replace those measurements.

The next highest-value experiment is to feed **recorded or actual target-relative observations with synchronized ego estimates through this same interface**, and measure bias/latency during the last two seconds and impact. A full SLAM stack or Residual RL is not required to answer that question. There is no current evidence that RL addresses the dominant remaining error.

## 7. Evidence, splits and reproducibility

The original request is preserved with SHA-256 in `artifacts/phase5-verification-01`. This work read the current Phase 4 report/launch logs, teacher/StudentSkill/deployment/observation code, full-field runner/evaluator, real-hardware/integration contracts, AP Guided/attitude-control and external-navigation test examples. Current source/config hashes, root/nested status and actual pytest outputs are also in that verification directory. Nested working trees were already dirty; their contents were not cleaned, relocated or rewritten.

The main campaign ledger is:

| Artifact | Trials | Role |
|---|---:|---|
| `phase5-sensitivity-01` | 91 | Nominal signed one-factor design sweep |
| `phase5-fine-modes-01` | 63 | Fine bias grid × three aggression modes |
| `phase5-guard-holdout-01` | 72 | 24 paired draws × fixed/guard/scheduled |
| `phase5-requirements-holdout-01` | 16 | New combined-error corners and physical draws |
| `phase5-common-frame-01` | 12 | Frame invariance and target-quality metadata controls |
| Four `phase5-sitl-correct-*` directories | 4 | Privileged component diagnostics |
| `phase5-sitl-externalnav-01` | 1 | Non-GPS design demo |
| `phase5-sitl-design-01` | 12 | Actual AP delays/biases/repeated EKF/abort controls |
| `phase5-sitl-holdout-01` | 12 | Frozen external-nav sensor-seed holdout |
| `phase5-sitl-common-frame-01` | 3 | Common-frame checks and truth repeat |

Development artifacts are additional: `phase5-smoke-01` contains four runner failures due to an evaluator-reference attribute missing from the wrapper; `phase5-smoke-02` contains four flights including the unsafe abrupt stop and initially weak abort flag; `phase5-smoke-03` contains four corrected tests. None was deleted or included in the main benchmark count. All main runs preserve failed trajectories, config/seed, sensor stream, actor records and source snapshot. Model overlays reference the immutable saved arena plant; the existing raw CAD/field assets remain local dependencies.

No train/test frame split or new neural training was used. Design configs and reference checkpoint hashes are frozen in manifests. The uncertainty holdout and combined-error holdout have independent physical/sensor seeds. The external-nav holdout has independent sensor seeds but nominal physical parameters; it does not validate physical robustness. Its results were not used to tune the controller or EKF.

### Actual commands and repeatable entry points

Commands below use new output directories; historical artifact names in the ledger are the directories actually used. The same module invocations ran successfully in this phase. The previous sandbox execution fault is resolved.

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sensitivity --config configs/contact_phase5/sensitivity.json --output artifacts/phase5-sensitivity-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sensitivity --config configs/contact_phase5/guard_holdout.json --output artifacts/phase5-guard-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sensitivity --config configs/contact_phase5/requirements_holdout.json --output artifacts/phase5-requirements-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.batch --config configs/contact_phase5/sitl_holdout.json --output artifacts/phase5-sitl-holdout-new
```

Small non-GPS full-field demo, actual local firmware:

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sitl --mode maneuver --observation telemetry --action-layer attitude --parameters configs/contact_phase5/externalnav.parm --study configs/contact_phase5/externalnav.json --instance 47 --duration 9 --output artifacts/phase5-externalnav-new
```

Watch the already recorded passing flight in MuJoCo:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/phase5-sitl-externalnav-01/evaluation/flight --speed 1 --loop
```

This is recorded-state playback of the checked firmware-motor replay. Space pauses; V switches overview/follow. It is not another closed-loop flight. [EGL full maneuver GIF](../artifacts/phase5-render-01/maneuver.gif) and [contact frame](../artifacts/phase5-render-01/contact.png) were generated and image-inspected. No live GUI session was claimed.

Reproduce analysis and checks:

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.error_budget --input artifacts/phase4-unshaped-telemetry-01 artifacts/phase5-sitl-correct-all-01 artifacts/phase5-sitl-externalnav-01 --output artifacts/phase5-error-budget-new
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

The before-change suite passed 97 tests; the final suite passed **104 tests in 6.82 s**. New checks cover covariance validity/projection, reference boundary continuity, commitment timing, synthetic source timestamps/bias, diagnostic isolation, external-nav frame/cadence and relative-target time/frame registration. No new dependency was introduced beyond the existing research environment; the SITL binary and local research bundle are still required.

Status: **IMPLEMENTED / RUN / TESTED**, conditional **SIM-VALIDATED**, and limited **SITL-VALIDATED** for the stated flights and synthetic source. **Not HARDWARE-VALIDATED.** No claim of reliable real visual contact, calibrated uncertainty, formal safe-stop feasibility or guaranteed recoverability.
