# PX4 return identification and landing software

Updated 2026-10-11. Work is in the independent PX4 migration worktree.

The downward camera has a dedicated implementation. `return_camera.py`
detects AprilTag 36h11 IDs 21–24 in actual current-flight rendered pixels and
computes metric position relative to the assigned slot. `visual_return.py`
admits only the assigned fresh image with fresh attitude/navigation and a
bounded registration. `flight.py` aligns, descends, reacquires after temporary
loss, requests normal PX4 LAND and checks landed state and automatic disarm.
`tag_height.py` additionally registers return height from metric tag pose,
attitude and the configured camera mount, preserving raw EKF telemetry.

These are host simulation programs. The camera mount, intrinsics and plane
offset are explicit simulation assumptions; they are not OS04A10 calibration
or qualified M3C deployment. The user selected a centre underside camera and
80 mm codes on 96 mm panels. Its estimated 2–3 g has not been added to the
150 g simulation profile, following the user's instruction to defer weight.

## Permanent camera loss development

The original flight runner raises on permanent reacquisition timeout. Closing
a SITL process while airborne cannot establish safe physical termination.
`experiments.degraded_return` therefore tests a separate bounded policy:
after a declared permanent pixel blackout and the original mission timeout,
it continues the same simulation, requests PX4 normal LAND and observes
actual landing/automatic disarm before closing the processes.

Admission requires fresh, valid, slow horizontal navigation and attitude,
an assigned tag previously aligned within 4 mm, its age at most 20 seconds,
and estimated height between 0.08 and 0.8 m. V2 uses 7 mm admission as explained
below. This policy is restricted to the
declared return-camera timeout. It does not authorize landing after invalid
navigation, arbitrary errors or elsewhere along the target route.

The independent physical scorer requires rest inside a declared 25 mm
horizontal termination bound, height error at most 5 mm, no unintended
contact, bounded contact force/penetration, actual landed/disarmed telemetry
and stopped motors. This wider termination bound is separate from the
unchanged 7 mm successful-slot gate. Termination is never task success.

Development `artifacts/px4-permanent-return-loss-development-01` passes once:
292 actual blacked-out frames, normal LAND ACK accepted, landing detected,
automatic disarm and stopped motors. Continued termination lasts 16.48 s;
independent resting XY error is 8.172 mm. The original mission summary remains
`passed: false`; `termination_summary.json` reports the different gate as
`passed: true, task_success: false`.

Five independent reserved V1 termination starts are declared before launch in
`configs/px4_permanent_return_loss_v1.json`, with blackout onsets 0.2/0.5/0.8/
1.1/1.4 s after DESCEND. This set excludes the development attempt and saves
all attempts. V1 fails **2/5**: three admission rejections when the last image
reports 4.4–4.6 mm, with valid navigation. Those cases do not execute a verified
termination and remain failures; their original summaries are preserved.

V2 keeps the separate 25 mm termination bound and adopts the existing 7 mm
successful-slot envelope for cached-image admission, leaving an explicitly
declared 18 mm motion allowance. That allowance is an engineering assumption,
not an estimator confidence guarantee or a relaxed task-success gate. A new
development seed 170211/onset 0.35 s passes with 4.014 mm resting XY, followed
by a separately reserved **5/5** termination batch in
`artifacts/px4-permanent-return-loss-batch-02`. Seeds 170301–170305 and blackout
onsets 0.45/0.75/1.05/1.35/1.65 s are fixed before its first attempt.
Task success remains **0/5**. This is a short-return profile, not the complete
target-contact mission or the remaining navigation/communication fault matrix.

## Final alignment and landed confirmation development

The complete-route fault study has exposed a final admission boundary:
fresh image alignment can drift just beyond3 mm after the stable ALIGN hold.
`tools.px4.ground_rest` saves an explicit experimental flight-runner copy
that resumes alignment instead of tearing down an airborne process. The
original3 mm admission and bounded retry count remain. A declared new image
pixel-translation experiment checks actual entry into that recovery branch.

The same runner confirms landed/disarmed state using fresh PX4 telemetry,
range and stopped actuator output below the flow floor. Independent physical
rest, velocity and motor-force gates remain mandatory. A new0.03 m/s POST-only
estimated-velocity bias flight passes the full image/VIO mission and its
separate audits. See [condition report](PX4_IMAGE_CONDITIONS.md) for actual
artifacts and limits. Active frozen batches continue to use their original
runner; these are development changes, not a relabelled acceptance.

The shared implementation now lives in `autopilot/px4/return_termination.py`;
the original development entry reuses it. Complete `image_mission` runs may
explicitly select `return_termination_configuration`. Camera reacquisition
timeout invokes the policy before process cleanup, using the last assigned
image and current telemetry. All other failures remain outside this policy.
Admission or physical termination failure is retained, and even a successful
termination leaves `summary.json` failed. Full-route fault experiments use
`experiments.full_return_faults`; their acceptance is separate from short-return
results and from a successful normal mission.

Complete-route development `px4-full-permanent-return-loss-development-01`
now passes one physical termination: target approach/contact/recovery precede
a permanent blacked-out downward camera; the unchanged reacquisition timeout
then invokes normal LAND in the same PX4 process. Automatic disarm and stopped
motors are observed, with independently scored resting XY 2.561 mm and
14.46 s continuation. `summary.json` remains failed. The separate retained
score in `px4-full-permanent-return-loss-development-score-01` checks the first
physical impact, no secondary contacts, stable recovery and the earlier hover.
One development case does not establish five-case fault acceptance.

The completed original complete-route fault gate fails:9 tasks,3 independently
safe terminations and3 unhandled cases out of15 in
`px4-full-return-fault-batch-01/summary.json`. Wrong-ID attempt09 rejects final
alignment; permanent-loss14/15 contact dock dividers/rim. Landing and eventual
disarm do not override the unintended-contact failure. A causal actor copy,
bounded alignment retry and3 s image-gap budget now have separate passing
development evidence; fresh reserved sets require their own acceptance.

Complete-route upstream command-loss development also passes through the
causal actor in `px4-causal-command-loss-development-01` and its independent
`-score-01`. Only after fresh assigned-tag alignment within3 mm and slow valid
navigation for0.25 s are upstream setpoints stopped. The same PX4 process must
enter onboard Offboard-loss LAND without an external LAND command, land,
automatically disarm and satisfy independent25 mm physical termination.
ACK/STATUSTEXT received before and during continuation are retained. Five
new reserved seeds are in `configs/px4_causal_command_loss_preregistered_v1.json`;
`tools.px4.command_loss_batch` freezes, launches and scores them. This does
not qualify command loss at arbitrary route positions or invalid navigation.

The reserved causal command-loss set exposes a real counterexample: attempt02
loses commands during RETURN at approximately0.601 m estimated height, despite
fresh tag alignment within3 mm. Onboard LAND and automatic disarm occur, but
the aircraft touches a dock divider. Final rest within25 mm does not satisfy
the no-unintended-contact requirement; the five-case gate fails.

A separate development entry, `tools.px4.low_alignment_command_loss`, restricts
interruption to ALIGN at10–14 cm estimated height, with the same fresh assigned
tag and0.25 s slow-navigation admission. It blocks position, mode and command
methods immediately after interruption; onboard PX4 must finish landing.
Seed424017 passes in `px4-low-alignment-command-loss-development-01` and
`px4-low-alignment-command-loss-development-score-01`: physical resting XY
1.698 mm, no unintended contact, actual prior EV fusion, normal LAND and
automatic disarm. The overall mission remains failed by design.

Read-only ULog confirmation in
`px4-user-pause-checkpoint-01/low_alignment_offboard_loss_ulog.json` observes
`offboard_control_signal_lost` at151.836 s after interruption at150.788 s,
followed by LAND/failsafe at151.848 s. A simulated battery-unhealthy flag
already exists before interruption without an active failsafe; it is retained
as a simulation limitation. This single development result neither repairs
nor replaces the earlier-return failed acceptance set. No new low-altitude
acceptance batch is launched at the user's pause.

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.experiments.degraded_return_batch \
  artifacts/px4-dock-tags-05/scene.xml \
  --config configs/px4_permanent_return_loss_v2.json \
  --workers 2 --instance 46 --output artifacts/px4-permanent-return-loss-new
```

Temporary blackout/stale/wrong-ID short-return results retain their narrower
15/15 scope in `PX4_MIGRATION_STATUS.md`. Complete image-mission evidence,
including the failed old 11/20 nominal batch, is in `PX4_IMAGE_MISSION.md`.
