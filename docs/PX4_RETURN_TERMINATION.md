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
