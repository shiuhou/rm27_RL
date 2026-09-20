# VSL-2 controlled localization recording protocol

This protocol prepares future data collection; it does not run an estimator.
Start with a rigid-camera/handheld capture where practical, then repeat the
same motion on the UAV after the camera is fixed and safe.

For every sequence keep camera mode, resolution, crop/resize, lens/focus,
exposure/gain, board/software build, calibration ID, recording command,
environment/lighting, and raw-file hashes fixed or explicitly recorded. Keep
calibration, tuning, stress, and evaluation recordings in separate sessions.

| Sequence | Tests | Hold fixed / vary |
|---|---|---|
| initialization + slow translation | feature initialization and parallax | fixed scene/lighting; vary slow forward/side translation |
| straight out-and-back | repeatability and drift | fixed speed target; reverse direction once |
| small closed loop | loop consistency | fixed route; vary loop direction only in a separate take |
| static hold after initialization | tracking stability | fixed pose and scene; no intentional motion |
| progressive yaw | rotational blur/rolling-shutter symptoms | fixed position; increase yaw rate by take |
| translation + yaw | coupled motion | fixed route; add one controlled yaw rate |
| close planar approach | planar/low-parallax failure | fixed target plane; vary distance |
| low-texture surface | texture sufficiency | fixed motion; use a documented blank wall/floor |
| lighting transition | exposure adaptation | fixed path; change lighting once |
| brief occlusion/freeze | recovery and continuity | fixed path; introduce one bounded occlusion |
| RM27/hangar repeated structures | perceptual ambiguity | fixed route and scene; document repeated geometry |

VSL-1B direct metadata capture should run in parallel with the first controlled
suite, preferably on the same full180/NV21 configuration. It is not required
to begin calibration, but it is required before VIO timing claims or hardware
drop-rate claims.
