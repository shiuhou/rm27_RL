# VSL-2 camera calibration procedure

Status: procedure implemented; calibration capture not executed.
Current result: `BLOCKED_ON_CALIBRATION_CAPTURE`.

## Fixed acquisition configuration

Use the same observed image geometry as `0921.mp4`: 1344 × 760, nominal
180 Hz, full180 business path. The actual sensor mode, crop/resize state,
lens model, focus setting, module revision, and exposure/gain must be recorded
from the device or operator log; they are currently `UNKNOWN`. Do not calibrate
at another resolution and silently scale the result.

Before capture, record the camera module and lens marking, focus mechanism and
setting, board/software build, camera configuration, exposure/gain, white
balance, output format, stride, and any crop/resize or stabilization setting.
Keep the lens mount and focus fixed for the entire calibration and validation
capture. Measure and record the physical target dimensions; do not invent a
checkerboard square size.

## Target and views

Use a rigid, flat OpenCV-supported checkerboard or AprilGrid. The operator must
provide the measured square/spacing dimension and target identifier. Capture at
least 30–50 valid views covering the optical center, all four corners, borders,
and intermediate regions, with varied distance and tilt. Include oblique views
and enough depth variation to constrain distortion. Reject frames with motion
blur, clipping, glare, partial target visibility, or unstable focus.

Hold out at least 20% of valid views, selected before fitting, for independent
validation. Keep calibration and validation image files, source timestamps,
capture configuration, and file hashes together.

## Model decision

Do not select pinhole or fisheye by sensor name. First identify the actual lens
and inspect edge distortion. If the lens and distortion class remain
ambiguous, calibration remains blocked. Once evidence is available, record the
chosen model and the reason; candidates are perspective/pinhole with a
documented distortion vector or OpenCV fisheye with its documented vector.

## Outputs and provisional gates

Produce a versioned calibration artifact using
`calibration_artifact.template.json`. It must contain the calibration ID,
module/lens/mode identifiers, dimensions and crop, model and coefficients,
measured target dimensions, tool/version, commit, capture date, fit metrics,
independent validation metrics, residual distribution, source dataset, and
file hashes. Unknown values remain `UNKNOWN`.

Use fixed, pre-declared engineering gates and report them as
`PROVISIONAL_ACCEPTANCE_CRITERIA`: no invalid solve, no non-finite parameters,
independent validation error no worse than the declared fit tolerance, no
systematic edge residual pattern, and visually sane undistortion at center and
corners. The project currently has no approved numerical threshold, so no
numeric pass claim may be made until the team fixes the threshold before
inspection of results.
