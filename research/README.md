# Independent research

This directory contains studies that support the RM27 project but are not the
main runtime package:

- `astra/`: observation-only terminal-guidance feasibility study.
- `Darts/`: simplified dart trajectory and CAD path experiment.
- `se3_radio_starter/`: SE3 controller and radio-input reproduction package.

The RM27 sensing path is kept under `rm27/perception/vision/`, because M3C /
OS04A10 localization and target recognition are part of the main system
boundary. Independent studies must state their own assumptions and must not be
treated as complete-mission evidence automatically.
