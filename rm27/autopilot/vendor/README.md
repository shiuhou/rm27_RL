# Autopilot vendor checkouts

This directory contains the external ArduPilot checkout and the ArduPilot
Gazebo plugin used by root launchers. They remain independent Git submodules;
the RM27 package owns the launch arguments and adapters around them, not their
upstream source history.

Do not put RM27 controller code here. Add that code under
`rm27/autopilot/` and record any upstream patch in `patches/`.
