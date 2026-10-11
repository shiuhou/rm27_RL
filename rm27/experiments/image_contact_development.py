"""Compatibility entry for the guard-tip development command; use image_mission."""
from ..autopilot.px4.image_route import ImageRoute as GuardTipRoute
from ..autopilot.px4.image_mission import main

if __name__ == "__main__":
    raise SystemExit(main())
