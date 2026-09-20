# Local component documentation changes

These patches preserve uncommitted component documentation without publishing
commits to third-party upstream repositories. They are already applied in the
original working copy. For a fresh checkout, from the root:

```bash
git -C rm27/autopilot/vendor/ardupilot apply --check ../../../../patches/ardupilot-local-guidance.patch
git -C rm27/autopilot/vendor/ardupilot apply ../../../../patches/ardupilot-local-guidance.patch
git -C rm27/simulation/legacy/mujoco_uav apply --check ../../../../patches/mujoco-uav-readme-paths.patch
git -C rm27/simulation/legacy/mujoco_uav apply ../../../../patches/mujoco-uav-readme-paths.patch
```

Do not apply twice. The root gitlinks retain the published component commits.
