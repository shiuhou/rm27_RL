#!/usr/bin/env bash
# Isolated GPU free-flight evaluation in the installed Isaac Lab image.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cache="$project_dir/.agents/runs/asm2-flight/cache"
mkdir -p "$cache/kit" "$cache/user"
image=xtrainer-astra:isaac-2.1.0
exec timeout --signal=TERM --kill-after=15s 300s docker run --rm --gpus all --network none \
  --name rm27-asm2-flight --shm-size=2g --memory=24g --cpus=12 --ulimit core=0 --init \
  --user "$(id -u):$(id -g)" --env HOME=/home/ubuntu --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N \
  --env PYTHONDONTWRITEBYTECODE=1 --env PYTHONUNBUFFERED=1 --env PYTHONPATH=/sf/src:/sf \
  --env OMP_NUM_THREADS=4 --env OPENBLAS_NUM_THREADS=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
  --tmpfs "/isaac-sim/kit/data:uid=$(id -u),gid=$(id -g),mode=0700" \
  --tmpfs "/isaac-sim/kit/logs:uid=$(id -u),gid=$(id -g),mode=0700" \
  --mount "type=bind,src=$cache/kit,dst=/isaac-sim/kit/cache" \
  --mount "type=bind,src=$cache/user,dst=/home/ubuntu/.cache" \
  --mount "type=bind,src=$project_dir,dst=/sf" --workdir /sf \
  --entrypoint /isaac-sim/python.sh "$image" -m rm27_asm2.flight_isaac "$@"
