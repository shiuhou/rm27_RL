#!/usr/bin/env bash
# Isolated collision diagnostics; never launches a learner or mounts W&B credentials.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run_dir="$project_dir/.agents/runs/asm2-collision"
mkdir -p "$run_dir/cache/kit" "$run_dir/cache/user"
image=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["isaac_image"])' "$project_dir/configs/rm27_asm2_collision.json")
exec timeout --signal=TERM --kill-after=15s 600s docker run --rm --gpus all --network none \
  --name rm27-asm2-collision --label "rm27.asm2.owner=$project_dir" \
  --shm-size=2g --memory=48g --cpus=16 --ulimit core=0 --init \
  --log-opt max-size=8m --log-opt max-file=1 --user "$(id -u):$(id -g)" \
  --env HOME=/home/ubuntu --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N \
  --env PYTHONDONTWRITEBYTECODE=1 --env PYTHONUNBUFFERED=1 --env PYTHONPATH=/sf/src \
  --env OMP_NUM_THREADS=4 --env OPENBLAS_NUM_THREADS=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
  --tmpfs "/isaac-sim/kit/data:uid=$(id -u),gid=$(id -g),mode=0700" \
  --tmpfs "/isaac-sim/kit/logs:uid=$(id -u),gid=$(id -g),mode=0700" \
  --mount "type=bind,src=$run_dir/cache/kit,dst=/isaac-sim/kit/cache" \
  --mount "type=bind,src=$run_dir/cache/user,dst=/home/ubuntu/.cache" \
  --mount "type=bind,src=$project_dir,dst=/sf" --workdir /sf \
  --entrypoint /isaac-sim/python.sh "$image" -m rm27_asm2.isaac "$@"
