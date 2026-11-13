#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$project_dir/docker/isaac-runtime/launch.sh"
run_dir="$project_dir/.agents/runs/simpleflight"
mkdir -p "$run_dir/cache/kit"
auth_mount=()
if [[ -f "$HOME/.netrc" ]]; then
  auth_mount=(--mount "type=bind,src=$HOME/.netrc,dst=/home/ubuntu/.netrc,readonly")
fi
if [[ ${1:-} != -* ]]; then
  set -- -m simpleflight_lab "$@"
fi
exec "${docker_command[@]}" run --rm "${container_home_mount[@]}" --gpus all --shm-size=2g --init \
  --user "$(id -u):$(id -g)" --env HOME=/home/ubuntu \
  --tmpfs "/isaac-sim/kit/data:uid=$(id -u),gid=$(id -g),mode=0700" \
  --tmpfs "/isaac-sim/kit/logs:uid=$(id -u),gid=$(id -g),mode=0700" \
  --mount "type=bind,src=$run_dir/cache/kit,dst=/isaac-sim/kit/cache" \
  --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N --env PYTHONDONTWRITEBYTECODE=1 \
  --env PYTHONUNBUFFERED=1 --env MPLBACKEND=Agg --env OMP_NUM_THREADS=4 \
  --env SF_WANDB --env SF_WANDB_CONFIG --env SF_WANDB_MODE --env SF_WANDB_GROUP \
  --env WANDB_BASE_URL --env WANDB_PROJECT --env WANDB_ENTITY --env WANDB_API_KEY \
  "${auth_mount[@]}" \
  --mount "type=bind,src=$project_dir,dst=/sf" \
  --mount "type=bind,src=$run_dir/cache,dst=/home/ubuntu/.cache" \
  --workdir /sf --entrypoint /isaac-sim/python.sh \
  "$image" "$@"
