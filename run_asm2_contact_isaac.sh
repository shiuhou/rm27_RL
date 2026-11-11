#!/usr/bin/env bash
# Isolated matched contact diagnostics on the existing GPU simulation image.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cache="$project_dir/.agents/runs/asm2-contact/cache"
mkdir -p "$cache/kit" "$cache/user"
module=rm27_asm2.contact_isaac
extra_mounts=()
if [[ ${1:-} == --handover ]]; then
  shift
  module=rm27_asm2.handover_isaac
  mkdir -p "$project_dir/.agents/runs/asm2-handover"
  extra_mounts=(--mount "type=bind,src=$project_dir/.agents/runs/asm2-handover,dst=/sf/.agents/runs/asm2-handover")
fi
image=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["isaac_image"])' "$project_dir/configs/rm27_asm2_collision.json")
exec timeout --signal=TERM --kill-after=15s 240s docker run --rm --gpus all --network none \
  --name rm27-asm2-contact-isaac --shm-size=2g --memory=48g --cpus=16 --ulimit core=0 --init \
  --user "$(id -u):$(id -g)" --env HOME=/home/ubuntu --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N \
  --env PYTHONDONTWRITEBYTECODE=1 --env PYTHONUNBUFFERED=1 --env PYTHONPATH=/sf/src:/sf \
  --env OMP_NUM_THREADS=4 --env OPENBLAS_NUM_THREADS=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
  --tmpfs "/isaac-sim/kit/data:uid=$(id -u),gid=$(id -g),mode=0700" \
  --tmpfs "/isaac-sim/kit/logs:uid=$(id -u),gid=$(id -g),mode=0700" \
  --mount "type=bind,src=$cache/kit,dst=/isaac-sim/kit/cache" \
  --mount "type=bind,src=$cache/user,dst=/home/ubuntu/.cache" \
  --mount "type=bind,src=$project_dir,dst=/sf,readonly" \
  --mount "type=bind,src=$project_dir/.agents/runs/asm2-contact,dst=/sf/.agents/runs/asm2-contact" --workdir /sf \
  "${extra_mounts[@]}" --entrypoint /isaac-sim/python.sh "$image" -m "$module" "$@"
