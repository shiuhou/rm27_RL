#!/usr/bin/env bash
set -euo pipefail
release=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ $# != 2 || ! $1 =~ ^[01]$ ]]; then
    echo "Usage: bash run_isaac_gate.sh {0|1} OUTPUT_DIRECTORY" >&2
    exit 2
fi
layout=$1
mkdir -p "$2"
output=$(realpath "$2")
if [[ $output == "$release" || $output == "$release/"* ]]; then
    echo "Choose an output directory outside the sealed release." >&2
    exit 2
fi
cache="$output/cache"
mkdir -p "$cache/kit" "$cache/user"
image=${ASM2_ISAAC_IMAGE:-xtrainer-astra:isaac-2.1.0}
exec timeout --signal=TERM --kill-after=15s 240s docker run --rm --gpus all --network none \
  --shm-size=2g --memory=48g --cpus=16 --ulimit core=0 --init \
  --user "$(id -u):$(id -g)" --env HOME=/home/ubuntu --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N \
  --env PYTHONDONTWRITEBYTECODE=1 --env PYTHONUNBUFFERED=1 --env PYTHONPATH=/release/src:/release \
  --env OMP_NUM_THREADS=4 --env OPENBLAS_NUM_THREADS=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
  --tmpfs "/isaac-sim/kit/data:uid=$(id -u),gid=$(id -g),mode=0700" \
  --tmpfs "/isaac-sim/kit/logs:uid=$(id -u),gid=$(id -g),mode=0700" \
  --mount "type=bind,src=$cache/kit,dst=/isaac-sim/kit/cache" \
  --mount "type=bind,src=$cache/user,dst=/home/ubuntu/.cache" \
  --mount "type=bind,src=$release,dst=/release,readonly" \
  --mount "type=bind,src=$output,dst=/results" --workdir /release \
  --entrypoint /isaac-sim/python.sh "$image" -m rm27_asm2.release_gate \
  --backend isaac --layout "$layout" --output "/results/isaac-layout$layout.json"
