#!/usr/bin/env bash
# Simulation-only ASM2 terminal pilot, gated by a pinned contact preflight.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cache="$project_dir/.agents/runs/asm2-contact/cache"
run_dir="$project_dir/.agents/runs/asm2-terminal/phase4-pilot"
case "${1:-}" in
  preflight) command=(-m rm27_asm2.terminal_preflight); timeout_s=240; container=rm27-asm2-terminal-check ;;
  train) command=(-m rm27_asm2.terminal_training); timeout_s=1900; container=rm27-asm2-terminal-pilot ;;
  recovery-check)
    run_dir="$project_dir/.agents/runs/asm2-terminal/phase4r"
    command=(/sf/.agents/runs/asm2-terminal/phase4r/contact_gate.py)
    timeout_s=180; container=rm27-asm2-terminal-check ;;
  matched-contact)
    run_dir="$project_dir/.agents/runs/asm2-terminal/phase4r2"
    command=(/sf/.agents/runs/asm2-terminal/phase4r2/matched_contact.py)
    timeout_s=150; container=rm27-asm2-terminal-check ;;
  braking-pilot)
    run_dir="$project_dir/.agents/runs/asm2-terminal/phase4r2"
    command=(/sf/.agents/runs/asm2-terminal/phase4r2/braking_pilot.py)
    timeout_s=1300; container=rm27-asm2-terminal-pilot ;;
  braking-continuation)
    run_dir="$project_dir/.agents/runs/asm2-terminal/phase4-meaningful"
    command=(/sf/.agents/runs/asm2-terminal/phase4-meaningful/continue_braking.py)
    timeout_s=7600; container=rm27-asm2-terminal-pilot ;;
  *) echo "Usage: $0 {preflight|train|recovery-check|matched-contact|braking-pilot|braking-continuation} [arguments]" >&2; exit 2 ;;
esac
shift
mkdir -p "$cache/kit" "$cache/user" "$run_dir"
image=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["isaac_image"])' "$project_dir/configs/rm27_asm2_collision.json")
exec timeout --signal=TERM --kill-after=15s "${timeout_s}s" docker run --rm --gpus all --network none \
  --name "$container" --shm-size=2g --memory=48g --cpus=16 --ulimit core=0 --init \
  --user "$(id -u):$(id -g)" --env HOME=/home/ubuntu --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N \
  --env PYTHONDONTWRITEBYTECODE=1 --env PYTHONUNBUFFERED=1 --env PYTHONPATH=/sf/src:/sf \
  --env OMP_NUM_THREADS=4 --env OPENBLAS_NUM_THREADS=1 --env NVIDIA_DRIVER_CAPABILITIES=all \
  --tmpfs "/isaac-sim/kit/data:uid=$(id -u),gid=$(id -g),mode=0700" \
  --tmpfs "/isaac-sim/kit/logs:uid=$(id -u),gid=$(id -g),mode=0700" \
  --mount "type=bind,src=$cache/kit,dst=/isaac-sim/kit/cache" \
  --mount "type=bind,src=$cache/user,dst=/home/ubuntu/.cache" \
  --mount "type=bind,src=$project_dir,dst=/sf,readonly" \
  --mount "type=bind,src=$run_dir,dst=/sf/${run_dir#"$project_dir/"}" \
  --workdir /sf --entrypoint /isaac-sim/python.sh "$image" "${command[@]}" "$@"
