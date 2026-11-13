#!/usr/bin/env bash
# Simulation-only ASM2 terminal pilot, gated by a pinned contact preflight.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$project_dir/docker/isaac-runtime/launch.sh"
cache="$project_dir/.agents/runs/asm2-contact/cache"
run_dir="$project_dir/.agents/runs/asm2-terminal/phase4-pilot"
case "${1:-}" in
  workflow)
    if [[ ${2:-} != --workflow-config || -z ${3:-} ]]; then
      echo 'workflow requires --workflow-config CONFIG' >&2
      exit 2
    fi
    readarray -t workflow_launch < <(python3 - "$project_dir" "$3" <<'PYCONFIG'
import json
import math
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve()
config = json.loads(Path(sys.argv[2]).read_text())
output = (root / config["output"]).resolve()
if not output.is_relative_to(root / ".agents/runs") or output == root / ".agents/runs":
    raise ValueError("Workflow output must be a run directory in the project")
print(output)
if config["schema"] == "rm27_asm2_rotation_bridge_instrumented_v1":
    print("transition-only")
else:
    budget = config["budget"]
    evaluations = 2 + math.ceil(budget["per_arm_transitions"] / config["evaluate_every"])
    print(math.ceil(budget["per_arm_learning_s"] + evaluations * config["evaluation_wait_s"] + 300))
PYCONFIG
)
    if [[ ${#workflow_launch[@]} != 2 ]]; then exit 2; fi
    run_dir=${workflow_launch[0]}
    timeout_s=${workflow_launch[1]}
    if [[ $timeout_s == transition-only ]]; then
      command=(-m rm27_asm2.rotation_bridge_experiment)
    else
      command=(-m rm27_asm2.terminal_training)
    fi
    container=rm27-asm2-terminal-workflow ;;
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
  rotation-continuation)
    run_dir="$project_dir/.agents/runs/asm2-terminal/phase5a"
    command=(/sf/.agents/runs/asm2-terminal/phase5a/continue_rotation.py)
    timeout_s=2600; container=rm27-asm2-terminal-pilot ;;
  rotation-recovery)
    run_dir="$project_dir/.agents/runs/asm2-terminal/phase5ar"
    command=(/sf/.agents/runs/asm2-terminal/phase5ar/train_recovery.py)
    timeout_s=7400; container=rm27-asm2-terminal-pilot ;;
  rotation-outcome)
    run_dir="$project_dir/.agents/runs/asm2-terminal/phase5-campaign"
    command=(/sf/.agents/runs/asm2-terminal/phase5-campaign/train_outcome.py)
    timeout_s=7600; container=rm27-asm2-terminal-pilot ;;
  rotation-margin)
    run_dir="$project_dir/.agents/runs/asm2-terminal/phase5-campaign"
    command=(/sf/.agents/runs/asm2-terminal/phase5-campaign/train_campaign.py margin)
    timeout_s=4900; container=rm27-asm2-terminal-pilot ;;
  *) echo "Usage: $0 {workflow|preflight|train|recovery-check|matched-contact|braking-pilot|braking-continuation|rotation-continuation|rotation-recovery|rotation-margin|rotation-outcome} [arguments]" >&2; exit 2 ;;
esac
shift
mkdir -p "$cache/kit" "$cache/user" "$run_dir"
runner=()
if [[ $timeout_s != transition-only ]]; then
  runner=(timeout --signal=TERM --kill-after=15s "${timeout_s}s")
fi
exec "${runner[@]}" "${docker_command[@]}" run --rm "${container_home_mount[@]}" --gpus all --network none \
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
