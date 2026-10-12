#!/usr/bin/env bash
set -euo pipefail

script_dir=$(CDPATH='' cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(CDPATH='' cd -- "$script_dir/../../.." && pwd)

usage() {
    cat >&2 <<'USAGE'
Usage:
  run.sh build [IMAGE]                         # default: rm27-contact-rl:i3-cpu
  run.sh run IMAGE RESULTS_ROOT RUN_ID [--source] [--check]
  run.sh train IMAGE RESULTS_ROOT RUN_ID [--source]
  run.sh evaluate IMAGE RESULTS_ROOT RUN_ID CHECKPOINT_RELATIVE_TO_RESULTS_ROOT [--source]
--source: read-only mount of this checkout at /workspace; uses image dependencies.
--check: import/config/reset/step smoke check, then the same single CLI baseline.
RESULTS_ROOT is a dedicated host results directory; RUN_ID must be new.
train uses the packaged PPO configuration and its fixed training budget.
evaluate loads a checkpoint directory under RESULTS_ROOT, read-only in use.
USAGE
    exit 1
}

# Capture Git on the host: a worktree's .git pointer is unusable in the image.
git_source() {
    python3 - "$repo_root" <<'PY'
import json
from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1])
def git(*args):
    return subprocess.run(['git', *args], cwd=root, text=True, capture_output=True,
                          check=True).stdout.strip()
status = git('status', '--porcelain', '--untracked-files=normal')
print(json.dumps(dict(git_commit=git('rev-parse', 'HEAD'), git_dirty=bool(status),
                      git_status=status.splitlines()), separators=(',', ':')))
PY
}

[[ $# -ge 1 ]] || usage
operation=$1
shift
case "$operation" in
    build)
        [[ $# -le 1 ]] || usage
        image=${1:-rm27-contact-rl:i3-cpu}
        source_identity=$(git_source)
        exec docker build --platform linux/amd64 \
            --build-arg "SOURCE_GIT_JSON=$source_identity" \
            --file "$script_dir/Dockerfile" --tag "$image" "$repo_root"
        ;;
    run|train|evaluate)
        [[ $# -ge 3 ]] || usage
        image=$1
        results_root=$2
        run_id=$3
        shift 3
        [[ "$run_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || usage
        checkpoint_relative=''
        if [[ "$operation" == evaluate ]]; then
            [[ $# -ge 1 ]] || usage
            checkpoint_relative=$1
            shift
            [[ -n "$checkpoint_relative" && "$checkpoint_relative" != /* ]] || usage
        fi
        source_mount=false
        check=false
        for option in "$@"; do
            case "$option" in
                --source) source_mount=true ;;
                --check) [[ "$operation" == run ]] || usage; check=true ;;
                *) usage ;;
            esac
        done
        # Resolve the tag once, then run that immutable local image identity.
        image_id=$(docker image inspect --format '{{.Id}}' "$image")
        mkdir -p -- "$results_root"
        results_root=$(realpath -e -- "$results_root")
        [[ "$results_root" != / && ! -e "$results_root/$run_id" ]] || {
            echo 'Use a dedicated results directory and a new run identifier.' >&2
            exit 1
        }
        checkpoint_container=''
        if [[ "$operation" == evaluate ]]; then
            checkpoint_host=$(realpath -e -- "$results_root/$checkpoint_relative")
            [[ "$checkpoint_host" == "$results_root/"* && -d "$checkpoint_host" ]] || {
                echo 'Checkpoint must be a directory within RESULTS_ROOT.' >&2
                exit 1
            }
            for checkpoint_file in model.zip normalize.pkl checkpoint.json; do
                [[ -f "$checkpoint_host/$checkpoint_file" ]] || {
                    echo "Missing checkpoint file: $checkpoint_file" >&2
                    exit 1
                }
                checkpoint_file_host=$(realpath -e -- "$checkpoint_host/$checkpoint_file")
                [[ "$checkpoint_file_host" == "$results_root/"* ]] || {
                    echo 'Checkpoint file cannot resolve outside RESULTS_ROOT.' >&2
                    exit 1
                }
            done
            checkpoint_container="/data/${checkpoint_host#"$results_root/"}"
        fi
        options=(--rm --init --network none --read-only --cap-drop ALL
                 --security-opt no-new-privileges --user "$(id -u):$(id -g)"
                 --tmpfs '/tmp:rw,nosuid,nodev,size=128m'
                 --mount "type=bind,src=$results_root,dst=/data"
                 --env "CONTACT_RL_IMAGE_ID=$image_id"
                 --env "CONTACT_RL_IMAGE_REFERENCE=$image"
                 --env CONTACT_RL_EXECUTION_SOURCE=IMAGE)
        if "$source_mount"; then
            options+=(--mount "type=bind,src=$repo_root,dst=/workspace,readonly"
                      --env "CONTACT_RL_GIT_SOURCE=$(git_source)"
                      --env CONTACT_RL_EXECUTION_SOURCE=BIND_MOUNT)
        fi
        case "$operation" in
            run) command=(baseline --output "/data/$run_id") ;;
            train) command=(train --output "/data/$run_id") ;;
            evaluate) command=(evaluate --checkpoint "$checkpoint_container" --output "/data/$run_id") ;;
        esac
        if "$check"; then
            check_command=$(cat <<'CHECK'
set -eu
python - <<'PY'
import json
from pathlib import Path
import sys
import numpy as np
from rm27_contact_rl import ContactRecoveryEnv
from rm27_contact_rl.rm27_adapter import DEFAULT_CONFIG

assert sys.prefix == '/opt/venv'
assert Path(DEFAULT_CONFIG).is_file()
env = ContactRecoveryEnv()
try:
    obs, info = env.reset(seed=0)
    assert env.observation_space.contains(obs) and obs.shape == (26,)
    assert np.isclose(env.plant.info['mass_kg'], 0.15)
    obs, reward, terminated, truncated, info = env.step(env.previous_action_ctbr.copy())
    assert env.observation_space.contains(obs)
    assert info['physics_substeps'] == 5 and np.isclose(info['time_s'], 0.005)
    assert not terminated and not truncated
    print(json.dumps(dict(container_smoke='PASS', python=sys.version.split()[0],
                          config=str(DEFAULT_CONFIG), time_s=info['time_s'])))
finally:
    env.close()
PY
exec python -m rm27_contact_rl "$@"
CHECK
)
            exec docker run "${options[@]}" --entrypoint /bin/sh \
                "$image_id" -c "$check_command" check "${command[@]}"
        fi
        exec docker run "${options[@]}" "$image_id" "${command[@]}"
        ;;
    *) usage ;;
esac
