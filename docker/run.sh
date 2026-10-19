#!/usr/bin/env bash
set -euo pipefail

script_dir=$(CDPATH='' cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(CDPATH='' cd -- "$script_dir/.." && pwd)

usage() {
    cat >&2 <<'USAGE'
Usage:
  run.sh build [IMAGE]                         # default: rm27-contact-rl:separated-cpu
  run.sh run IMAGE RESULTS_ROOT RUN_ID [--source] [--check]
  run.sh train IMAGE RESULTS_ROOT RUN_ID [--source] [--config CONFIG] [--wandb]
  run.sh arrival-baseline IMAGE RESULTS_ROOT RUN_ID [--source] [--config CONFIG]
  run.sh evaluate IMAGE RESULTS_ROOT RUN_ID CHECKPOINT_DIRECTORY [--source]
--source: read-only mount of this checkout at /workspace; uses image dependencies.
--check: import/config/reset/step smoke check, then the same single CLI baseline.
--wandb: train only; log to Weights & Biases. Enables container networking and passes
    WANDB_API_KEY from ~/.config/rm27/wandb.env, else from the host `wandb login` (~/.netrc).
RESULTS_ROOT is a dedicated host results directory; RUN_ID must be new.
train defaults to the packaged Contact PPO configuration; --config selects another
packaged configuration using its /workspace-relative path or absolute container path.
arrival-baseline defaults to the packaged Agile Terminal Arrival configuration.
evaluate mounts CHECKPOINT_DIRECTORY separately and read-only at /checkpoint.
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
        image=${1:-rm27-contact-rl:separated-cpu}
        source_identity=$(git_source)
        exec docker build --platform linux/amd64 \
            --build-arg "SOURCE_GIT_JSON=$source_identity" \
            --file "$script_dir/Dockerfile" --tag "$image" "$repo_root"
        ;;
    run|train|evaluate|arrival-baseline)
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
            [[ -n "$checkpoint_relative" ]] || usage
        fi
        source_mount=false
        check=false
        wandb=false
        config_container=''
        while [[ $# -gt 0 ]]; do
            case "$1" in
                --source) source_mount=true; shift ;;
                --check) [[ "$operation" == run ]] || usage; check=true; shift ;;
                --wandb) [[ "$operation" == train ]] || usage; wandb=true; shift ;;
                --config)
                    [[ "$operation" == train || "$operation" == arrival-baseline ]] || usage
                    [[ $# -ge 2 && -z "$config_container" && -n "$2" ]] || usage
                    case "$2" in
                        /*) config_container=$2 ;;
                        *) config_container="/workspace/$2" ;;
                    esac
                    shift 2
                    ;;
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
            checkpoint_host=$(realpath -e -- "$checkpoint_relative")
            [[ -d "$checkpoint_host" ]] || usage
            for checkpoint_file in model.zip normalize.pkl checkpoint.json; do
                [[ -f "$checkpoint_host/$checkpoint_file" && ! -L "$checkpoint_host/$checkpoint_file" ]] || {
                    echo "Expected ordinary checkpoint file: $checkpoint_file" >&2
                    exit 1
                }
            done
            [[ "$results_root/" != "$checkpoint_host/"* && "$checkpoint_host/" != "$results_root/"* ]] || {
                echo 'Checkpoint input and writable results must be separate directories.' >&2
                exit 1
            }
            checkpoint_container=/checkpoint
        fi
        network=none
        if "$wandb"; then
            # Credentials: the private env file wins, else the host login; never on the command line.
            key_file="$HOME/.config/rm27/wandb.env"
            if [[ -f "$key_file" ]]; then
                file_key=$(sed -n 's/^WANDB_API_KEY=//p' "$key_file" | tail -n 1)
                file_entity=$(sed -n 's/^WANDB_ENTITY=//p' "$key_file" | tail -n 1)
                [[ -n "$file_key" ]] && WANDB_API_KEY=$file_key
                [[ -n "$file_entity" ]] && WANDB_ENTITY=$file_entity
            fi
            if [[ -z "${WANDB_API_KEY:-}" ]]; then
                WANDB_API_KEY=$(python3 -c 'import netrc; a = netrc.netrc().authenticators("api.wandb.ai"); print(a[2] if a else "")' 2>/dev/null || true)
            fi
            [[ -n "${WANDB_API_KEY:-}" ]] || { echo 'No W&B key: run `wandb login` or fill ~/.config/rm27/wandb.env.' >&2; exit 1; }
            export WANDB_API_KEY WANDB_ENTITY="${WANDB_ENTITY:-}" RM27_WANDB_PROJECT="${RM27_WANDB_PROJECT:-rm27-agile-flight-rl}"
            network=bridge
        fi
        options=(--rm --init --network "$network" --read-only --cap-drop ALL
                 --security-opt no-new-privileges --user "$(id -u):$(id -g)"
                 --tmpfs '/tmp:rw,nosuid,nodev,size=128m'
                 --mount "type=bind,src=$results_root,dst=/data"
                 --env "CONTACT_RL_IMAGE_ID=$image_id"
                 --env "CONTACT_RL_IMAGE_REFERENCE=$image"
                 --env CONTACT_RL_EXECUTION_SOURCE=IMAGE)
        if [[ "$operation" == evaluate ]]; then
            options+=(--mount "type=bind,src=$checkpoint_host,dst=/checkpoint,readonly")
        fi
        if "$wandb"; then
            options+=(--env WANDB_API_KEY --env WANDB_ENTITY --env RM27_WANDB_PROJECT --env RM27_WANDB=1
                      --env WANDB_CONFIG_DIR=/tmp/wandb-config --env WANDB_CACHE_DIR=/tmp/wandb-cache
                      --env WANDB_DATA_DIR=/tmp/wandb-data)
        fi
        if "$source_mount"; then
            options+=(--mount "type=bind,src=$repo_root,dst=/workspace,readonly"
                      --env "CONTACT_RL_GIT_SOURCE=$(git_source)"
                      --env CONTACT_RL_EXECUTION_SOURCE=BIND_MOUNT)
        fi
        case "$operation" in
            run) command=(baseline --output "/data/$run_id") ;;
            train) command=(train --output "/data/$run_id") ;;
            arrival-baseline) command=(arrival-baseline --output "/data/$run_id") ;;
            evaluate) command=(evaluate --checkpoint "$checkpoint_container" --output "/data/$run_id") ;;
        esac
        if [[ -n "$config_container" ]]; then
            command+=(--config "$config_container")
        fi
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
