#!/usr/bin/env bash
# Private-LAN Isaac playback of a recorded ASM2 free-flight run.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
container=rm27-asm2-flight-viewer
run_dir="$project_dir/.agents/runs/asm2-flight/viewer"

if [[ ${1:-} == --stop ]]; then
  owner=$(docker inspect --format '{{index .Config.Labels "rm27.asm2.viewer"}}' "$container" 2>/dev/null || true)
  [[ "$owner" == "$project_dir" ]] || { echo 'No project-owned ASM2 flight viewer is running.' >&2; exit 1; }
  exec docker stop --timeout 12 "$container"
fi
if [[ ${1:-} == --help || $# == 0 ]]; then
  cat <<'EOF'
Usage: ./run_asm2_flight_viewer.sh REPLAY.npz [--detach] [--paused] [--episode NAME] [--bind PRIVATE_IP]
       ./run_asm2_flight_viewer.sh REPLAY.npz --self-test [--episode NAME]
       ./run_asm2_flight_viewer.sh REPLAY.npz --self-test-output NEW_PNG [--episode NAME]
       ./run_asm2_flight_viewer.sh --stop

Use the printed private IP in NVIDIA Isaac Sim WebRTC Streaming Client 1.0.6.
The selected address needs TCP 49100 and UDP 47998 available.
--self-test renders a screenshot and exits without publishing ports.
EOF
  exit 0
fi

replay=$(realpath -e -- "$1")
shift
[[ "$replay" == "$project_dir/"* && -f "$replay" ]] || { echo 'Choose a replay inside this project.' >&2; exit 1; }
detach=()
args=("/sf/${replay#"$project_dir/"}")
ports=()
test=0
snapshot_mount=()
bind_ip=${RM27_VIEWER_BIND:-}
while (($#)); do
  case "$1" in
    --detach) detach=(-d); shift ;;
    --bind) bind_ip=${2:?--bind requires a private IPv4 address}; shift 2 ;;
    --self-test) test=1; args+=(--self-test /sf/.agents/runs/asm2-flight/viewer/check.png); shift ;;
    --self-test-output)
      snapshot=$(realpath -m -- "${2:?--self-test-output requires a fresh PNG path}")
      [[ "$snapshot" == "$project_dir/.agents/runs/"*.png && ! -e "$snapshot" ]] || {
        echo 'Choose an unused PNG path under .agents/runs/.' >&2; exit 1;
      }
      snapshot_dir=$(dirname -- "$snapshot")
      mkdir -p "$snapshot_dir"
      snapshot_mount=(--mount "type=bind,src=$snapshot_dir,dst=/sf/${snapshot_dir#"$project_dir/"}")
      test=1; args+=(--self-test "/sf/${snapshot#"$project_dir/"}"); shift 2 ;;
    *) args+=("$1"); shift ;;
  esac
done
if ((test == 0)); then
  bind_ip=$(python3 - "$bind_ip" <<'PY'
import ipaddress
import json
import subprocess
import sys

networks = tuple(map(ipaddress.ip_network, ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16', '127.0.0.0/8')))
if sys.argv[1]:
    address = ipaddress.IPv4Address(sys.argv[1])
    if not any(address in network for network in networks):
        raise SystemExit('Use a private LAN/VPN or loopback IPv4 address.')
    print(address)
else:
    rows = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address', 'show', 'scope', 'global']))
    candidates = (ipaddress.IPv4Address(info['local']) for row in rows
                  if not row['ifname'].startswith(('docker', 'veth', 'br-')) for info in row['addr_info'])
    print(next((address for address in candidates if any(address in network for network in networks)), '127.0.0.1'))
PY
)
  ports=(--publish "$bind_ip:49100:49100/tcp" --publish "$bind_ip:47998:47998/udp")
  echo "ASM2 private WebRTC: $bind_ip (TCP 49100 / UDP 47998)"
else
  bind_ip=127.0.0.1
fi
if docker container inspect "$container" >/dev/null 2>&1; then
  echo 'ASM2 viewer already exists. Stop it with ./run_asm2_flight_viewer.sh --stop.' >&2
  exit 1
fi
mkdir -p "$run_dir/cache/kit" "$run_dir/cache/data" "$run_dir/cache/ov" "$run_dir/cache/user"
image=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["isaac_image"])' "$project_dir/configs/rm27_asm2_collision.json")
exec docker run --rm "${detach[@]}" --name "$container" --label "rm27.asm2.viewer=$project_dir" \
  --gpus all --shm-size=2g --init --stop-timeout=12 --ulimit core=0 --log-opt max-size=4m --log-opt max-file=1 \
  --user "$(id -u):$(id -g)" --env HOME=/home/ubuntu --env PUBLIC_IP="$bind_ip" \
  --env NVIDIA_DRIVER_CAPABILITIES=all --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N \
  --env PYTHONDONTWRITEBYTECODE=1 --env PYTHONUNBUFFERED=1 --env PYTHONPATH=/sf/src \
  --env OMP_NUM_THREADS=4 --env OPENBLAS_NUM_THREADS=1 \
  --tmpfs "/isaac-sim/kit/logs:uid=$(id -u),gid=$(id -g),mode=0700" \
  --mount "type=bind,src=$run_dir/cache/kit,dst=/isaac-sim/kit/cache" \
  --mount "type=bind,src=$run_dir/cache/data,dst=/isaac-sim/kit/data" \
  --mount "type=bind,src=$run_dir/cache/ov,dst=/home/ubuntu/.local/share/ov" \
  --mount "type=bind,src=$run_dir/cache/user,dst=/home/ubuntu/.cache" \
  --mount "type=bind,src=$project_dir,dst=/sf,readonly" \
  --mount "type=bind,src=$run_dir,dst=/sf/.agents/runs/asm2-flight/viewer" \
  --workdir /tmp "${ports[@]}" "${snapshot_mount[@]}" --entrypoint /isaac-sim/python.sh "$image" \
  -m rm27_asm2.flight_viewer "${args[@]}"
