#!/usr/bin/env bash
# Private-LAN Isaac Sim viewer; independent of every learner/training process.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$project_dir/docker/isaac-runtime/launch.sh"
container=sf-rm27-visual-replay
run_dir="$project_dir/.agents/runs/rm27-viewer"
if [[ ${1:-} == --stop ]]; then
  owner=$("${docker_command[@]}" inspect --format '{{index .Config.Labels "sf.rm27.viewer"}}' "$container" 2>/dev/null || true)
  [[ "$owner" == "$project_dir" ]] || { echo 'No SF-owned RM27 viewer is running.'; exit 1; }
  "${docker_command[@]}" stop --timeout 12 "$container"
  for attempt in {1..50}; do
    "${docker_command[@]}" container inspect "$container" >/dev/null 2>&1 || exit 0
    sleep .1
  done
  echo "Container stopped; Docker is still removing it." >&2
  exit 1
fi
if [[ ${1:-} == --help ]]; then
  cat <<'EOF'
Usage: ./run_visual_replay.sh [--detach] [--bind PRIVATE_IP] [--demo contact|witness|failure] [--paused]
       ./run_visual_replay.sh --self-test
       ./run_visual_replay.sh --stop
Default: private-LAN WebRTC on TCP 49100 and UDP 47998. No public bind accepted.
Connect NVIDIA Isaac Sim WebRTC Streaming Client 1.0.6 to the printed address.
Use the RM27 Replay panel to switch demonstrations/modes, pause, restart,
choose 0.25x/0.5x/1x and follow/fixed/detail cameras. Free camera uses Alt+drag.
--self-test performs bounded rendering/inference checks, saves verification,
and exits. It does not run PPO or modify authoritative evaluation records.
EOF
  exit 0
fi
detach=()
viewer_args=()
stream=2
bind_ip=${RM27_VIEWER_BIND:-}
while (($#)); do
  case "$1" in
    --detach) detach=(-d); shift ;;
    --bind) bind_ip=${2:?--bind requires a private IPv4 address}; shift 2 ;;
    --self-test) viewer_args+=(--self-test /sf/.agents/runs/rm27-viewer/verification); shift ;;
    *) viewer_args+=("$1"); shift ;;
  esac
done
if [[ -z "$bind_ip" ]]; then
  bind_ip=$(python3 - <<'PY'
import ipaddress,json,subprocess
rows=json.loads(subprocess.check_output(['ip','-j','-4','address','show','scope','global']))
networks=[ipaddress.ip_network(n) for n in ('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16')]
for row in rows:
    if row['ifname'].startswith(('docker','veth','br-')): continue
    for info in row['addr_info']:
        address=ipaddress.ip_address(info['local'])
        if any(address in net for net in networks):
            print(address); raise SystemExit
print('127.0.0.1')
PY
)
fi
python3 - "$bind_ip" <<'PY'
import ipaddress,sys
address=ipaddress.IPv4Address(sys.argv[1])
if not any(address in ipaddress.ip_network(n) for n in ('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16','127.0.0.0/8')):
    raise SystemExit('Refusing a public/non-RFC1918 viewer bind; use private LAN or VPN.')
PY
if "${docker_command[@]}" container inspect "$container" >/dev/null 2>&1; then
  echo "Viewer container already exists. Use ./run_visual_replay.sh --stop before relaunch." >&2
  exit 1
fi
mkdir -p "$run_dir/cache/kit" "$run_dir/cache/user" "$run_dir/cache/ov" "$run_dir/cache/data"
ports=()
if ((stream)); then
  ports=(--publish "$bind_ip:49100:49100/tcp" --publish "$bind_ip:47998:47998/udp")
  echo "RM27 private WebRTC: $bind_ip (TCP49100 / UDP47998)"
fi
exec "${docker_command[@]}" run --rm "${container_home_mount[@]}" "${detach[@]}" --name "$container" --label "sf.rm27.viewer=$project_dir" \
  --gpus all --shm-size=2g --init --log-opt max-size=4m --log-opt max-file=1 \
  --user "$(id -u):$(id -g)" --env HOME=/home/ubuntu \
  --env NVIDIA_DRIVER_CAPABILITIES=all --env PUBLIC_IP="$bind_ip" \
  --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N --env PYTHONDONTWRITEBYTECODE=1 \
  --env PYTHONUNBUFFERED=1 --env OMP_NUM_THREADS=4 --env OPENBLAS_NUM_THREADS=1 \
  --tmpfs "/isaac-sim/kit/logs:uid=$(id -u),gid=$(id -g),mode=0700" \
  --mount "type=bind,src=$run_dir/cache/kit,dst=/isaac-sim/kit/cache" \
  --mount "type=bind,src=$run_dir/cache/data,dst=/isaac-sim/kit/data" \
  --mount "type=bind,src=$run_dir/cache/ov,dst=/home/ubuntu/.local/share/ov" \
  --mount "type=bind,src=$run_dir/cache/user,dst=/home/ubuntu/.cache" \
  --mount "type=bind,src=$project_dir,dst=/sf" "${ports[@]}" \
  --workdir /sf --entrypoint /isaac-sim/python.sh "$image" \
  -m simpleflight_lab.visual_replay --livestream "$stream" "${viewer_args[@]}"
