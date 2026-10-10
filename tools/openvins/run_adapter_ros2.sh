#!/usr/bin/env bash
# Runtime adapter wrapper; same native command/config/rate as clean baseline.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=117
export ROS_LOCALHOST_ONLY=1
export HOME=/out
cd /out
test ! -e native.log
native_pid=''
recorder_pid=''
player_pid=''
observer_pid=''
cleanup() {
  if [ -n "$observer_pid" ] && kill -0 "$observer_pid" 2>/dev/null; then
    kill -TERM "$observer_pid" 2>/dev/null || true
    wait "$observer_pid" || true
  fi
  for task_pid in "$player_pid" "$native_pid"; do
    if [ -n "$task_pid" ] && kill -0 "$task_pid" 2>/dev/null; then
      kill -INT "$task_pid" 2>/dev/null || true
    fi
  done
  if [ -n "$recorder_pid" ] && kill -0 "$recorder_pid" 2>/dev/null; then
    kill -TERM "$recorder_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
export PYTHONPATH=/repo:${PYTHONPATH:-}
python3 /repo/tools/openvins/observe_ros2.py > /out/observer.log 2>&1 &
observer_pid=$!
for attempt in $(seq 1 30); do
  kill -0 "$observer_pid"
  [ -e /out/observer.ready ] && break
  sleep 1
done
test -e /out/observer.ready
/usr/bin/time -v -o /out/native-resource.txt \
  /ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf --ros-args \
  -p config_path:=/upstream/config/euroc_mav/estimator_config.yaml \
  -p use_stereo:=false -p max_cameras:=1 -p verbosity:=INFO \
  -p save_total_state:=true -p filepath_est:=/out/state_estimate.txt \
  -p filepath_std:=/out/state_deviation.txt \
  -p record_timing_information:=true -p record_timing_filepath:=/out/timing.txt \
  > /out/native.log 2>&1 &
meter_pid=$!
for attempt in $(seq 1 20); do
  native_pid=$(pgrep -P "$meter_pid" -f run_subscribe_msckf || true)
  [ -n "$native_pid" ] && break
  sleep 0.1
done
test -n "$native_pid"
ready=false
for attempt in $(seq 1 30); do
  kill -0 "$native_pid"
  ros2 topic info /cam0/image_raw > /out/camera-readiness.txt 2>&1 || true
  ros2 topic info /imu0 > /out/imu-readiness.txt 2>&1 || true
  if grep -q 'Subscription count: 2' /out/camera-readiness.txt && grep -q 'Subscription count: 2' /out/imu-readiness.txt; then
    ready=true
    break
  fi
  sleep 1
done
test "$ready" = true
ros2 topic info /cam0/image_raw --verbose > /out/camera-endpoints.txt
ros2 topic info /imu0 --verbose > /out/imu-endpoints.txt
ros2 bag record -o /out/online-output /poseimu /odomimu > /out/recorder.log 2>&1 &
recorder_pid=$!
ready=false
for attempt in $(seq 1 30); do
  kill -0 "$recorder_pid"
  ros2 topic info /poseimu > /out/output-readiness.txt 2>&1 || true
  ros2 topic info /odomimu > /out/odom-readiness.txt 2>&1 || true
  if grep -q 'Subscription count: 2' /out/output-readiness.txt && grep -q 'Subscription count: 2' /out/odom-readiness.txt; then
    ready=true
    break
  fi
  sleep 1
done
test "$ready" = true
ros2 bag play /data --rate 0.5 --disable-keyboard-controls --wait-for-all-acked 5000 > /out/player.log 2>&1 &
player_pid=$!
set +e
wait "$player_pid"
player_status=$?
set -e
printf '%s\n' "$player_status" > /out/player.exit
test "$player_status" -eq 0
player_pid=''
# Observe file stability after playback. This is not proof of IMU delivery.
stable=0
previous=''
for attempt in $(seq 1 30); do
  kill -0 "$native_pid"
  current=$(stat -c '%s' /out/state_estimate.txt)
  if [ "$current" = "$previous" ]; then stable=$((stable + 1)); else stable=0; fi
  [ "$stable" -ge 5 ] && break
  previous=$current
  sleep 1
done
kill -INT "$native_pid"
set +e
wait "$meter_pid"
native_status=$?
set -e
printf '%s\n' "$native_status" > /out/native.exit
native_pid=''
# ros2bag ignored SIGINT in this non-interactive background invocation.
# SIGTERM was observed to flush its cache, write metadata and return zero.
kill -TERM "$recorder_pid"
set +e
wait "$recorder_pid"
recorder_status=$?
set -e
printf '%s\n' "$recorder_status" > /out/recorder.exit
recorder_pid=''
test "$native_status" -eq 0
test "$recorder_status" -eq 0
test "$stable" -ge 5
kill -TERM "$observer_pid"
set +e
wait "$observer_pid"
observer_status=$?
set -e
printf '%s\n' "$observer_status" > /out/observer.exit
observer_pid=''
test "$observer_status" -eq 0
find /out -maxdepth 1 -type f ! -name run-artifacts.sha256 -print0 | sort -z | xargs -0 sha256sum > /out/run-artifacts.sha256
