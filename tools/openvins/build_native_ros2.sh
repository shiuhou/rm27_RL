#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /ws
cp /research-dependencies.tsv /ws/dependencies.tsv
g++ --version > /ws/compiler.txt
cmake --version > /ws/cmake.txt
find /upstream -type f -print0 | sort -z | xargs -0 sha256sum > /ws/upstream-files.sha256
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
set +e
/usr/bin/time -v -o /ws/build-resource.txt colcon --log-base /ws/log build \
  --base-paths /upstream --packages-up-to ov_msckf \
  --executor sequential --build-base /ws/build --install-base /ws/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON -DDISABLE_MATPLOTLIB=ON \
  > /ws/build.log 2>&1
build_status=$?
printf '%s\n' "$build_status" > /ws/build.exit
if [ "$build_status" -eq 0 ]; then
  find /ws/install -type f \( -name '*.so' -o -name run_subscribe_msckf \) -print0 | sort -z | xargs -0 sha256sum > /ws/binaries.sha256
fi
exit "$build_status"
