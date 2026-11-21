#!/usr/bin/env bash
# Builds and tests the optional ROS 2 adapter (ros2/multirobotcore_ros).
# Must run in a sourced ROS 2 environment (e.g. source /opt/ros/jazzy/setup.bash).
#
#   1. builds the ROS-independent core library and installs it into
#      build/ros2-core-install (tests, demos and benchmarks disabled);
#   2. builds the adapter with colcon against that prefix;
#   3. runs the adapter's gtest and launch_testing suites.
#
# Usage: scripts/ci/ros2_build.sh
set -euo pipefail

: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
root=$(cd "$(dirname "$0")/../.." && pwd)
jobs=${BUILD_JOBS:-4}
core_build="$root/build/ros2-core"
core_prefix="$root/build/ros2-core-install"

cmake -S "$root" -B "$core_build" -DCMAKE_BUILD_TYPE=Release \
  -DMULTIROBOTCORE_BUILD_TESTS=OFF -DMULTIROBOTCORE_BUILD_APPS=OFF \
  -DMULTIROBOTCORE_BUILD_BENCHMARKS=OFF -DCMAKE_INSTALL_PREFIX="$core_prefix"
cmake --build "$core_build" --parallel "$jobs"
cmake --install "$core_build"

cd "$root/ros2"
colcon --log-base "$root/build/ros2-log" build \
  --build-base "$root/build/ros2-build" --install-base "$root/build/ros2-install" \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$core_prefix"
# shellcheck disable=SC1091
set +u  # ROS setup scripts reference unset variables
source "$root/build/ros2-install/setup.bash"
set -u
colcon --log-base "$root/build/ros2-log" test \
  --build-base "$root/build/ros2-build" --install-base "$root/build/ros2-install" \
  --event-handlers console_direct+
colcon test-result --test-result-base "$root/build/ros2-build" --verbose
