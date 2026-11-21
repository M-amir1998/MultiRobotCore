#!/usr/bin/env bash
# Linux verification in a disposable ROS 2 Jazzy container (Ubuntu 24.04,
# GCC 13): builds the core with warnings as errors and runs every test suite,
# optionally the ASan/UBSan build (clang) and the ROS 2 adapter.
#
# Usage: scripts/ci/docker_linux.sh [core|sanitize|ros2|all]
set -euo pipefail

mode=${1:-all}
image=${ROS_IMAGE:-ros:jazzy-ros-base}
root=$(cd "$(dirname "$0")/../.." && pwd)
jobs=${BUILD_JOBS:-4}

# On Windows hosts docker needs a Windows-style path for the bind mount.
mount_src="$root"
if command -v cygpath > /dev/null 2>&1; then
  mount_src=$(cygpath -w "$root")
fi

read -r -d '' inner << 'EOS' || true
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq --no-install-recommends ninja-build nlohmann-json3-dev libgtest-dev \
  clang ros-${ROS_DISTRO}-launch-testing-ament-cmake ros-${ROS_DISTRO}-launch-testing-ros \
  python3-pytest > /dev/null
# Work on a copy with LF line endings, independent of the host checkout.
mkdir -p /ws
tar -C /src --exclude=./build --exclude=./.venv --exclude=./.deps --exclude=./results \
  --exclude=./.git -cf - . | tar -C /ws -xf -
find /ws -type f \( -name '*.sh' -o -name '*.py' -o -name '*.yaml' -o -name '*.xml' \) \
  -exec sed -i 's/\r$//' {} +
mkdir -p /ws/.deps/downloads
cp -n /src/.deps/downloads/* /ws/.deps/downloads/ 2> /dev/null || true
cd /ws
set +u  # ROS setup scripts reference unset variables
source /opt/ros/${ROS_DISTRO}/setup.bash
set -u
gcc --version | head -n1
if [ "$MODE" = core ] || [ "$MODE" = all ]; then
  cmake --preset ci-debug
  cmake --build --preset ci-debug --parallel "$JOBS" -- -k 0
  ctest --test-dir build/ci-debug --output-on-failure --parallel "$JOBS"
fi
if [ "$MODE" = sanitize ] || [ "$MODE" = all ]; then
  cmake --preset sanitize
  cmake --build --preset sanitize --parallel "$JOBS"
  ASAN_OPTIONS=detect_leaks=1 ctest --test-dir build/sanitize --output-on-failure --parallel "$JOBS"
fi
if [ "$MODE" = ros2 ] || [ "$MODE" = all ]; then
  BUILD_JOBS="$JOBS" bash scripts/ci/ros2_build.sh
fi
EOS

docker run --rm -e MODE="$mode" -e JOBS="$jobs" -v "$mount_src:/src:ro" "$image" bash -c "$inner"
