#!/usr/bin/env bash
# Installs a configured build tree into a temporary prefix and builds the
# downstream consumer in tests/package with find_package(MultiRobotCore).
# Usage: scripts/ci/install_test.sh <build-dir> [config]
set -euo pipefail

build_dir=${1:?build directory required}
config=${2:-Release}
root=$(cd "$(dirname "$0")/../.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cmake --install "$build_dir" --prefix "$work/prefix" --config "$config"
cmake -S "$root/tests/package" -B "$work/build" -DCMAKE_BUILD_TYPE="$config" \
  -DCMAKE_PREFIX_PATH="$work/prefix"
cmake --build "$work/build" --config "$config"
exe=$(find "$work/build" -maxdepth 2 -type f \( -name package_consumer -o -name package_consumer.exe \) | head -n1)
if [ -z "$exe" ]; then
  echo "package_consumer was not built"
  exit 1
fi
"$exe"
echo "install/package test passed"
