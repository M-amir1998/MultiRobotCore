#!/usr/bin/env bash
# Fails when any tracked C++ file deviates from .clang-format.
# Usage: scripts/ci/check_format.sh [clang-format-binary]
set -euo pipefail

clang_format=${1:-${CLANG_FORMAT:-clang-format}}
root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
mapfile -t files < <(git ls-files '*.cpp' '*.hpp' | grep -v '^ros2/.*/build/')
"$clang_format" --version
"$clang_format" --dry-run --Werror --style=file "${files[@]}"
echo "clang-format: ${#files[@]} files clean"
