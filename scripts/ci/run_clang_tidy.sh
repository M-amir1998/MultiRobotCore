#!/usr/bin/env bash
# Runs clang-tidy (.clang-tidy profile) on every library source using the
# compile database of a configured build tree. Exits non-zero on findings.
# Usage: scripts/ci/run_clang_tidy.sh <build-dir> [clang-tidy-binary] [jobs]
set -euo pipefail

build_dir=${1:?build directory with compile_commands.json required}
clang_tidy=${2:-${CLANG_TIDY:-clang-tidy}}
jobs=${3:-4}
root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
if [ ! -f "$build_dir/compile_commands.json" ]; then
  echo "no compile_commands.json in $build_dir (configure with CMAKE_EXPORT_COMPILE_COMMANDS=ON)"
  exit 2
fi
mapfile -t files < <(git ls-files 'src/*.cpp')
log=$(mktemp)
printf '%s\n' "${files[@]}" | xargs -P "$jobs" -I{} "$clang_tidy" -p "$build_dir" --quiet {} > "$log" 2>&1 || true
warnings=$(grep -cE "(warning|error): " "$log" || true)
grep -E "(warning|error): " "$log" | sort -u || true
rm -f "$log"
echo "clang-tidy: ${#files[@]} files, $warnings findings"
[ "$warnings" -eq 0 ]
