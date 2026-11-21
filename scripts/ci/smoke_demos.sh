#!/usr/bin/env bash
# Runs every demo executable once and checks that it writes valid JSON.
# Usage: scripts/ci/smoke_demos.sh <build-dir>
set -euo pipefail

build_dir=${1:?build directory required}
root=$(cd "$(dirname "$0")/../.." && pwd)
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
cd "$root"
count=0
for exe in "$build_dir"/apps/mrc_demo_*; do
  case "$exe" in *.pdb|*.ilk|*.manifest) continue ;; esac
  [ -x "$exe" ] || continue
  name=$(basename "$exe")
  name=${name%.exe}
  echo "== $name"
  "$exe" --output "$out/$name.json" > "$out/$name.log"
  python3 -c "import json,sys; json.load(open(sys.argv[1]))" "$out/$name.json" 2> /dev/null || \
    python -c "import json,sys; json.load(open(sys.argv[1]))" "$out/$name.json"
  count=$((count + 1))
done
if [ "$count" -eq 0 ]; then
  echo "no demos found in $build_dir/apps"
  exit 1
fi
echo "smoke test: $count demos produced valid JSON"
