#!/usr/bin/env bash
# Creates the Python virtual environment used by the renderers, benchmark
# reports and Python tests.
# Usage: scripts/setup/bootstrap_python.sh [python-executable]
set -euo pipefail

python_bin=${1:-python3}
root=$(cd "$(dirname "$0")/../.." && pwd)
"$python_bin" -m venv "$root/.venv"
if [ -x "$root/.venv/bin/python" ]; then
  venv_python="$root/.venv/bin/python"
else
  venv_python="$root/.venv/Scripts/python"
fi
"$venv_python" -m pip install --quiet -e "$root[dev]"
echo "Python environment ready: $venv_python"
