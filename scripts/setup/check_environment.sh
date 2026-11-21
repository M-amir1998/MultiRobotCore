#!/usr/bin/env bash
# Validates the build environment and prints tool versions.
# Usage: scripts/setup/check_environment.sh
set -euo pipefail

status=0
require() {
  if ! command -v "$1" > /dev/null 2>&1; then
    echo "missing required tool: $1"
    status=1
  fi
}

require cmake
require ctest
require git
if ! command -v c++ > /dev/null 2>&1 && ! command -v g++ > /dev/null 2>&1 && \
   ! command -v clang++ > /dev/null 2>&1 && ! command -v cl > /dev/null 2>&1; then
  echo "missing a C++20 compiler (g++, clang++ or cl)"
  status=1
fi

if command -v cmake > /dev/null 2>&1; then
  version=$(cmake --version | head -n1 | awk '{print $3}')
  major=${version%%.*}
  minor=${version#*.}
  minor=${minor%%.*}
  if [ "$major" -lt 3 ] || { [ "$major" -eq 3 ] && [ "$minor" -lt 24 ]; }; then
    echo "CMake >= 3.24 required (found $version)"
    status=1
  fi
fi

echo "cmake:       $(cmake --version 2>/dev/null | head -n1 || echo missing)"
echo "ninja:       $(ninja --version 2>/dev/null || echo 'not found (Unix Makefiles will be used)')"
echo "c++:         $( (c++ --version || g++ --version || clang++ --version) 2>/dev/null | head -n1 || echo missing)"
echo "clang-format: $(clang-format --version 2>/dev/null || echo 'not found')"
echo "clang-tidy:  $(clang-tidy --version 2>/dev/null | head -n1 || echo 'not found')"
echo "python:      $(python3 --version 2>/dev/null || python --version 2>/dev/null || echo missing)"
echo "nproc:       $(nproc 2>/dev/null || echo unknown)"
echo "memory:      $(free -h 2>/dev/null | awk '/Mem:/ {print $2}' || echo unknown)"

exit "$status"
