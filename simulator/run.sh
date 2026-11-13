#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
build_dir="$project_root/build-simulator"

cmake -S "$script_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$build_dir" --parallel "$(sysctl -n hw.ncpu 2>/dev/null || getconf _NPROCESSORS_ONLN || echo 4)"
ctest --test-dir "$build_dir" --output-on-failure

if [[ "${1:-}" == "--test-only" ]]; then
  exit 0
fi

scenario="${1:-$script_dir/scenarios/nominal-flight.csv}"
duration="${2:-10000}"
exec "$build_dir/eppg-sim" --scenario "$scenario" --duration "$duration"
