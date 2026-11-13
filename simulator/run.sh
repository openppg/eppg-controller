#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
backend="${EPPG_SIM_BACKEND:-}"
case "${1:-}" in --rust) backend=rust; shift ;; --native) backend=native; shift ;; esac
if [[ -z "$backend" ]]; then
  if [[ -f "$project_root/rust/apps/controller-sim/Cargo.toml" ]]; then backend=rust; else backend=native; fi
fi
if [[ "$backend" = rust ]]; then
  cd "$project_root/rust"
  cargo +stable test --locked -p controller-sim
  if [[ "${1:-}" = --test-only ]]; then
    cargo +stable build --locked -p controller-sim
    exit 0
  fi
  scenario="${1:-$script_dir/scenarios/nominal-flight.csv}"
  if [[ "$scenario" != /* ]]; then scenario="$project_root/$scenario"; fi
  exec cargo +stable run --locked -p controller-sim -- --scenario "$scenario" --duration "${2:-10000}"
fi
if [[ "$backend" != native ]]; then echo "EPPG_SIM_BACKEND must be native or rust" >&2; exit 2; fi
build_dir="$project_root/build-simulator"
cmake -S "$script_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$build_dir" --parallel "${EPPG_BUILD_JOBS:-4}"
ctest --test-dir "$build_dir" --output-on-failure
if [[ "${1:-}" = --test-only ]]; then exit 0; fi
exec "$build_dir/eppg-sim" --scenario "${1:-$script_dir/scenarios/nominal-flight.csv}" --duration "${2:-10000}"
