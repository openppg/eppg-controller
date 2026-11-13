#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
cd "$project_root"

echo "[1/5] Building real ESP32-S3 firmware and merged QEMU flash"
pio run -e OpenPPG-CESP32S3-CAN-SP140 --target mergebin

echo "[2/5] Running native production-logic tests"
pio test -e native-test

echo "[3/5] Building and testing deterministic whole-controller simulator"
./simulator/run.sh --test-only

echo "[4/5] Exercising real SINE/DroneCAN and ANT BMS byte protocols"
cmake -S tools/host-can-protocol-probe -B build-host-can-protocol \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build-host-can-protocol --parallel \
  "$(sysctl -n hw.ncpu 2>/dev/null || getconf _NPROCESSORS_ONLN || echo 4)"
./build-host-can-protocol/host_can_protocol_probe

echo "[5/5] Rendering and comparing all real LVGL screens offline"
./test/test_screenshots/build_and_run.sh --local

echo "All controller emulation/build checks passed."
