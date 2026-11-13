#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
cd "$project_root"

printf '%s\n' '[1/5] Build native ESP32-S3 firmware and validate the release image'
IDF_COMPONENT_MANAGER=0 idf.py -B build-idf build
python3 tools/check_native_release.py build-idf

printf '%s\n' '[2/5] Run production controller regressions'
cmake -S tools/host-native-tests -B build-host-native-tests ${GTEST_DIR:+-DGTEST_DIR="$GTEST_DIR"}
cmake --build build-host-native-tests --parallel
ctest --test-dir build-host-native-tests --output-on-failure

printf '%s\n' '[3/5] Test deterministic controller and register-level device models'
./simulator/run.sh --test-only

printf '%s\n' '[4/5] Exercise pinned SINE/DroneCAN and ANT BMS byte protocols'
cmake -S tools/host-can-protocol-probe -B build-host-can-protocol
cmake --build build-host-can-protocol --parallel
./build-host-can-protocol/host_can_protocol_probe

printf '%s\n' '[5/5] Compare production LVGL screenshots with existing references'
cmake -S tools/host-screenshot-offline -B build-host-screenshots ${GTEST_DIR:+-DGTEST_DIR="$GTEST_DIR"}
cmake --build build-host-screenshots --parallel
ctest --test-dir build-host-screenshots --output-on-failure
printf '%s\n' 'All native firmware and host verification gates passed.'
