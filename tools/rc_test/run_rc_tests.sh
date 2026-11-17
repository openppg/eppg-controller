#!/usr/bin/env bash
# Runs every automated release-candidate check on Linux and writes
# .rc-results/rc-results.json, the file the RC checklist page loads.
#
#   tools/rc_test/run_rc_tests.sh            # everything
#   RC_SKIP=firmware tools/rc_test/run_rc_tests.sh
#
# Needs: PlatformIO, cmake, g++, python3, cpplint (pipx install cpplint).

set -u
ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT" || exit 1
OUT="${RC_OUT:-$ROOT/.rc-results}"
ENV_FW="OpenPPG-CESP32S3-CAN-SP140"
rm -rf "$OUT"
mkdir -p "$OUT/logs"
export PATH="$HOME/.local/bin:$PATH"
# Recorded before anything runs: the build and tests write into the tree.
git status --porcelain --untracked-files=no > "$OUT/dirty.txt"
echo "${RC_BRANCH:-$(git rev-parse --abbrev-ref HEAD)}" > "$OUT/branch.txt"

: > "$OUT/steps.tsv"
step() {
  local id=$1 name=$2
  shift 2
  if [[ ",${RC_SKIP:-}," == *",$id,"* ]]; then
    printf '%s\t%s\tskip\t0\n' "$id" "$name" >> "$OUT/steps.tsv"
    echo "SKIP  $name"
    return
  fi
  echo "----  $name"
  local t0=$SECONDS
  "$@" > "$OUT/logs/$id.log" 2>&1
  local rc=$?
  local status=pass
  [ $rc -ne 0 ] && status=fail
  printf '%s\t%s\t%s\t%s\n' "$id" "$name" "$status" "$((SECONDS - t0))" >> "$OUT/steps.tsv"
  echo "$(echo $status | tr a-z A-Z)  $name ($((SECONDS - t0)) s)"
}

has_env() { grep -q "^\[env:$1\]" platformio.ini; }

step lint "cpplint (CI rules)" \
  cpplint --linelength 140 \
  --filter=-legal/copyright,-runtime/int,-build/include_subdir,-readability/casting,-readability/todo,-build/include_order,-build/include_what_you_use,-whitespace/newline \
  --recursive ./inc/ ./lib/ ./src/

step unit "Native unit tests" \
  pio test -e native-test --junit-output-path "$OUT/unit.xml"

if has_env native-emu-ota; then
  step emu_ota "BLE OTA protocol emulation" \
    pio test -e native-emu-ota --junit-output-path "$OUT/emu_ota.xml"
fi

if has_env native-emu-settings; then
  step emu_settings "Settings storage and factory QC emulation" \
    pio test -e native-emu-settings --junit-output-path "$OUT/emu_settings.xml"
fi

# Also builds hc_emulator and climb_eff_replay (same CMake project).
rm -f test/test_screenshots/output/*.bmp
step screens "Display screenshot regression" \
  ./test/test_screenshots/build_and_run.sh --gtest_output="xml:$OUT/screens.xml"

step firmware "ESP32-S3 firmware build" pio run -e "$ENV_FW"

step static "Firmware image and source checks" \
  python3 tools/rc_test/static_checks.py --json "$OUT/static.json"

if [ -f tools/rc_test/climb_emulation.py ]; then
  step climb "Climb power bar flight emulation" \
    python3 tools/rc_test/climb_emulation.py \
    --replay build-screenshot/climb_eff_replay \
    --emulator build-screenshot/hc_emulator \
    --json "$OUT/climb.json" --frames-dir "$OUT/frames"
fi

python3 tools/rc_test/collect_results.py --out "$OUT" --env "$ENV_FW"
