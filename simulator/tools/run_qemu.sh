#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/../.." && pwd)"
build_dir="${EPPG_QEMU_BUILD_DIR:-$project_root/build-idf-qemu}"
qemu_binary="${QEMU_BIN:-}"
image=""
build=0
smoke=0
qemu_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) build=1 ;;
    --smoke) smoke=1 ;;
    --build-dir) build_dir="$2"; shift ;;
    --image) image="$2"; shift ;;
    --gdb) qemu_args+=(-s -S) ;;
    --trace) qemu_args+=(-d in_asm -D "${EPPG_QEMU_TRACE_LOG:-$project_root/qemu-instructions.log}") ;;
    *) echo "Usage: $0 [--build] [--smoke] [--build-dir DIR] [--image BIN] [--gdb|--trace]" >&2; exit 2 ;;
  esac
  shift
done
if [[ -z "$qemu_binary" ]]; then qemu_binary="$(command -v qemu-system-xtensa || true)"; fi
if [[ -z "$qemu_binary" && -x "$project_root/.tools/qemu/bin/qemu-system-xtensa" ]]; then
  qemu_binary="$project_root/.tools/qemu/bin/qemu-system-xtensa"
fi
if [[ -z "$qemu_binary" ]]; then
  echo "Set QEMU_BIN to Espressif's qemu-system-xtensa or install it through ESP-IDF." >&2
  exit 2
fi
cd "$project_root"
if [[ "$build" = 1 ]]; then
  if ! command -v idf.py >/dev/null; then echo "Source ESP-IDF export.sh before --build." >&2; exit 2; fi
  IDF_COMPONENT_MANAGER=0 idf.py -B "$build_dir" -D EPPG_QEMU=ON \
    -D "SDKCONFIG=$build_dir/sdkconfig" \
    -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.qemu.defaults' build
fi
if [[ -z "$image" ]]; then
  image="$build_dir/qemu-flash-8mb.bin"
  python3 "$script_dir/merge_idf_flash.py" "$build_dir" "$image"
fi
if [[ "$smoke" = 1 ]]; then
  exec python3 "$script_dir/qemu_smoke.py" --qemu "$qemu_binary" --image "$image" \
    --control-proof --expect '=== Boot complete ===' --expect 'pwm_01us=17700' \
    --reject 'Error initializing BMP3xx' \
    --report "$build_dir/qemu-smoke.json"
fi
# The interactive session owns a writable copy, preserving the release image.
qemu_copy="$(mktemp "${TMPDIR:-/tmp}/eppg-qemu.XXXXXX")"
trap 'rm -f "$qemu_copy"' EXIT
python3 "$script_dir/prepare_qemu_flash.py" "$image" "$qemu_copy"
"$qemu_binary" -machine esp32s3 -display none -serial stdio -monitor none \
  "${qemu_args[@]}" -drive "file=$qemu_copy,if=mtd,format=raw"
