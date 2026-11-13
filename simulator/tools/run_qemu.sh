#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/../.." && pwd)"
environment="OpenPPG-CESP32S3-CAN-SP140"
flash_image="$project_root/.pio/build/$environment/firmware-merged.bin"
qemu_flash_image="$project_root/.pio/build/$environment/qemu-flash-8mb.bin"

qemu_binary="${QEMU_BIN:-}"
qemu_args=()
case "${1:-}" in
  "") ;;
  --gdb) qemu_args=(-s -S) ;;
  --trace) qemu_args=(-d in_asm,exec,cpu -D "$project_root/qemu-instructions.log") ;;
  *)
    echo "Usage: $0 [--gdb|--trace]" >&2
    exit 2
    ;;
esac
if [[ -z "$qemu_binary" ]]; then
  qemu_binary="$(command -v qemu-system-xtensa || true)"
fi
if [[ -z "$qemu_binary" && -x "$project_root/.tools/qemu/bin/qemu-system-xtensa" ]]; then
  qemu_binary="$project_root/.tools/qemu/bin/qemu-system-xtensa"
fi
if [[ -z "$qemu_binary" ]]; then
  echo "Espressif qemu-system-xtensa is not installed." >&2
  echo "Install the official build, add it to PATH, or set QEMU_BIN." >&2
  echo "https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/tools/qemu.html" >&2
  exit 2
fi

cd "$project_root"
pio run -e "$environment" --target mergebin

if [[ ! -f "$flash_image" ]]; then
  echo "merged flash image was not created: $flash_image" >&2
  exit 1
fi

python3 "$script_dir/prepare_qemu_flash.py" \
  "$flash_image" "$qemu_flash_image"

exec "$qemu_binary" -nographic \
  -machine esp32s3 \
  "${qemu_args[@]}" \
  -drive "file=$qemu_flash_image,if=mtd,format=raw"
