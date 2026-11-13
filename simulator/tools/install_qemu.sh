#!/usr/bin/env bash
# Pinned official Espressif release; hashes match esp-idf/tools/tools.json.
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
project_root="$(cd "$script_dir/../.." && pwd)"
destination="${1:-$project_root/.tools}"
version=esp_develop_9.2.2_20260417
case "$(uname -s)/$(uname -m)" in
  Linux/x86_64)
    platform=x86_64-linux-gnu
    checksum=0eecb2a34a5586c0e59110f77b9343b7b336e82fdb0e1a30e1dc1bab8a547e35 ;;
  Linux/aarch64|Linux/arm64)
    platform=aarch64-linux-gnu
    checksum=00de5985094c14e47d1b38464a006b8ed64fd0fa7a289c56da24f3a329f65339 ;;
  Darwin/arm64)
    platform=aarch64-apple-darwin
    checksum=bb8c15810565d3df1665dc34962430885e11bc95575b228fb44698146be1e9d6 ;;
  Darwin/x86_64)
    platform=x86_64-apple-darwin
    checksum=ae8170fe46bcdfa54a7c0d7afcdb7a066711991be680f72ff7bbc6c4ae3ad88f ;;
  *) printf '%s\n' 'This installer supports Linux and macOS x86_64/arm64.' >&2; exit 2 ;;
esac
archive="$(mktemp "${TMPDIR:-/tmp}/eppg-qemu-download.XXXXXX")"
trap 'rm -f "$archive"' EXIT
curl --fail --location --retry 3 -o "$archive" \
  "https://github.com/espressif/qemu/releases/download/esp-develop-9.2.2-20260417/qemu-xtensa-softmmu-$version-$platform.tar.xz"
python3 - "$archive" "$checksum" <<'PY'
import hashlib
from pathlib import Path
import sys
actual = hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest()
if actual != sys.argv[2]:
    raise SystemExit(f'QEMU download checksum mismatch: {actual}')
PY
mkdir -p "$destination"
tar -xJf "$archive" -C "$destination"
"$destination/qemu/bin/qemu-system-xtensa" --version
