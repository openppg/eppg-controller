#!/usr/bin/env python3
"""Inspect ESP32-S3 firmware ELF sections, symbols, and Xtensa bytecode."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path


def find_objdump() -> str:
    direct = shutil.which("xtensa-esp32s3-elf-objdump")
    if direct:
        return direct
    candidate = (Path.home() / ".platformio/packages/toolchain-xtensa-esp32s3/bin/"
                 "xtensa-esp32s3-elf-objdump")
    if candidate.exists():
        return str(candidate)
    raise SystemExit("xtensa-esp32s3-elf-objdump not found; build with PlatformIO first")


def command(tool: str, *args: str) -> str:
    return subprocess.run(
        [tool, *args], check=True, text=True, stdout=subprocess.PIPE
    ).stdout


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "elf", nargs="?", default=".pio/build/OpenPPG-CESP32S3-CAN-SP140/firmware.elf"
    )
    parser.add_argument("--disassembly", help="write complete annotated disassembly here")
    args = parser.parse_args()
    elf = Path(args.elf).resolve()
    if not elf.exists():
        raise SystemExit(f"firmware ELF not found: {elf}; run pio run first")

    objdump = find_objdump()
    headers = command(objdump, "-h", str(elf))
    symbols = command(objdump, "-t", str(elf))
    interesting = []
    for line in symbols.splitlines():
        if any(name in line for name in (
            " app_main", " setup", " loop", "throttleTask", "uiTask", "bmsTask",
            "bleNotifyTask", "setupAltimeter", "initESC", "initBMSCAN"
        )):
            interesting.append(line.strip())

    sections = []
    for line in headers.splitlines():
        fields = line.split()
        if len(fields) >= 7 and fields[0].isdigit():
            try:
                sections.append({
                    "name": fields[1], "size_bytes": int(fields[2], 16),
                    "vma": fields[3], "file_offset": fields[5],
                })
            except ValueError:
                pass

    report = {
        "elf": str(elf),
        "sha256": hashlib.sha256(elf.read_bytes()).hexdigest(),
        "file_size_bytes": elf.stat().st_size,
        "objdump": objdump,
        "sections": sections,
        "firmware_symbols": interesting,
    }
    print(json.dumps(report, indent=2))

    if args.disassembly:
        output = Path(args.disassembly)
        output.write_text(command(objdump, "-d", "-S", "-C", str(elf)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
