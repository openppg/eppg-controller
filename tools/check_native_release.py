#!/usr/bin/env python3
"""Check native image identity, partition compatibility and linked dependencies."""
import argparse
import json
from pathlib import Path
import re
import struct


def validate(build: Path) -> dict:
    image = build / "openppg_controller.bin"
    content = image.read_bytes()
    if len(content) > 0x330000:
        raise ValueError("application exceeds the installed OTA slot")
    marker = b"OPPGFWV1"
    if content.count(marker) != 1:
        raise ValueError("expected exactly one retained firmware version marker")
    offset = content.index(marker)
    major, minor, counter = struct.unpack_from("<BBI", content, offset + 8)
    link_map = (build / "openppg_controller.map").read_text()
    forbidden = re.compile(r"libarduino|framework-arduino|ArduinoJson|Adafruit_|Preferences::|NimBLE-Arduino", re.I)
    match = forbidden.search(link_map)
    if match:
        raise ValueError(f"Arduino dependency in production map: {match.group()}")
    commands = json.loads((build / "compile_commands.json").read_text())
    for entry in commands:
        match = forbidden.search(entry["file"])
        if match:
            raise ValueError(f"Arduino source in production compilation: {match.group()}")
    partitions = (build / "partition_table/partition-table.bin").read_bytes()
    expected = {"nvs": (0x9000, 0x5000), "otadata": (0xE000, 0x2000),
                "app0": (0x10000, 0x330000), "app1": (0x340000, 0x330000),
                "spiffs": (0x670000, 0x180000), "coredump": (0x7F0000, 0x10000)}
    actual = {}
    for n in range(0, len(partitions), 32):
        magic, kind, subtype, start, length, label, flags = struct.unpack_from("<HBBII16sI", partitions, n)
        if magic != 0x50AA:
            break
        actual[label.rstrip(b"\0").decode()] = (start, length)
    if actual != expected:
        raise ValueError(f"partition table does not match installed devices: {actual}")
    if not (build / "openppg_controller.elf").is_file():
        raise ValueError("missing ELF for crash decoding")
    return {"image": str(image), "bytes": len(content), "ota_slot_bytes": 0x330000,
            "version": f"{major}.{minor}", "build": counter,
            "marker_offset": offset, "arduino_objects": False,
            "partition_layout": "installed-8MB", "compiled_sources": len(commands)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(validate(args.build), indent=2))
    except (OSError, ValueError, KeyError, struct.error) as error:
        parser.exit(1, f"native-release-check: {error}\n")
