#!/usr/bin/env python3
"""Assemble a persistent QEMU NOR image from ESP-IDF flasher_args.json."""
from __future__ import annotations
import argparse
import json
from pathlib import Path


def merge(build: Path, output: Path, size: int = 8 * 1024 * 1024) -> list[dict]:
    arguments = json.loads((build / "flasher_args.json").read_text())
    chip = arguments.get("extra_esptool_args", {}).get("chip", "esp32s3")
    if chip != "esp32s3":
        raise ValueError(f"expected ESP32-S3 image, got {chip}")
    segments = []
    for address, file in arguments["flash_files"].items():
        offset = int(address, 0)
        path = Path(file)
        if not path.is_absolute():
            path = build / path
        segments.append((offset, path.read_bytes(), str(path)))
    segments.sort()
    end = 0
    result = []
    for offset, data, name in segments:
        if offset < end or offset < 0 or offset + len(data) > size:
            raise ValueError(f"overlapping or out-of-bounds flash segment: {name}")
        end = offset + len(data)
        result.append({"offset": offset, "size": len(data), "source": name})
    if not segments or segments[0][0] != 0:
        raise ValueError("ESP32-S3 bootloader must be included at offset 0")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        written = 0
        erased = b"\xff" * 65536
        for offset, data, _ in segments:
            while written < offset:
                chunk = min(offset - written, len(erased))
                stream.write(erased[:chunk]); written += chunk
            stream.write(data); written += len(data)
        while written < size:
            chunk = min(size - written, len(erased))
            stream.write(erased[:chunk]); written += chunk
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--size", type=int, default=8 * 1024 * 1024)
    args = parser.parse_args()
    try:
        segments = merge(args.build.resolve(), args.output, args.size)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(2, f"merge-idf-flash: {error}\n")
    print(json.dumps({"image": str(args.output), "size": args.size, "segments": segments}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
