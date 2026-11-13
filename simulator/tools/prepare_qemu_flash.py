#!/usr/bin/env python3
"""Pad a merged ESP32-S3 image to the exact NOR flash size QEMU requires."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("destination")
    parser.add_argument("--size", type=int, default=8 * 1024 * 1024)
    args = parser.parse_args()

    source = Path(args.source)
    destination = Path(args.destination)
    payload_size = source.stat().st_size
    if payload_size > args.size:
        raise SystemExit(
            f"merged image is {payload_size} bytes, larger than {args.size}-byte flash"
        )

    with source.open("rb") as incoming, destination.open("wb") as outgoing:
        while chunk := incoming.read(1024 * 1024):
            outgoing.write(chunk)
        remaining = args.size - payload_size
        erased = b"\xff" * (1024 * 1024)
        while remaining:
            chunk_size = min(remaining, len(erased))
            outgoing.write(erased[:chunk_size])
            remaining -= chunk_size
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
