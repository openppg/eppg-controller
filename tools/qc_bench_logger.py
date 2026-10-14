#!/usr/bin/env python3
"""Factory QC bench logger.

Tails the controller's USB serial output and appends every FIRST_BOOT_QC
record (single-line JSON starting with {"qc":) to a JSONL file, wrapped with a
capture timestamp and the serial port. Everything else is echoed to stdout so
the operator still sees the live boot/QC log.

Usage:
  python3 tools/qc_bench_logger.py --port /dev/tty.usbmodem101
  python3 tools/qc_bench_logger.py --port COM7 --out qc-records.jsonl

Reconnects automatically when the device reboots (QC runs at boot, so the
port comes and goes). Companion to tools/usb_diag_logger.py.
"""

import argparse
import datetime
import json
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial is required: pip3 install pyserial")


def parse_args():
    p = argparse.ArgumentParser(description="OpenPPG factory QC bench logger")
    p.add_argument("--port", required=True, help="Serial port (e.g. /dev/tty.usbmodem101)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default="qc-records.jsonl", help="JSONL output path")
    return p.parse_args()


def record_line(out_path, port, line):
    entry = {
        "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "port": port,
    }
    try:
        entry["record"] = json.loads(line)
    except json.JSONDecodeError:
        entry["raw"] = line  # keep malformed records for debugging
    with open(out_path, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry) + "\n")


def main():
    args = parse_args()
    print(f"qc_bench_logger: port={args.port} -> {args.out} (Ctrl-C to stop)")
    while True:
        try:
            with serial.Serial(args.port, args.baud, timeout=1) as ser:
                print(f"qc_bench_logger: connected to {args.port}")
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    print(line)
                    if line.startswith('{"qc":'):
                        record_line(args.out, args.port, line)
                        print(f"qc_bench_logger: >>> QC record captured to {args.out}")
        except serial.SerialException:
            print("qc_bench_logger: port unavailable, retrying in 2s...")
            time.sleep(2)
        except KeyboardInterrupt:
            print("\nqc_bench_logger: stopped")
            return


if __name__ == "__main__":
    main()
