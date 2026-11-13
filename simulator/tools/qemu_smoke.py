#!/usr/bin/env python3
"""Run a bounded, evidence-producing ESP32-S3 instruction-level QEMU test.

The supplied image is copied before launch: writes and crash records never alter
an input/release artifact. Success requires application markers, not ROM boot.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import selectors
import shutil
import subprocess
import tempfile
import time


def find_qemu(explicit: str | None) -> str:
    candidate = explicit or os.environ.get("QEMU_BIN") or shutil.which("qemu-system-xtensa")
    if candidate and Path(candidate).is_file():
        return str(Path(candidate).resolve())
    local = Path(__file__).resolve().parents[2] / ".tools/qemu/bin/qemu-system-xtensa"
    if local.is_file():
        return str(local)
    raise ValueError("Set --qemu or QEMU_BIN to Espressif's qemu-system-xtensa")


def control_evidence(log: str) -> dict:
    reports = []
    for line in log.splitlines():
        if line.startswith("EPPG_QEMU {"):
            try:
                reports.append(json.loads(line[len("EPPG_QEMU "):]))
            except json.JSONDecodeError:
                pass
    armed = [item for item in reports if item.get("state") in (1, 2)]
    screens = [item for item in reports if item.get("pixels", 0) > 0]
    can = [tuple(map(int, match)) for match in re.findall(
        r"EPPG_QEMU_CAN tx=(\d+) rx=(\d+) pwm_01us=(\d+)", log)]
    checks = {
        "armed_on_s3": bool(armed),
        "throttle_task_fresh": any(0 <= item.get("throttle_age_ms", 1000) <= 100 for item in armed),
        "spi_screen_changed": len({item.get("screen_fnv1a") for item in screens}) >= 2,
        "raw_can_bidirectional": any(tx > 0 and rx > 0 for tx, rx, _ in can),
        "raw_can_throttle_above_idle": any(pwm > 10000 for _, _, pwm in can),
    }
    return {"passed": all(checks.values()), "checks": checks,
            "model_reports": len(reports), "can_reports": len(can)}


def run(args: argparse.Namespace) -> dict:
    qemu = find_qemu(args.qemu)
    source = Path(args.image).resolve()
    size = source.stat().st_size
    if size > args.flash_size:
        raise ValueError("merged image exceeds configured flash size")
    if not args.expect:
        raise ValueError("at least one --expect application marker is required")
    started = time.monotonic()
    output = bytearray()
    missing = list(args.expect)
    failure = None
    result_code = None
    with tempfile.TemporaryDirectory(prefix="eppg-qemu-") as directory:
        image = Path(directory) / "flash.bin"
        with source.open("rb") as incoming, image.open("wb") as outgoing:
            shutil.copyfileobj(incoming, outgoing)
            remaining = args.flash_size - size
            while remaining:
                chunk = min(remaining, 1024 * 1024)
                outgoing.write(b"\xff" * chunk)
                remaining -= chunk
        command = [qemu, "-machine", "esp32s3", "-display", "none",
                   "-serial", "stdio", "-monitor", "none",
                   "-drive", f"file={image},if=mtd,format=raw"]
        if args.elf:
            command += ["-kernel", str(Path(args.elf).resolve())]
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, bufsize=0)
        assert process.stdout is not None
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        sent = False
        decoded = ""
        try:
            while time.monotonic() - started < args.timeout:
                for key, _ in selector.select(timeout=0.1):
                    chunk = os.read(key.fileobj.fileno(), 16384)
                    if chunk:
                        output.extend(chunk)
                    else:
                        result_code = process.poll()
                        if result_code is not None:
                            failure = f"QEMU exited before required markers (exit {result_code})"
                decoded = output.decode("utf-8", errors="replace")
                if args.send and not sent and args.send_after in decoded:
                    assert process.stdin is not None
                    process.stdin.write((args.send + "\n").encode())
                    process.stdin.flush()
                    sent = True
                missing = [marker for marker in args.expect if marker not in decoded]
                rejected = next((marker for marker in args.reject if marker in decoded), None)
                if rejected:
                    failure = f"failure marker observed: {rejected}"
                    break
                if not missing and (not args.control_proof or control_evidence(decoded)["passed"]):
                    break
                if failure:
                    break
            if not failure and args.control_proof and not control_evidence(decoded)["passed"]:
                failure = "timeout waiting for production control/SPI/CAN proof"
            if missing and not failure:
                failure = f"timeout after {args.timeout}s waiting for application markers"
        finally:
            selector.close()
            if process.poll() is None:
                process.terminate()
                try:
                    tail, _ = process.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    tail, _ = process.communicate()
                output.extend(tail or b"")
            result_code = process.returncode
    return {
        "passed": failure is None and not missing,
        "elapsed_seconds": round(time.monotonic() - started, 3),
        "qemu": qemu,
        "image": str(source),
        "image_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "flash_size": args.flash_size,
        "expected": args.expect,
        "missing": missing,
        "failure": failure,
        "control_proof": control_evidence(output.decode("utf-8", errors="replace")),
        "qemu_exit_code": result_code,
        "uart_log": output.decode("utf-8", errors="replace"),
        "fidelity": "ESP32-S3 CPU/RTOS/flash; application peripheral models selected by image build",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, help="merged flash image with bootloader/partitions/app")
    parser.add_argument("--elf", help="optional ELF direct-load mode; flash still backed by copied image")
    parser.add_argument("--qemu")
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--flash-size", type=int, default=8 * 1024 * 1024)
    parser.add_argument("--control-proof", action="store_true", help="require armed state, fresh task, changing SPI screen, and bidirectional non-idle CAN")
    parser.add_argument("--expect", action="append", default=[])
    parser.add_argument("--reject", action="append", default=[
        "Guru Meditation Error", "assert failed", "ABORTED", "stack overflow",
        "panic_abort", "Task watchdog got triggered"])
    parser.add_argument("--send", help="one UART command to send after --send-after marker")
    parser.add_argument("--send-after", default="=== Boot complete ===")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.timeout <= 0 or args.timeout > 600:
        parser.error("--timeout must be within (0, 600] seconds")
    try:
        report = run(args)
    except (OSError, ValueError) as error:
        parser.exit(2, f"qemu-smoke: {error}\n")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(report["uart_log"], end="")
    print("PASS" if report["passed"] else "FAIL", "ESP32-S3 QEMU application smoke")
    if report["failure"]:
        print(report["failure"])
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
