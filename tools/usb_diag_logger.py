#!/usr/bin/env python3

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import serial
from serial import SerialException
from serial.tools import list_ports


DEFAULT_BAUD = 115200
DEFAULT_QUERY_INTERVAL = 2.0
DEFAULT_RECONNECT_DELAY = 1.0
TARGET_VID = 0x303A
TARGET_PID = 0x1001


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reconnectable USB diagnostics logger for the OpenPPG controller."
    )
    parser.add_argument("--port", help="Serial device path. Auto-detected if omitted.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--out-dir",
        default="./diagnostics-logs",
        help="Directory for serial.log and diag-history.jsonl",
    )
    parser.add_argument(
        "--query-interval",
        type=float,
        default=DEFAULT_QUERY_INTERVAL,
        help="Seconds between diag_sync retries while waiting for a response.",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=DEFAULT_RECONNECT_DELAY,
        help="Seconds between reconnect attempts.",
    )
    return parser.parse_args()


def find_controller_port(explicit_port: Optional[str]) -> Optional[str]:
    if explicit_port:
        return explicit_port

    matches = []
    for port in list_ports.comports():
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            matches.append(port.device)

    if not matches:
        return None

    matches.sort()
    return matches[0]


class UsbDiagLogger:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.output_dir = Path(args.out_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.serial_log = (self.output_dir / "serial.log").open(
            "a", encoding="utf-8", buffering=1
        )
        self.diag_log = (self.output_dir / "diag-history.jsonl").open(
            "a", encoding="utf-8", buffering=1
        )
        self.serial_port: Optional[serial.Serial] = None
        self.port_name: Optional[str] = None
        self.buffer = bytearray()
        self.awaiting_diag = False
        self.next_query_time = 0.0

    def close(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except SerialException:
                pass
        self.serial_port = None
        self.port_name = None
        self.buffer.clear()

    def host_log(self, message: str) -> None:
        line = f"{utc_now()} [HOST] {message}"
        print(line, flush=True)
        self.serial_log.write(line + "\n")

    def connect(self) -> bool:
        port_name = find_controller_port(self.args.port)
        if not port_name:
            return False

        if self.serial_port is not None and self.port_name == port_name:
            return True

        self.close()
        try:
            serial_port = serial.Serial()
            serial_port.port = port_name
            serial_port.baudrate = self.args.baud
            serial_port.timeout = 0.25
            serial_port.write_timeout = 1.0
            serial_port.rtscts = False
            serial_port.dsrdtr = False
            serial_port.xonxoff = False
            serial_port.dtr = False
            serial_port.rts = False
            serial_port.open()
            self.serial_port = serial_port
            self.serial_port.reset_input_buffer()
            self.port_name = port_name
            self.awaiting_diag = True
            self.next_query_time = 0.0
            self.host_log(f"connected to {port_name}")
            return True
        except SerialException as exc:
            self.close()
            self.host_log(f"connect failed for {port_name}: {exc}")
            return False

    def send_diag_sync(self) -> None:
        if self.serial_port is None:
            return

        payload = b'{"command":"diag_sync"}\n'
        try:
            self.serial_port.write(payload)
            self.serial_port.flush()
            self.next_query_time = time.monotonic() + self.args.query_interval
        except (SerialException, OSError) as exc:
            self.host_log(f"diag_sync write failed: {exc}")
            self.close()

    def log_raw_line(self, line: str) -> None:
        self.serial_log.write(f"{utc_now()} {line}\n")

    def log_diag_payload(self, payload: dict) -> None:
        entry = {
            "host_ts": utc_now(),
            "port": self.port_name,
            "diag": payload,
        }
        self.diag_log.write(json.dumps(entry, sort_keys=True) + "\n")

    def handle_line(self, raw_line: bytes) -> None:
        line = raw_line.decode("utf-8", errors="replace").rstrip("\r")
        self.log_raw_line(line)

        candidates = [line]
        start_idx = line.find("{")
        end_idx = line.rfind("}")
        if start_idx >= 0 and end_idx > start_idx:
            candidates.insert(0, line[start_idx : end_idx + 1])

        payload = None
        for candidate in candidates:
            try:
                payload = json.loads(candidate)
                break
            except json.JSONDecodeError:
                continue
        if payload is None:
            return

        if payload.get("diag_v") == 1:
            self.awaiting_diag = False
            self.log_diag_payload(payload)

    def pump_serial(self) -> None:
        if self.serial_port is None:
            return

        chunk = self.serial_port.read(256)
        if not chunk:
            return

        self.buffer.extend(chunk)
        while True:
            newline_index = self.buffer.find(b"\n")
            if newline_index < 0:
                break
            line = self.buffer[:newline_index]
            del self.buffer[: newline_index + 1]
            self.handle_line(line)

    def run(self) -> int:
        try:
            while True:
                if self.serial_port is None:
                    if not self.connect():
                        time.sleep(self.args.reconnect_delay)
                        continue

                if self.awaiting_diag and time.monotonic() >= self.next_query_time:
                    self.send_diag_sync()

                try:
                    self.pump_serial()
                except (SerialException, OSError) as exc:
                    self.host_log(f"serial error on {self.port_name}: {exc}")
                    self.close()
                    time.sleep(self.args.reconnect_delay)
                    continue

                time.sleep(0.05)
        except KeyboardInterrupt:
            self.host_log("stopped by user")
            return 0
        finally:
            self.close()
            self.serial_log.close()
            self.diag_log.close()


def main() -> int:
    args = parse_args()
    logger = UsbDiagLogger(args)
    return logger.run()


if __name__ == "__main__":
    sys.exit(main())
