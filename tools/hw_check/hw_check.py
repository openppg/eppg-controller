#!/usr/bin/env python3
"""Layer 1 hardware checks for an SP140 hand controller plugged in over USB.

Runs the release-checklist items the firmware already reports over USB serial
and (optionally) BLE: firmware version, settings write/persist/restore,
boot diagnostics and crash history, first-boot QC results, live sensor/ESC/BMS
telemetry, and OTA readiness. Nothing here arms the motor or sends throttle.

    python tools/hw_check/hw_check.py                    # serial checks, auto-detect port
    python tools/hw_check/hw_check.py --ble --expect-bms # add BLE checks, BMS must be connected
    python tools/hw_check/hw_check.py --dry-run          # against a simulated controller

Exit status is 0 when nothing FAILed. See README.md for what each check covers.
"""

import argparse
import asyncio
import json
import re
import subprocess
import sys
import tempfile
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

import protocol  # noqa: E402
from links import BleLink, LinkError, SerialLink  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]

PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"


@dataclass
class Result:
    id: str
    status: str
    detail: str
    data: dict = field(default_factory=dict)


@dataclass
class Timeouts:
    reply: float = 6.0
    reconnect: float = 20.0
    boot: float = 20.0
    telemetry: float = 3.0
    qc: float = 300.0

    @classmethod
    def fast(cls) -> "Timeouts":
        return cls(reply=3.0, reconnect=5.0, boot=5.0, telemetry=0.5, qc=10.0)


def repo_firmware_version() -> Optional[str]:
    """VERSION_MAJOR.VERSION_MINOR from inc/version.h in this checkout."""
    try:
        text = (REPO_ROOT / "inc" / "version.h").read_text()
    except OSError:
        return None
    major = re.search(r"#define VERSION_MAJOR (\d+)", text)
    minor = re.search(r"#define VERSION_MINOR (\d+)", text)
    return f"{major.group(1)}.{minor.group(1)}" if major and minor else None


def esptool_read_partition_table(port: str) -> bytes:
    """Read the partition table with esptool. This resets the controller."""
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "partitions.bin"
        cmd = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", port,
               "--after", "hard_reset", "read_flash",
               hex(protocol.PARTITION_TABLE_OFFSET), hex(protocol.PARTITION_TABLE_SIZE), str(out)]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        if proc.returncode != 0:
            raise LinkError(f"esptool failed: {(proc.stderr or proc.stdout).strip()[-300:]}")
        return out.read_bytes()


class HardwareCheck:
    def __init__(self, args: argparse.Namespace, serial_link: SerialLink,
                 ble_factory: Optional[Callable[[], BleLink]],
                 read_partition_table: Callable[[str], bytes], timeouts: Timeouts):
        self.args = args
        self.serial = serial_link
        self.ble_factory = ble_factory
        self.read_partition_table = read_partition_table
        self.t = timeouts
        self.results: List[Result] = []
        self.version: Optional[str] = None

    # -- reporting ----------------------------------------------------------

    def record(self, check_id: str, status: str, detail: str, **data) -> Result:
        result = Result(check_id, status, detail, data)
        self.results.append(result)
        print(f"  {status:<4}  {check_id:<24} {detail}", flush=True)
        return result

    @property
    def failed(self) -> bool:
        return any(r.status == FAIL for r in self.results)

    # -- serial helpers -----------------------------------------------------

    def sync(self) -> dict:
        return self.serial.request('{"command":"sync"}', protocol.is_sync_response, self.t.reply)

    def diag(self) -> dict:
        return self.serial.request('{"command":"diag_sync"}', protocol.is_diag_response, self.t.reply)

    def write_settings(self, settings: dict) -> dict:
        payload = json.dumps({"settings": settings}, separators=(",", ":"))
        return self.serial.request(payload, protocol.is_sync_response, self.t.reply, retries=1)

    # -- checks -------------------------------------------------------------

    def run(self) -> None:
        print(f"OpenPPG hardware check  {datetime.now(timezone.utc).isoformat(timespec='seconds')}")
        try:
            self.serial.open(timeout=self.t.reconnect)
        except LinkError as exc:
            self.record("serial.connect", FAIL, str(exc))
            return
        self.record("serial.connect", PASS, f"opened {self.serial.port_name}")

        try:
            original = self.check_version()
            self.check_crash_history()
            self.check_settings_and_reboot(original)
            self.check_qc_over_serial()
            if self.ble_factory is not None:
                asyncio.run(self.run_ble_checks())
            self.check_partition_table()
        except LinkError as exc:
            self.record("serial.link", FAIL, f"lost the controller: {exc}")
        finally:
            self.serial.close()

    def check_version(self) -> dict:
        sync = self.sync()
        self.version = protocol.sync_version(sync)
        expected = self.args.expect_version
        detail = f"{self.version} ({sync.get('arch')}), armed time {sync.get('ar_tme')} min"
        if sync.get("arch") != "ESP32S3":
            self.record("fw.version", FAIL, f"unexpected arch: {detail}")
        elif expected and self.version != expected:
            self.record("fw.version", FAIL, f"expected {expected}, unit reports {detail}")
        else:
            self.record("fw.version", PASS, detail, version=self.version)
        return sync

    def check_crash_history(self) -> None:
        diag = self.diag()
        records = [diag.get("cur") or {}] + list(diag.get("hist") or [])
        crashes = []
        for rec in records:
            dump = rec.get("cdmp") or {}
            if rec.get("rst") in protocol.CRASH_RESET_REASONS or dump.get("present"):
                where = f" in {dump['task']} pc=0x{dump.get('pc', 0):08x}" if dump.get("task") else ""
                crashes.append(f"boot {rec.get('boot')}: {rec.get('rst')}{where}")
        if crashes:
            self.record("diag.crash_history", FAIL,
                        f"{len(crashes)} crash/watchdog/brownout reset(s) retained: " + "; ".join(crashes[:4]),
                        crashes=crashes)
        else:
            self.record("diag.crash_history", PASS,
                        f"no crash resets in {len(records)} retained boot record(s)",
                        boot=(diag.get("cur") or {}).get("boot"))

    def check_settings_and_reboot(self, original_sync: dict) -> None:
        original = protocol.settings_from_sync(original_sync)
        try:
            sanitized = self.write_settings(
                {"screen_rot": 2, "sea_pressure": 50.0, "performance_mode": 5, "theme": 9})
            problems = protocol.settings_mismatches(
                {"screen_rot": 3, "sea_pressure": 1013.25, "performance_mode": 0, "theme": 0}, sanitized)
            self.record("settings.sanitize", FAIL if problems else PASS,
                        "; ".join(problems) if problems else "out-of-range values replaced with defaults")

            target = protocol.non_default_settings(original_sync)
            problems = protocol.settings_mismatches(target, self.write_settings(target))
            self.record("settings.write", FAIL if problems else PASS,
                        "; ".join(problems) if problems else f"wrote and read back {len(target)} settings",
                        written=target)

            if self.args.no_reboot:
                self.record("settings.persist", SKIP, "--no-reboot given")
                return
            self.reboot_checks(target)
        finally:
            self.restore_settings(original)

    def reboot_checks(self, target: dict) -> None:
        lines = self.serial.reboot_and_capture(self.t.reconnect, self.t.boot)
        self.check_boot_log(lines)

        problems = protocol.settings_mismatches(target, self.sync())
        self.record("settings.persist", FAIL if problems else PASS,
                    "; ".join(problems) if problems else "all settings survived a reboot")

        cur = self.diag().get("cur") or {}
        if cur.get("rst") == "SOFTWARE" and cur.get("plan") == "usb_command_reboot":
            self.record("diag.reset_reason", PASS, "reboot recorded as SOFTWARE / usb_command_reboot")
        else:
            self.record("diag.reset_reason", FAIL,
                        f"expected SOFTWARE / usb_command_reboot, got {cur.get('rst')} / {cur.get('plan')}")

    def check_boot_log(self, lines: List[str]) -> None:
        banner = next((m for m in map(protocol.BOOT_BANNER_RE.search, lines) if m), None)
        ota = next((m for m in map(protocol.OTA_PARTITION_RE.search, lines) if m), None)
        alerts = [m.group(0) for m in map(protocol.MONITOR_ALERT_RE.match, lines) if m]
        complete = any(protocol.BOOT_COMPLETE in line for line in lines)

        if any(protocol.BARO_INIT_ERROR in line for line in lines):
            self.record("sensors.baro_init", FAIL, "boot log: " + protocol.BARO_INIT_ERROR)
        if banner and self.version and banner.group("ver") != self.version:
            self.record("boot.banner", FAIL, f"boot banner says V{banner.group('ver')}, sync says {self.version}")
        elif banner:
            self.record("boot.banner", PASS, f"V{banner.group('ver')} git:{banner.group('git')}")
        self.record("boot.complete", PASS if complete else WARN,
                    f"boot finished ({len(lines)} log lines captured)" if complete
                    else "did not see '=== Boot complete ===' after the reboot")

        if ota is None:
            self.record("ota.boot_partition", SKIP, "OTA partition line not captured (printed before the port reopened)")
        elif ota.group("running") != ota.group("boot"):
            self.record("ota.boot_partition", FAIL,
                        f"running {ota.group('running')} but boot partition is {ota.group('boot')}")
        else:
            self.record("ota.boot_partition", PASS, f"running and booting {ota.group('running')}")

        if alerts:
            self.record("boot.alerts", WARN, f"{len(alerts)} monitor alert(s): " + "; ".join(alerts[:3]),
                        alerts=alerts)

        qc = next((obj for obj in map(protocol.parse_json_line, lines) if obj and protocol.is_qc_record(obj)), None)
        if qc is not None:
            self.evaluate_qc_record("qc.boot_record", qc)

    def restore_settings(self, original: dict) -> None:
        try:
            problems = protocol.settings_mismatches(original, self.write_settings(original))
        except LinkError as exc:
            self.record("settings.restore", FAIL, f"could not restore the unit's settings: {exc}", original=original)
            return
        self.record("settings.restore", FAIL if problems else PASS,
                    "; ".join(problems) if problems else "restored the unit's original settings",
                    original=original)

    def evaluate_qc_record(self, check_id: str, record: dict) -> None:
        failures = protocol.qc_failures(record)
        summary = f"QC {record.get('result')} on fw {record.get('fw')}, pot {record.get('pot_min')}-{record.get('pot_max')}"
        if record.get("result") != "PASSED" or failures:
            self.record(check_id, FAIL, f"{summary}; not passed: {', '.join(failures) or 'result flag'}", record=record)
        else:
            self.record(check_id, PASS, summary, record=record)

    def check_qc_over_serial(self) -> None:
        if not self.args.run_qc:
            return
        print("  ....  running factory QC: follow the prompts on the controller screen", flush=True)
        self.serial.send('{"command":"run_qc"}')
        deadline = time.monotonic() + self.t.qc
        while time.monotonic() < deadline:
            try:
                record = self.serial.wait_for(protocol.is_qc_record, deadline - time.monotonic())
            except LinkError:
                self.serial.open(timeout=self.t.reconnect)
                continue
            if record is not None:
                self.evaluate_qc_record("qc.run", record)
                return
            break
        self.record("qc.run", FAIL, f"no QC record within {self.t.qc:.0f}s "
                                    "(firmware without first-boot QC ignores run_qc)")

    def check_partition_table(self) -> None:
        if not self.args.esptool:
            return
        self.serial.close()
        try:
            raw = self.read_partition_table(self.serial.port_name)
        except (LinkError, OSError, subprocess.SubprocessError) as exc:
            self.record("ota.partition_table", FAIL, str(exc))
            return
        partitions = protocol.parse_partition_table(raw)
        slots = [p for p in partitions if p["type"] == protocol.APP_TYPE and p["subtype"] in protocol.OTA_APP_SUBTYPES]
        otadata = [p for p in partitions if p["type"] == protocol.DATA_TYPE and p["subtype"] == protocol.OTA_DATA_SUBTYPE]
        layout = ", ".join(f"{p['label']}@0x{p['offset']:x}/{p['size'] // 1024}K" for p in partitions)
        if len(slots) < 2 or not otadata:
            self.record("ota.partition_table", FAIL, f"need 2 OTA app slots and otadata, found: {layout}")
            return
        if self.args.firmware:
            size = Path(self.args.firmware).stat().st_size
            smallest = min(p["size"] for p in slots)
            if size > smallest:
                self.record("ota.partition_table", FAIL, f"{self.args.firmware} is {size} bytes, slot is {smallest}")
                return
            layout += f"; image {size // 1024}K fits ({100 * size // smallest}% of a slot)"
        self.record("ota.partition_table", PASS, layout, partitions=partitions)

    # -- BLE ----------------------------------------------------------------

    async def run_ble_checks(self) -> None:
        ble = self.ble_factory()
        try:
            await ble.connect()
        except (LinkError, Exception) as exc:  # noqa: BLE001 - bleak raises backend-specific errors
            self.record("ble.connect", FAIL, str(exc))
            return
        self.record("ble.connect", PASS, f"connected to {ble.name or 'controller'}")
        try:
            services = ble.services()
            await self.check_ble_version(ble)
            await self.check_ble_setting(ble)
            await self.check_telemetry(ble, services)
            self.check_ota_service(services)
            await self.check_ble_qc_record(ble, services)
        except LinkError:
            raise  # serial trouble, reported by run()
        except Exception as exc:  # noqa: BLE001 - bleak raises backend-specific errors
            self.record("ble.link", FAIL, f"BLE error: {exc!r} (is this computer bonded to the controller?)")
        finally:
            await ble.disconnect()

    async def check_ble_version(self, ble: BleLink) -> None:
        raw = await ble.read(protocol.FW_VERSION_UUID)
        if not raw:
            self.record("ble.fw_version", WARN, "firmware version read came back empty")
            return
        version, build = protocol.parse_fw_version(raw)
        detail = f"{version} build {build}" if build is not None else version
        if self.version and version != self.version:
            self.record("ble.fw_version", FAIL, f"BLE reports {detail}, serial reports {self.version}")
        else:
            self.record("ble.fw_version", PASS, detail, build=build)

    async def check_ble_setting(self, ble: BleLink) -> None:
        """Write the theme over BLE and confirm the controller saved it (via serial)."""
        before = self.sync().get("thm")
        flipped = 0 if before == 1 else 1
        try:
            await ble.write(protocol.THEME_UUID, bytes([flipped]))
            after = self.sync().get("thm")
        finally:
            await ble.write(protocol.THEME_UUID, bytes([before]))
        restored = self.sync().get("thm")
        if after != flipped:
            self.record("ble.settings_write", FAIL, f"wrote theme={flipped} over BLE, serial reads {after}")
        elif restored != before:
            self.record("ble.settings_write", FAIL, f"could not restore theme={before} over BLE (reads {restored})")
        else:
            self.record("ble.settings_write", PASS, "theme written over BLE reached the controller and was restored")

    async def check_telemetry(self, ble: BleLink, services: dict) -> None:
        if protocol.FAST_LINK_TELEMETRY_UUID not in services.get(protocol.FAST_LINK_SERVICE_UUID, []):
            self.record("telemetry.stream", FAIL, "Fast-Link telemetry service not found")
            return
        raw_packets = await ble.collect_notifications(protocol.FAST_LINK_TELEMETRY_UUID, self.t.telemetry)
        packets = []
        for raw in raw_packets:
            try:
                packets.append(protocol.parse_fastlink(raw))
            except ValueError:
                pass
        if not packets:
            self.record("telemetry.stream", FAIL,
                        f"no complete Fast-Link packets in {self.t.telemetry:.0f}s ({len(raw_packets)} short ones;"
                        " the link needs MTU >= 201)")
            return
        bad_version = [p["version"] for p in packets if p["version"] != protocol.FASTLINK_VERSION]
        ids = [p["packet_id"] for p in packets]
        dropped = (ids[-1] - ids[0] + 1) - len(ids)
        rate = len(packets) / self.t.telemetry
        if bad_version:
            self.record("telemetry.stream", FAIL, f"protocol version {bad_version[0]}, expected {protocol.FASTLINK_VERSION}")
        else:
            self.record("telemetry.stream", PASS if dropped <= len(ids) // 10 else WARN,
                        f"{len(packets)} packets at {rate:.0f} Hz, {dropped} dropped")

        last = packets[-1]
        state = protocol.DEVICE_STATES.get(last["device_state"], last["device_state"])
        self.record("state.disarmed", PASS if last["device_state"] == 0 else FAIL, f"device state {state}")

        hpa = last["baro_pressure_dHPa"] / 10
        self.record("sensors.baro", PASS if 300 <= hpa <= 1100 else FAIL,
                    f"{hpa:.1f} hPa, {last['baro_temp_dC'] / 10:.1f} °C, altitude {last['altitude_cm'] / 100:.1f} m")
        cpu = last["mcu_temp_dC"] / 10
        self.record("sensors.cpu_temp", PASS if 0 < cpu < 85 else FAIL, f"CPU {cpu:.1f} °C")
        self.record("sensors.throttle_pot", PASS if 0 <= last["pot_raw"] <= 4095 else FAIL,
                    f"pot raw {last['pot_raw']}")

        self.check_esc(last)
        self.check_bms(last)

    def check_esc(self, p: dict) -> None:
        if p["esc_status"] != protocol.TELEMETRY_CONNECTED:
            self.record("telemetry.esc", FAIL if self.args.expect_esc else SKIP, "no ESC telemetry (ESC not connected)")
            return
        problems = []
        if p["esc_error"] or p["esc_selfcheck"]:
            problems.append(f"error 0x{p['esc_error']:04x} selfcheck 0x{p['esc_selfcheck']:04x}")
        if p["esc_rpm"] != 0:
            problems.append(f"motor turning ({p['esc_rpm']} eRPM) while disarmed")
        detail = (f"{p['esc_volts_dV'] / 10:.1f} V, MOS {p['esc_temp_mos_dC'] / 10:.0f} °C, "
                  f"hw 0x{p['esc_hardware_id']:04x} fw 0x{p['esc_fw_version']:04x}, SN {p['esc_sn_code'] or '?'}")
        self.record("telemetry.esc", FAIL if problems else PASS, "; ".join(problems + [detail]))

    def check_bms(self, p: dict) -> None:
        if p["bms_status"] != protocol.TELEMETRY_CONNECTED:
            self.record("telemetry.bms", FAIL if self.args.expect_bms else SKIP, "no BMS telemetry (battery not connected)")
            return
        cells = [mv for mv in p["bms_cell_voltages_mV"] if mv]
        problems = []
        if not 0 <= p["bms_soc"] <= 100:
            problems.append(f"SOC {p['bms_soc']}%")
        if p["bms_fail_level"]:
            problems.append(f"fail level {p['bms_fail_level']}")
        if cells and not all(2500 <= mv <= 4300 for mv in cells):
            problems.append(f"cell out of range ({min(cells)}-{max(cells)} mV)")
        if cells and abs(sum(cells) / 100 - p["bms_volts_dV"]) > 0.05 * p["bms_volts_dV"]:
            problems.append(f"cells sum to {sum(cells) / 1000:.1f} V but pack reports {p['bms_volts_dV'] / 10:.1f} V")
        detail = (f"{p['bms_volts_dV'] / 10:.1f} V, SOC {p['bms_soc']}%, {len(cells)} cells "
                  f"{p['bms_cell_min_mV']}-{p['bms_cell_max_mV']} mV, id {p['bms_battery_id'] or '?'}")
        self.record("telemetry.bms", FAIL if problems else PASS, "; ".join(problems + [detail]))

    def check_ota_service(self, services: dict) -> None:
        chars = services.get(protocol.OTA_SERVICE_UUID)
        if chars is None:
            self.record("ota.ble_service", FAIL, "OTA service not advertised")
            return
        missing = [name for name, uuid in protocol.OTA_CHARACTERISTIC_UUIDS.items() if uuid not in chars]
        self.record("ota.ble_service", FAIL if missing else PASS,
                    f"missing characteristics: {', '.join(missing)}" if missing
                    else "OTA service and all 4 characteristics present")

    async def check_ble_qc_record(self, ble: BleLink, services: dict) -> None:
        if protocol.QC_RECORD_UUID not in services.get(protocol.CONFIG_SERVICE_UUID, []):
            self.record("qc.stored_record", SKIP, "this firmware has no QC record characteristic")
            return
        blob = b""
        while len(blob) < 4096:
            await ble.write(protocol.QC_RECORD_UUID, len(blob).to_bytes(4, "little"))
            page = await ble.read(protocol.QC_RECORD_UUID)
            if not page:
                break
            blob += page
        text = blob.split(b"\0", 1)[0].decode("utf-8", errors="replace")
        if not text:
            self.record("qc.stored_record", FAIL, "no QC record stored (unit never completed first-boot QC)")
            return
        record = protocol.parse_json_line(text)
        if record is None or not protocol.is_qc_record(record):
            self.record("qc.stored_record", FAIL, f"stored QC record is not valid JSON: {text[:80]!r}")
            return
        self.evaluate_qc_record("qc.stored_record", record)


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--port", help="serial port; auto-detected by USB VID/PID if omitted")
    p.add_argument("--expect-version", default=repo_firmware_version(),
                   help="firmware MAJOR.MINOR the unit must report (default: inc/version.h, %(default)s)")
    p.add_argument("--no-reboot", action="store_true",
                   help="skip the reboot (no settings-persist, reset-reason or boot-log checks)")
    p.add_argument("--run-qc", action="store_true",
                   help="send run_qc and wait for the factory QC record (operator follows the screen prompts)")
    p.add_argument("--ble", action="store_true", help="also run the BLE checks (needs bleak and a bonded controller)")
    p.add_argument("--ble-address", help="BLE address/UUID of the controller (default: first 'OpenPPG SP140' seen)")
    p.add_argument("--expect-esc", action="store_true", help="fail if the ESC is not reporting telemetry")
    p.add_argument("--expect-bms", action="store_true", help="fail if the BMS is not reporting telemetry")
    p.add_argument("--esptool", action="store_true",
                   help="read the flash partition table with esptool at the end (resets the unit)")
    p.add_argument("--firmware", help="firmware.bin to check against the OTA slot size (with --esptool)")
    p.add_argument("--json-out", help="write the results as JSON to this file")
    p.add_argument("-v", "--verbose", action="store_true", help="echo serial traffic")
    p.add_argument("--dry-run", action="store_true",
                   help="run every check against a simulated controller instead of hardware")
    p.add_argument("--fault", action="append", default=[],
                   help="with --dry-run, inject a fault into the simulated controller (repeatable); "
                        "see fake_controller.FAULTS")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    log = (lambda line: print(f"        {line}")) if args.verbose else (lambda line: None)

    fake = None
    if args.dry_run:
        from fake_controller import FakeController

        fake = FakeController(version=repo_firmware_version() or "8.1", faults=args.fault)
        fake.start()
        args.port = fake.serial_url
        args.ble = True
        args.esptool = True
        timeouts = Timeouts.fast()
        ble_factory = fake.ble_link
        read_partitions = lambda _port: fake.partition_table()  # noqa: E731
    else:
        if args.fault:
            print("--fault only works with --dry-run", file=sys.stderr)
            return 2
        timeouts = Timeouts()
        ble_factory = (lambda: BleLink(args.ble_address)) if args.ble else None
        read_partitions = esptool_read_partition_table

    checker = HardwareCheck(args, SerialLink(args.port, log), ble_factory if args.ble else None,
                            read_partitions, timeouts)
    try:
        checker.run()
    finally:
        if fake is not None:
            fake.stop()

    counts = {s: sum(r.status == s for r in checker.results) for s in (PASS, FAIL, WARN, SKIP)}
    print("\n" + "  ".join(f"{n} {s}" for s, n in counts.items()))
    if args.json_out:
        Path(args.json_out).write_text(json.dumps({
            "tool": "hw_check",
            "time": datetime.now(timezone.utc).isoformat(),
            "port": checker.serial.port_name,
            "dry_run": args.dry_run,
            "results": [asdict(r) for r in checker.results],
        }, indent=2))
    return 1 if checker.failed else 0


if __name__ == "__main__":
    sys.exit(main())
