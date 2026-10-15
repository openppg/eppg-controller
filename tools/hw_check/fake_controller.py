"""Simulated SP140 hand controller for `hw_check.py --dry-run` and the tests.

It speaks the firmware's USB serial line protocol on a local TCP socket (so
hw_check uses its real pyserial code path via a socket:// URL, on any OS) and
offers an in-process BLE link with the firmware's GATT layout and Fast-Link
packets. A reboot drops the socket the way USB re-enumeration drops the port.

The display emulator in tools/hc_emulator only models the LVGL screen (no
serial, BLE or NVS), so this fake is what lets CI exercise the checks. The
protocol details come from protocol.py, which test_hw_check.py keeps in step
with the firmware sources.

FAULTS lists the misbehaviours that can be injected to prove each check fails.
"""

import json
import socket
import struct
import threading
import time
from typing import Dict, Iterable, List, Optional

import protocol

FAULTS = {
    "settings-not-persisted": "settings revert to defaults on reboot",
    "no-sanitize": "out-of-range settings are stored as sent",
    "crash-history": "diagnostics retain a TASK_WDT reset with a core dump",
    "version-mismatch": "BLE reports a different firmware version than serial",
    "ota-pending": "boot log shows the unit booting a different partition than it runs",
    "baro-missing": "barometer fails to initialise",
    "no-bms": "BMS sends no telemetry",
    "qc-failed": "stored first-boot QC record failed the button check",
    "armed": "telemetry reports the unit ARMED",
}

DEFAULTS = {"screen_rot": 3, "sea_pressure": 1013.25, "metric_temp": True, "metric_alt": True,
            "performance_mode": 0, "theme": 0}

PARTITIONS = [  # default_8MB.csv
    ("nvs", 0x01, 0x02, 0x9000, 0x5000),
    ("otadata", 0x01, 0x00, 0xE000, 0x2000),
    ("app0", 0x00, 0x10, 0x10000, 0x330000),
    ("app1", 0x00, 0x11, 0x340000, 0x330000),
    ("spiffs", 0x01, 0x82, 0x670000, 0x180000),
    ("coredump", 0x01, 0x03, 0x7F0000, 0x10000),
]

QC_RECORD = {
    "qc": 1, "fw": "8.1", "build": "Oct 15 2026 10:00:00", "result": "PASSED",
    "pot_min": 212, "pot_max": 3890, "span": 3678, "cal_saved": True,
    "baro_hpa": 1009.8, "cpu_c": 41.5, "pack_v": 98.4,
    "esc_hw_id": "0x0201", "esc_sn": "ESC0001", "bms_id": "BMS0001",
    "checks": {name: "pass" for name in protocol.QC_CHECK_NAMES},
}


class FakeController:
    def __init__(self, version: str = "8.1", faults: Iterable[str] = ()):
        unknown = set(faults) - set(FAULTS)
        if unknown:
            raise ValueError(f"unknown fault(s) {sorted(unknown)}; known: {sorted(FAULTS)}")
        self.faults = set(faults)
        self.major, self.minor = (int(x) for x in version.split("."))
        self.settings: Dict[str, object] = dict(DEFAULTS, theme=1, sea_pressure=1009.0)
        self.armed_time = 37
        self.boot = 12
        self.reset_reason, self.planned = "POWERON", "none"
        self.qc_record = dict(QC_RECORD, fw=version)
        if "qc-failed" in self.faults:
            self.qc_record["result"] = "FAILED"
            self.qc_record["checks"] = dict(self.qc_record["checks"], button="fail")
        self.pending_qc = False
        self._server: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    # -- lifecycle ------------------------------------------------------------

    def start(self) -> None:
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", 0))
        self._server.listen(1)
        self._server.settimeout(0.2)
        self._thread = threading.Thread(target=self._serve, name="fake-controller", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self._server:
            self._server.close()

    @property
    def serial_url(self) -> str:
        return f"socket://127.0.0.1:{self._server.getsockname()[1]}"

    # -- serial side ------------------------------------------------------------

    def _serve(self) -> None:
        pending_boot_log: List[str] = []
        while not self._stop.is_set():
            try:
                conn, _ = self._server.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(0.1)
                if pending_boot_log:
                    # pyserial's socket:// open() discards whatever has already
                    # arrived, so give it time to finish, as a real boot would.
                    time.sleep(0.5)
                for line in pending_boot_log:
                    conn.sendall(line.encode() + b"\r\n")
                pending_boot_log = []
                buffer = b""
                while not self._stop.is_set():
                    try:
                        chunk = conn.recv(1024)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    buffer += chunk
                    rebooted = False
                    while b"\n" in buffer:
                        raw, buffer = buffer.split(b"\n", 1)
                        replies, rebooted = self.handle_line(raw.decode().strip())
                        for reply in replies:
                            conn.sendall(reply.encode() + b"\r\n")
                        if rebooted:
                            pending_boot_log = self.boot_log()
                            break
                    if rebooted:
                        break  # drop the connection like USB re-enumeration

    def handle_line(self, line: str):
        """Return (reply lines, rebooted) for one command line, as parse_serial_command_line does."""
        try:
            doc = json.loads(line)
        except json.JSONDecodeError:
            return [], False
        command = doc.get("command")
        with self._lock:
            if command == "reboot":
                self._reboot("SOFTWARE", "usb_command_reboot")
                return ["Rebooting"], True
            if command == "run_qc":
                self._reboot("SOFTWARE", "none")
                self.pending_qc = True
                return ["QC rerun requested - rebooting into factory QC"], True
            if command == "sync":
                return [json.dumps(self.sync())], False
            if command in ("diag_sync", "diag_clear"):
                return [json.dumps(self.diag())], False
            settings = doc.get("settings")
            if isinstance(settings, dict):
                for key in protocol.SETTINGS_TO_SYNC:
                    if key in settings:
                        self.settings[key] = settings[key]
                self._sanitize()
                return ["Device data saved to Preferences", json.dumps(self.sync())], False
        return [], False

    def _sanitize(self) -> None:
        if "no-sanitize" in self.faults:
            return
        s = self.settings
        if s["screen_rot"] not in (1, 3):
            s["screen_rot"] = 3
        if not 300 <= float(s["sea_pressure"]) <= 1200:
            s["sea_pressure"] = 1013.25
        if s["performance_mode"] not in (0, 1):
            s["performance_mode"] = 0
        if s["theme"] not in (0, 1):
            s["theme"] = 0

    def _reboot(self, reason: str, planned: str) -> None:
        self.boot += 1
        self.reset_reason, self.planned = reason, planned
        if "settings-not-persisted" in self.faults:
            self.settings = dict(DEFAULTS)

    def sync(self) -> dict:
        s = self.settings
        return {"mj_v": self.major, "mi_v": self.minor, "arch": "ESP32S3", "scr_rt": s["screen_rot"],
                "ar_tme": self.armed_time, "m_tmp": s["metric_temp"], "m_alt": s["metric_alt"],
                "prf": s["performance_mode"], "sea_p": s["sea_pressure"], "thm": s["theme"]}

    def _diag_record(self, boot: int, rst: str, plan: str) -> dict:
        return {"boot": boot, "rst": rst, "plan": plan, "snap_ok": True, "prev_state": "DISARMED",
                "prev_up_ms": 600000, "thr_age_ms": 4, "ui_age_ms": 30, "bms_age_ms": 90,
                "free_heap": 151000, "min_heap": 120000, "esc_ok": True, "bms_ok": True,
                "cdmp": {"present": False, "valid": False}}

    def diag(self) -> dict:
        history = [self._diag_record(self.boot - n, "POWERON", "none") for n in range(1, 4)]
        if "crash-history" in self.faults:
            crash = self._diag_record(self.boot - 2, "TASK_WDT", "none")
            crash["cdmp"] = {"present": True, "valid": True, "task": "throttle", "pc": 0x42001234,
                             "cause": 0, "vaddr": 0, "bt": [0x42001234]}
            history[1] = crash
        return {"diag_v": 1, "hist_n": len(history),
                "cur": self._diag_record(self.boot, self.reset_reason, self.planned), "hist": history}

    def boot_log(self) -> List[str]:
        running = "app1" if "ota-pending" in self.faults else "app0"
        lines = ["Build date/time: Oct 15 2026 10:00:00",
                 f"Booting up V{self.major}.{self.minor} git:abc1234",
                 f"OTA: Running partition {running} @ 0x{0x340000 if running == 'app1' else 0x10000:06x}, "
                 "boot app0 @ 0x010000",
                 "Initializing hardware..."]
        if "baro-missing" in self.faults:
            lines.append(protocol.BARO_INIT_ERROR)
        if self.pending_qc:
            lines.append(json.dumps(self.qc_record, separators=(",", ":")))
            self.pending_qc = False
        lines += ["Starting UI...", "BLE advertising started", protocol.BOOT_COMPLETE]
        return lines

    def partition_table(self) -> bytes:
        entries = b"".join(struct.pack("<2sBBII16sI", b"\xaa\x50", ptype, subtype, offset, size,
                                       label.encode(), 0)
                           for label, ptype, subtype, offset, size in PARTITIONS)
        return entries + b"\xff" * (protocol.PARTITION_TABLE_SIZE - len(entries))

    # -- BLE side -----------------------------------------------------------------

    def ble_link(self) -> "FakeBleLink":
        return FakeBleLink(self)

    def telemetry_packet(self, packet_id: int) -> bytes:
        no_bms = "no-bms" in self.faults
        cells = [] if no_bms else [4012 + (i % 3) for i in range(protocol.BMS_CELLS_NUM)]
        return protocol.build_fastlink(
            version=protocol.FASTLINK_VERSION, packet_id=packet_id, uptime_ms=60000 + packet_id * 20,
            altitude_cm=152, baro_temp_dC=243,
            baro_pressure_dHPa=0 if "baro-missing" in self.faults else 10098,
            vario_cmps=-3, mcu_temp_dC=415, pot_raw=214, device_state=1 if "armed" in self.faults else 0,
            esc_status=1, esc_volts_dV=963, esc_temp_mos_dC=262, esc_temp_cap_dC=255, esc_temp_mcu_dC=301,
            esc_temp_motor_dC=-32768, esc_hardware_id=0x0201, esc_fw_version=0x0112,
            esc_bootloader_version=0x0003, esc_sn_code="ESC0001",
            bms_status=0 if no_bms else 1, bms_soc=0 if no_bms else 87,
            bms_volts_dV=0 if no_bms else sum(cells) // 100, bms_battery_cycle=42,
            bms_highest_temp_C=26, bms_lowest_temp_C=24,
            bms_cell_max_mV=max(cells, default=0), bms_cell_min_mV=min(cells, default=0),
            bms_voltage_diff_mV=2 if cells else 0, bms_battery_id="" if no_bms else "BMS0001",
            bms_type=2, bms_cell_voltages_mV=cells + [0] * (protocol.BMS_CELLS_NUM - len(cells)))


class FakeBleLink:
    """Same async interface as links.BleLink, backed by a FakeController."""

    def __init__(self, fake: FakeController):
        self.fake = fake
        self.name = "OpenPPG SP140 (simulated)"
        self.qc_offset = 0

    async def connect(self) -> None:
        pass

    async def disconnect(self) -> None:
        pass

    def services(self) -> Dict[str, List[str]]:
        return {
            protocol.CONFIG_SERVICE_UUID: [protocol.METRIC_ALT_UUID, protocol.SCREEN_ROTATION_UUID,
                                           protocol.SEA_PRESSURE_UUID, protocol.METRIC_TEMP_UUID,
                                           protocol.PERFORMANCE_MODE_UUID, protocol.THEME_UUID,
                                           protocol.DEVICE_STATE_UUID, protocol.FW_VERSION_UUID,
                                           protocol.HW_REVISION_UUID, protocol.QC_RECORD_UUID],
            protocol.FAST_LINK_SERVICE_UUID: [protocol.FAST_LINK_TELEMETRY_UUID],
            protocol.OTA_SERVICE_UUID: list(protocol.OTA_CHARACTERISTIC_UUIDS.values()),
        }

    async def read(self, uuid: str) -> bytes:
        fake = self.fake
        if uuid == protocol.FW_VERSION_UUID:
            minor = fake.minor + (1 if "version-mismatch" in fake.faults else 0)
            return bytes([fake.major, minor]) + struct.pack("<I", 1234)
        if uuid == protocol.QC_RECORD_UUID:
            blob = json.dumps(fake.qc_record, separators=(",", ":")).encode() + b"\0"
            return blob[self.qc_offset:self.qc_offset + 240]
        if uuid == protocol.THEME_UUID:
            return bytes([fake.settings["theme"]])
        raise KeyError(uuid)

    async def write(self, uuid: str, data: bytes) -> None:
        fake = self.fake
        if uuid == protocol.THEME_UUID and len(data) == 1 and data[0] <= 1:
            with fake._lock:
                fake.settings["theme"] = data[0]
        elif uuid == protocol.QC_RECORD_UUID and len(data) >= 4:
            self.qc_offset = int.from_bytes(data[:4], "little")

    async def collect_notifications(self, uuid: str, seconds: float) -> List[bytes]:
        count = max(1, int(seconds * 50))
        return [self.fake.telemetry_packet(n) for n in range(count)]
