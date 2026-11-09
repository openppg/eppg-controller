"""Wire formats the SP140 hand controller exposes over USB serial and BLE.

Everything here mirrors a firmware source file. test_hw_check.py parses those
files and fails if they drift apart, so keep the references below accurate:

- USB serial line protocol: src/sp140/device_settings.cpp
  (parse_serial_command_line, send_device_data)
- Boot diagnostics JSON: src/sp140/diagnostics.cpp (diagPacketToJson)
- BLE UUIDs: inc/sp140/ble/ble_ids.h
- Fast-Link telemetry packet: inc/sp140/structs.h (BLE_FastLink_Telemetry)
- First-boot QC record: src/sp140/qc_logic.cpp (qcRecordToJson), on firmware
  that carries first-boot QC
"""

import json
import math
import re
import struct
from typing import Dict, List, Optional, Tuple

# --- USB serial -------------------------------------------------------------

USB_VID = 0x303A  # Espressif
USB_PID = 0x1001  # ESP32-S3 USB Serial/JTAG
BAUD = 115200

# Keys of the `sync` response (send_device_data).
SYNC_KEYS = ("mj_v", "mi_v", "arch", "scr_rt", "ar_tme", "m_tmp", "m_alt", "prf", "sea_p", "thm")

# Keys accepted under {"settings": {...}}, and the sync key each one reads back as.
SETTINGS_TO_SYNC = {
    "screen_rot": "scr_rt",
    "sea_pressure": "sea_p",
    "metric_temp": "m_tmp",
    "metric_alt": "m_alt",
    "performance_mode": "prf",
    "theme": "thm",
}

# Reset reasons in the diag JSON that mean the unit crashed or browned out.
CRASH_RESET_REASONS = {"PANIC", "INT_WDT", "TASK_WDT", "WDT", "BROWNOUT"}

BOOT_BANNER_RE = re.compile(r"Booting up V(?P<ver>[\d.]+) git:(?P<git>\S+)")
OTA_PARTITION_RE = re.compile(
    r"OTA: Running partition (?P<running>\w+) @ 0x(?P<raddr>[0-9a-fA-F]+), "
    r"boot (?P<boot>\w+) @ 0x(?P<baddr>[0-9a-fA-F]+)")
BOOT_COMPLETE = "=== Boot complete ==="
BARO_INIT_ERROR = "Error initializing BMP3xx barometer"
# simple_monitor.cpp: "[<millis>] [<LEVEL>] <sensor> = ..."
MONITOR_ALERT_RE = re.compile(r"^\[\d+\] \[(?P<level>WARN|CRIT)\w*\] (?P<text>(?!.*\(cleared\)$).*)$")


def parse_json_line(line: str) -> Optional[dict]:
    """Return the JSON object on a serial line, or None.

    The firmware prints JSON and plain log text on the same port, so tolerate
    log noise around the object.
    """
    start, end = line.find("{"), line.rfind("}")
    if start < 0 or end <= start:
        return None
    try:
        value = json.loads(line[start:end + 1])
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def is_sync_response(obj: dict) -> bool:
    # Require every key: a reply that lost bytes can still parse as JSON.
    return all(key in obj for key in SYNC_KEYS)


def looks_like_broken_reply(line: str, obj: Optional[dict]) -> bool:
    """A line that started a JSON reply but is not a complete, known one."""
    if "{" not in line:
        return False
    if obj is None:
        return True
    # Parsed, but a sync reply with keys missing is damaged, not a different message.
    return "mj_v" in obj and not is_sync_response(obj)


def is_diag_response(obj: dict) -> bool:
    return obj.get("diag_v") == 1


def is_qc_record(obj: dict) -> bool:
    return obj.get("qc") == 1


def sync_version(obj: dict) -> str:
    return f"{obj['mj_v']}.{obj['mi_v']}"


def non_default_settings(current: dict) -> Dict[str, object]:
    """Pick a value for every setting that differs from what the unit has now."""
    sea = float(current.get("sea_p", 1013.25))
    return {
        "screen_rot": 1 if current.get("scr_rt") == 3 else 3,
        "sea_pressure": 1001.5 if abs(sea - 1001.5) > 0.05 else 1022.5,
        "metric_temp": not bool(current.get("m_tmp", True)),
        "metric_alt": not bool(current.get("m_alt", True)),
        "performance_mode": 0 if current.get("prf") == 1 else 1,
        "theme": 0 if current.get("thm") == 1 else 1,
    }


def settings_from_sync(sync: dict) -> Dict[str, object]:
    return {setting: sync[key] for setting, key in SETTINGS_TO_SYNC.items() if key in sync}


def settings_mismatches(wanted: Dict[str, object], sync: dict) -> List[str]:
    """List the settings whose sync value differs from what was written."""
    problems = []
    for setting, value in wanted.items():
        key = SETTINGS_TO_SYNC[setting]
        got = sync.get(key)
        if isinstance(value, float):
            ok = isinstance(got, (int, float)) and not math.isnan(got) and abs(got - value) < 0.05
        elif isinstance(value, bool):
            ok = isinstance(got, bool) and got == value
        else:
            ok = got == value
        if not ok:
            problems.append(f"{setting}: expected {value!r}, read {got!r}")
    return problems


# --- BLE --------------------------------------------------------------------

ADVERTISED_NAME_PREFIX = "OpenPPG SP140"

CONFIG_SERVICE_UUID = "1779a55b-deb8-4482-a5d1-a12e62146138"
METRIC_ALT_UUID = "df63f19e-7295-4a44-a0dc-184d1afeddf7"
SCREEN_ROTATION_UUID = "9cbab736-3705-4ecf-8086-fb7c5fb86282"
SEA_PRESSURE_UUID = "db47e20e-d8c1-405a-971a-da0a2df7e0f6"
METRIC_TEMP_UUID = "d4962473-a3fb-4754-ad6a-90b079c3fb38"
PERFORMANCE_MODE_UUID = "d76c2e92-3547-4f5f-afb4-515c5c08b06b"
THEME_UUID = "ad0e4309-1eb2-461a-b36c-697b2e1604d2"
DEVICE_STATE_UUID = "8f80bcf5-b58f-4908-b079-e8ad6f5ee257"
QC_RECORD_UUID = "e5c0c0de-0007-4a5c-9b21-7e5c0f1a2b30"


def ble_uuid16(short: str) -> str:
    return f"0000{short.lower()}-0000-1000-8000-00805f9b34fb"


FW_VERSION_UUID = ble_uuid16("2A26")
HW_REVISION_UUID = ble_uuid16("2A27")

FAST_LINK_SERVICE_UUID = "45a17001-b73b-49e1-8b39-5e9ed5e1b930"
FAST_LINK_TELEMETRY_UUID = "45a17002-b73b-49e1-8b39-5e9ed5e1b930"

OTA_SERVICE_UUID = ble_uuid16("8018")
OTA_CHARACTERISTIC_UUIDS = {
    "recv_fw": ble_uuid16("8020"),
    "progress": ble_uuid16("8021"),
    "command": ble_uuid16("8022"),
    "customer": ble_uuid16("8023"),
}

DEVICE_STATES = {0: "DISARMED", 1: "ARMED", 2: "ARMED_CRUISING"}


def parse_fw_version(raw: bytes) -> Tuple[str, Optional[int]]:
    """FW_VERSION characteristic: [major, minor, build u32 LE] (build since 8.0)."""
    if len(raw) < 2:
        raise ValueError(f"firmware version characteristic too short ({len(raw)} bytes)")
    build = struct.unpack_from("<I", raw, 2)[0] if len(raw) >= 6 else None
    return f"{raw[0]}.{raw[1]}", build


# Fast-Link V3 packet, field for field with BLE_FastLink_Telemetry (packed).
FASTLINK_VERSION = 3
BMS_CELLS_NUM = 24
FASTLINK_FIELDS: List[Tuple[str, str]] = [
    ("version", "B"),
    ("packet_id", "I"),
    ("uptime_ms", "I"),
    ("altitude_cm", "i"),
    ("baro_temp_dC", "h"),
    ("baro_pressure_dHPa", "H"),
    ("vario_cmps", "h"),
    ("mcu_temp_dC", "h"),
    ("pot_raw", "H"),
    ("device_state", "B"),
    ("esc_status", "B"),
    ("esc_volts_dV", "H"),
    ("esc_amps_dA", "h"),
    ("esc_phase_current_dA", "h"),
    ("esc_rpm", "i"),
    ("esc_temp_mos_dC", "h"),
    ("esc_temp_cap_dC", "h"),
    ("esc_temp_mcu_dC", "h"),
    ("esc_temp_motor_dC", "h"),
    ("esc_inPWM", "H"),
    ("esc_outPWM", "H"),
    ("esc_v_modulation", "H"),
    ("esc_error", "H"),
    ("esc_selfcheck", "H"),
    ("esc_hardware_id", "H"),
    ("esc_fw_version", "H"),
    ("esc_bootloader_version", "H"),
    ("esc_runtime_ms", "I"),
    ("esc_sn_code", "16s"),
    ("bms_status", "B"),
    ("bms_soc", "B"),
    ("bms_volts_dV", "H"),
    ("bms_amps_dA", "h"),
    ("bms_energy_cycle_mAh", "I"),
    ("bms_battery_cycle", "I"),
    ("bms_fail_level", "B"),
    ("bms_is_charging", "B"),
    ("bms_is_charge_mos", "B"),
    ("bms_is_discharge_mos", "B"),
    ("bms_charge_wire", "B"),
    ("bms_low_soc_warning", "B"),
    ("bms_battery_ready", "B"),
    ("bms_highest_temp_C", "b"),
    ("bms_lowest_temp_C", "b"),
    ("bms_cell_max_mV", "H"),
    ("bms_cell_min_mV", "H"),
    ("bms_voltage_diff_mV", "H"),
    ("bms_battery_id", "33s"),
    ("bms_type", "B"),
    ("bms_cell_voltages_mV", f"{BMS_CELLS_NUM}H"),
    ("bms_temp_sensors_C", "8b"),
]
FASTLINK_STRUCT = struct.Struct("<" + "".join(code for _, code in FASTLINK_FIELDS))
FASTLINK_SIZE = 198  # static_assert in structs.h
TELEMETRY_CONNECTED = 1  # TelemetryState::CONNECTED


def parse_fastlink(raw: bytes) -> dict:
    """Decode one Fast-Link notification into a dict keyed by struct field name."""
    if len(raw) < FASTLINK_STRUCT.size:
        raise ValueError(f"Fast-Link packet is {len(raw)} bytes, expected {FASTLINK_STRUCT.size}")
    values = list(FASTLINK_STRUCT.unpack_from(raw))
    packet = {}
    for name, code in FASTLINK_FIELDS:
        count = int(code[:-1]) if code[:-1] and code[-1] != "s" else 1
        if count > 1:
            packet[name] = values[:count]
            del values[:count]
        else:
            packet[name] = values.pop(0)
    for name in ("esc_sn_code", "bms_battery_id"):
        packet[name] = packet[name].split(b"\0", 1)[0].decode("ascii", errors="replace")
    return packet


def build_fastlink(**fields) -> bytes:
    """Encode a Fast-Link packet; unspecified fields are zero. Used by the fake."""
    values = []
    for name, code in FASTLINK_FIELDS:
        count = int(code[:-1]) if code[:-1] and code[-1] != "s" else 1
        value = fields.get(name)
        if code.endswith("s"):
            values.append((value or "").encode("ascii") if isinstance(value, str) else (value or b""))
        elif count > 1:
            values.extend(value if value is not None else [0] * count)
        else:
            values.append(value or 0)
    return FASTLINK_STRUCT.pack(*values)


# --- First-boot QC ------------------------------------------------------------

QC_CHECK_NAMES = ("display", "i2c_baro", "spi_bms", "can_esc", "can_bms", "cpu", "nvs",
                  "throttle", "cal", "buzzer", "vibe", "button")


def qc_failures(record: dict) -> List[str]:
    """Checks in a QC record that did not pass ("skip" means deliberately absent, e.g. no ESC)."""
    checks = record.get("checks") or {}
    return [f"{name}={checks.get(name, 'missing')}" for name in QC_CHECK_NAMES
            if checks.get(name) not in ("pass", "skip")]


# --- Flash partition table (read with esptool) --------------------------------

PARTITION_TABLE_OFFSET = 0x8000
PARTITION_TABLE_SIZE = 0xC00
_PARTITION_ENTRY = struct.Struct("<2sBBII16sI")
APP_TYPE, DATA_TYPE = 0x00, 0x01
OTA_DATA_SUBTYPE = 0x00
OTA_APP_SUBTYPES = range(0x10, 0x20)


def parse_partition_table(raw: bytes) -> List[dict]:
    partitions = []
    for offset in range(0, len(raw) - _PARTITION_ENTRY.size + 1, _PARTITION_ENTRY.size):
        magic, ptype, subtype, address, size, label, _flags = _PARTITION_ENTRY.unpack_from(raw, offset)
        if magic != b"\xaa\x50":
            break
        partitions.append({
            "type": ptype,
            "subtype": subtype,
            "offset": address,
            "size": size,
            "label": label.split(b"\0", 1)[0].decode("ascii", errors="replace"),
        })
    return partitions
