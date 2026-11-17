"""Tests for hw_check: end-to-end dry runs against the simulated controller, and
checks that protocol.py still matches the firmware sources it mirrors.

    python -m unittest discover -s tools/hw_check -v
"""

import contextlib
import io
import json
import re
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import fake_controller  # noqa: E402
import hw_check  # noqa: E402
import protocol  # noqa: E402

REPO = HERE.parents[1]


def source(relative: str) -> str:
    return (REPO / relative).read_text(encoding="utf-8")


def function_body(text: str, signature: str) -> str:
    start = text.index(signature)
    depth, i = 0, text.index("{", start)
    for j in range(i, len(text)):
        depth += {"{": 1, "}": -1}.get(text[j], 0)
        if depth == 0:
            return text[i:j + 1]
    raise ValueError(signature)


def dry_run(*extra):
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "results.json"
        with contextlib.redirect_stdout(io.StringIO()):
            code = hw_check.main(["--dry-run", "--json-out", str(out), *extra])
        results = {r["id"]: r for r in json.loads(out.read_text())["results"]}
    return code, results


class DryRunTest(unittest.TestCase):
    def test_healthy_unit_passes_every_check(self):
        code, results = dry_run("--run-qc", "--expect-esc", "--expect-bms")
        not_passed = {k: r["detail"] for k, r in results.items() if r["status"] != "PASS"}
        self.assertEqual(not_passed, {})
        self.assertEqual(code, 0)
        for check in ("fw.version", "settings.persist", "settings.restore", "diag.reset_reason",
                      "qc.run", "qc.stored_record", "telemetry.bms", "ota.ble_service",
                      "ota.partition_table"):
            self.assertIn(check, results)

    def test_each_fault_fails_its_check(self):
        cases = {
            "settings-not-persisted": "settings.persist",
            "no-sanitize": "settings.sanitize",
            "crash-history": "diag.crash_history",
            "version-mismatch": "ble.fw_version",
            "ota-pending": "ota.boot_partition",
            "baro-missing": "sensors.baro_init",
            "no-bms": "telemetry.bms",
            "qc-failed": "qc.stored_record",
            "armed": "state.disarmed",
            "garbled-sync": "serial.link",
        }
        self.assertEqual(set(cases), set(fake_controller.FAULTS))
        for fault, check in cases.items():
            with self.subTest(fault=fault):
                code, results = dry_run("--fault", fault, "--expect-bms")
                self.assertEqual(code, 1)
                self.assertEqual(results[check]["status"], "FAIL", results[check]["detail"])

    def test_garbled_reply_is_reported_as_garbled(self):
        code, results = dry_run("--fault", "garbled-sync")
        self.assertEqual(code, 1)
        self.assertIn("garbled reply", results["serial.link"]["detail"])

    def test_missing_bms_is_skipped_unless_expected(self):
        code, results = dry_run("--fault", "no-bms")
        self.assertEqual(code, 0)
        self.assertEqual(results["telemetry.bms"]["status"], "SKIP")

    def test_settings_are_restored_even_when_persist_fails(self):
        fake = fake_controller.FakeController(faults=["settings-not-persisted"])
        before = dict(fake.settings)
        fake.start()
        try:
            args = hw_check.parse_args(["--port", fake.serial_url])
            link = hw_check.SerialLink(fake.serial_url)
            checker = hw_check.HardwareCheck(args, link, None, lambda _p: b"", hw_check.Timeouts.fast())
            with contextlib.redirect_stdout(io.StringIO()):
                checker.run()
        finally:
            fake.stop()
        self.assertEqual(fake.settings, before)

    def test_wrong_expected_version_fails(self):
        code, results = dry_run("--expect-version", "8.0")
        self.assertEqual(results["fw.version"]["status"], "FAIL")


class ParsingTest(unittest.TestCase):
    def test_damaged_sync_replies_are_not_accepted(self):
        full = dict.fromkeys(protocol.SYNC_KEYS, 1)
        partial = {k: v for k, v in full.items() if k != "prf"}
        self.assertTrue(protocol.is_sync_response(full))
        self.assertFalse(protocol.is_sync_response(partial))
        self.assertTrue(protocol.looks_like_broken_reply('{"mj_v":8,"mi_v":1,"arch":""scr_rt":3', None))
        self.assertTrue(protocol.looks_like_broken_reply("{}", partial))
        self.assertFalse(protocol.looks_like_broken_reply("[FASTLINK] stats ok=0", None))

    def test_json_is_found_inside_log_noise(self):
        self.assertEqual(protocol.parse_json_line('junk {"mj_v":8,"mi_v":1} junk'), {"mj_v": 8, "mi_v": 1})
        self.assertIsNone(protocol.parse_json_line("Device data saved to Preferences"))

    def test_monitor_alerts_ignore_cleared_lines(self):
        self.assertTrue(protocol.MONITOR_ALERT_RE.match("[1200] [CRIT_HIGH] ESC_MOS_Temp = 101.00"))
        self.assertIsNone(protocol.MONITOR_ALERT_RE.match("[1300] [WARN_HIGH] ESC_Error = OFF (cleared)"))
        self.assertIsNone(protocol.MONITOR_ALERT_RE.match("[1400] [OK] ESC_MOS_Temp = 40.00"))

    def test_fastlink_round_trip(self):
        raw = protocol.build_fastlink(version=3, packet_id=7, bms_battery_id="ABC",
                                      bms_cell_voltages_mV=list(range(protocol.BMS_CELLS_NUM)))
        packet = protocol.parse_fastlink(raw)
        self.assertEqual((packet["packet_id"], packet["bms_battery_id"]), (7, "ABC"))
        self.assertEqual(packet["bms_cell_voltages_mV"][-1], protocol.BMS_CELLS_NUM - 1)

    def test_partition_table_matches_the_build(self):
        ini = source("platformio.ini")
        csv_name = re.search(r"board_build\.partitions\s*=\s*(\S+)", ini).group(1)
        rows = [line.split(",") for line in source(csv_name).splitlines()
                if line.strip() and not line.startswith("#")]
        expected = [(r[0].strip(), int(r[3], 16), int(r[4], 16)) for r in rows]
        parsed = protocol.parse_partition_table(fake_controller.FakeController().partition_table())
        self.assertEqual([(p["label"], p["offset"], p["size"]) for p in parsed], expected)


class FirmwareDriftTest(unittest.TestCase):
    """protocol.py and the fake must describe the firmware actually in this repo."""

    def test_sync_reply_keys(self):
        body = function_body(source("src/sp140/device_settings.cpp"), "void send_device_data()")
        self.assertEqual(tuple(re.findall(r'doc\["(\w+)"\]', body)), protocol.SYNC_KEYS)

    def test_settings_keys_and_commands(self):
        body = function_body(source("src/sp140/device_settings.cpp"), "void parse_serial_command_line(")
        self.assertEqual(set(re.findall(r'settings\["(\w+)"\]', body)), set(protocol.SETTINGS_TO_SYNC))
        commands = set(re.findall(r'command == "(\w+)"', body))
        self.assertTrue({"reboot", "sync", "diag_sync"} <= commands, commands)

    def test_boot_log_strings(self):
        main = source("src/sp140/main.cpp")
        for text in ('"Booting up V"', '" git:"', '"OTA: Running partition %s @ 0x%06lx, boot %s @ 0x%06lx\\n"',
                     f'"{protocol.BOOT_COMPLETE}"', f'"{protocol.BARO_INIT_ERROR}"'):
            self.assertIn(text, main)

    def test_diag_json_keys(self):
        diag = source("src/sp140/diagnostics.cpp")
        for key in ("diag_v", "cur", "hist", "boot", "rst", "plan", "cdmp", "present", "task", "pc"):
            self.assertIn(f'["{key}"]', diag)
        for reason in protocol.CRASH_RESET_REASONS | {"SOFTWARE", "usb_command_reboot"}:
            self.assertIn(f'"{reason}"', diag)
        fake = fake_controller.FakeController()
        self.assertEqual(set(fake.diag()["cur"]) - set(re.findall(r'obj\["(\w+)"\]', diag)), set())

    def test_ble_uuids(self):
        ids = {name: value.lower() for name, value in
               re.findall(r'#define (\w+_UUID) "([0-9A-Fa-f-]+)"', source("inc/sp140/ble/ble_ids.h"))}
        for name in ("CONFIG_SERVICE_UUID", "METRIC_ALT_UUID", "SCREEN_ROTATION_UUID", "SEA_PRESSURE_UUID",
                     "METRIC_TEMP_UUID", "PERFORMANCE_MODE_UUID", "THEME_UUID", "DEVICE_STATE_UUID",
                     "FAST_LINK_TELEMETRY_SERVICE_UUID", "FAST_LINK_TELEMETRY_UUID"):
            ours = getattr(protocol, name.replace("FAST_LINK_TELEMETRY_SERVICE", "FAST_LINK_SERVICE"))
            self.assertEqual(ours, ids[name], name)
        self.assertEqual(protocol.FW_VERSION_UUID, protocol.ble_uuid16(ids["FW_VERSION_UUID"]))
        ota = re.findall(r'(OTA_\w+_UUID)\("([0-9a-f-]+)"\)', source("inc/sp140/ble/ble_ids.h"))
        ota = {name: value for name, value in ota}
        self.assertEqual(protocol.OTA_SERVICE_UUID, ota["OTA_SERVICE_UUID"])
        self.assertEqual(set(protocol.OTA_CHARACTERISTIC_UUIDS.values()),
                         {v for k, v in ota.items() if k != "OTA_SERVICE_UUID"})
        if "QC_RECORD_UUID" in ids:
            self.assertEqual(protocol.QC_RECORD_UUID, ids["QC_RECORD_UUID"])

    def test_advertised_name(self):
        self.assertIn(f'"{protocol.ADVERTISED_NAME_PREFIX}"', source("src/sp140/ble/ble_core.cpp"))

    def test_fastlink_struct(self):
        structs = source("inc/sp140/structs.h")
        body = structs[structs.index("typedef struct {\n  uint8_t version;"):structs.index("} BLE_FastLink_Telemetry;")]
        codes = {"uint8_t": "B", "int8_t": "b", "uint16_t": "H", "int16_t": "h",
                 "uint32_t": "I", "int32_t": "i", "char": "s"}
        fields = []
        for ctype, name, count in re.findall(r"^\s*(u?int\d+_t|char)\s+(\w+)(?:\[(\w+)\])?;", body, re.M):
            n = protocol.BMS_CELLS_NUM if count == "BMS_CELLS_NUM" else (int(count) if count else 1)
            code = codes[ctype]
            fields.append((name, f"{n}{code}" if n > 1 or code == "s" else code))
        # A uint8_t[N] byte field and a char[N] string pack the same way.
        same_layout = [(name, code.replace("s", "B")) for name, code in protocol.FASTLINK_FIELDS]
        self.assertEqual([(name, code.replace("s", "B")) for name, code in fields], same_layout)
        self.assertIn(f"static constexpr uint8_t FASTLINK_PROTOCOL_VERSION = {protocol.FASTLINK_VERSION};", structs)
        self.assertIn(f"sizeof(BLE_FastLink_Telemetry) == {protocol.FASTLINK_SIZE}", structs)
        self.assertEqual(protocol.FASTLINK_STRUCT.size, protocol.FASTLINK_SIZE)
        cells = re.search(r"#define BMS_CELLS_NUM (\d+)|BMS_CELLS_NUM\s*=\s*(\d+)", "".join(
            p.read_text(encoding="utf-8") for p in (REPO / "inc").rglob("*.h")))
        self.assertEqual(int(cells.group(1) or cells.group(2)), protocol.BMS_CELLS_NUM)

    @unittest.skipUnless((REPO / "src/sp140/qc_logic.cpp").exists(), "firmware has no first-boot QC")
    def test_qc_record_check_names(self):
        qc = source("src/sp140/qc_logic.cpp")
        names = re.findall(r'\\"(\w+)\\":\\"%s\\"', qc[qc.index('"\\"checks\\"'):])
        self.assertEqual(tuple(names), protocol.QC_CHECK_NAMES)
        for status in ("pass", "skip", "fail"):
            self.assertIn(f'return "{status}";', qc)
        self.assertIn('? "PASSED" : "FAILED"', qc)


if __name__ == "__main__":
    unittest.main()
