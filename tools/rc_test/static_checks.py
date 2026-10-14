#!/usr/bin/env python3
"""Static release checks on the source tree and the built firmware image.

Run after `pio run -e OpenPPG-CESP32S3-CAN-SP140`. Writes a JSON list of
{"id", "name", "status", "detail"} entries (status: pass | fail | warn).

  python3 tools/rc_test/static_checks.py --json out.json
"""

import argparse
import json
import os
import re
import shutil
import struct
import subprocess
import sys

ENV = "OpenPPG-CESP32S3-CAN-SP140"
results = []


def add(cid, name, ok, detail, warn_only=False):
    status = "pass" if ok else ("warn" if warn_only else "fail")
    results.append({"id": cid, "name": name, "status": status, "detail": detail})


def read(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def strip_comments(src):
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


def firmware_sources(root):
    for base in ("src/sp140", "inc/sp140"):
        for dirpath, _, files in os.walk(os.path.join(root, base)):
            for name in files:
                if name.endswith((".cpp", ".h")):
                    yield os.path.join(dirpath, name)


def function_body(src, name):
    """Body of `void name(...) {` up to its matching brace, or None."""
    m = re.search(r"\bvoid\s+" + re.escape(name) + r"\s*\([^)]*\)\s*\{", src)
    if not m:
        return None
    depth, i = 1, m.end()
    while i < len(src) and depth:
        depth += {"{": 1, "}": -1}.get(src[i], 0)
        i += 1
    return src[m.end():i - 1]


def parse_partitions(path):
    parts = []
    data = open(path, "rb").read()
    for off in range(0, len(data), 32):
        entry = data[off:off + 32]
        if len(entry) < 32 or entry[:2] != b"\xaa\x50":
            break
        ptype, subtype, offset, size = struct.unpack("<BBII", entry[2:12])
        label = entry[12:28].split(b"\0")[0].decode()
        parts.append({"type": ptype, "subtype": subtype, "offset": offset,
                      "size": size, "label": label})
    return parts


def find_tool(name):
    pio = os.path.expanduser("~/.platformio/packages/toolchain-xtensa-esp32s3/bin")
    cand = os.path.join(pio, name)
    return cand if os.path.exists(cand) else shutil.which(name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.getcwd())
    ap.add_argument("--json", required=True)
    args = ap.parse_args()
    root = args.root
    build = os.path.join(root, ".pio", "build", ENV)

    # --- version -----------------------------------------------------------
    ver = read(os.path.join(root, "inc/version.h"))
    major = int(re.search(r"#define VERSION_MAJOR (\d+)", ver).group(1))
    minor = int(re.search(r"#define VERSION_MINOR (\d+)", ver).group(1))
    add("static.version", "Version is 8.1", (major, minor) == (8, 1),
        f"inc/version.h says {major}.{minor}")

    try:
        commits = int(subprocess.check_output(
            ["git", "-C", root, "rev-list", "--count", "HEAD"], text=True).strip())
        shallow = subprocess.check_output(
            ["git", "-C", root, "rev-parse", "--is-shallow-repository"], text=True).strip()
    except (subprocess.CalledProcessError, OSError):
        commits, shallow = None, "unknown"

    fw_path = os.path.join(build, "firmware.bin")
    if not os.path.exists(fw_path):
        add("static.firmware", "Firmware image built", False, f"{fw_path} missing")
        return finish(args.json)
    fw = open(fw_path, "rb").read()
    add("static.image_header", "Image header is ESP32-S3",
        fw[0] == 0xE9 and struct.unpack("<H", fw[12:14])[0] == 9,
        f"magic 0x{fw[0]:02X}, chip id {struct.unpack('<H', fw[12:14])[0]} (ESP32-S3 = 9)")

    i = fw.find(b"OPPGFWV1")
    if i < 0:
        add("static.marker", "Firmware marker (OPPGFWV1) present", False,
            "magic not found; the OTA backend could not identify this image")
    else:
        m_major, m_minor, m_build = struct.unpack("<BBI", fw[i + 8:i + 14])
        ok = (m_major, m_minor) == (major, minor) and m_build > 1
        detail = f"marker says {m_major}.{m_minor} build {m_build}"
        if commits is not None:
            ok = ok and m_build == commits
            detail += f"; git history has {commits} commits (shallow={shallow})"
        add("static.marker", "Firmware marker matches version and build number", ok, detail)

    # --- OTA slot fits -------------------------------------------------------
    parts_path = os.path.join(build, "partitions.bin")
    parts = parse_partitions(parts_path) if os.path.exists(parts_path) else []
    ota_slots = [p for p in parts if p["type"] == 0 and 0x10 <= p["subtype"] <= 0x1F]
    if ota_slots:
        slot = min(p["size"] for p in ota_slots)
        used = len(fw) / slot
        add("static.ota_fit", "Image fits the OTA slot with headroom", used < 0.90,
            f"{len(fw):,} bytes of {slot:,} ({used:.1%}); {len(ota_slots)} OTA slots")
    else:
        add("static.ota_fit", "Image fits the OTA slot with headroom", False,
            "no OTA app partitions found in partitions.bin")
    has_otadata = any(p["type"] == 1 and p["subtype"] == 0 for p in parts)
    add("static.otadata", "Partition table has otadata (needed for rollback)", has_otadata,
        ", ".join(f"{p['label']}@0x{p['offset']:X}/{p['size'] // 1024}K" for p in parts))
    has_coredump = any(p["type"] == 1 and p["subtype"] == 3 for p in parts)
    add("static.coredump_part", "Coredump partition exists for ELF coredumps", has_coredump,
        "coredump partition present" if has_coredump else
        "CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH is set but the table has no coredump partition",
        warn_only=True)

    # --- rollback hook -------------------------------------------------------
    nm = find_tool("xtensa-esp32s3-elf-nm")
    elf = os.path.join(build, "firmware.elf")
    if nm and os.path.exists(elf):
        out = subprocess.check_output([nm, elf], text=True)
        line = next((l for l in out.splitlines() if l.endswith(" verifyRollbackLater")), "")
        kind = line.split()[1] if line else "missing"
        add("static.rollback_hook", "verifyRollbackLater() is the app's strong override",
            kind in ("T", "t"), f"nm: {line or 'symbol not found'} (W would mean the weak "
            "Arduino default, which marks every new image valid before setup())")
    else:
        add("static.rollback_hook", "verifyRollbackLater() is the app's strong override", False,
            "nm or firmware.elf not found")

    # --- sdkconfig -----------------------------------------------------------
    sdk = read(os.path.join(root, f"sdkconfig.{ENV}"))
    wanted = {
        "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE": "bootloader rollback",
        "CONFIG_APP_ROLLBACK_ENABLE": "app rollback",
        "CONFIG_COMPILER_OPTIMIZATION_PERF": "-O2 (#124)",
        "CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_ENABLE": "assertions kept",
        "CONFIG_ESP_TASK_WDT_PANIC": "task watchdog panics",
        "CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH": "coredump to flash",
    }
    missing = [k for k in wanted if not re.search(rf"^{k}=y$", sdk, re.M)]
    add("static.sdkconfig", "sdkconfig: rollback, -O2, assertions, WDT panic, coredump",
        not missing, "missing: " + ", ".join(missing) if missing else
        "; ".join(f"{v} on" for v in wanted.values()))
    pio_ini = read(os.path.join(root, "platformio.ini"))
    env_block = pio_ini.split(f"[env:{ENV}]", 1)[1].split("\n[", 1)[0]
    add("static.release_build", "Production env builds in release mode",
        re.search(r"^build_type\s*=\s*release", env_block, re.M) is not None,
        "build_type = release" if "release" in env_block else "build_type is not release")

    # --- source invariants ---------------------------------------------------
    sources = {p: strip_comments(read(p)) for p in firmware_sources(root)}
    rel = lambda p: os.path.relpath(p, root).replace("\\", "/")

    ota = sources.get(os.path.join(root, "src/sp140/ble/ota_service.cpp"), "")
    chars = re.findall(r"createCharacteristic\(([^;]*?)\);", ota, re.S)
    enc = [c for c in chars if "WRITE_ENC" in c]
    add("static.ota_write_enc", "OTA write characteristics require encryption",
        len(chars) >= 2 and all("WRITE_ENC" in c for c in chars if "WRITE" in c),
        f"{len(enc)} of {len([c for c in chars if 'WRITE' in c])} writable OTA "
        "characteristics use WRITE_ENC")

    callers = sorted({rel(p) for p, s in sources.items()
                      if re.search(r"\btemperatureRead\s*\(", s)})
    add("static.tsens_single_writer", "Only system_monitors.cpp reads the CPU temp sensor (#125)",
        callers == ["src/sp140/system_monitors.cpp"], "temperatureRead() in: " + ", ".join(callers))

    main_src = sources.get(os.path.join(root, "src/sp140/main.cpp"), "")
    baro = sorted({rel(p) for p, s in sources.items() if re.search(r"\bsampleBarometer\s*\(", s)
                   and not p.endswith("altimeter.cpp")})
    task_body = function_body(main_src, "barometerTask") or ""
    add("static.baro_single_reader", "Barometer sampled only by barometerTask and factory QC (#126)",
        "sampleBarometer" in task_body and set(baro) <= {"src/sp140/main.cpp",
                                                         "src/sp140/first_boot_qc.cpp"},
        "sampleBarometer() called from: " + ", ".join(baro))

    # FreeRTOS task entry points must never return: with -O2 ESP-IDF 4.4 drops
    # the task-function wrapper that would otherwise catch it.
    all_src = "\n".join(sources.values())
    tasks = sorted(set(re.findall(r"xTaskCreate(?:PinnedToCore)?\(\s*(\w+)\s*,", all_src)))
    bad = []
    for t in tasks:
        body = next((function_body(s, t) for s in sources.values() if function_body(s, t)), None)
        if body is None:
            bad.append(f"{t} (body not found)")
            continue
        loops = re.search(r"for\s*\(\s*;\s*;\s*\)|while\s*\(\s*(true|1)\s*\)", body)
        tail = body.rstrip().rstrip("}").rstrip()
        if not loops and "vTaskDelete" not in body:
            bad.append(t)
        elif re.search(r"\breturn\s*;", body) and "vTaskDelete" not in body:
            bad.append(f"{t} (has return)")
    add("static.tasks_never_return", "Every FreeRTOS task loops forever or deletes itself (-O2)",
        not bad, f"{len(tasks)} tasks checked" + (": " + ", ".join(bad) if bad else ""))

    leftovers = sorted({rel(p) for p, s in sources.items()
                        if re.search(r"(?<![\w.])(millis|delay)\s*\(", s)
                        and not p.endswith("time_utils.h")})
    add("static.time_shim", "Firmware uses timeMillis()/timeDelay() everywhere (#127)",
        not leftovers, "Arduino millis()/delay() still in: " + ", ".join(leftovers)
        if leftovers else "no Arduino millis()/delay() calls left", warn_only=True)

    # --- size from the build log -------------------------------------------
    size = find_tool("xtensa-esp32s3-elf-size")
    if size and os.path.exists(elf):
        out = subprocess.check_output([size, "-A", elf], text=True)
        sec = {m.group(1): int(m.group(2)) for m in re.finditer(r"^(\.\S+)\s+(\d+)", out, re.M)}
        dram = sum(v for k, v in sec.items() if k in (".dram0.data", ".dram0.bss"))
        add("static.ram", "Static DRAM use leaves room for task stacks and heap",
            dram < 200 * 1024, f"{dram:,} bytes static .data+.bss", warn_only=True)

    finish(args.json)


def finish(path):
    with open(path, "w") as f:
        json.dump(results, f, indent=2)
    for r in results:
        print(f"{r['status'].upper():5} {r['name']} -- {r['detail']}")
    sys.exit(1 if any(r["status"] == "fail" for r in results) else 0)


if __name__ == "__main__":
    main()
