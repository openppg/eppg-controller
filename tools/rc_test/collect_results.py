#!/usr/bin/env python3
"""Bundle one run_rc_tests.sh run into rc-results.json for the checklist page.

Reads steps.tsv, the JUnit XML files, static.json and climb.json from --out,
plus the screenshot renders, and embeds the screens as PNG data URIs so the
single JSON file is all the page needs.
"""

import argparse
import base64
import datetime
import glob
import json
import os
import re
import socket
import struct
import subprocess
import xml.etree.ElementTree as ET
import zlib

SUITE_NAMES = {
    "lint": "cpplint (CI rules)",
    "unit": "Native unit tests",
    "emu_ota": "BLE OTA protocol emulation",
    "emu_settings": "Settings storage and factory QC emulation",
    "screens": "Display screenshot regression",
    "firmware": "ESP32-S3 firmware build",
    "static": "Firmware image and source checks",
    "climb": "Climb power bar flight emulation",
}


def git(*args):
    try:
        return subprocess.check_output(["git", *args], text=True).strip()
    except (subprocess.CalledProcessError, OSError):
        return ""


def bmp_to_png(path):
    """24/32-bit uncompressed BMP -> PNG bytes (stdlib only)."""
    data = open(path, "rb").read()
    off = struct.unpack_from("<I", data, 10)[0]
    w, h = struct.unpack_from("<ii", data, 18)
    bpp = struct.unpack_from("<H", data, 28)[0]
    bottom_up = h > 0
    h = abs(h)
    bpr = bpp // 8
    stride = (w * bpr + 3) & ~3
    rows = []
    for y in range(h):
        src_y = h - 1 - y if bottom_up else y
        row = data[off + src_y * stride: off + src_y * stride + w * bpr]
        rgb = bytearray(w * 3)
        for x in range(w):
            b, g, r = row[x * bpr: x * bpr + 3]
            rgb[x * 3: x * 3 + 3] = bytes((r, g, b))
        rows.append(b"\0" + bytes(rgb))
    return png_bytes(w, h, b"".join(rows))


def png_bytes(w, h, raw):
    def chunk(tag, body):
        return (struct.pack(">I", len(body)) + tag + body
                + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def data_uri(png):
    return "data:image/png;base64," + base64.b64encode(png).decode()


def junit_tests(path):
    tests = []
    if not os.path.exists(path):
        return tests
    root = ET.parse(path).getroot()
    for case in root.iter("testcase"):
        suite = case.get("classname") or ""
        name = case.get("name") or ""
        fail = case.find("failure")
        if fail is None:
            fail = case.find("error")
        skipped = case.find("skipped") is not None or case.get("status") in ("notrun", "skipped")
        status = "fail" if fail is not None else ("skip" if skipped else "pass")
        detail = ""
        if fail is not None:
            detail = (fail.get("message") or fail.text or "").strip()[:600]
        tests.append({
            "id": f"{suite}.{name}".strip("."),
            "name": name,
            "group": suite.split(".")[-1].split("/")[-1],
            "status": status,
            "detail": detail,
            "secs": float(case.get("time") or 0),
        })
    return tests


def suite_status(tests, step_status):
    if step_status == "skip":
        return "skip"
    if step_status == "fail" or any(t["status"] == "fail" for t in tests):
        return "fail"
    return "pass"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--env", default="OpenPPG-CESP32S3-CAN-SP140")
    args = ap.parse_args()
    out = args.out

    steps = []
    with open(os.path.join(out, "steps.tsv")) as f:
        for line in f:
            sid, name, status, secs = line.rstrip("\n").split("\t")
            steps.append({"id": sid, "name": name, "status": status, "secs": int(secs)})

    suites = []
    for step in steps:
        sid = step["id"]
        tests = []
        if os.path.exists(os.path.join(out, f"{sid}.xml")):
            tests = junit_tests(os.path.join(out, f"{sid}.xml"))
        elif os.path.exists(os.path.join(out, f"{sid}.json")):
            for r in json.load(open(os.path.join(out, f"{sid}.json"))):
                tests.append({"id": r["id"], "name": r["name"], "group": sid,
                              "status": r["status"], "detail": r.get("detail", ""), "secs": 0})
        log = os.path.join(out, "logs", f"{sid}.log")
        tail = ""
        if os.path.exists(log):
            lines = open(log, errors="replace").read().splitlines()
            tail = "\n".join(lines[-25:]) if step["status"] == "fail" else ""
            if sid == "lint":
                errs = [l for l in lines if re.search(r":\d+:\s", l)]
                tests = [{"id": "lint.all", "name": "cpplint over inc/ lib/ src/",
                          "group": "lint", "status": step["status"],
                          "detail": f"{len(errs)} findings" if errs else "clean", "secs": step["secs"]}]
            if sid == "firmware":
                usage = [l.strip() for l in lines if l.startswith(("RAM:", "Flash:"))]
                tests = [{"id": "firmware.build", "name": f"pio run -e {args.env}",
                          "group": "firmware", "status": step["status"],
                          "detail": "; ".join(usage), "secs": step["secs"]}]
        suites.append({"id": sid, "name": SUITE_NAMES.get(sid, step["name"]),
                       "status": suite_status(tests, step["status"]),
                       "secs": step["secs"], "tests": tests, "logTail": tail})

    screens = []
    shot_dir = "test/test_screenshots/output"
    # The harness writes <name>_diff.bmp only for a mismatch; the runner
    # clears the output folder first, so a diff file means this run failed.
    failed_shots = {os.path.basename(p)[:-9]
                    for p in glob.glob(os.path.join(shot_dir, "*_diff.bmp"))}
    for path in sorted(glob.glob(os.path.join(shot_dir, "*.bmp"))):
        name = os.path.basename(path)[:-4]
        if name.endswith("_diff"):
            continue
        entry = {"name": name, "png": data_uri(bmp_to_png(path)),
                 "status": "fail" if name in failed_shots else "pass"}
        diff = os.path.join(shot_dir, name + "_diff.bmp")
        if name in failed_shots and os.path.exists(diff):
            entry["diff"] = data_uri(bmp_to_png(diff))
        screens.append(entry)

    frames = []
    for path in sorted(glob.glob(os.path.join(out, "frames", "*.png"))):
        frames.append({"name": os.path.basename(path)[:-4],
                       "png": data_uri(open(path, "rb").read())})

    ver = open("inc/version.h").read()
    major = re.search(r"VERSION_MAJOR (\d+)", ver).group(1)
    minor = re.search(r"VERSION_MINOR (\d+)", ver).group(1)
    commit = git("rev-parse", "--short", "HEAD")
    now = datetime.datetime.now(datetime.timezone.utc)
    result = {
        "schema": 1,
        "run": {
            "id": f"{commit}-{now.strftime('%Y%m%dT%H%M%SZ')}",
            "commit": commit,
            "subject": git("log", "-1", "--format=%s"),
            "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "build": int(git("rev-list", "--count", "HEAD") or 0),
            "version": f"{major}.{minor}",
            "dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
            "finishedAt": now.isoformat(timespec="seconds"),
            "host": socket.gethostname(),
        },
        "suites": suites,
        "screens": screens,
        "frames": frames,
    }
    path = os.path.join(out, "rc-results.json")
    with open(path, "w") as f:
        json.dump(result, f)

    total = sum(len(s["tests"]) for s in suites)
    failed = [t["name"] for s in suites for t in s["tests"] if t["status"] == "fail"]
    bad_steps = [s["name"] for s in suites if s["status"] == "fail" and not s["tests"]]
    print(f"\n{total} automated checks, {len(failed)} failed"
          + (f", {len(bad_steps)} steps errored" if bad_steps else ""))
    for name in failed[:20] + bad_steps:
        print(f"  FAIL {name}")
    print(f"\nResults: {os.path.abspath(path)} ({os.path.getsize(path) // 1024} KB)")


if __name__ == "__main__":
    main()
