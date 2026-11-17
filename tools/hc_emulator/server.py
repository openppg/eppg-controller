#!/usr/bin/env python3
"""Hand-controller emulator server.

Starts the hc_emulator binary (the real LVGL UI + climb efficiency estimator
running natively on a virtual clock) and serves a browser dashboard that
drives it with a paramotor flight simulator or a recorded flight log.

    python tools/hc_emulator/server.py [--port 8140] [--exe PATH] [--logs DIR]

Build the binary first (it is part of the screenshot-test CMake project):

    cmake -S test/test_screenshots -B build-screenshot -G Ninja
    cmake --build build-screenshot --target hc_emulator

No third-party Python packages are needed.
"""

import argparse
import base64
import json
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
WEB = HERE / "web"
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".csv": "text/csv; charset=utf-8",
}


def find_exe():
    for name in ("hc_emulator.exe", "hc_emulator"):
        path = REPO / "build-screenshot" / name
        if path.exists():
            return path
    return None


class Emulator:
    """Owns the hc_emulator process; one request talks to it at a time."""

    def __init__(self, exe):
        self.exe = str(exe)
        self.lock = threading.Lock()
        self.proc = None
        self._start()

    def _start(self):
        self.proc = subprocess.Popen(
            [self.exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    def run(self, text):
        """Send command lines; return (header, frame) for the last render."""
        lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
        renders = sum(1 for ln in lines if ln == "render")
        with self.lock:
            if self.proc.poll() is not None:
                self._start()
            self.proc.stdin.write(("\n".join(lines) + "\n").encode())
            self.proc.stdin.flush()
            header, frame = None, b""
            for _ in range(renders):
                raw = self.proc.stdout.readline()
                if not raw:
                    raise RuntimeError("hc_emulator exited")
                header = json.loads(raw)
                frame = self.proc.stdout.read(header["frame"])
            return header, frame


class Handler(BaseHTTPRequestHandler):
    emulator = None
    log_dirs = []

    def log_message(self, fmt, *args):  # keep the console quiet
        pass

    def _send(self, code, body, content_type="application/json"):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _log_files(self):
        files = {}
        for folder in self.log_dirs:
            for path in sorted(Path(folder).glob("*.csv")):
                files[path.name] = path
        return files

    def do_GET(self):
        path = unquote(urlparse(self.path).path)
        if path == "/api/logs":
            names = [{"name": n, "size": p.stat().st_size}
                     for n, p in self._log_files().items()]
            return self._send(200, json.dumps(names))
        if path.startswith("/api/logs/"):
            match = self._log_files().get(path[len("/api/logs/"):])
            if not match:
                return self._send(404, '{"error":"no such log"}')
            return self._send(200, match.read_bytes(), CONTENT_TYPES[".csv"])

        rel = "index.html" if path in ("", "/") else path.lstrip("/")
        target = (WEB / rel).resolve()
        if WEB not in target.parents or not target.is_file():
            return self._send(404, "not found", "text/plain")
        ctype = CONTENT_TYPES.get(target.suffix, "application/octet-stream")
        return self._send(200, target.read_bytes(), ctype)

    def do_POST(self):
        if urlparse(self.path).path != "/api/cmd":
            return self._send(404, '{"error":"unknown endpoint"}')
        length = int(self.headers.get("Content-Length", 0))
        text = self.rfile.read(length).decode()
        try:
            header, frame = self.emulator.run(text)
        except (RuntimeError, OSError, ValueError) as exc:
            return self._send(500, json.dumps({"error": str(exc)}))
        body = {"state": header}
        if header is not None:
            body["frame"] = base64.b64encode(frame).decode()
        return self._send(200, json.dumps(body))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=8140)
    parser.add_argument("--exe", type=Path, default=None,
                        help="hc_emulator binary (default: build-screenshot/)")
    parser.add_argument("--logs", type=Path, action="append", default=[],
                        help="folder of flight-log CSVs to offer for replay")
    args = parser.parse_args()

    exe = args.exe or find_exe()
    if exe is None or not Path(exe).exists():
        sys.exit("hc_emulator binary not found - build it first (see --help)")

    Handler.emulator = Emulator(exe)
    Handler.log_dirs = [d for d in args.logs if d.is_dir()]
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Hand-controller emulator: http://127.0.0.1:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
