"""Transports to a hand controller: USB serial (pyserial) and BLE (bleak)."""

import asyncio
import time
from typing import Callable, Dict, List, Optional

import protocol


class LinkError(Exception):
    pass


def find_controller_port() -> Optional[str]:
    from serial.tools import list_ports

    matches = sorted(p.device for p in list_ports.comports()
                     if p.vid == protocol.USB_VID and p.pid == protocol.USB_PID)
    return matches[0] if matches else None


class SerialLink:
    """Line-oriented USB serial link that survives the controller rebooting.

    `port` is a device path, a pyserial URL such as socket://127.0.0.1:5000
    (used by the dry run), or None to auto-detect by USB VID/PID.
    """

    def __init__(self, port: Optional[str], log: Callable[[str], None] = lambda line: None):
        self.requested_port = port
        self.port_name: Optional[str] = None
        self.serial = None
        self.buffer = bytearray()
        self.log = log
        self.lines: List[str] = []  # every line received, for the report

    def open(self, timeout: float = 5.0) -> None:
        import serial

        deadline = time.monotonic() + timeout
        last_error = "no controller found (USB VID 303A, PID 1001)"
        while time.monotonic() < deadline:
            port = self.requested_port or find_controller_port()
            if port:
                try:
                    # Keep DTR/RTS low: toggling them on the ESP32-S3 USB
                    # Serial/JTAG port resets the chip, possibly into the bootloader.
                    link = serial.serial_for_url(port, do_not_open=True, baudrate=protocol.BAUD,
                                                 timeout=0.1, write_timeout=1.0)
                    link.dtr = False
                    link.rts = False
                    link.open()
                    self.serial = link
                    self.port_name = port
                    self.buffer.clear()
                    return
                except (serial.SerialException, OSError) as exc:
                    last_error = f"{port}: {exc}"
            time.sleep(0.25)
        raise LinkError(last_error)

    def close(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:  # noqa: BLE001 - closing a vanished port can raise anything
                pass
        self.serial = None

    def send(self, payload: str) -> None:
        if self.serial is None:
            raise LinkError("serial port is not open")
        self.log(f">> {payload}")
        self.serial.write(payload.encode("utf-8") + b"\n")
        self.serial.flush()

    def read_line(self, timeout: float) -> Optional[str]:
        """Return the next line, or None on timeout. Raises LinkError if the port drops."""
        import serial

        deadline = time.monotonic() + timeout
        while True:
            newline = self.buffer.find(b"\n")
            if newline >= 0:
                raw = bytes(self.buffer[:newline])
                del self.buffer[:newline + 1]
                line = raw.decode("utf-8", errors="replace").rstrip("\r")
                self.lines.append(line)
                self.log(f"<< {line}")
                return line
            if time.monotonic() >= deadline:
                return None
            try:
                chunk = self.serial.read(self.serial.in_waiting or 1)
            except (serial.SerialException, OSError) as exc:
                self.close()
                raise LinkError(f"serial port dropped: {exc}") from exc
            if chunk:
                self.buffer.extend(chunk)

    def request(self, payload: str, match: Callable[[dict], bool], timeout: float,
                retries: int = 3) -> dict:
        """Send a JSON command and wait for the first JSON reply that `match`es."""
        per_try = timeout / retries
        garbled: List[str] = []
        for _ in range(retries):
            self.send(payload)
            deadline = time.monotonic() + per_try
            while time.monotonic() < deadline:
                line = self.read_line(deadline - time.monotonic())
                if line is None:
                    break
                obj = protocol.parse_json_line(line)
                if obj is not None and match(obj):
                    return obj
                if protocol.looks_like_broken_reply(line, obj):
                    garbled.append(line)
        if garbled:
            # The firmware answered but bytes were lost on the way. Report it
            # as a fault rather than retrying until a clean copy slips through.
            raise LinkError(f"garbled reply to {payload} ({len(garbled)} damaged line(s), "
                            f"e.g. {len(garbled[-1])} bytes: {garbled[-1][:160]!r})")
        raise LinkError(f"no reply to {payload} within {timeout:.0f}s")

    def reboot_and_capture(self, reconnect_timeout: float, boot_timeout: float) -> List[str]:
        """Send `reboot`, reopen the port when the unit re-enumerates, and return
        the boot log lines seen up to "=== Boot complete ===" (or the timeout).

        The first lines of the boot log are often printed before the host has
        reopened the port and are lost; callers must not require them.
        """
        self.send('{"command":"reboot"}')
        captured: List[str] = []
        # Drain until the port drops (USB re-enumeration) or goes quiet.
        quiet_deadline = time.monotonic() + 3.0
        try:
            while time.monotonic() < quiet_deadline:
                line = self.read_line(0.2)
                if line is not None:
                    captured.append(line)
                    if protocol.BOOT_COMPLETE in line:
                        return captured
        except LinkError:
            pass
        self.close()
        time.sleep(0.5)
        self.open(timeout=reconnect_timeout)
        deadline = time.monotonic() + boot_timeout
        while time.monotonic() < deadline:
            try:
                line = self.read_line(deadline - time.monotonic())
            except LinkError:
                self.open(timeout=reconnect_timeout)
                continue
            if line is None:
                break
            captured.append(line)
            if protocol.BOOT_COMPLETE in line:
                break
        return captured

    def wait_for(self, match: Callable[[dict], bool], timeout: float) -> Optional[dict]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.read_line(deadline - time.monotonic())
            if line is None:
                return None
            obj = protocol.parse_json_line(line)
            if obj is not None and match(obj):
                return obj
        return None


class BleLink:
    """Async BLE client for a controller this computer is already bonded with.

    The controller only accepts encrypted reads/writes, and only accepts a new
    bond during the 60 s pairing window after power-up, so pair it with the OS
    Bluetooth settings first (see README).
    """

    def __init__(self, address: Optional[str], scan_timeout: float = 10.0):
        self.address = address
        self.scan_timeout = scan_timeout
        self.client = None
        self.name: Optional[str] = None

    async def connect(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as exc:
            raise LinkError("BLE checks need bleak: pip install bleak") from exc

        if self.address:
            device = await BleakScanner.find_device_by_address(self.address, timeout=self.scan_timeout)
        else:
            device = await BleakScanner.find_device_by_filter(
                lambda d, adv: (d.name or adv.local_name or "").startswith(protocol.ADVERTISED_NAME_PREFIX),
                timeout=self.scan_timeout)
        if device is None:
            raise LinkError("no OpenPPG controller advertising (is it bonded to this computer and powered?)")
        self.name = device.name
        self.client = BleakClient(device)
        await self.client.connect()

    async def disconnect(self) -> None:
        if self.client is not None:
            await self.client.disconnect()

    def services(self) -> Dict[str, List[str]]:
        return {s.uuid.lower(): [c.uuid.lower() for c in s.characteristics]
                for s in self.client.services}

    async def read(self, uuid: str) -> bytes:
        return bytes(await self.client.read_gatt_char(uuid))

    async def write(self, uuid: str, data: bytes) -> None:
        await self.client.write_gatt_char(uuid, data, response=True)

    async def collect_notifications(self, uuid: str, seconds: float) -> List[bytes]:
        packets: List[bytes] = []
        await self.client.start_notify(uuid, lambda _char, data: packets.append(bytes(data)))
        await asyncio.sleep(seconds)
        await self.client.stop_notify(uuid)
        return packets
