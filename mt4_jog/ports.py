"""Auto-detect the MT4 serial port when none is specified."""

from __future__ import annotations

import re
import time

import serial
from serial.tools import list_ports

from mt4_jog.joints import DEFAULT_BAUD

# Typical USB-UART bridges seen on MT4 / Arduino Mega setups.
_PREFERRED_USB_IDS = (
    (0x1A86, 0x7523),  # CH340
    (0x2341, 0x0010),  # Arduino Mega 2560
    (0x2341, 0x0042),  # Arduino Mega 2560 (older USB id)
    (0x0403, 0x6001),  # FTDI
    (0x10C4, 0xEA60),  # CP210x
)

_PROBE_MARKERS = ("MT4 jog", "MODE=", "--- MT4 jog ---")
_BLUETOOTH_RE = re.compile(r"bluetooth", re.I)


class Mt4PortError(Exception):
    """Raised when no suitable MT4 serial port can be found."""


def _score_port(info: list_ports.ListPortInfo) -> int:
    desc = info.description or ""
    hwid = info.hwid or ""
    combined = f"{desc} {hwid}".lower()

    if _BLUETOOTH_RE.search(combined):
        return -1000

    score = 0
    if info.vid is not None and info.pid is not None:
        if (info.vid, info.pid) in _PREFERRED_USB_IDS:
            score += 100
        if info.vid == 0x1A86:
            score += 50

    for needle, pts in (
        ("ch340", 40),
        ("usb-serial", 20),
        ("arduino", 30),
        ("mega", 25),
        ("wch", 20),
        ("ftdi", 20),
        ("cp210", 20),
    ):
        if needle in combined:
            score += pts

    return score


def list_port_candidates() -> list[tuple[int, str, str]]:
    """Return `(score, device, description)` sorted best-first."""
    ranked: list[tuple[int, str, str]] = []
    for info in list_ports.comports():
        if not info.device:
            continue
        ranked.append((_score_port(info), info.device, info.description or ""))
    ranked.sort(key=lambda row: (-row[0], row[1]))
    return ranked


def probe_port(port: str, baud: int = DEFAULT_BAUD, timeout: float = 1.5) -> bool:
    """Return True if `port` responds like MT4 jog firmware to `?`."""
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.3
        ser.dtr = False
        ser.rts = False
        ser.open()
    except serial.SerialException:
        return False

    try:
        time.sleep(0.4)
        if ser.in_waiting:
            ser.read(ser.in_waiting)
        ser.write(b"?\n")
        ser.flush()
        deadline = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < deadline:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                if any(marker.encode() in buf for marker in _PROBE_MARKERS):
                    return True
            else:
                time.sleep(0.05)
        text = buf.decode("utf-8", errors="replace")
        return any(marker in text for marker in _PROBE_MARKERS)
    finally:
        ser.close()


def find_mt4_port(*, baud: int = DEFAULT_BAUD, probe: bool = True) -> str | None:
    """Pick the most likely MT4 COM port, optionally confirming with `?`."""
    candidates = list_port_candidates()
    if not candidates:
        return None

    viable = [row for row in candidates if row[0] > -1000] or candidates

    if probe:
        for _, device, _ in viable:
            if probe_port(device, baud=baud):
                return device

    if viable and viable[0][0] > 0:
        return viable[0][1]
    return None


def resolve_port(port: str | None, *, baud: int = DEFAULT_BAUD, probe: bool = True) -> str:
    """Use `port` when given, otherwise auto-detect the MT4 serial port."""
    if port:
        return port
    found = find_mt4_port(baud=baud, probe=probe)
    if found:
        return found
    raise Mt4PortError(
        "Could not find an MT4 serial port. Plug in the arm or pass --port COMx."
    )


def port_display(port: str, *, baud: int = DEFAULT_BAUD, explicit: bool) -> str:
    suffix = "" if explicit else " (auto-detected)"
    return f"Port {port} @ {baud}{suffix}"
