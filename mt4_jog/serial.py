"""Serial helpers for MT4 jog firmware (DTR/RTS off)."""

from __future__ import annotations

import time

import serial

from mt4_jog.joints import DEFAULT_BAUD
from mt4_jog.ports import resolve_port

BOOT_WAIT_S = 1.0
STATUS_WAIT_S = 2.0
# Opening the CH340 can reset the MCU into its serial bootloader (DTR pulse
# on first open after USB enumeration / power-on). The bootloader swallows
# every line AND restarts its ~9.4s exit timer on each received byte, so
# retrying `?` keeps the app from ever starting. Cure: stay quiet until the
# boot banner, then confirm once (~9.4s window + ~1.3s app boot + margin).
BOOTLOADER_QUIET_S = 13.0
POLL_WAIT_S = 0.3
READ_HARD_LIMIT_S = 1.0
BOOT_BANNER = "MT4 jog firmware ready"


class FirmwareNotReadyError(Exception):
    """Raised when the port opens but application firmware never answers `?`."""


def open_serial(port: str | None = None, baud: int = DEFAULT_BAUD) -> serial.Serial:
    port = resolve_port(port, baud=baud)
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.5
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def read_lines(
    ser: serial.Serial,
    timeout: float = 1.5,
    *,
    hard_limit: float | None = None,
) -> list[str]:
    start = time.monotonic()
    deadline = start + timeout
    cap = start + hard_limit if hard_limit is not None else None
    chunks: list[bytes] = []
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            chunks.append(ser.read(waiting))
            deadline = time.monotonic() + 0.1
            if cap is not None:
                deadline = min(deadline, cap)
        else:
            time.sleep(0.02)
    text = b"".join(chunks).decode("utf-8", "replace")
    return [line.rstrip() for line in text.splitlines() if line.rstrip()]


def send(
    ser: serial.Serial,
    cmd: str,
    wait: float = 1.5,
    *,
    hard_limit: float | None = None,
) -> list[str]:
    ser.write(f"{cmd}\n".encode("ascii"))
    ser.flush()
    time.sleep(0.05)
    return read_lines(ser, wait, hard_limit=hard_limit)


def send_quick(ser: serial.Serial, cmd: str) -> None:
    ser.write(f"{cmd}\n".encode("ascii"))
    ser.flush()


def drain_lines(ser: serial.Serial, buffer: list[str] | None = None) -> list[str]:
    if buffer is None:
        buffer = [""]
    if ser.in_waiting:
        buffer[0] += ser.read(ser.in_waiting).decode("utf-8", "replace")
    lines: list[str] = []
    while "\n" in buffer[0]:
        line, buffer[0] = buffer[0].split("\n", 1)
        line = line.rstrip("\r")
        if line:
            lines.append(line)
    return lines


def _status_looks_alive(lines: list[str]) -> bool:
    return any(
        line.startswith("MODE=") or line.startswith("pos ") or line.startswith("EN=")
        for line in lines
    )


def await_firmware_alive(
    ser: serial.Serial,
    *,
    port_label: str | None = None,
) -> list[str]:
    """Wait until application firmware answers `?`; return those status lines.

    Call once after open_serial() before sending other commands. Handles the
    cold-start bootloader trap: if the first `?` is swallowed, stop sending
    and wait up to BOOTLOADER_QUIET_S for the boot banner, then try `?` again.
    """
    time.sleep(BOOT_WAIT_S)
    read_lines(ser, 0.5, hard_limit=0.6)  # discard any boot banner already present

    label = port_label or getattr(ser, "port", None) or "serial port"
    for attempt in range(2):
        lines = send(
            ser,
            "?",
            wait=STATUS_WAIT_S,
            hard_limit=STATUS_WAIT_S + READ_HARD_LIMIT_S,
        )
        if _status_looks_alive(lines):
            return lines
        if attempt > 0:
            break
        deadline = time.monotonic() + BOOTLOADER_QUIET_S
        while time.monotonic() < deadline:
            quiet_lines = read_lines(
                ser, POLL_WAIT_S, hard_limit=POLL_WAIT_S + READ_HARD_LIMIT_S
            )
            if any(BOOT_BANNER in line for line in quiet_lines):
                # Drain the rest of the banner so the next `?` is clean.
                read_lines(ser, 0.2, hard_limit=0.4)
                break

    raise FirmwareNotReadyError(
        f"{label} opened but the MT4 firmware never responded to `?` "
        "(device stuck in bootloader or not an MT4?)"
    )
