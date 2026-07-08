"""Serial helpers for MT4 jog firmware (DTR/RTS off)."""

from __future__ import annotations

import time

import serial

from mt4_jog.joints import DEFAULT_BAUD, DEFAULT_PORT


def open_serial(port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD) -> serial.Serial:
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
