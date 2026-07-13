"""Serial helpers for MT4 jog firmware (DTR/RTS off)."""

from __future__ import annotations

import time

import serial

from mt4_jog.joints import DEFAULT_BAUD
from mt4_jog.ports import resolve_port


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


def read_lines(ser: serial.Serial, timeout: float = 1.5) -> list[str]:
    deadline = time.monotonic() + timeout
    chunks: list[bytes] = []
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            chunks.append(ser.read(waiting))
            deadline = time.monotonic() + 0.1
        else:
            time.sleep(0.02)
    text = b"".join(chunks).decode("utf-8", "replace")
    return [line.rstrip() for line in text.splitlines()]


def send(ser: serial.Serial, cmd: str, wait: float = 1.5) -> list[str]:
    ser.write(f"{cmd}\n".encode("ascii"))
    ser.flush()
    time.sleep(0.05)
    return read_lines(ser, wait)


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
