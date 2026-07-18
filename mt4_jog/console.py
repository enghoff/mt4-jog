"""Shared console message style for MT4 tooling.

Conventions for user-facing status lines:
- Sentence case (capitalize the first letter)
- No trailing period on short status / exit lines ("Bye", "Ready", "Home ok")
- Firmware wire protocol stays lowercase; pretty-print when echoing to the user
"""

from __future__ import annotations

import re

from mt4_jog.status import parse_status_lines

_GRIP_RE = re.compile(r"GRIP S=(\d+)\s+pwm=(\w+)\s+sweep=(\w+)")
_LIM_RE = re.compile(r"LIM\s+(.+?)(?:\s{2,}|\s*$)")
_STEP_RE = re.compile(r"^STEP=(.+)$")


def format_firmware_line(line: str) -> str:
    """Pretty-print a single firmware event line (wire format is unchanged)."""
    if line == "home start":
        return "Home start"
    if line == "home ok":
        return "Home ok"
    if line.startswith("home fail"):
        return "Home fail" + line[len("home fail") :]
    if line.startswith("ok speed "):
        return f"Speed: {line[len('ok speed '):]}"
    if line.startswith("pos "):
        return "Joints: " + "  ".join(line[4:].split())
    return line


def format_status_report(lines: list[str]) -> list[str]:
    """Turn a `?` reply into a short, consistent status block."""
    status = parse_status_lines(lines)
    out: list[str] = []

    if status.mode:
        out.append(
            f"Mode: {status.mode}  Orient: {status.orient}  "
            f"Homed: {'yes' if status.homed else 'no'}  Speed: {status.speed_us}"
        )
    if status.joints:
        j = status.joints
        out.append(
            f"Joints: J1={j['j1']}  J2={j['j2']}  J3={j['j3']}  J4={j['j4']}"
        )
    if status.tcp is not None:
        t = status.tcp
        out.append(
            f"TCP: x={t.x:.1f}  y={t.y:.1f}  z={t.z:.1f}  "
            f"j4={t.j4:.1f}  grip={t.grip:.0f}"
        )

    step = ""
    lim = ""
    grip = ""
    for line in lines:
        m = _STEP_RE.match(line)
        if m:
            step = m.group(1)
        m = _LIM_RE.search(line)
        if m:
            lim = m.group(1).strip()
        m = _GRIP_RE.search(line)
        if m:
            grip = f"S={m.group(1)}  pwm={m.group(2)}  sweep={m.group(3)}"

    drive = (
        f"Drivers: {'on' if status.drivers_enabled else 'off'}  "
        f"Jog: {'on' if status.jog_active else 'off'}"
    )
    if step:
        drive += f"  Step: {step}"
    if lim:
        drive += f"  Limits: {lim}"
    if status.mode or status.joints or status.tcp is not None:
        out.append(drive)
    if grip:
        out.append(f"Gripper: {grip}")

    if not out:
        # Fallback: show non-noise lines rather than going silent.
        noise = (
            "---",
            "ok stop",
            "ok grip",
            "ok all",
            "ok enable",
            "ok m",
            "ok mp",
            "ok cj",
            "ok jog",
            "ok pin",
            "ok step",
            "ok orient",
        )
        for line in lines:
            if not line or line.startswith(noise) or line.startswith("---"):
                continue
            out.append(format_firmware_line(line))
    return out


def print_status(lines: list[str]) -> None:
    """Print a formatted `?` status report to stdout."""
    for line in format_status_report(lines):
        print(line)
