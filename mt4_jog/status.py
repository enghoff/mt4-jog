"""Parse MT4 firmware status lines from `?` and `pos` responses."""

from __future__ import annotations

import re
from dataclasses import dataclass, field

TCP_FIELDS = ("x", "y", "z", "j4", "grip", "speed")


@dataclass
class TcpPose:
    x: float
    y: float
    z: float
    j4: float
    grip: float
    speed: int

    def as_dict(self) -> dict[str, float | int]:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "j4": self.j4,
            "grip": self.grip,
            "speed": self.speed,
        }


@dataclass
class Mt4Status:
    homed: bool = False
    mode: str = ""
    orient: str = ""
    speed_us: int = 0
    joints: dict[str, int] = field(default_factory=dict)
    tcp: TcpPose | None = None
    drivers_enabled: bool = False
    jog_active: bool = False
    stepping: bool = False
    # True when we couldn't find a parsable TCP pose or joint positions
    # in the raw firmware lines.
    parse_failed: bool = False
    raw_lines: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "homed": self.homed,
            "mode": self.mode,
            "orient": self.orient,
            "speed_us": self.speed_us,
            "joints": dict(self.joints),
            "tcp": self.tcp.as_dict() if self.tcp else None,
            "drivers_enabled": self.drivers_enabled,
            "jog_active": self.jog_active,
            "stepping": self.stepping,
            "parse_failed": self.parse_failed,
            # Keep this bounded so tool responses don't explode.
            "raw_lines": self.raw_lines[:50],
        }


_POS_RE = re.compile(r"^pos J1=(-?\d+) J2=(-?\d+) J3=(-?\d+) J4=(-?\d+)$")
_MODE_RE = re.compile(
    r"MODE=(\w+)\s+ORIENT=(\w+)\s+HOMED=(\w+)\s+SPEED=(\d+)"
)
_EN_JOG_RE = re.compile(r"EN=(\w+)\s+JOG=(\w+)")


def parse_tcp_line(line: str) -> TcpPose | None:
    if not line.startswith("tcp "):
        return None
    out: dict[str, float] = {}
    for tok in line[4:].split():
        key, _, val = tok.partition("=")
        if key not in TCP_FIELDS:
            continue
        try:
            out[key] = float(val)
        except ValueError:
            return None
    if len(out) != len(TCP_FIELDS):
        return None
    return TcpPose(
        x=out["x"],
        y=out["y"],
        z=out["z"],
        j4=out["j4"],
        grip=out["grip"],
        speed=int(out["speed"]),
    )


def parse_status_lines(lines: list[str]) -> Mt4Status:
    status = Mt4Status(raw_lines=list(lines))
    for line in lines:
        if "HOMED=yes" in line:
            status.homed = True
        m = _MODE_RE.search(line)
        if m:
            status.mode = m.group(1)
            status.orient = m.group(2)
            status.homed = m.group(3) == "yes"
            status.speed_us = int(m.group(4))
        m = _POS_RE.match(line)
        if m:
            status.joints = {
                "j1": int(m.group(1)),
                "j2": int(m.group(2)),
                "j3": int(m.group(3)),
                "j4": int(m.group(4)),
            }
        tcp = parse_tcp_line(line)
        if tcp is not None:
            status.tcp = tcp
        if line.startswith("STEP="):
            status.stepping = line != "STEP=none"
        m = _EN_JOG_RE.search(line)
        if m:
            status.drivers_enabled = m.group(1) == "on"
            status.jog_active = m.group(2) == "on"
    return status
