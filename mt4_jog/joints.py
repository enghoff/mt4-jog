"""Joint and pin map for MT4 custom jog firmware."""

from __future__ import annotations

from dataclasses import dataclass

DEFAULT_PORT: str | None = None
DEFAULT_BAUD = 115200

J1_HOME_CENTER_STEPS = 4580
J2_HOME_PULLOFF_STEPS = 1000

# Enforced in jog firmware (g o / g c sweep); client only starts/stops sweep.
GRIPPER_S_OPEN = 120
GRIPPER_S_CLOSED = 285

# Shared jog / `mp` move step period (microseconds between DDA ticks).
JOG_SPEED_MIN_US = 700
JOG_SPEED_MAX_US = 4000


@dataclass(frozen=True)
class Joint:
    number: int
    name: str
    gcode: str
    drive: int
    direction: int
    limit_pin: int | None = None

    @property
    def label(self) -> str:
        return f"J{self.number} {self.name}"


JOINTS: tuple[Joint, ...] = (
    Joint(1, "base", "X", 23, 22, limit_pin=21),
    Joint(2, "shoulder", "Y", 25, 24, limit_pin=20),
    Joint(3, "elbow", "Z", 27, 26),
    Joint(4, "wrist", "A", 35, 36),
)

LIMIT_JOINTS: dict[str, str] = {
    f"I{j.limit_pin}": j.label for j in JOINTS if j.limit_pin is not None
}
