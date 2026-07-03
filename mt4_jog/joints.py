"""Joint and pin map for MT4 custom jog firmware."""

from __future__ import annotations

from dataclasses import dataclass

DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 115200

J1_HOME_CENTER_STEPS = 4580
J2_HOME_PULLOFF_STEPS = 1000

# Enforced in jog firmware (g o / g c sweep); client only starts/stops sweep.
GRIPPER_S_OPEN = 120
GRIPPER_S_CLOSED = 285


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

JOINT_BY_GCODE: dict[str, Joint] = {j.gcode: j for j in JOINTS}

# Keyboard layout (left to right): Q/A=J1, W/S=J2, E/D=J3, R/F=J4
KEYBOARD_JOINTS: tuple[Joint, ...] = JOINTS

LIMIT_JOINTS: dict[str, str] = {
    f"I{j.limit_pin}": j.label for j in JOINTS if j.limit_pin is not None
}
