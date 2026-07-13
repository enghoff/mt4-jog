"""Pick and place sequences for cubes on the calibrated work surface."""

from __future__ import annotations

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import Calibration

# Slow the final descent/ascent around the grip so a slightly-off Z estimate
# nudges rather than slams. Firmware speed is us-per-step: larger = slower.
APPROACH_SPEED_US = 2400


def _check(result: dict[str, object], step: str) -> dict[str, object]:
    """move_to/home/gripper report failure via {"ok": False, ...}, not an
    exception -- callers here must not chain past a step that never
    happened, so turn a failed result into one."""
    if not result.get("ok"):
        raise Mt4ClientError(f"{step} failed: {result.get('error', result)}")
    return result


def ensure_homed(client: Mt4Client) -> None:
    status = client.get_status()
    if not status.homed:
        _check(client.home(), "home")


def pick(client: Mt4Client, calib: Calibration, x: float, y: float) -> dict[str, object]:
    """Grip a cube at robot-frame (x, y): open, descend, close, lift."""
    ensure_homed(client)
    client.gripper(calib.grip_open_s)
    _check(client.move_to(x, y, calib.safe_z), "move to safe height")
    _check(
        client.move_to(x, y, calib.pick_z, speed_us=APPROACH_SPEED_US),
        "descend to pick height",
    )
    result = client.gripper(calib.grip_close_s)
    if not result.get("ok"):
        # Lift clear anyway so a failed grip doesn't leave the TCP parked
        # against the cube.
        client.move_to(x, y, calib.safe_z, speed_us=APPROACH_SPEED_US)
        raise Mt4ClientError(f"gripper close failed: {result}")
    _check(
        client.move_to(x, y, calib.safe_z, speed_us=APPROACH_SPEED_US),
        "lift after grip",
    )
    return {"ok": True, "picked_at": [x, y]}


def place(client: Mt4Client, calib: Calibration, x: float, y: float) -> dict[str, object]:
    """Release the held cube at robot-frame (x, y).

    Releases slightly above pick height so the cube drops the last couple of
    mm instead of being pressed into the table.
    """
    ensure_homed(client)
    release_z = calib.pick_z + 3.0
    _check(client.move_to(x, y, calib.safe_z), "move to safe height")
    _check(
        client.move_to(x, y, release_z, speed_us=APPROACH_SPEED_US),
        "descend to release height",
    )
    client.gripper(calib.grip_open_s)
    _check(
        client.move_to(x, y, calib.safe_z, speed_us=APPROACH_SPEED_US),
        "lift after release",
    )
    return {"ok": True, "placed_at": [x, y]}
