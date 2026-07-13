"""Pick and place sequences for cubes on the calibrated work surface."""

from __future__ import annotations

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import Calibration


def _check(result: dict[str, object], step: str) -> dict[str, object]:
    """move_to/home/gripper report failure via {"ok": False, ...}, not an
    exception -- callers here must not chain past a step that never
    happened, so turn a failed result into one."""
    if not result.get("ok"):
        raise Mt4ClientError(f"{step} failed: {result.get('error', result)}")
    return result


def _travel(
    client: Mt4Client, calib: Calibration, x: float, y: float, z: float, step: str
) -> dict[str, object]:
    """Horizontal or lift move at safe travel speed (firmware mp ramp active)."""
    return _check(
        client.move_to(x, y, z, speed_us=calib.travel_speed_us),
        step,
    )


def _approach(
    client: Mt4Client, calib: Calibration, x: float, y: float, z: float, step: str
) -> dict[str, object]:
    """Slow final descent near the table (firmware ramp off)."""
    return _check(
        client.move_to(x, y, z, speed_us=calib.approach_speed_us),
        step,
    )


def ensure_homed(client: Mt4Client) -> None:
    status = client.get_status()
    if not status.homed:
        _check(client.home(), "home")


def pick(client: Mt4Client, calib: Calibration, x: float, y: float) -> dict[str, object]:
    """Grip a cube at robot-frame (x, y): open, descend, close, lift."""
    ensure_homed(client)
    client.gripper(calib.grip_open_s)
    _travel(client, calib, x, y, calib.safe_z, "move to safe height")
    _approach(client, calib, x, y, calib.pick_z, "descend to pick height")
    result = client.gripper(calib.grip_close_s)
    if not result.get("ok"):
        # Lift clear anyway so a failed grip doesn't leave the TCP parked
        # against the cube.
        _travel(client, calib, x, y, calib.safe_z, "lift after failed grip")
        raise Mt4ClientError(f"gripper close failed: {result}")
    _travel(client, calib, x, y, calib.safe_z, "lift after grip")
    return {"ok": True, "picked_at": [x, y]}


def place(client: Mt4Client, calib: Calibration, x: float, y: float) -> dict[str, object]:
    """Release the held cube at robot-frame (x, y).

    Releases slightly above pick height so the cube drops the last couple of
    mm instead of being pressed into the table.
    """
    ensure_homed(client)
    release_z = calib.pick_z + 3.0
    _travel(client, calib, x, y, calib.safe_z, "move to safe height")
    _approach(client, calib, x, y, release_z, "descend to release height")
    client.gripper(calib.grip_open_s)
    _travel(client, calib, x, y, calib.safe_z, "lift after release")
    return {"ok": True, "placed_at": [x, y]}


def place_here(client: Mt4Client, calib: Calibration) -> dict[str, object]:
    """Release the held cube at the current TCP xy."""
    tcp = client.get_tcp()
    return place(client, calib, tcp.x, tcp.y)


def goto_marker(
    client: Mt4Client, calib: Calibration, x: float, y: float, *, touch: bool = False
) -> dict[str, object]:
    """Move the TCP over robot-frame (x, y) -- a calibration accuracy check:
    hover at the safe travel height by default (won't crash into the table
    even if the calibration is off), or descend to the measured table
    surface with `touch=True` for a physical go/no-go check.
    """
    ensure_homed(client)
    _travel(client, calib, x, y, calib.safe_z, "move to safe height")
    if touch:
        _approach(client, calib, x, y, calib.table_z, "descend to table")
    return {"ok": True, "moved_to": [x, y], "touched": touch}
