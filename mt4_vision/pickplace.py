"""Pick and place sequences for cubes on the calibrated work surface."""

from __future__ import annotations

import math
from collections.abc import Callable

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_jog.joints import JOINT_SOFT_MAX_STEPS, JOINT_SOFT_MIN_STEPS, JOG_SPEED_MIN_US
from mt4_jog.kinematics import STEPS_PER_DEG
from mt4_vision.calib import Calibration
from mt4_vision.detect import CubeDetection
from mt4_vision.workspace import KEEPOUT_RADIUS_MM, is_mp_reachable_xy


def fold_square_yaw_deg(yaw_deg: float) -> float:
    """Map any angle into (-45, 45] -- one face of a square (90° period)."""
    return (yaw_deg + 45.0) % 90.0 - 45.0


def j4_for_face_align(
    cube_yaw_deg: float,
    *,
    current_j4_deg: float | None = None,
) -> float:
    """World-frame J4 (deg) so the jaws meet a cube face, not a corner.

    Assumes firmware ``j4zero``: jaws along the arm ⇒ world J4 = 0.
    ``cube_yaw_deg`` is a robot-frame edge angle from detection. Squares are
    90°-periodic; when ``current_j4_deg`` is given, pick the equivalent that
    minimizes wrist travel.
    """
    if current_j4_deg is None:
        return fold_square_yaw_deg(cube_yaw_deg)
    # j4 ≡ cube_yaw (mod 90), closest to current -- avoids ±360 duplicates.
    delta = (cube_yaw_deg - current_j4_deg + 45.0) % 90.0 - 45.0
    return current_j4_deg + delta


def j4_preserve_wrist(
    x: float,
    y: float,
    *,
    from_x: float,
    from_y: float,
    from_j4: float,
) -> float:
    """World-frame J4 that keeps joint J4 fixed across a J1 swing.

    ``Mt4Client.move_to(j4=None)`` holds *world* yaw, which commands
    ``joint_j4 = world_j4 - j1``. Large base swings (e.g. to marker 0 at
    j1≈−80°) then drive J4 past soft limits → ``err mp joints``. Holding
    the wrist joint instead yields ``world_j4 = j1_tgt + (from_j4 - j1_from)``.
    """
    j1_from = math.degrees(math.atan2(from_y, from_x))
    j1_to = math.degrees(math.atan2(y, x))
    return j1_to + (from_j4 - j1_from)


def _resolve_travel_j4(
    client: Mt4Client,
    x: float,
    y: float,
    j4: float | None,
) -> float:
    """Explicit j4, or wrist-preserving world yaw from the current TCP."""
    if j4 is not None:
        return float(j4)
    tcp = client.get_tcp()
    if tcp is None:
        raise Mt4ClientError("Could not read TCP to resolve wrist-safe j4")
    return j4_preserve_wrist(
        x, y, from_x=float(tcp.x), from_y=float(tcp.y), from_j4=float(tcp.j4)
    )


def _check(result: dict[str, object], step: str) -> dict[str, object]:
    """move_to/home/gripper report failure via {"ok": False, ...}, not an
    exception -- callers here must not chain past a step that never
    happened, so turn a failed result into one."""
    if not result.get("ok"):
        raise Mt4ClientError(f"{step} failed: {result.get('error', result)}")
    return result


def _travel(
    client: Mt4Client,
    calib: Calibration,
    x: float,
    y: float,
    z: float,
    step: str,
    *,
    j4: float | None = None,
) -> dict[str, object]:
    """Horizontal or lift move at safe travel speed (firmware mp ramp active)."""
    return _check(
        client.move_to(
            x, y, z, j4=_resolve_travel_j4(client, x, y, j4),
            speed_us=calib.travel_speed_us,
        ),
        step,
    )


def _approach(
    client: Mt4Client,
    calib: Calibration,
    x: float,
    y: float,
    z: float,
    step: str,
    *,
    j4: float | None = None,
) -> dict[str, object]:
    """Slow final descent near the table (firmware ramp off)."""
    return _check(
        client.move_to(
            x, y, z, j4=_resolve_travel_j4(client, x, y, j4),
            speed_us=calib.approach_speed_us,
        ),
        step,
    )


# Camera-clear parking spot for between-move captures: the homed TCP pose.
# From the front-mounted camera the arm parked here only occludes the strip
# behind it -- essentially the mp keep-out region, where nothing pickable or
# placeable ever sits. Anywhere over the workspace, the forearm hides cubes
# and markers AND reads as cube-sized red blobs inside the workspace hull.
CAMERA_PARK_X = 200.0
CAMERA_PARK_Y = 0.0
CAMERA_PARK_Z = 260.0
CAMERA_PARK_CLEARANCE_MM = 80.0


def near_camera_park(x: float, y: float) -> bool:
    """True when (x, y) is too close to the camera-park TCP to place/pick."""
    return (
        (x - CAMERA_PARK_X) ** 2 + (y - CAMERA_PARK_Y) ** 2
    ) < CAMERA_PARK_CLEARANCE_MM**2


def retreat_for_camera(client: Mt4Client, calib: Calibration) -> dict[str, object]:
    """Move the TCP to the camera-clear park pose (post-move capture prep)."""
    return _travel(
        client, calib, CAMERA_PARK_X, CAMERA_PARK_Y, CAMERA_PARK_Z,
        "retreat to camera park",
    )


def ensure_homed(client: Mt4Client) -> None:
    status = client.get_status()
    if not status.homed:
        home_arm(client)


def home_arm(client: Mt4Client) -> None:
    """Run firmware homing regardless of the session homed flag."""
    _check(client.home(), "home")


def _require_mp_reachable(x: float, y: float, step: str) -> None:
    if not is_mp_reachable_xy(x, y):
        raise Mt4ClientError(
            f"{step}: ({x:.1f}, {y:.1f}) is inside the {KEEPOUT_RADIUS_MM:.0f}mm "
            f"J1 keep-out zone (mp cannot move there)"
        )


def resolve_pick_j4(
    client: Mt4Client,
    calib: Calibration,
    yaw_deg: float | None,
    *,
    face_align: bool = True,
) -> float | None:
    """Face-align world J4, or None so ``_travel`` preserves joint J4 instead.

    None must not mean world-yaw hold: that trips J4 soft limits on large
    J1 swings (marker 0 / far −Y). ``_travel``/``_approach`` map None to
    ``j4_preserve_wrist``.
    """
    if not face_align or yaw_deg is None:
        return None
    tcp = client.get_tcp()
    current = tcp.j4 if tcp is not None else None
    return j4_for_face_align(yaw_deg, current_j4_deg=current)


def resolve_place_j4(
    client: Mt4Client,
    calib: Calibration,
    *,
    axis_align: bool = True,
) -> float | None:
    """World-frame J4 that lands the held cube square to the X/Y axes.

    A gripped cube's orientation relative to the jaws is fixed at pick time,
    so driving J4 to 0° (mod 90°, closest to the current wrist) squares
    whatever face is held to the world axes — assumes ``j4zero``.

    Defaults on unconditionally (validated safe on hardware): even for a
    pick that wasn't face-aligned, squaring the wrist costs nothing worse
    than the unaligned yaw it would otherwise land at.
    """
    if not axis_align:
        return None
    return resolve_pick_j4(client, calib, 0.0, face_align=True)


def pick(
    client: Mt4Client,
    calib: Calibration,
    x: float,
    y: float,
    *,
    yaw_deg: float | None = None,
    face_align: bool | None = None,
) -> dict[str, object]:
    """Grip a cube at robot-frame (x, y): open, descend, close, lift.

    When ``yaw_deg`` is set (robot-frame cube-edge angle from detection) and
    face-align is enabled, world-frame J4 is commanded so the jaws meet a
    face rather than a corner. Face-align defaults on and assumes firmware
    ``j4zero`` (``calibrate_j4.py``): world J4 = 0 means jaws along the arm.
    """
    ensure_homed(client)
    _require_mp_reachable(x, y, "pick target")
    if face_align is None:
        face_align = bool(getattr(calib, "face_align_picks", True))
    j4 = resolve_pick_j4(client, calib, yaw_deg, face_align=face_align)
    client.gripper(calib.grip_open_s)
    _travel(client, calib, x, y, calib.safe_z, "move to safe height", j4=j4)
    _approach(client, calib, x, y, calib.pick_z, "descend to pick height", j4=j4)
    result = client.gripper(calib.grip_close_s)
    if not result.get("ok"):
        # Lift clear anyway so a failed grip doesn't leave the TCP parked
        # against the cube.
        _travel(client, calib, x, y, calib.safe_z, "lift after failed grip")
        raise Mt4ClientError(f"gripper close failed: {result}")
    _travel(client, calib, x, y, calib.safe_z, "lift after grip")
    out: dict[str, object] = {"ok": True, "picked_at": [x, y]}
    if j4 is not None:
        out["j4"] = round(j4, 2)
        out["yaw_deg"] = None if yaw_deg is None else round(float(yaw_deg), 2)
    return out


def pick_cube(
    client: Mt4Client,
    calib: Calibration,
    cube: CubeDetection,
    *,
    face_align: bool | None = None,
) -> dict[str, object]:
    """Vision pick from a ``CubeDetection`` (central entry for shuffle/MCP/etc.)."""
    if cube.x is None or cube.y is None:
        raise Mt4ClientError("pick_cube: detection has no robot XY")
    result = pick(
        client,
        calib,
        float(cube.x),
        float(cube.y),
        yaw_deg=cube.yaw_deg,
        face_align=face_align,
    )
    result["color"] = cube.color
    return result


def place(
    client: Mt4Client,
    calib: Calibration,
    x: float,
    y: float,
    *,
    on_released: Callable[[], None] | None = None,
    axis_align: bool = True,
    lift_after: bool = True,
) -> dict[str, object]:
    """Release the held cube at robot-frame (x, y).

    Releases slightly above pick height so the cube drops the last couple of
    mm instead of being pressed into the table. By default world-frame J4 is
    driven square to the X/Y axes during the approach (world J4 = 0 after
    ``j4zero``) so the released cube lands aligned rather than at whatever
    orientation it happened to be picked in.

    When ``lift_after`` is False the TCP stays at release height over the
    target (for in-place centering immediately after).
    """
    ensure_homed(client)
    _require_mp_reachable(x, y, "place target")
    j4 = resolve_place_j4(client, calib, axis_align=axis_align)
    release_z = calib.pick_z + 3.0
    _travel(client, calib, x, y, calib.safe_z, "move to safe height", j4=j4)
    _approach(client, calib, x, y, release_z, "descend to release height", j4=j4)
    client.gripper(calib.grip_open_s)
    if on_released is not None:
        on_released()
    if lift_after:
        _travel(client, calib, x, y, calib.safe_z, "lift after release")
    return {"ok": True, "placed_at": [x, y]}


def place_here(client: Mt4Client, calib: Calibration) -> dict[str, object]:
    """Release the held cube at the current TCP xy."""
    tcp = client.get_tcp()
    return place(client, calib, tcp.x, tcp.y)


def _rotate_j4_90_in_place(client: Mt4Client) -> None:
    """Rotate J4 ±90° via ``m``, picking the direction with more soft-limit headroom."""
    dj4_90 = round(90.0 * STEPS_PER_DEG[3])
    j4_min, j4_max = JOINT_SOFT_MIN_STEPS[3], JOINT_SOFT_MAX_STEPS[3]
    j4 = client.get_status().joints.get("j4", 0)
    options: list[tuple[int, int]] = []
    for dj4 in (dj4_90, -dj4_90):
        end = j4 + dj4
        if j4_min <= end <= j4_max:
            margin = min(end - j4_min, j4_max - end)
            options.append((margin, dj4))
    if not options:
        raise Mt4ClientError("center: no j4 ±90° rotation within soft limits")
    options.sort(key=lambda item: item[0], reverse=True)
    status = client.get_status()
    prev_speed = status.speed_us or (
        int(status.tcp.speed) if status.tcp is not None else JOG_SPEED_MIN_US
    )
    _check(client.set_speed(JOG_SPEED_MIN_US), "center: max speed for j4 rotate")
    try:
        last_err: object = None
        for _, dj4 in options:
            result = client.move_relative(0, 0, 0, dj4)
            if result.get("ok"):
                return
            last_err = result.get("error", result)
        raise Mt4ClientError(f"center: rotate j4 ±90° failed: {last_err}")
    finally:
        if prev_speed != JOG_SPEED_MIN_US:
            _check(client.set_speed(prev_speed), "center: restore speed")


def center_placed_cube(
    client: Mt4Client,
    calib: Calibration,
    x: float,
    y: float,
    *,
    lift_before_rotate: bool = False,
) -> dict[str, object]:
    """Re-grip a placed cube after rotating J4 90° and release in place.

    Centers the cube under the TCP (corrects placement/release drag). The
    gripper closes and opens at pick height, then lifts straight up.

    Expects ``place(..., lift_after=False)`` to have left the TCP at release
    height over (x, y). The wrist is rotated in place with a relative joint
    move (`m`); commanding absolute j4+90 through ``mp`` can exceed soft
    limits because firmware sets joint_j4 = world_j4 - j1.

    When ``lift_before_rotate`` is True (first calibration placement only),
    lift to ``safe_z`` before the wrist rotation, then descend for grip/release.
    """
    ensure_homed(client)
    _require_mp_reachable(x, y, "center target")
    if lift_before_rotate:
        tcp = client.get_tcp()
        _travel(
            client, calib, tcp.x, tcp.y, calib.safe_z,
            "center: lift before rotate",
        )
    _rotate_j4_90_in_place(client)
    _approach(client, calib, x, y, calib.pick_z, "center: descend to cube")
    _check(client.gripper(calib.grip_close_s), "center: grip")
    _check(client.gripper(calib.grip_open_s), "center: release")
    tcp = client.get_tcp()
    _travel(
        client, calib, tcp.x, tcp.y, calib.safe_z,
        "center: lift straight after release",
    )
    return {"ok": True, "centered_at": [x, y]}


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
