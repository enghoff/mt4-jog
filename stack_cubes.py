#!/usr/bin/env python3
"""Build a cube stack on a calibrated ArUco marker.

The stack site is a marker id passed on the CLI (required -- no default).
Any cubes within SITE_CLEAR_MM of that marker are nudged aside along the
marker→cube direction to CLEAR_PARK_MM (keep-clear + margin) first. Each stack cube is taken with the calibrate_height centering
sequence (yaw-pick → release → lift → rotate J4 90° → re-grip) via
``pick_centered``, then placed at the marker's calibrated XY by dead
reckoning. Placement Z steps by ``cube_height_mm`` from the calibration;
there is no visual alignment or post-place verification. Stack place/retreat
moves are vertical-then-horizontal within ``STACK_AXIS_CLEAR_MM`` of the
site so the gripper cannot clip the growing column on a diagonal ``mp``.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_jog.kinematics import JointAnglesDeg, ik_position
from mt4_vision.calib import DEFAULT_CALIB_PATH, CalibrationError, load_calibration
from mt4_vision.camera import capture_frame
from mt4_vision.detect import CubeDetection
from mt4_vision.pickplace import (
    STACK_AXIS_CLEAR_MM,
    home_arm,
    pick,
    pick_centered,
    place,
    retreat_for_camera,
)
from mt4_vision.scene import Scene, capture_scene
from mt4_vision.workspace import (
    MAX_REACH_MM,
    MarkerSlot,
    dist_mm,
    is_mp_reachable_xy,
    marker_slots_from_calibration,
)

# Cubes this close to the stack marker are moved aside before building.
SITE_CLEAR_MM = 70.0
# Clear landings sit this far past the keep-clear radius so release drag /
# vision scatter can't bounce them straight back into the zone (the old
# free-slot path parked at ~70mm and re-cleared the same cube forever).
CLEAR_MARGIN_MM = 40.0
CLEAR_PARK_MM = SITE_CLEAR_MM + CLEAR_MARGIN_MM  # 110mm from marker
# Finger clearance from other cubes when parking a cleared cube.
CLEAR_SEP_MM = 45.0
# Transit clearance above the current release height when carrying over the
# growing stack (safe_z alone is only ~34mm above the table).
TRAVEL_ABOVE_MM = 35.0
# Settle after retreat before a fresh scene capture.
CAMERA_SETTLE_S = 0.8
SITE_CLEAR_ATTEMPTS = 6
# Camera line-of-sight shadow behind the stack: raised stack tops map
# (via the 1-cube cube-top homography) to phantom table cubes further from
# the camera than the site. Ignore pick candidates in that corridor.
# Measured 2026-07-21 on marker 3: true (179,180) → phantoms ~(115,227)
# (~79mm along the away-from-camera axis).
STACK_SHADOW_LATERAL_MM = 45.0
STACK_SHADOW_ALONG_MIN_MM = 25.0
STACK_SHADOW_ALONG_PER_LEVEL_MM = 35.0
STACK_SHADOW_ALONG_FLOOR_MM = 90.0


def marker_by_id(calib, marker_id: int) -> MarkerSlot:
    slots = marker_slots_from_calibration(calib)
    for m in slots:
        if m.marker_id == marker_id:
            return m
    known = [m.marker_id for m in slots]
    raise SystemExit(
        f"marker {marker_id} not in calibration; known ids: {known}"
    )


def cubes_near_site(
    scene: Scene, sx: float, sy: float, radius_mm: float = SITE_CLEAR_MM
) -> list[CubeDetection]:
    return [
        c
        for c in scene.raw_cubes
        if c.x is not None
        and c.y is not None
        and dist_mm(float(c.x), float(c.y), sx, sy) < radius_mm
    ]


def clear_aside_xy(
    sx: float,
    sy: float,
    cx: float,
    cy: float,
    occupied: list[tuple[float, float]],
) -> tuple[float, float] | None:
    """Push a cube away from the marker along marker→cube, with margin.

    Lands at ~CLEAR_PARK_MM from the site (not on a barely-outside free
    slot that vision will still read as "near site"). Tries a few angles
    and radii if the primary landing is blocked or unreachable.
    """
    dx, dy = cx - sx, cy - sy
    r = math.hypot(dx, dy)
    if r < 1.0:
        # Sitting on the tag: push outward along the marker's base bearing.
        dx, dy = sx, sy
        r = math.hypot(dx, dy) or 1.0
    ux, uy = dx / r, dy / r
    for dist in (CLEAR_PARK_MM, CLEAR_PARK_MM + 30.0, CLEAR_PARK_MM + 60.0):
        for angle_deg in (0.0, 35.0, -35.0, 70.0, -70.0, 110.0, -110.0):
            ang = math.radians(angle_deg)
            ca, sa = math.cos(ang), math.sin(ang)
            vx, vy = ux * ca - uy * sa, ux * sa + uy * ca
            tx, ty = sx + vx * dist, sy + vy * dist
            if not is_mp_reachable_xy(tx, ty):
                continue
            if math.hypot(tx, ty) > MAX_REACH_MM:
                continue
            if any(dist_mm(tx, ty, ox, oy) < CLEAR_SEP_MM for ox, oy in occupied):
                continue
            return (tx, ty)
    return None


def choose_park_slot(
    scene: Scene,
    sx: float,
    sy: float,
    *,
    avoid: list[tuple[float, float]] | None = None,
) -> tuple[float, float] | None:
    """Nearest free open-table slot well clear of the stack site."""
    avoid = avoid or []
    candidates = [
        (x, y)
        for x, y in scene.free_slots
        if dist_mm(x, y, sx, sy) >= CLEAR_PARK_MM
        and all(dist_mm(x, y, ax, ay) >= CLEAR_SEP_MM for ax, ay in avoid)
        and is_mp_reachable_xy(x, y)
        and math.hypot(x, y) <= MAX_REACH_MM
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda p: dist_mm(p[0], p[1], sx, sy))


def stack_shadow_behind_unit(
    calib, sx: float, sy: float
) -> tuple[float, float] | None:
    """Unit XY vector from the stack site away from the camera.

    Derived from table vs cube-top maps at the site: mapping the site's
    table pixel through the cube-top homography shifts toward the camera;
    the opposite direction is "behind the stack" along the camera LOS.
    """
    if not calib.cube_top_homography:
        return None
    ht_inv = np.linalg.inv(np.array(calib.homography, dtype=np.float64))
    v = ht_inv @ np.array([sx, sy, 1.0])
    px, py = float(v[0] / v[2]), float(v[1] / v[2])
    cx, cy = calib.pixel_to_robot(px, py, on_cube_top=True)
    # cube-top reading of the table-site pixel sits toward the camera.
    toward_cam_x, toward_cam_y = cx - sx, cy - sy
    length = math.hypot(toward_cam_x, toward_cam_y)
    if length < 1.0:
        return None
    return (-toward_cam_x / length, -toward_cam_y / length)


def in_stack_camera_shadow(
    x: float,
    y: float,
    sx: float,
    sy: float,
    behind_u: tuple[float, float],
    *,
    stack_levels: int,
) -> bool:
    """True when (x, y) lies behind the stack along the camera LOS.

    A real cube there would be occluded by the stack; detections in this
    corridor are almost always raised stack tops mis-mapped as table cubes.
    """
    dx, dy = x - sx, y - sy
    ux, uy = behind_u
    along = dx * ux + dy * uy
    lateral = abs(dx * uy - dy * ux)
    along_max = max(
        STACK_SHADOW_ALONG_FLOOR_MM,
        stack_levels * STACK_SHADOW_ALONG_PER_LEVEL_MM,
    )
    return (
        along >= STACK_SHADOW_ALONG_MIN_MM
        and along <= along_max
        and lateral <= STACK_SHADOW_LATERAL_MM
    )


def stack_candidates(
    scene: Scene,
    sx: float,
    sy: float,
    *,
    calib=None,
    stack_levels: int = 0,
) -> list[CubeDetection]:
    """Reachable pickable cubes outside the site keep-clear radius.

    When the stack already has cubes, also drop detections in the camera
    line-of-sight shadow behind the site (stack-top phantoms).
    """
    behind_u = None
    if stack_levels > 0 and calib is not None:
        behind_u = stack_shadow_behind_unit(calib, sx, sy)
    out: list[CubeDetection] = []
    for c in scene.pickable(scene.cubes):
        if dist_mm(float(c.x), float(c.y), sx, sy) < SITE_CLEAR_MM:
            continue
        if (
            behind_u is not None
            and in_stack_camera_shadow(
                float(c.x), float(c.y), sx, sy, behind_u,
                stack_levels=stack_levels,
            )
        ):
            continue
        out.append(c)
    return out


def release_z_for_level(calib, level: int) -> float:
    """TCP release height: 4mm above the current stack top.

    Stack top before placing ``level`` (1-based) is the top of the uppermost
    cube already seated -- ``pick_z + (level-1)*cube_height_mm`` in the same
    TCP frame as table grips (empty marker when level==1).
    """
    stack_top = float(calib.pick_z) + (level - 1) * float(calib.cube_height_mm)
    return stack_top + 4.0


def travel_z_for_level(calib, level: int) -> float:
    rz = release_z_for_level(calib, level)
    return max(float(calib.safe_z), rz + TRAVEL_ABOVE_MM)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Stack cubes on a calibrated marker (cartesian place)"
    )
    parser.add_argument(
        "--marker",
        type=int,
        required=True,
        help="calibration ArUco marker id to build the stack on (required)",
    )
    parser.add_argument("--port", default=None)
    parser.add_argument("--camera", type=int, default=None)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument(
        "--max-levels",
        type=int,
        default=8,
        help="stop after this many levels (default 8)",
    )
    args = parser.parse_args()

    try:
        calib = load_calibration(Path(args.calib))
    except CalibrationError as exc:
        print(exc, file=sys.stderr)
        return 1

    marker = marker_by_id(calib, args.marker)
    sx, sy = marker.x, marker.y
    if not is_mp_reachable_xy(sx, sy) or math.hypot(sx, sy) > MAX_REACH_MM:
        print(
            f"marker {marker.marker_id} at ({sx:.1f},{sy:.1f}) is out of reach",
            file=sys.stderr,
        )
        return 1

    camera_kwargs = {} if args.camera is None else {"index": args.camera}
    client = Mt4Client() if not args.port else Mt4Client(port=args.port)
    cube_h = float(calib.cube_height_mm)
    home_q = JointAnglesDeg(0.0, 0.0, 0.0, 0.0)

    def snap_scene() -> Scene:
        retreat_for_camera(client, calib)
        time.sleep(CAMERA_SETTLE_S)
        return capture_scene(calib, capture_frame(**camera_kwargs))

    try:
        client.ensure_connected()
        status = client.get_status()
        if not status.homed:
            print("Homing...")
            home_arm(client)
        else:
            print("Already homed")
        status = client.get_status()
        # Max cruise (lowest step period) for the session.
        client.move_to(
            status.tcp.x, status.tcp.y, status.tcp.z,
            speed_us=calib.travel_speed_us,
        )

        print(
            f"Stack site: marker {marker.marker_id} at "
            f"({sx:.1f},{sy:.1f}), cube_height={cube_h:.1f}mm"
        )

        # --- Clear cubes near the stack marker ---------------------------------
        for attempt in range(1, SITE_CLEAR_ATTEMPTS + 1):
            scene = snap_scene()
            near = cubes_near_site(scene, sx, sy)
            if not near:
                print("Site clear")
                break
            # Prefer pickable detections; fall back to raw occupants.
            pickable_near = [
                c for c in scene.pickable(scene.cubes)
                if dist_mm(float(c.x), float(c.y), sx, sy) < SITE_CLEAR_MM
            ]
            target = (pickable_near or near)[0]
            occupied = [
                (float(c.x), float(c.y))
                for c in scene.raw_cubes
                if c is not target and c.x is not None and c.y is not None
            ]
            dest = clear_aside_xy(
                sx, sy, float(target.x), float(target.y), occupied,
            )
            if dest is None:
                print(
                    f"No reachable clear spot for {target.color} at "
                    f"({target.x:.0f},{target.y:.0f})",
                    file=sys.stderr,
                )
                return 1
            print(
                f"Clearing {target.color} at ({target.x:.0f},{target.y:.0f}) "
                f"-> ({dest[0]:.0f},{dest[1]:.0f}) "
                f"[attempt {attempt}/{SITE_CLEAR_ATTEMPTS}]"
            )
            try:
                pick(
                    client, calib, float(target.x), float(target.y),
                    yaw_deg=target.yaw_deg,
                )
                place(client, calib, dest[0], dest[1])
            except Mt4ClientError as exc:
                print(f"  clear failed: {exc}", file=sys.stderr)
                return 1
        else:
            scene = snap_scene()
            still = cubes_near_site(scene, sx, sy)
            if still:
                print(
                    "Site still occupied after clear attempts: "
                    + ", ".join(f"{c.color}({c.x:.0f},{c.y:.0f})" for c in still),
                    file=sys.stderr,
                )
                return 1

        # --- Build the stack ---------------------------------------------------
        built = 0
        for level in range(1, args.max_levels + 1):
            tz = travel_z_for_level(calib, level)
            rz = release_z_for_level(calib, level)
            if ik_position(sx, sy, tz, near=home_q) is None:
                print(
                    f"level {level}: travel height {tz:.0f}mm unreachable -- stopping"
                )
                break

            scene = snap_scene()
            # Drop stack-top phantoms behind the site along the camera LOS
            # (they appear once level 1+ is built).
            shadowed = []
            behind_u = (
                stack_shadow_behind_unit(calib, sx, sy) if built > 0 else None
            )
            if behind_u is not None:
                for c in scene.pickable(scene.cubes):
                    if dist_mm(float(c.x), float(c.y), sx, sy) < SITE_CLEAR_MM:
                        continue
                    if in_stack_camera_shadow(
                        float(c.x), float(c.y), sx, sy, behind_u,
                        stack_levels=built,
                    ):
                        shadowed.append(c)
            if shadowed:
                print(
                    "Ignoring stack-shadow phantom(s): "
                    + ", ".join(
                        f"{c.color}({c.x:.0f},{c.y:.0f})" for c in shadowed
                    )
                )
            cands = stack_candidates(
                scene, sx, sy, calib=calib, stack_levels=built,
            )
            if not cands:
                print(f"level {level}: no reachable cube outside site -- done")
                break
            cube = cands[0]
            print(
                f"\nLevel {level}: align-pick {cube.color} at "
                f"({cube.x:.1f},{cube.y:.1f}) yaw={cube.yaw_deg}"
            )
            try:
                pick_centered(
                    client, calib, float(cube.x), float(cube.y),
                    yaw_deg=cube.yaw_deg,
                )
                print(
                    f"  placing at marker ({sx:.1f},{sy:.1f}) "
                    f"release_z={rz:.1f} travel_z={tz:.1f}"
                )
                place(
                    client, calib, sx, sy,
                    release_z=rz,
                    travel_z=tz,
                    along_arm=True,
                    # No XYZ diagonal inside this radius of the stack axis.
                    axis_clear_mm=STACK_AXIS_CLEAR_MM,
                )
            except Mt4ClientError as exc:
                print(f"  level {level} failed: {exc}", file=sys.stderr)
                return 1
            built = level
            print(f"  placed level {level}")

        print(f"\nBuilt {built} level(s) on marker {marker.marker_id}")
        retreat_for_camera(client, calib)
        return 0
    except Mt4ClientError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
