#!/usr/bin/env python3
"""Dismantle a marker stack built by stack_cubes.py.

Picks are dead-reckoned at the marker's calibrated XY: grip height steps
down by ``cube_height_mm`` from ``--levels`` (or ``--max-levels`` when the
height is unknown). Each cube is parked on a free open-table slot clear of
the site. No visual height estimation.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_jog.kinematics import JointAnglesDeg, ik_position
from mt4_vision.calib import DEFAULT_CALIB_PATH, CalibrationError, load_calibration
from mt4_vision.camera import capture_frame
from mt4_vision.pickplace import (
    _approach,
    _travel,
    home_arm,
    place,
    retreat_for_camera,
)
from mt4_vision.scene import capture_scene
from mt4_vision.workspace import (
    MAX_REACH_MM,
    is_mp_reachable_xy,
)
from stack_cubes import (
    CAMERA_SETTLE_S,
    SITE_CLEAR_MM,
    choose_park_slot,
    cubes_near_site,
    marker_by_id,
    travel_z_for_level,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Unstack cubes from a calibrated marker (cartesian)"
    )
    parser.add_argument(
        "--marker",
        type=int,
        required=True,
        help="calibration ArUco marker id the stack sits on (required)",
    )
    parser.add_argument("--port", default=None)
    parser.add_argument("--camera", type=int, default=None)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument(
        "--levels",
        type=int,
        default=None,
        help="exact number of levels to remove (default: probe from --max-levels)",
    )
    parser.add_argument(
        "--max-levels",
        type=int,
        default=8,
        help="upper bound when --levels is omitted (default 8)",
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
    start_level = args.levels if args.levels is not None else args.max_levels

    def snap_scene():
        retreat_for_camera(client, calib)
        time.sleep(CAMERA_SETTLE_S)
        return capture_scene(calib, capture_frame(**camera_kwargs))

    def grab_level(level: int) -> None:
        z_grip = float(calib.pick_z) + (level - 1) * cube_h
        z_tr = travel_z_for_level(calib, level)
        if ik_position(sx, sy, z_tr, near=home_q) is None:
            raise Mt4ClientError(
                f"travel height {z_tr:.0f}mm unreachable for level {level}"
            )
        client.gripper(calib.grip_open_s)
        _travel(client, calib, sx, sy, z_tr, "over stack")
        _approach(client, calib, sx, sy, z_grip, "descend to cube")
        result = client.gripper(calib.grip_close_s)
        if not result.get("ok"):
            _travel(client, calib, sx, sy, z_tr, "lift after failed grip")
            raise Mt4ClientError(f"gripper close failed: {result}")
        _travel(client, calib, sx, sy, z_tr, "lift cube")

    try:
        client.ensure_connected()
        status = client.get_status()
        if not status.homed:
            print("Homing...")
            home_arm(client)
        else:
            print("Already homed")
        status = client.get_status()
        client.move_to(
            status.tcp.x, status.tcp.y, status.tcp.z,
            speed_us=calib.travel_speed_us,
        )

        print(
            f"Unstack site: marker {marker.marker_id} at "
            f"({sx:.1f},{sy:.1f}), starting from level {start_level}"
        )

        removed = 0
        parked: list[tuple[float, float]] = []
        for level in range(start_level, 0, -1):
            scene = snap_scene()
            near = cubes_near_site(scene, sx, sy)
            if not near and args.levels is None:
                print(f"no cube near site at level {level} -- done")
                break

            slot = choose_park_slot(scene, sx, sy, avoid=parked)
            if slot is None:
                r = math.hypot(sx, sy)
                slot = (
                    sx / r * min(r + SITE_CLEAR_MM, MAX_REACH_MM - 5.0),
                    sy / r * min(r + SITE_CLEAR_MM, MAX_REACH_MM - 5.0),
                )
                print(
                    f"\nLevel {level}: no free slot -- "
                    f"fallback ({slot[0]:.0f},{slot[1]:.0f})"
                )
            else:
                print(
                    f"\nLevel {level}: grab at ({sx:.1f},{sy:.1f}), "
                    f"park ({slot[0]:.0f},{slot[1]:.0f})"
                )

            try:
                grab_level(level)
                place(
                    client, calib, slot[0], slot[1],
                    travel_z=travel_z_for_level(calib, level),
                )
            except Mt4ClientError as exc:
                print(f"  failed: {exc}", file=sys.stderr)
                if args.levels is not None:
                    return 1
                print("  continuing lower..." if level > 1 else "  stopping")
                continue

            parked.append(slot)
            removed += 1
            print(f"  removed level {level}")

        print(f"\nUnstack complete: removed {removed} cube(s)")
        retreat_for_camera(client, calib)
        return 0
    except Mt4ClientError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
