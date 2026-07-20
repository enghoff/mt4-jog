#!/usr/bin/env python3
"""Dismantle the cube stack at the stack site, parking each cube on a free
marker/slot -- clears the work area between stacking attempts.

The top face is located by vision from the capture pose: each color is
segmented near the parallax-predicted column pixel, the highest hit wins,
and its along-axis displacement gives the level count (rounded to the
20mm grid -- the stack was placed on-column, so XY->height coupling is
small). The cube is then gripped at that height, carried out along the
site's own radius, and parked on a free spot chosen with the held-cube
phantom filter active.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration
from mt4_vision.camera import FrameStream
from mt4_vision.detect import COLOR_RANGES, detect_cubes
from mt4_vision.pickplace import _approach, _travel, home_arm, place
from mt4_vision.scene import capture_scene
from stack_cubes import (
    CAPTURE_POSE,
    STACK_XY,
    TRAVEL_ABOVE_MM,
    VIA_RADIUS_MM,
    ParallaxHeightModel,
    find_top_face,
    ground_offset_mm,
    park_spot_for_clear,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Unstack cubes at the stack site")
    parser.add_argument("--port", default="")
    parser.add_argument("--camera", type=int, default=-1)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument("--x", type=float, default=STACK_XY[0])
    parser.add_argument("--y", type=float, default=STACK_XY[1])
    parser.add_argument("--max-levels", type=int, default=8,
                        help="upper bound on levels to remove (safety cap)")
    args = parser.parse_args()

    calib = load_calibration(Path(args.calib))
    cube = calib.cube_height_mm
    sx, sy = args.x, args.y
    via = (
        sx / math.hypot(sx, sy) * VIA_RADIUS_MM,
        sy / math.hypot(sx, sy) * VIA_RADIUS_MM,
    )

    ht_inv = np.linalg.inv(np.array(calib.homography))
    hc_inv = np.linalg.inv(np.array(calib.cube_top_homography))

    def px_of(hinv, x, y):
        v = hinv @ np.array([x, y, 1.0])
        return float(v[0] / v[2]), float(v[1] / v[2])

    model = ParallaxHeightModel(
        px_of(ht_inv, sx, sy), px_of(hc_inv, sx, sy), cube
    )
    ht = np.array(calib.homography)

    def robot_of(x_px, y_px):
        v = ht @ np.array([x_px, y_px, 1.0])
        return np.array([v[0] / v[2], v[1] / v[2]])

    p_a = px_of(ht_inv, sx, sy)
    r0 = robot_of(*p_a)
    jac = np.column_stack([
        robot_of(p_a[0] + 1.0, p_a[1]) - r0,
        robot_of(p_a[0], p_a[1] + 1.0) - r0,
    ])
    cx0, cy0, cz0 = CAPTURE_POSE
    cmodel = ParallaxHeightModel(
        px_of(ht_inv, cx0, cy0), px_of(hc_inv, cx0, cy0), cube
    )
    held_px = cmodel.predict_px((cz0 - calib.pick_z) + cube)

    def find_stack_top(frame) -> tuple[str, float, float, int] | None:
        """(color, x, y, level) of the current stack top, or None.

        Gated, never clamped: clamping bad estimates manufactured a
        plausible-looking 'red level 4' out of an off-column table cube
        (coupling faked h=136mm, the 15mm offset cap faked its XY). A
        stack-top candidate must sit in a tight perpendicular corridor
        around the column line, read a height close to a whole level,
        and imply a small XY offset -- otherwise it is not a stack.
        """
        best: tuple[int, str, tuple[float, float]] | None = None
        seen = set()
        for n in range(args.max_levels, 1, -1):
            # n stops at 2: a "level 1 stack" is just a table cube -- the
            # stack script's site-clearing handles those; treating one as
            # a stack here caused 20mm-high air grabs.
            pred = model.predict_px(n * cube)
            for color in COLOR_RANGES:
                g = find_top_face(frame, color, pred)
                if g is None or (color, round(g[0]), round(g[1])) in seen:
                    continue
                seen.add((color, round(g[0]), round(g[1])))
                along, perp = model.components(*g)
                h_est = model.h_of_s(along)
                n_est = round(h_est / cube)
                if n_est < 2 or n_est > args.max_levels:
                    continue
                if abs(perp) > 14.0:
                    continue  # off the column line laterally
                if abs(h_est - n_est * cube) > 10.0:
                    continue  # not a whole level: coupling artefact
                off = xy_offset_for(g, n_est)
                if math.hypot(*off) > 20.0:
                    continue  # too far from the site to be this stack
                if best is None or n_est > best[0]:
                    best = (n_est, color, g)
        if best is None:
            return None
        n_est, color, g = best
        return color, g, n_est

    def xy_offset_for(g: tuple[float, float], n: int) -> tuple[float, float]:
        """Robot XY offset from the column implied by blob g IF it is the
        top of an n-level stack. The height hypothesis and the XY are one
        joint interpretation -- retrying a grab one level lower MUST also
        recompute the XY (the parallax scaling differs per level; reusing
        the level-3 XY for a level-1 grab missed by ~10mm)."""
        p_n = model.predict_px(n * cube)
        return ground_offset_mm(
            (g[0] - p_n[0], g[1] - p_n[1]), jac, model.hc, n * cube,
        )

    client = Mt4Client() if not args.port else Mt4Client(port=args.port)
    cam = FrameStream(args.camera)
    removed = 0
    try:
        client.ensure_connected()
        if not client.get_status().homed:
            home_arm(client)
        _travel(client, calib, *CAPTURE_POSE, "capture pose")

        from mt4_vision.detect import detect_cubes as _dc
        for _warm in range(4):
            frame = cam.fresh()
            if _dc(frame, calib):
                break
            print("camera settling, retrying")
            time.sleep(2.0)

        def grab_at(tx: float, ty: float, n: int) -> None:
            """One grab attempt at level n, ending at the capture pose."""
            z_grip = calib.pick_z + (n - 1) * cube
            z_tr = z_grip + cube + TRAVEL_ABOVE_MM - 20.0
            client.gripper(calib.grip_open_s)
            _travel(client, calib, via[0], via[1], z_tr, "via point")
            _travel(client, calib, tx, ty, z_tr, "over stack top")
            _approach(client, calib, tx, ty, z_grip, "descend to top cube")
            client.gripper(calib.grip_close_s)
            _approach(client, calib, tx, ty, z_tr, "lift top cube")
            _approach(client, calib, via[0], via[1], z_tr, "retreat via")
            # Loaded climb to the capture pose: slow -- fast cruise on
            # loaded high-z extended moves loses steps (r=315 precedent).
            _approach(client, calib, *CAPTURE_POSE, "capture pose (held)")
            time.sleep(0.8)

        # Where a held cube ACTUALLY reads at the capture pose: measured
        # (641,309)/(640,307) vs parallax prediction (686,339) -- the
        # model extrapolation error at h~211mm is systematic. A loose 90px
        # radius around the raw prediction reached desk cubes near the
        # site (74px away) and reported an empty gripper as holding.
        held_anchor = (held_px[0] - 45.0, held_px[1] - 30.0)

        def holding() -> str | None:
            """Color of the cube visibly in the gripper at the capture
            pose, or None. Checks every color: a stepped-down grab on an
            offset formation can catch a different cube than the one
            aimed at -- treating that as a miss would re-open the gripper
            at height."""
            frame = cam.fresh()
            for c in COLOR_RANGES:
                if find_top_face(frame, c, held_anchor,
                                 radius=40.0) is not None:
                    return c
            return None

        prev_top_px: tuple[float, float] | None = None
        for _ in range(args.max_levels):
            time.sleep(1.0)
            frame = cam.fresh()
            top = find_stack_top(frame)
            if top is None:
                print("no stack top found -- site clear")
                break
            color, g, n = top
            if (prev_top_px is not None
                    and math.hypot(g[0] - prev_top_px[0],
                                   g[1] - prev_top_px[1]) < 4.0):
                print("stack top did not change after a removal pass -- "
                      "stopping (would loop forever)", file=sys.stderr)
                break
            prev_top_px = g
            # The monocular height estimate couples with XY offset (an
            # offset top face fakes extra height), so a grab can close on
            # air ABOVE the real top. Aim at the estimate, verify a cube
            # is actually held, and step DOWN one level per miss (with the
            # XY recomputed for EACH height hypothesis) -- and never park
            # thin air (the first version "removed" the same phantom four
            # times).
            got = None
            for try_n in range(n, max(1, n - 3), -1):
                off = xy_offset_for(g, try_n)
                tx, ty = sx + off[0], sy + off[1]
                if try_n == n:
                    print(f"top: {color} level {n} at ({tx:.1f},{ty:.1f})")
                else:
                    print(f"  grab missed -- retrying one level lower "
                          f"({try_n}) at ({tx:.1f},{ty:.1f})")
                if math.hypot(*off) > 35.0:
                    print(f"  implied position {math.hypot(*off):.0f}mm off "
                          f"the column for level {try_n} -- not this stack")
                    continue
                grab_at(tx, ty, try_n)
                got = holding()
                if got is not None:
                    if got != color:
                        print(f"  gripped a {got} cube (aimed at {color})")
                    color = got
                    break
            if got is None:
                print(f"nothing gripped down to level {max(1, n - 2)} -- "
                      f"stopping", file=sys.stderr)
                client.gripper(calib.grip_open_s)
                break
            sc = capture_scene(
                calib, cam.fresh(),
                held_cube_px=held_px, held_color=color,
            )
            spot = park_spot_for_clear(sc, sx, sy)
            if spot is None:
                print("no free park spot -- releasing at via radius table",
                      file=sys.stderr)
                place(client, calib, via[0], via[1])
            else:
                px_, py_, where = spot
                print(f"  parking {color} at {where} ({px_:.0f},{py_:.0f})")
                place(client, calib, px_, py_)
            removed += 1
            _travel(client, calib, *CAPTURE_POSE, "capture pose")

        print(f"\nUnstack complete: removed {removed} cube(s)")
        return 0
    except Mt4ClientError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        cam.close()
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
