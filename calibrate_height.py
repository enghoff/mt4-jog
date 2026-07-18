#!/usr/bin/env python3
"""Calibrate the cube-top-height pixel->robot correction, fully autonomously.

The main calibration (calibrate_vision.py) fits a homography from markers
lying flat on the table -- accurate for anything at table height (confirmed
by goto-marker landing dead-on), but cubes are detected by their top face,
cube_height_mm above the table, and camera parallax shifts that face's pixel
position enough to matter: ~15-19mm of pick error measured directly against
known-accurate marker coordinates.

This script measures that shift directly by placing a cube at a grid of
robot-frame points spread across the reachable workspace and photographing
where it lands -- the ground truth here is the arm's own positioning (we
choose the target and command the arm there directly), not a marker at all,
so unlike marker-only calibration this doesn't need any correspondence to
already-calibrated points.

A first attempt fit only 3 points (the 3 reachable markers) exactly (zero
residual) and made pick accuracy *worse*, not better: unlike marker corner
detection (sub-pixel precise), a cube's color-blob centroid is noisy --
lighting, slight rotation, partial occlusion -- and a 3-point exact affine
has zero redundancy to average that noise out; it just bakes the noise into
the fit. Sampling more points across the workspace gives real least-squares
averaging (and enables a full homography once >=4 points are collected).

No human interaction needed -- the arm places its own calibration probe.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import (
    DEFAULT_CALIB_PATH,
    CalibrationError,
    fit_transform,
    load_calibration,
    reprojection_errors,
)
from mt4_vision.camera import capture_frame
from mt4_vision.detect import CubeDetection, detect_cubes
from mt4_vision.pickplace import home_arm, pick, place
from mt4_vision.scene import filter_phantoms
from mt4_vision.workspace import (
    MAX_REACH_MM,
    PLACE_CLEARANCE_MM,
    PLACEMENT_SLOTS,
    marker_slots_from_calibration,
)

# Grid of robot-frame (x, y) targets spread across the reachable workspace,
# one quadrant/radius at a time -- ground truth is the arm's own positioning,
# not vision, so these don't need to be markers or anywhere special. Radii
# chosen to stay comfortably under MAX_REACH_MM at any angle in the set.
# Ordered to start in the well-lit region near the base and finish with the
# far, dimmer area (a detection failure there then costs one point at the
# end of the chain rather than resetting it at the start).
# Note no point near (100, 0): the homed arm's gripper hangs over that
# region in the camera view, so a cube placed there can't be re-detected.
GRID_POINTS = PLACEMENT_SLOTS + [
    # Probe-only densification of the +y quadrant: the 6-point 2026-07-18
    # collection left 9-12mm similarity residuals there (slowly-varying
    # centroid bias a 4-DOF similarity cannot absorb); >=8 usable points
    # upgrade the fit to bundle+affine, which can. The last point doubles as
    # the holdout, so the validation number measures the marker-3 region
    # picks actually struggled in.
    (140.0, 160.0),
    (215.0, 205.0),
]
# How far (px) the placed cube's centroid may land from where it's expected
# and still count as "the probe" -- generous given measured parallax shifts
# were under 15px, but tight enough to reject the arm's own body (which
# reads as a color blob elsewhere in the frame).
MATCH_RADIUS_PX = 120.0
# pick() is open-loop -- no force/current sensing, so a closed gripper isn't
# proof of a grasp. If the cube is still within this many px of where it sat
# before the pick, the grasp almost certainly failed (confirmed happening:
# multiple "successful" placements turned out to be the untouched cube still
# sitting at its very first starting position).
GRASP_FAIL_RADIUS_PX = 30.0
# Vision-bootstrap picks (no arm-known position yet) go through the table
# homography, which reads a cube-top ~this much low in X (height parallax,
# measured 15-19mm with the camera at large +X beyond the desk edge). Once a
# probe has been arm-placed, its position is known exactly and no nudge is
# used -- that is precisely the error this whole calibration will remove.
BOOTSTRAP_NUDGE_X_MM = 17.0
# After placing, the *raw* table-plane pixel->robot estimate of the detected
# position (before any height correction) should land within this many mm of
# the intended target -- parallax is ~15-20mm and the interim table map can
# be off by a few more cm in places; this is a mis-grasp/slip detector, not
# a tight tolerance. Anything further off means the placement didn't land
# where intended and the point should be discarded rather than poisoning
# the fit.
PLACEMENT_SANITY_MM = 100.0
# Reset cruise speed before starting: whatever a prior session left it at
# (e.g. leftover from gripper-timing/APPROACH_SPEED_US testing) could be slow
# enough that a full cross-table pick/place leg exceeds the client's move
# timeout even though the arm completes it fine -- confirmed happening at a
# leftover 2400us during manual testing.
RESET_SPEED_US = 1524
HOME_SETTLE_S = 0.5


def find_probe(frame, color: str, near_px: tuple[float, float]) -> CubeDetection | None:
    candidates = [c for c in detect_cubes(frame, calibration=None) if c.color == color]
    if not candidates:
        return None
    nx, ny = near_px
    best = min(candidates, key=lambda c: math.hypot(c.px - nx, c.py - ny))
    if math.hypot(best.px - nx, best.py - ny) > MATCH_RADIUS_PX:
        return None
    return best


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate cube-top-height parallax")
    parser.add_argument("--port", default=None)
    parser.add_argument("--camera", type=int, default=None)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument(
        "--holdout", type=int, default=1,
        help="reserve this many grid points to validate the fit instead of using them in it (default 1)",
    )
    parser.add_argument(
        "--probe-at", type=float, nargs=2, default=None, metavar=("X", "Y"),
        help="exact robot XY of a cube the arm previously placed -- skips the "
             "less-accurate vision bootstrap for the first pick",
    )
    parser.add_argument(
        "--avoid", type=float, nargs=2, action="append", default=[],
        metavar=("X", "Y"),
        help="skip grid targets within place clearance of this robot XY "
             "(e.g. a cube that must stay put); repeatable",
    )
    args = parser.parse_args()

    try:
        calib = load_calibration(Path(args.calib))
    except CalibrationError as exc:
        print(exc, file=sys.stderr)
        return 1

    camera_kwargs = {} if args.camera is None else {"index": args.camera}
    client = Mt4Client() if not args.port else Mt4Client(port=args.port)

    try:
        client.ensure_connected()
        status = client.get_status()
        if not status.homed:
            print("Homing first...")
            home_arm(client)  # raises on failure instead of sailing on unhomed
            status = client.get_status()
        # No-op move at the current pose, purely to reset cruise speed (see
        # RESET_SPEED_US) before the real moves below.
        client.move_to(status.tcp.x, status.tcp.y, status.tcp.z, speed_us=RESET_SPEED_US)

        grid = [(x, y) for x, y in GRID_POINTS if math.hypot(x, y) <= MAX_REACH_MM]
        for ax, ay in args.avoid:
            dropped_targets = [
                (x, y) for x, y in grid
                if math.hypot(x - ax, y - ay) < PLACE_CLEARANCE_MM
            ]
            if dropped_targets:
                print(f"Skipping target(s) {dropped_targets} -- within place "
                      f"clearance of avoided ({ax:.0f},{ay:.0f})")
            grid = [t for t in grid if t not in dropped_targets]
        if len(grid) < 5:
            print(f"Only {len(grid)} grid points within MAX_REACH_MM; need >=5", file=sys.stderr)
            return 1

        frame = capture_frame(**camera_kwargs)
        # Phantom-filter the probe pool (keep-out cylinder, area, hull, reach):
        # the arm base's own hardware intermittently reads as small blue blobs
        # inside the keep-out zone, and the grasp-failure rotation below would
        # otherwise eventually send the gripper at one.
        all_blobs = detect_cubes(frame, calib)
        cubes = filter_phantoms(all_blobs, marker_slots_from_calibration(calib))
        dropped = [c for c in all_blobs if c not in cubes]
        if dropped:
            print("Ignoring phantom blob(s): "
                  + ", ".join(f"{c.color}({c.x:.0f},{c.y:.0f})" for c in dropped))
        if not cubes:
            print("No reachable cube in view to use as a height probe", file=sys.stderr)
            return 1
        # Rotate through every reachable cube on repeated grasp failure --
        # the bootstrap map's error varies across the table, so a different
        # cube in a better-mapped spot may grasp fine.
        probe_pool = [(c.color, (c.x, c.y)) for c in cubes]
        if args.probe_at:
            # The arm-known cube must be the probe -- order the pool by
            # distance to it.
            probe_pool.sort(
                key=lambda e: math.hypot(e[1][0] - args.probe_at[0], e[1][1] - args.probe_at[1])
            )
        probe_color, probe_xy = probe_pool[0]
        print(f"Probe candidates: {[(c, (round(x), round(y))) for c, (x, y) in probe_pool]}")
        print(f"Using the {probe_color} cube at ({probe_xy[0]:.1f},{probe_xy[1]:.1f}) first")
        print(f"Sampling {len(grid)} points, holding out last {args.holdout} for validation")

        pixel_pts: list[tuple[float, float]] = []
        robot_pts: list[tuple[float, float]] = []
        probe_colors: list[str] = []

        def keep_probe_observations() -> None:
            """Persist raw (pixel, robot, color) observations -- on every exit path that
            has any, not just success: a run aborted by a dropped cube already
            paid the arm time for its points, and offline refits can merge
            them with a later session's. Color matters: red- and blue-cube
            centroids of the same physical position measured ~10px apart
            (different HSV masks include different side faces), so
            cross-color merges must model that or stay single-color."""
            if not pixel_pts:
                return
            calib.probe_observations = [
                {"pixel": [round(p[0], 2), round(p[1], 2)], "robot": list(r), "color": c}
                for p, r, c in zip(pixel_pts, robot_pts, probe_colors)
            ]
            calib.save(Path(args.calib))

        def locate_probe(near_xy: tuple[float, float]) -> tuple[float, float] | None:
            """Re-detect the probe fresh (never trust a carried-over
            assumption -- avoids compounding drift after any failed
            grasp/placement earlier in the run). Returns its pixel position;
            find_probe() bypasses the hull filter via calibration=None, which
            leaves CubeDetection.x/.y unset, so robot XY is computed
            separately here via the raw (uncorrected) table homography --
            good enough to re-locate and pick up the probe by.
            """
            frame = capture_frame(**camera_kwargs)
            found = find_probe(frame, probe_color, _inverse_guess(calib, *near_xy))
            if found is not None:
                return (found.px, found.py)
            candidates = [
                c for c in detect_cubes(frame, calib)
                if c.color == probe_color and math.hypot(c.x, c.y) <= MAX_REACH_MM
            ]
            if not candidates:
                return None
            best = min(candidates, key=lambda c: math.hypot(c.x - near_xy[0], c.y - near_xy[1]))
            return (best.px, best.py)

        grasp_fails = 0
        # Once the probe has been arm-placed somewhere, its position is known
        # EXACTLY (the arm put it there) -- vastly better than re-deriving it
        # through the still-uncorrected camera map. Vision only verifies the
        # cube is still where it was left; the pick targets the known coords.
        known_xy: tuple[float, float] | None = args.probe_at
        for gx, gy in grid:
            pre_pick_px = locate_probe(known_xy or probe_xy)
            if pre_pick_px is None:
                if known_xy is not None:
                    # Not visible but the arm knows where it left it -- e.g.
                    # the homed arm occludes that spot. Pick blind; the
                    # post-place check still validates the data point.
                    print(f"\nTarget ({gx:.1f},{gy:.1f}): probe not visible, picking blind "
                          f"at arm-known ({known_xy[0]:.1f},{known_xy[1]:.1f})")
                else:
                    print(f"\nTarget ({gx:.1f},{gy:.1f}): lost track of the probe cube -- aborting", file=sys.stderr)
                    keep_probe_observations()
                    if pixel_pts:
                        print(f"Kept {len(pixel_pts)} collected probe observation(s) "
                              "in the calibration file (cube_top_homography unchanged)")
                    return 1
            seen_xy = calib.pixel_to_robot(*pre_pick_px) if pre_pick_px else None
            if (
                known_xy is not None
                and seen_xy is not None
                and math.hypot(seen_xy[0] - known_xy[0], seen_xy[1] - known_xy[1])
                > PLACEMENT_SANITY_MM
            ):
                print(f"  Probe not where it was left (seen ~({seen_xy[0]:.0f},{seen_xy[1]:.0f}), "
                      f"expected ({known_xy[0]:.0f},{known_xy[1]:.0f})) -- falling back to vision")
                known_xy = None
            if known_xy is not None:
                probe_xy = known_xy
                origin = "arm-known"
            elif calib.cube_top_homography:
                # A cube-top map (even the crude interim one) already contains
                # the parallax correction -- use it, no nudge.
                probe_xy = calib.pixel_to_robot(*pre_pick_px, on_cube_top=True)
                origin = "vision(cube-top)"
            else:
                probe_xy = (seen_xy[0] + BOOTSTRAP_NUDGE_X_MM, seen_xy[1])
                origin = "vision+nudge"
            print(f"\nTarget ({gx:.1f},{gy:.1f}): probe at "
                  f"({probe_xy[0]:.1f},{probe_xy[1]:.1f}) [{origin}], picking...")

            try:
                pick(client, calib, *probe_xy)
                home_arm(client)
            except Mt4ClientError as exc:
                print(f"  Pick failed ({exc}), skipping this point")
                known_xy = None
                continue

            # Grasp verification: pick() has no force/current sensing, so a
            # closed gripper isn't proof of a grasp -- check the cube is
            # actually gone from where it was. Blind picks (no pre-pick
            # sighting) skip this; the post-place check still gates the data.
            frame = capture_frame(**camera_kwargs)
            still_there = (
                find_probe(frame, probe_color, pre_pick_px) if pre_pick_px else None
            )
            if still_there is not None and math.hypot(
                still_there.px - pre_pick_px[0], still_there.py - pre_pick_px[1]
            ) < GRASP_FAIL_RADIUS_PX:
                grasp_fails += 1
                known_xy = None
                print("  Grasp likely failed (cube still at its start position), skipping")
                if grasp_fails >= 2 and len(probe_pool) > 1:
                    # The bootstrap map may be off where this cube sits --
                    # switch to a different cube in a better-mapped spot.
                    probe_pool.append(probe_pool.pop(0))
                    probe_color, probe_xy = probe_pool[0]
                    grasp_fails = 0
                    print(f"  Switching probe to the {probe_color} cube at "
                          f"({probe_xy[0]:.1f},{probe_xy[1]:.1f})")
                continue
            grasp_fails = 0

            try:
                # base avoidance is the firmware's job now: mp routes around
                # the keep-out cylinder on its own
                place(client, calib, gx, gy)
                home_arm(client)
            except Mt4ClientError as exc:
                print(f"  Place failed ({exc}), skipping this point")
                known_xy = None
                continue
            time.sleep(HOME_SETTLE_S)

            frame = capture_frame(**camera_kwargs)
            # Expected pixel is unknown a priori (that's what we're
            # measuring) -- fall back to the table homography's own guess
            # for the search center, which is at worst ~20mm (a few px) off.
            guess_px = _inverse_guess(calib, gx, gy)
            found = find_probe(frame, probe_color, guess_px)
            if found is None:
                print(f"  Could not find {probe_color} probe near ({gx:.1f},{gy:.1f}), skipping")
                known_xy = None
                continue

            raw_x, raw_y = calib.pixel_to_robot(found.px, found.py)
            sanity_err = math.hypot(raw_x - gx, raw_y - gy)
            if sanity_err > PLACEMENT_SANITY_MM:
                print(f"  Detected position ({raw_x:.1f},{raw_y:.1f}) is {sanity_err:.0f}mm "
                      f"from the target -- likely a bad placement, discarding")
                known_xy = None
                continue

            print(f"  Probe detected at pixel ({found.px:.1f},{found.py:.1f}), "
                  f"raw estimate ({raw_x:.1f},{raw_y:.1f}) [{sanity_err:.0f}mm off]")
            pixel_pts.append((found.px, found.py))
            robot_pts.append((gx, gy))
            probe_colors.append(probe_color)
            probe_xy = (gx, gy)
            known_xy = (gx, gy)

        holdout = args.holdout if args.holdout > 0 else 0
        if len(pixel_pts) - holdout < 4:
            print(
                f"\nonly {len(pixel_pts)} usable point(s) ({holdout} held out); "
                "need >=4 to fit with -- not refitting",
                file=sys.stderr,
            )
            keep_probe_observations()
            return 1

        fit_px = pixel_pts[: len(pixel_pts) - holdout] if holdout else pixel_pts
        fit_rb = robot_pts[: len(robot_pts) - holdout] if holdout else robot_pts

        if calib.bundle_homography:
            # Preferred: reuse the corner bundle's perspective and fit only a
            # low-DOF alignment on top -- a similarity (4 DOF) in the metric
            # plane frame, or an affine (6 DOF) with >=8 points to also absorb
            # the slowly-varying part of the centroid bias. An unconstrained
            # 8-DOF homography from a handful of noisy blob centroids is
            # exactly the overfit that broke the table map; never again.
            import numpy as np

            from mt4_vision.calib import fit_affine
            from mt4_vision.table_fit import _apply_h, fit_similarity_2d

            hb = np.array(calib.bundle_homography)
            src = _apply_h(hb, np.array(fit_px, dtype=np.float64))
            dst = np.array(fit_rb, dtype=np.float64)
            if len(fit_px) >= 8:
                align = np.array(fit_affine([tuple(p) for p in src], [tuple(r) for r in dst]))
                kind = "bundle+affine"
            else:
                align = fit_similarity_2d(src, dst)
                kind = "bundle+similarity"
            matrix = (align @ hb).tolist()
        else:
            matrix, kind = fit_transform(fit_px, fit_rb)

        errors = reprojection_errors(matrix, fit_px, fit_rb)
        print(f"\n{kind} fit from {len(fit_px)} point(s)")
        print(f"In-fit reprojection error (mm): {[round(e, 2) for e in errors]}")

        if holdout:
            held_px = pixel_pts[-holdout:]
            held_rb = robot_pts[-holdout:]
            held_errors = reprojection_errors(matrix, held_px, held_rb)
            print(f"Held-out validation error (mm): {[round(e, 2) for e in held_errors]}")
            print("(This is the number that matters -- in-fit error is expected to look good regardless)")

        calib.cube_top_homography = matrix
        keep_probe_observations()
        print(f"\nSaved cube_top_homography ({len(pixel_pts)} raw probe "
              f"observations kept) to {args.calib}")
        return 0
    except Mt4ClientError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        client.close()


def _inverse_guess(calib, x: float, y: float) -> tuple[float, float]:
    """Rough pixel-space search center for a robot XY: numerically invert
    the table homography (good to a few px; cube-top parallax is <15px)."""
    import numpy as np

    h = np.array(calib.homography, dtype=np.float64)
    h_inv = np.linalg.inv(h)
    v = h_inv @ np.array([x, y, 1.0])
    return float(v[0] / v[2]), float(v[1] / v[2])


if __name__ == "__main__":
    raise SystemExit(main())
