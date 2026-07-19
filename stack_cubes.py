#!/usr/bin/env python3
"""Build the tallest possible cube stack at the arm's highest-reach point.

The stack site defaults to (0, 178): along x=0 the IK envelope peaks right
at the keep-out boundary (max TCP Z ~389mm, enough for 12 cube levels), and
the +y side is clear desk (the controller box lives at -y).

Placement is dead-reckoned: XY is the same arm-frame coordinate every level
(arm repeatability ~1mm), Z steps by cube_height_mm per level, released a
few mm above the resting height exactly like a table place.

Verification is vision-based via height-from-parallax: the calibration
carries two reference planes (table at 0mm, cube tops at cube_height_mm),
so the stack site's image point slides along a known parallax line as the
stack grows. After each place, the new top face is segmented near its
predicted pixel; its displacement *along* the parallax line is a monocular
height measurement (the projective model's one unknown, the camera height,
is refined from the accumulating observations), and displacement
*perpendicular* to the line measures stack lean. A missing/short/leaning
top face stops the build before the arm knocks anything over.

Pick order alternates colors so each new level is visually unambiguous
against the level below it.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_jog.kinematics import JointAnglesDeg, ik_position
from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration
from mt4_vision.camera import capture_frame
from mt4_vision.detect import COLOR_RANGES, _top_face_centroid
from mt4_vision.pickplace import _approach, _travel, home_arm, pick
from mt4_vision.scene import capture_scene

STACK_XY = (0.0, 178.0)
# Camera-clear pose for captures: high and folded, like the homed pose but
# reached with a plain move -- no homing cycle (steps aren't lost in normal
# operation), and unlike pickplace's camera park it doesn't lean over the
# desk (the park pose shadows reads around (200, +-60)).
CAPTURE_POSE = (175.0, 0.0, 340.0)
# Clearance climb above the current place height for every traverse near the
# stack -- the gripper (with cube) must never cross the stack column lower.
TRAVEL_ABOVE_MM = 35.0
# Radial via point on the stack's bearing: approach the column from outside
# along its own radius so no traverse arcs over the stack.
VIA_RADIUS_MM = 250.0
# Verification thresholds. Drift is measured PAIRWISE (level N's top vs
# level N-1's observed top, one parallax step apart) -- measuring against
# the model-chained ideal column accumulated anchor/model/color bias and
# produced false misalignments that "corrections" then turned into real
# topples. A physical re-pick additionally requires a height anomaly: a
# perched cube reads low, a well-seated cube with a color-biased centroid
# reads correct height.
HEIGHT_TOL_MM = 8.0       # |h_est - h_expect| beyond this corroborates a perch
DRIFT_OK_MM = 6.0         # pairwise offset accepted outright
DRIFT_REPICK_MM = 9.0     # re-pick only past this AND with height anomaly
DRIFT_FIXABLE_MM = 45.0   # beyond this the cube is somewhere unexpected: abort
FIX_ATTEMPTS = 2          # re-pick + corrected re-place tries per level
SERVO_CAP_MM = 4.0        # max no-contact nudge of the NEXT level's command
SEARCH_RADIUS_PX = 45.0
CAM_HEIGHT_GUESS_MM = 700.0


def color_sequence(counts: dict[str, int], levels: int) -> list[str]:
    """Alternating color order: never two equal colors adjacent, spending
    the most abundant color first so the alternation stays feasible."""
    remaining = dict(counts)
    seq: list[str] = []
    for _ in range(levels):
        options = [c for c, n in remaining.items() if n > 0 and (not seq or c != seq[-1])]
        if not options:
            break
        pick_c = max(options, key=lambda c: (remaining[c], c))
        seq.append(pick_c)
        remaining[pick_c] -= 1
    return seq


class ParallaxHeightModel:
    """Monocular stack height from top-face pixel displacement.

    Displacement along the parallax direction follows s(h) = A*h/(Hc - h)
    with A fixed by the calibrated cube-top plane (s(cube) = |p_cube - p0|)
    and Hc (camera height over the table) the single unknown, refined by a
    1-D least-squares over the accumulated (nominal height, measured s)
    pairs.
    """

    def __init__(self, p0: tuple[float, float], p_cube: tuple[float, float],
                 cube_mm: float) -> None:
        self.p0 = np.array(p0)
        d = np.array(p_cube) - self.p0
        self.s_cube = float(np.linalg.norm(d))
        self.u = d / self.s_cube
        self.cube_mm = cube_mm
        self.hc = CAM_HEIGHT_GUESS_MM
        self.obs: list[tuple[float, float]] = []  # (nominal h, measured s)

    def _a(self, hc: float) -> float:
        return self.s_cube * (hc - self.cube_mm) / self.cube_mm

    def s_of_h(self, h: float, hc: float | None = None) -> float:
        hc = self.hc if hc is None else hc
        return self._a(hc) * h / (hc - h)

    def h_of_s(self, s: float) -> float:
        return s * self.hc / (self._a(self.hc) + s)

    def predict_px(self, h: float) -> tuple[float, float]:
        p = self.p0 + self.u * self.s_of_h(h)
        return float(p[0]), float(p[1])

    def components(self, px: float, py: float) -> tuple[float, float]:
        """(along, perpendicular) pixel components of a measured top pixel."""
        d = np.array([px, py]) - self.p0
        along = float(d @ self.u)
        perp = float(d @ np.array([-self.u[1], self.u[0]]))
        return along, perp

    def set_anchor(self, px: float, py: float, h_anchor: float) -> None:
        """Anchor the model on an OBSERVED top face of known height.

        The map's own pixel prediction for the stack site carries that
        spot's XY mapping error, which projects onto the parallax line and
        corrupts absolute height readings; measuring every later level
        relative to the first placed cube's observed pixel cancels it.
        """
        self.anchor = np.array([px, py], dtype=float)
        self.h_anchor = h_anchor

    def rel_components(self, px: float, py: float) -> tuple[float, float]:
        d = np.array([px, py]) - self.anchor
        along = float(d @ self.u)
        perp = float(d @ np.array([-self.u[1], self.u[0]]))
        return along, perp

    def h_from_rel(self, along: float) -> float:
        return self.h_of_s(self.s_of_h(self.h_anchor) + along)

    def predict_px_rel(self, h: float) -> tuple[float, float]:
        p = self.anchor + self.u * (self.s_of_h(h) - self.s_of_h(self.h_anchor))
        return float(p[0]), float(p[1])

    def add_observation(self, h_nominal: float, along_rel: float) -> None:
        """Relative observation: displacement from the anchor along u."""
        self.obs.append((h_nominal, along_rel))
        if len(self.obs) < 2:
            return
        grid = np.linspace(300.0, 2000.0, 341)
        costs = [
            sum(
                ((self.s_of_h(h, hc) - self.s_of_h(self.h_anchor, hc)) - s) ** 2
                for h, s in self.obs
            )
            for hc in grid
        ]
        self.hc = float(grid[int(np.argmin(costs))])


def ground_offset_mm(
    dp_px: tuple[float, float],
    jac: np.ndarray,
    hc_mm: float,
    h_mm: float,
    cap_mm: float | None = None,
) -> tuple[float, float]:
    """Robot-frame XY offset for a pixel deviation of a face at height h.

    jac is the table-plane d(robot)/d(pixel) 2x2; a face at height h moves
    (hc/(hc-h)) more pixels per mm than the table, hence the scaling.
    """
    o = jac @ np.array(dp_px) * (hc_mm - h_mm) / hc_mm
    if cap_mm is not None:
        n = float(np.linalg.norm(o))
        if n > cap_mm:
            o *= cap_mm / n
    return float(o[0]), float(o[1])


def find_top_face(
    frame: np.ndarray, color: str, near: tuple[float, float]
) -> tuple[float, float] | None:
    """Segment `color` near the predicted top pixel; no global area caps --
    a high stack top is closer to the camera and larger than table cubes."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in COLOR_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best: tuple[float, tuple[float, float]] | None = None
    for c in contours:
        if cv2.contourArea(c) < 100:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        d = math.hypot(cx - near[0], cy - near[1])
        if d > SEARCH_RADIUS_PX:
            continue
        px, py = _top_face_centroid(hsv, c, (cx, cy))
        if best is None or d < best[0]:
            best = (d, (px, py))
    return None if best is None else best[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="Stack cubes as high as possible")
    parser.add_argument("--port", default="")
    parser.add_argument("--camera", type=int, default=-1)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument("--x", type=float, default=STACK_XY[0])
    parser.add_argument("--y", type=float, default=STACK_XY[1])
    parser.add_argument("--max-levels", type=int, default=12)
    parser.add_argument(
        "--shift-per-level", type=float, nargs=2, default=(0.0, 0.0),
        metavar=("DX", "DY"),
        help="feed-forward XY compensation added per level: near the reach "
             "envelope the arm's true XY walks with commanded z (measured "
             "~(-8,-6)mm per 20mm at (0,190) by hovering a held cube); pass "
             "the NEGATED measured shift",
    )
    parser.add_argument("--snapshots", default="", help="directory for per-level photos")
    args = parser.parse_args()

    calib = load_calibration(Path(args.calib))
    cube = calib.cube_height_mm
    sx, sy = args.x, args.y
    home_q = JointAnglesDeg(0.0, 0.0, 0.0, 0.0)
    via = (
        sx / math.hypot(sx, sy) * VIA_RADIUS_MM,
        sy / math.hypot(sx, sy) * VIA_RADIUS_MM,
    )

    # Parallax model from the two calibrated planes at the stack site.
    ht_inv = np.linalg.inv(np.array(calib.homography))
    hc_inv = np.linalg.inv(np.array(calib.cube_top_homography))

    def px_of(hinv, x, y):
        v = hinv @ np.array([x, y, 1.0])
        return float(v[0] / v[2]), float(v[1] / v[2])

    model = ParallaxHeightModel(px_of(ht_inv, sx, sy), px_of(hc_inv, sx, sy), cube)
    # Local scale for reporting lean in mm, and the table-plane
    # pixel->robot Jacobian for vision-servoed placement corrections.
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
    mm_per_px = float(np.linalg.norm(jac @ np.array([1.0, 0.0])))

    client = Mt4Client() if not args.port else Mt4Client(port=args.port)
    snapdir = Path(args.snapshots) if args.snapshots else None
    if snapdir:
        snapdir.mkdir(parents=True, exist_ok=True)

    def snap(level: int) -> np.ndarray:
        time.sleep(1.0)
        frame = capture_frame(args.camera)
        if snapdir:
            cv2.imwrite(str(snapdir / f"level{level}.png"), frame)
        return frame

    built = 0
    try:
        client.ensure_connected()
        if not client.get_status().homed:
            home_arm(client)
        _travel(client, calib, *CAPTURE_POSE, "capture pose")

        # First capture after another process just released the camera can
        # come back with unconverged exposure (near-zero detections) despite
        # the open-time warmup -- retry until the frame is usable.
        from mt4_vision.detect import detect_cubes
        for _warm in range(4):
            frame0 = capture_frame(args.camera)
            if detect_cubes(frame0, calib):
                break
            print("fresh frame has no detections -- camera settling, retrying")
            time.sleep(2.0)
        p1 = model.predict_px(cube)
        for c0 in COLOR_RANGES:
            if find_top_face(frame0, c0, p1) is not None:
                print(f"stack site ({sx:.0f},{sy:.0f}) already holds a {c0} cube "
                      "-- clear it first (it was placed at the site coordinate, "
                      "so an arm-known pick there will lift it)", file=sys.stderr)
                return 1

        scene = capture_scene(calib, frame0)
        pickables = [
            c for c in scene.pickable(scene.cubes)
            if math.hypot(c.x - sx, c.y - sy) > 70.0
        ]
        counts: dict[str, int] = {}
        for c in pickables:
            counts[c.color] = counts.get(c.color, 0) + 1
        seq = color_sequence(counts, min(args.max_levels, len(pickables)))
        print(f"{len(pickables)} pickable cubes {counts}; planned stack: {seq}")
        # Closed-loop placement: the arm's XY at this near-envelope radius
        # shifts systematically with commanded z (measured ~8-10mm between
        # consecutive levels, same direction every run), so open-loop
        # stacking perches cubes on corners. Each level is measured after
        # landing; a perched cube is re-picked (its position AND height come
        # from the parallax model) and re-placed with the measured error
        # subtracted. The winning correction carries forward as the next
        # level's prior.
        carry = np.zeros(2)

        def deliver(x: float, y: float, z_pl: float, z_tr: float) -> None:
            tcp = client.get_tcp()
            _travel(client, calib, tcp.x, tcp.y, z_tr, "climb with cube")
            _travel(client, calib, via[0], via[1], z_tr, "via point")
            _travel(client, calib, x, y, z_tr, "over stack")
            _approach(client, calib, x, y, z_pl, "lower onto stack")
            client.gripper(calib.grip_open_s)
            _approach(client, calib, x, y, z_tr, "clear stack")
            _travel(client, calib, via[0], via[1], z_tr, "retreat via")
            _travel(client, calib, *CAPTURE_POSE, "capture pose")

        def grab_off_stack(x: float, y: float, h_top: float, z_tr: float) -> None:
            client.gripper(calib.grip_open_s)
            _travel(client, calib, via[0], via[1], z_tr, "via point")
            _travel(client, calib, x, y, z_tr, "over perched cube")
            _approach(client, calib, x, y, calib.pick_z + (h_top - cube), "descend to cube")
            client.gripper(calib.grip_close_s)
            _approach(client, calib, x, y, z_tr, "lift cube")

        for level, color in enumerate(seq, start=1):
            # +1mm, not the table-place +3mm: a 3mm drop onto a slightly
            # misaligned cube edge BOUNCES it off the stack (observed 30-40mm
            # scatter); setting it down in near-contact cannot.
            z_place = calib.pick_z + (level - 1) * cube + 1.0
            z_travel = z_place + TRAVEL_ABOVE_MM
            if ik_position(sx, sy, z_travel, near=home_q) is None:
                print(f"level {level}: travel height {z_travel:.0f}mm not solvable -- stopping")
                break

            # Fresh scene each level: pick the planned color, verified grasp.
            scene = capture_scene(calib, capture_frame(args.camera))
            cands = [
                c for c in scene.pickable(scene.cubes)
                if c.color == color and math.hypot(c.x - sx, c.y - sy) > 70.0
            ]
            if not cands:
                print(f"level {level}: no pickable {color} cube left -- stopping")
                break
            target = max(cands, key=lambda c: c.area)
            grabbed = False
            for attempt in (1, 2):
                print(f"level {level}: pick {color} at ({target.x:.1f},{target.y:.1f}) "
                      f"(attempt {attempt})")
                pick(client, calib, target.x, target.y)
                # Clear the view BEFORE the grasp check: at safe_z over the
                # pick spot the held cube itself images near the pick pixel
                # and reads as a false "still there". A camera-park retreat
                # suffices -- no homing cycle needed (steps aren't lost in
                # normal operation).
                _travel(client, calib, *CAPTURE_POSE, "capture pose")
                frame = snap(0)
                still = find_top_face(frame, color, (target.px, target.py))
                if still is None:
                    grabbed = True
                    break
                print("  grasp failed; retrying from fresh reading")
                scene = capture_scene(calib, frame)
                fresh = [
                    c for c in scene.cubes
                    if c.color == color
                    and math.hypot(c.px - target.px, c.py - target.py) < 60.0
                ]
                if not fresh:
                    break
                target = fresh[0]
            if not grabbed:
                print(f"level {level}: could not grasp a {color} cube -- stopping")
                break

            h_expect = level * cube
            cmd = (np.array([sx, sy]) + carry
                   + np.array(args.shift_per_level) * (level - 1))
            level_ok = False
            for fix in range(FIX_ATTEMPTS + 1):
                deliver(float(cmd[0]), float(cmd[1]), z_place, z_travel)
                frame = snap(level)
                p_pred = (model.predict_px(h_expect) if level == 1
                          else model.predict_px_rel(h_expect))
                found = find_top_face(frame, color, p_pred)
                if found is None:
                    print(f"level {level}: top face NOT FOUND near "
                          f"({p_pred[0]:.0f},{p_pred[1]:.0f}) -- stopping")
                    break
                if level == 1:
                    # First cube anchors the model: its observed pixel IS
                    # height `cube` -- absolute height at this extrapolated
                    # site would inherit the map's local XY error.
                    model.set_anchor(found[0], found[1], h_expect)
                    print(f"level 1: top at ({found[0]:.1f},{found[1]:.1f}) -- anchor set")
                    prev_found = found
                    level_ok = True
                    break
                along, _perp = model.rel_components(*found)
                h_est = model.h_from_rel(along)
                # PAIRWISE drift: level N's top vs level N-1's observed top,
                # minus the expected one-level parallax step. Cube heights
                # are quantized, so what remains is the physical offset of
                # this cube relative to the one it rests on.
                step = model.s_of_h(h_expect) - model.s_of_h(h_expect - cube)
                dp = (
                    found[0] - prev_found[0] - model.u[0] * step,
                    found[1] - prev_found[1] - model.u[1] * step,
                )
                off = ground_offset_mm(dp, jac, model.hc, h_expect)
                drift = math.hypot(*off)
                h_err = h_est - h_expect
                print(f"level {level}: top at ({found[0]:.1f},{found[1]:.1f})  "
                      f"height {h_est:.0f}mm (expect {h_expect:.0f})  "
                      f"pair-drift ({off[0]:+.1f},{off[1]:+.1f})mm  "
                      f"cam-height est {model.hc:.0f}mm")
                perched = drift > DRIFT_REPICK_MM and abs(h_err) > HEIGHT_TOL_MM
                if not perched and drift <= DRIFT_FIXABLE_MM:
                    # Seated (possibly imperfectly). Moderate drift readings
                    # without a height anomaly are as likely measurement bias
                    # as real offset -- never touch the placed cube for them,
                    # at most nudge the NEXT level's command.
                    model.add_observation(h_expect, along)
                    prev_found = found
                    if drift > DRIFT_OK_MM:
                        nudge = ground_offset_mm(dp, jac, model.hc, h_expect,
                                                 cap_mm=SERVO_CAP_MM)
                        cmd = cmd - np.array(nudge)
                        print(f"  seated but offset: nudging next level by "
                              f"({-nudge[0]:+.1f},{-nudge[1]:+.1f})mm (no contact)")
                    level_ok = True
                    break
                if (drift > DRIFT_FIXABLE_MM
                        or h_est < h_expect - cube - HEIGHT_TOL_MM
                        or fix == FIX_ATTEMPTS):
                    print(f"level {level}: not correctable "
                          f"(drift {drift:.1f}mm, h {h_est:.0f}mm) -- stopping")
                    break
                # Perched (large drift corroborated by height anomaly):
                # re-pick the cube where vision says it is and re-place with
                # the measured error subtracted.
                cube_xy = (float(cmd[0]) + off[0], float(cmd[1]) + off[1])
                print(f"  correction {fix + 1}: perched -- re-pick at "
                      f"({cube_xy[0]:.1f},{cube_xy[1]:.1f}) h~{h_est:.0f}mm, "
                      f"re-place shifted ({-off[0]:+.1f},{-off[1]:+.1f})mm")
                grab_off_stack(cube_xy[0], cube_xy[1], max(h_est, cube), z_travel)
                err = np.array(off)
                n = float(np.linalg.norm(err))
                if n > 2 * SERVO_CAP_MM:
                    err *= 2 * SERVO_CAP_MM / n
                cmd = cmd - err
            if not level_ok:
                break
            # Nudges/corrections persist via carry; the per-level
            # feed-forward term must not compound into it.
            carry = (cmd - np.array([sx, sy])
                     - np.array(args.shift_per_level) * (level - 1))
            built = level

        print(f"\nStack complete: {built} level(s), "
              f"~{built * cube:.0f}mm tall at ({sx:.0f},{sy:.0f})")
        return 0
    except Mt4ClientError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
