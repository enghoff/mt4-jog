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
from mt4_vision.camera import FrameStream
from mt4_vision.detect import COLOR_RANGES, _top_face_centroid
from mt4_vision.pickplace import (
    _approach,
    _travel,
    home_arm,
    near_camera_park,
    pick,
    place,
    resolve_place_j4,
)
from mt4_vision.scene import Scene, capture_scene, is_phantom_detection
from mt4_vision.workspace import is_mp_reachable_xy

# Default site (200, 60): a calibrated probe point (2-4mm map accuracy),
# r=209 -- inside the validated torque-safe envelope rather than at the
# reach limit, where placement scattered 10-65mm. Max TCP Z here ~370mm:
# 10 cube levels of headroom. (--x/--y override; the original max-height
# brief pointed at x=0, whose every point has J1 at +-90deg and r pinned
# to the keep-out boundary -- the arm's worst accuracy zone.)
STACK_XY = (200.0, 60.0)
# Fixed grip-inspection pose: after every pick the held cube is hovered
# here and photographed. The first cube of each color defines that color's
# reference pixel; later cubes' deviation from it is their differential
# grip offset (picks land the cube off-center by several mm, and the
# offset rides along to the placement) -- subtracted from the place
# command. Fixed pose = arm repeatability is common mode; per-color
# references = centroid color bias is common mode.
INSPECT_POSE = (220.0, 66.0, 250.0)
GRIP_CAP_MM = 8.0
# Grip validation (Sigurd's policy): a measurably off-center grip is parked
# on a free spot and re-gripped rather than compensated -- re-picking from
# the arm-known park spot re-rolls the grip; only verified-good grips build
# the stack. After GRIP_TRIES the cube is rejected (parked on a free spot,
# never dropped) and another cube is tried for the level.
GRIP_OK_MM = 3.5
GRIP_TRIES = 3
GRIP_CUBE_ATTEMPTS = 3  # distinct cubes to try per level after grip rejects
# Soft avoid radius: don't immediately re-pick a cube we just parked.
AVOID_PARKED_MM = 40.0
# Camera-clear pose for captures: high and folded, like the homed pose but
# reached with a plain move -- no homing cycle (steps aren't lost in normal
# operation), and unlike pickplace's camera park it doesn't lean over the
# desk (the park pose shadows reads around (200, +-60)).
# z capped by soft J3 max (1150): (172,0,370) needs J3~1578 and `mp` rejects
# it with `err mp joints`. 340mm is the highest keep-out-pinned pose with
# ~60 steps of J3 headroom under the 2026-07-19 envelope limits.
CAPTURE_POSE = (172.0, 0.0, 340.0)
# The lean of lower capture poses shadows reads around (200, +-60) -- never
# park or read cubes there; (172,0,340) is nearly vertical (r pinned by the
# keep-out cylinder).
# Clearance climb above the current place height for every traverse near the
# stack -- the gripper (with cube) must never cross the stack column lower.
TRAVEL_ABOVE_MM = 35.0
# Radial via point on the stack's bearing: approach the column from outside
# along its own radius so no traverse arcs over the stack.
VIA_RADIUS_MM = 235.0  # outside the site radius, inside the torque-safe r<=245
# Verification thresholds. Drift is measured PAIRWISE (level N's top vs
# level N-1's observed top, one parallax step apart) -- measuring against
# the model-chained ideal column accumulated anchor/model/color bias and
# produced false misalignments that "corrections" then turned into real
# topples.
#
# CRITICAL: residual along the parallax axis is ONE observation shared by
# height and XY. A stacked cube short in X reads "too tall" (coupling);
# never treat positive h_err as a table miss. Height is only consulted when
# pairwise drift is within DRIFT_OK_MM (on-column).
HEIGHT_TOL_MM = 8.0       # on-column |h_est - h_expect| band
DRIFT_OK_MM = 6.0         # pairwise offset accepted outright; height trusted
DRIFT_FIXABLE_MM = 45.0   # beyond this the cube is somewhere unexpected: abort
# Closed-loop landing: the held cube's top face is measured over the stack
# BEFORE release -- at hover (z_travel) and again at release height -- and
# the arm is corrected until the cube is on target. This removes the
# z-walk / grip-offset / map-transfer errors from the landing entirely;
# what remains is release drag (2-6mm, semi-random). Falls back to the
# open-loop dead-reckoned place when the held cube isn't visible.
SERVO_HOVER_TOL_MM = 2.0     # accept hover position below this
SERVO_HOVER_ITERS = 5        # measure/correct rounds at hover
# Hover corrections in x respond with ~2x the commanded change (attempt-6:
# +9.9 -> -9.2 -> +9.1 oscillation); damping keeps the loop gain under 1.
SERVO_HOVER_GAIN = 0.55
SERVO_RELEASE_TOL_MM = 3.0   # max pre-release offset; else raise + retry
SERVO_RELEASE_RETRIES = 2
# Small reversing corrections at place height showed ~2-3x apparent gain
# in x (attempt-1 logs: backlash/stiction at r=209), so the learned
# descent compensation is damped and a badly positioned cube triggers a
# full re-approach (raise to hover, re-servo, descend) instead of endless
# in-place nudging -- and is never released more than the abort limit off.
SERVO_COMP_GAIN = 0.6
SERVO_APPROACH_CYCLES = 2    # hover+descend cycles before giving up
SERVO_RELEASE_ABORT_MM = 6.0
SERVO_SETTLE_S = 0.9         # camera settle before a servo measurement
# Near-miss picks shove neighbouring cubes (pick tolerance ~+-10mm):
# prefer candidates with clear space around them.
PICK_ISOLATION_MM = 45.0
FIX_ATTEMPTS = 2          # re-pick + corrected re-place tries per level
SERVO_CAP_MM = 4.0        # max no-contact nudge of the NEXT level's command
SEARCH_RADIUS_PX = 45.0
# Held-cube measurements must not match static cubes: the grip-inspect
# reference pixel sits only ~27-31px from the stack column's top-face
# pixels, and the 45px search radius let inspection (and the delivery
# servo) read the STACK instead of the held cube (attempt-3). Tight radii
# for held-cube searches + hard exclusion around known static tops.
INSPECT_SEARCH_RADIUS_PX = 20.0
TRACK_RADIUS_PX = 28.0    # re-finding a blob we already confirmed
EXCLUDE_RADIUS_PX = 14.0
# A cube whose landing drifted beyond this is NOT on the column -- a 20mm
# cube with >14mm offset cannot carry further levels; building on it is
# fiction (attempt-3 "built" phantom levels this way). Honest stop.
OFF_COLUMN_MM = 14.0
CAM_HEIGHT_GUESS_MM = 700.0
# Keep parked cubes this far from the stack site (and the symmetric shadow
# zone at (200, -60) that capture-pose occlusion used to hide).
SITE_KEEP_CLEAR_MM = 70.0
SITE_CLEAR_ATTEMPTS = 4


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


def classify_level(h_err: float, drift: float, cube: float) -> str:
    """Return 'misplaced' | 'perched' | 'seated' | 'abort' for a level read.

    Along-axis pixel residual is shared by monocular height and pairwise XY
    drift -- they are NOT independent evidence. A true-h=40 cube offset by
    dXY=(-16.5,+1)mm reads h_est=67.0 (matches field to 0.1mm): the old
    'missed_low' / height-overshoot heuristic was just XY.u coupling. Height
    anomalies are only trusted when pairwise drift is small enough that
    along-contamination is negligible; otherwise treat as seated and nudge.
    """
    if drift > DRIFT_FIXABLE_MM:
        return "abort"
    if drift > OFF_COLUMN_MM:
        # Not on the column: a 20mm cube offset >14mm cannot carry further
        # levels. Report honestly instead of "seating" it (attempt-3 built
        # phantom levels by accepting 30mm drifts as seated).
        return "misplaced"
    if drift <= DRIFT_OK_MM and h_err < -cube - HEIGHT_TOL_MM:
        return "abort"
    if drift <= DRIFT_OK_MM and abs(h_err) > HEIGHT_TOL_MM:
        # On-column with a height anomaly: LOW = hanging off an edge;
        # HIGH = tilted / corner-perched (attempt-6: level 2 read +11mm at
        # 4mm drift and the next cube slid off it). Both need a re-seat.
        return "perched"
    return "seated"


def site_occupant_color(
    frame: np.ndarray, near_px: tuple[float, float]
) -> str | None:
    for color in COLOR_RANGES:
        if find_top_face(frame, color, near_px) is not None:
            return color
    return None


def park_spot_for_clear(
    scene: Scene, sx: float, sy: float
) -> tuple[float, float, str] | None:
    """Free marker or table slot far enough from the stack / shadow zones."""
    shadow = ((sx, sy), (200.0, -60.0))

    def ok(x: float, y: float) -> bool:
        if near_camera_park(x, y):
            return False
        if not is_mp_reachable_xy(x, y):
            return False
        if ik_position(x, y, 185.0, near=JointAnglesDeg(0, 0, 0, 0)) is None:
            # A spot the arm cannot actually reach at safe height crashes
            # the run mid-place with `err mp joints` (attempt-9).
            return False
        return all(
            math.hypot(x - px, y - py) >= SITE_KEEP_CLEAR_MM for px, py in shadow
        )

    markers = [m for m in scene.placeable_markers() if ok(m.x, m.y)]
    if markers:
        m = max(markers, key=lambda m: math.hypot(m.x - sx, m.y - sy))
        return m.x, m.y, f"marker {m.marker_id}"
    slots = [(x, y) for x, y in scene.free_slots if ok(x, y)]
    if slots:
        x, y = max(slots, key=lambda p: math.hypot(p[0] - sx, p[1] - sy))
        return x, y, f"slot ({x:.0f},{y:.0f})"
    return None


def stack_pickable(scene: Scene) -> list:
    """Pick candidates WITHOUT the marker-hull phantom test: this desk's
    cubes legitimately sit 60-80mm outside the marker pattern and the hull
    filter was dropping them (measured 2026-07-19: 3 blues + 1 green at
    59-82mm outside, all real). Area and reachability checks stay active;
    the arm's own silhouette blobs are not a risk here because all stack
    captures happen from the capture pose, where they map into the
    keep-out and fail the reachability test."""
    raw = scene.raw_cubes if scene.raw_cubes is not None else scene.cubes
    return [c for c in raw if not is_phantom_detection(c, [], hull=None)]


def find_top_face(
    frame: np.ndarray,
    color: str,
    near: tuple[float, float],
    radius: float = SEARCH_RADIUS_PX,
    exclude: tuple[tuple[float, float], ...] = (),
) -> tuple[float, float] | None:
    """Segment `color` near the predicted top pixel; no global area caps --
    a high stack top is closer to the camera and larger than table cubes.

    ``exclude``: pixels of KNOWN static cubes (e.g. the current stack top)
    that must never be mistaken for the blob being sought -- attempt-3 logs
    showed held-cube searches locking onto the stack's own exposed faces.
    """
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
        if d > radius:
            continue
        if any(
            math.hypot(cx - ex, cy - ey) <= EXCLUDE_RADIUS_PX
            for ex, ey in exclude
        ):
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
    if not calib.cube_top_homography:
        print(
            "cube_top_homography missing -- stacking needs the cube-top "
            "parallax map (cleared by recalibrate_camera / table-plane "
            "refits). Run: python calibrate_height.py --camera 1",
            file=sys.stderr,
        )
        return 1
    ht_inv = np.linalg.inv(np.array(calib.homography))
    hc_inv = np.linalg.inv(np.array(calib.cube_top_homography))

    def px_of(hinv, x, y):
        v = hinv @ np.array([x, y, 1.0])
        return float(v[0] / v[2]), float(v[1] / v[2])

    model = ParallaxHeightModel(px_of(ht_inv, sx, sy), px_of(hc_inv, sx, sy), cube)
    ix, iy, iz = INSPECT_POSE
    h_ins = (iz - calib.pick_z) + cube  # held cube's top height at inspect
    imodel = ParallaxHeightModel(px_of(ht_inv, ix, iy), px_of(hc_inv, ix, iy), cube)
    # Predicted pixel of a gripped cube hovering at the capture pose: park
    # selection captures the scene while holding, and the held cube's top
    # (~(z-pick_z)+cube over the table) can pass detect's area cap and
    # falsely occupy markers/slots -- capture_scene drops the matching
    # oversized blob (scene.is_held_cube_blob).
    cx0, cy0, cz0 = CAPTURE_POSE
    cmodel = ParallaxHeightModel(px_of(ht_inv, cx0, cy0), px_of(hc_inv, cx0, cy0), cube)
    held_px = cmodel.predict_px((cz0 - calib.pick_z) + cube)
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
    p_i = px_of(ht_inv, ix, iy)
    r0_i = robot_of(*p_i)
    jac_i = np.column_stack([
        robot_of(p_i[0] + 1.0, p_i[1]) - r0_i,
        robot_of(p_i[0], p_i[1] + 1.0) - r0_i,
    ])
    mm_per_px = float(np.linalg.norm(jac @ np.array([1.0, 0.0])))

    client = Mt4Client() if not args.port else Mt4Client(port=args.port)
    snapdir = Path(args.snapshots) if args.snapshots else None
    if snapdir:
        snapdir.mkdir(parents=True, exist_ok=True)

    # One continuously-drained camera for the whole session: fresh frames
    # without the 2-3s per-capture reopen (see FrameStream).
    cam = FrameStream(args.camera)

    def snap(level: int) -> np.ndarray:
        time.sleep(0.8)
        frame = cam.fresh()
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
            frame0 = cam.fresh()
            if detect_cubes(frame0, calib):
                break
            print("fresh frame has no detections -- camera settling, retrying")
            time.sleep(2.0)
        p1 = model.predict_px(cube)
        # One extra iteration so occupancy is re-checked AFTER the last
        # clear; the abort only fires on a fresh still-occupied reading.
        for attempt in range(1, SITE_CLEAR_ATTEMPTS + 2):
            occ = site_occupant_color(frame0, p1)
            if occ is None:
                break
            if attempt > SITE_CLEAR_ATTEMPTS:
                print(f"stack site ({sx:.0f},{sy:.0f}) still occupied after "
                      f"{SITE_CLEAR_ATTEMPTS} clears -- stopping", file=sys.stderr)
                return 1
            scene0 = capture_scene(calib, frame0)
            park = park_spot_for_clear(scene0, sx, sy)
            if park is None:
                print(f"stack site ({sx:.0f},{sy:.0f}) holds a {occ} cube but "
                      f"no free park spot -- clear space and retry",
                      file=sys.stderr)
                return 1
            px, py, where = park
            near = [
                c for c in scene0.cubes
                if math.hypot(c.x - sx, c.y - sy) < SITE_KEEP_CLEAR_MM
            ]
            if near:
                t = max(near, key=lambda c: c.area)
                pick_xy = (float(t.x), float(t.y))
            else:
                # Placed at the commanded site: arm-known pick is reliable.
                pick_xy = (sx, sy)
            print(f"stack site occupied by {occ} -- clearing to {where} "
                  f"(attempt {attempt}/{SITE_CLEAR_ATTEMPTS})")
            pick(client, calib, *pick_xy)
            place(client, calib, px, py)
            _travel(client, calib, *CAPTURE_POSE, "capture pose")
            time.sleep(1.0)
            frame0 = cam.fresh()

        scene = capture_scene(calib, frame0)
        pickables = [
            c for c in stack_pickable(scene)
            if math.hypot(c.x - sx, c.y - sy) > SITE_KEEP_CLEAR_MM
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
        grip_ref: dict[str, tuple[float, float]] = {}
        # SESSION-scoped avoid list: park spots of rejected cubes and
        # grasp-hostile positions. Per-level scoping let the same bad
        # cube/spot be revisited level after level (attempt-8 looped on
        # one green six times across two runs).
        avoid_xys: list[tuple[float, float]] = []

        def choose_park(held_color: str) -> tuple[float, float, str] | None:
            """Capture pose + free marker/slot away from the stack site."""
            # Loaded leg to a high-z pose: slow (r=315 stall precedent).
            _approach(client, calib, *CAPTURE_POSE, "capture pose (held)")
            time.sleep(0.5)
            sc = capture_scene(
                calib, cam.fresh(),
                held_cube_px=held_px, held_color=held_color,
            )
            return park_spot_for_clear(sc, sx, sy)

        def park_held(label: str, held_color: str) -> tuple[float, float] | None:
            """Place the held cube on an unoccupied spot. Never open mid-air."""
            park = choose_park(held_color)
            if park is None:
                print(f"  {label}: no free park spot available", file=sys.stderr)
                return None
            px, py, where = park
            print(f"  {label}: placing at {where} ({px:.0f},{py:.0f})")
            place(client, calib, px, py)
            return px, py

        # Observed pixel of the current stack top: a KNOWN static cube that
        # held-cube searches (inspection + servo) must never match.
        stack_top_px: dict[str, tuple[float, float] | None] = {"px": None}

        def grip_offset(color: str):
            """Hover the held cube at the fixed inspect pose; returns
            (offset_mm, reading_px) or None when no cube is visible (grasp
            lost). Tight search radius + stack-top exclusion: the stack
            column's top faces sit only ~27-31px from the inspect reference
            and the old 45px search read the STACK as the held cube."""
            _approach(client, calib, ix, iy, iz, "inspect pose")
            time.sleep(0.8)
            frame = cam.fresh()
            guess = grip_ref.get(color) or imodel.predict_px(h_ins)
            excl = ((stack_top_px["px"],)
                    if stack_top_px["px"] is not None else ())
            # First reference for a color: the model prediction carries
            # ~30px systematic error (measured at the capture pose too),
            # so the tight radius only applies once a reference anchors
            # the search; the exclusion still guards the wide search.
            radius = (INSPECT_SEARCH_RADIUS_PX if color in grip_ref
                      else SEARCH_RADIUS_PX)
            g = find_top_face(frame, color, guess, radius=radius,
                              exclude=excl)
            if g is None:
                print("  grip inspection: cube not seen (grasp lost?)")
                return None
            if color not in grip_ref:
                # Motion-verify before trusting a FIRST reference: the wide
                # first-search can match a parked desk cube (attempt-6
                # anchored green's reference on one 75px off, poisoning
                # every later green check and blinding the servo). The held
                # cube must follow a small arm shift; static blobs don't.
                dv = (6.0, 0.0)
                inv_jac_i = np.linalg.inv(jac_i)
                dpx = inv_jac_i @ np.array(dv) * imodel.hc / (imodel.hc - h_ins)
                _approach(client, calib, ix + dv[0], iy + dv[1], iz,
                          "reference motion check")
                time.sleep(0.6)
                exp = np.array(g) + dpx
                g2 = find_top_face(cam.fresh(), color,
                                   (float(exp[0]), float(exp[1])),
                                   radius=TRACK_RADIUS_PX, exclude=excl)
                _approach(client, calib, ix, iy, iz, "inspect pose")
                moved_ok = False
                if g2 is not None:
                    moved = math.hypot(g2[0] - g[0], g2[1] - g[1])
                    moved_ok = moved >= 0.4 * float(np.linalg.norm(dpx))
                if not moved_ok:
                    print("  grip inspection: blob failed the motion check "
                          "(static desk cube, not the held one)")
                    return None
                time.sleep(0.6)
                g3 = find_top_face(cam.fresh(), color, g,
                                   radius=TRACK_RADIUS_PX, exclude=excl)
                if g3 is not None:
                    g = g3
                grip_ref[color] = g
                print(f"  grip reference for {color}: ({g[0]:.1f},{g[1]:.1f})"
                      f" (motion-verified)")
                return (0.0, 0.0), g
            ref = grip_ref[color]
            raw = ground_offset_mm(
                (g[0] - ref[0], g[1] - ref[1]), jac_i, imodel.hc, h_ins,
            )
            off = ground_offset_mm(
                (g[0] - ref[0], g[1] - ref[1]), jac_i, imodel.hc, h_ins,
                cap_mm=GRIP_CAP_MM,
            )
            print(f"  grip offset vs {color} reference: "
                  f"({raw[0]:+.1f},{raw[1]:+.1f})mm")
            return off, g

        def validated_grip(color: str, set_down_xy: tuple[float, float]):
            """Accept a centered grip, or park+re-grip; reject by parking.

            Returns (offset, reading, good) on accept, (\"parked\", xy) if the
            cube was rejected and placed on a free spot, None if the grasp
            was lost mid-inspect.
            """
            pick_xy = set_down_xy
            prev_px: tuple[float, float] | None = None
            for gtry in range(1, GRIP_TRIES + 1):
                res = grip_offset(color)
                if res is None:
                    return None
                off, reading = res
                n = math.hypot(*off)
                if n <= GRIP_OK_MM:
                    return off, reading, True
                if (prev_px is not None
                        and math.hypot(reading[0] - prev_px[0],
                                       reading[1] - prev_px[1]) < 2.0):
                    # A re-grip re-rolls the grip by several mm; a reading
                    # identical to the pre-park one is a static blob, not
                    # the held cube. Proceed ungated -- the delivery servo
                    # measures the cube itself anyway.
                    print("  identical reading after re-grip -- inspection "
                          "suspect; proceeding, delivery servo will correct")
                    return (0.0, 0.0), reading, False
                prev_px = reading
                if gtry < GRIP_TRIES:
                    print(f"  grip {n:.1f}mm off -- parking and re-gripping "
                          f"({gtry}/{GRIP_TRIES})")
                    parked = park_held("re-grip set-down", color)
                    if parked is None:
                        # Last resort: arm-known original pick XY, still a place.
                        place(client, calib, *pick_xy)
                    else:
                        pick_xy = parked
                    pick(client, calib, *pick_xy)
                    continue
                print(f"  grip still {n:.1f}mm off after {GRIP_TRIES} tries "
                      f"-- rejecting cube")
                parked_xy = park_held("rejected cube", color)
                if parked_xy is None:
                    # Must not drop: fall back to arm-known place.
                    print("  fallback place at last pick XY", file=sys.stderr)
                    place(client, calib, *pick_xy)
                    parked_xy = pick_xy
                return ("parked", parked_xy)
            return ("parked", pick_xy)

        inv_jac = np.linalg.inv(jac)

        def px_shift(dxy, h: float) -> np.ndarray:
            """Pixel displacement of a face at height h for a robot-frame
            XY displacement."""
            return inv_jac @ np.array(dxy) * model.hc / (model.hc - h)

        def held_target_px(cube_xy: tuple[float, float], h: float) -> np.ndarray:
            """Predicted pixel of the held cube's top if it sat exactly at
            cube_xy at height h. Anchor-relative once level 1 has anchored
            the model (same frame as verification); absolute two-plane
            extrapolation before that."""
            if getattr(model, "anchor", None) is not None:
                base = np.array(model.predict_px_rel(h))
            else:
                base = np.array(model.predict_px(h))
            dxy = np.array(cube_xy) - np.array([sx, sy])
            return base + px_shift(dxy, h)

        def measure_held(
            color: str,
            cube_xy: tuple[float, float],
            h: float,
            guess_px=None,
            radius: float = SEARCH_RADIUS_PX,
            exclude: tuple = (),
        ):
            """(offset_mm, found_px) of the held cube's top vs cube_xy at
            height h; None when not visible. ``guess_px`` narrows the
            search to a tracked blob instead of the absolute prediction."""
            time.sleep(SERVO_SETTLE_S)
            frame = cam.fresh()
            pred = held_target_px(cube_xy, h)
            near = ((float(guess_px[0]), float(guess_px[1]))
                    if guess_px is not None
                    else (float(pred[0]), float(pred[1])))
            g = find_top_face(frame, color, near, radius=radius,
                              exclude=exclude)
            if g is None:
                return None
            off = ground_offset_mm(
                (g[0] - pred[0], g[1] - pred[1]), jac, model.hc, h,
            )
            return off, g

        def _clamp(v: np.ndarray, cap: float) -> np.ndarray:
            n = float(np.linalg.norm(v))
            return v if n <= cap else v * (cap / n)

        TAKEUP_MM = 6.0

        def seat_xy(tx: float, ty: float, z: float, label: str) -> None:
            """Reach (tx,ty) from a consistent direction: backlash at the
            place radius ate the first 1-2 servo corrections per level
            (attempt 1+3 logs), so every fine XY move takes up the gears
            the same way before settling on the target."""
            _approach(client, calib, tx - TAKEUP_MM, ty - TAKEUP_MM, z,
                      label + " (take-up)")
            _approach(client, calib, tx, ty, z, label)

        # Learned arm XY walk of the fixed z_travel -> z_place descent
        # (same delta-z every level, one adaptive damped vector serves
        # all). Seeded from the CLI feed-forward, clamped so one misread
        # can never fling it (attempt-3 drove it to -25mm on a static
        # blob before static-lock detection existed).
        descent_comp = _clamp(
            np.array(args.shift_per_level, dtype=float)
            * (TRAVEL_ABOVE_MM / cube),
            12.0,
        )

        def deliver(
            x: float, y: float, z_pl: float, z_tr: float,
            color: str, cube_xy: tuple[float, float],
        ) -> None:
            # Loaded legs at the slow approach speed: fast cruise on
            # loaded, high-z, extended moves stalls steppers (the documented
            # r=315 lost-steps failure) -- landings scattered 10-60mm until
            # this leg was slowed, while every freshly-homed pick was fine.
            nonlocal descent_comp
            tcp = client.get_tcp()
            _approach(client, calib, tcp.x, tcp.y, z_tr, "climb with cube")
            _approach(client, calib, via[0], via[1], z_tr, "via point")
            seat_xy(x, y, z_tr, "over stack")
            # Square the held cube to the X/Y axes BEFORE the hover servo:
            # the rotation shifts the cube's XY by its grip offset, and the
            # servo then measures and cancels that shift like any other.
            # Later mm-scale servo moves barely change J1, so the world yaw
            # set here survives the wrist-preserving moves below.
            j4_sq = resolve_place_j4(client, calib)
            if j4_sq is not None:
                _approach(client, calib, x, y, z_tr, "square wrist", j4=j4_sq)
            ax, ay = x, y
            h_hover = (z_tr - calib.pick_z) + cube
            h_rel = (z_pl - calib.pick_z) + cube
            excl = ((stack_top_px["px"],)
                    if stack_top_px["px"] is not None else ())
            trusted = False   # vision positively tracking the held cube
            tracked = None    # last confirmed pixel of the held cube
            last_off = None
            for cycle in range(SERVO_APPROACH_CYCLES):
                # Closed loop at hover: measure the cube itself (cancels
                # grip offset, z-walk and map-transfer error in one
                # observation). Identity is verified by response: after a
                # correction the reading must follow the arm; a stationary
                # reading is a static blob (attempt-3 locked onto the
                # stack's own exposed faces and chased garbage).
                res = measure_held(color, cube_xy, h_hover, exclude=excl)
                for _ in range(SERVO_HOVER_ITERS):
                    if res is None:
                        break
                    off, gpx = res
                    print(f"  servo hover: cube off "
                          f"({off[0]:+.1f},{off[1]:+.1f})mm")
                    trusted, tracked = True, gpx
                    if math.hypot(*off) <= SERVO_HOVER_TOL_MM:
                        break
                    step = _clamp(SERVO_HOVER_GAIN * np.array(off), 12.0)
                    ax -= float(step[0])
                    ay -= float(step[1])
                    seat_xy(ax, ay, z_tr, "servo hover correct")
                    exp = np.array(gpx) + px_shift(-step, h_hover)
                    res2 = measure_held(color, cube_xy, h_hover,
                                        guess_px=exp,
                                        radius=TRACK_RADIUS_PX,
                                        exclude=excl)
                    if res2 is not None:
                        moved = math.hypot(res2[1][0] - gpx[0],
                                           res2[1][1] - gpx[1])
                        cmd_px = float(np.linalg.norm(px_shift(step, h_hover)))
                        if cmd_px >= 3.0 and moved < 0.3 * cmd_px:
                            print("  servo: reading ignored the correction "
                                  "-- static blob, distrusting vision")
                            trusted, tracked, res2 = False, None, None
                    res = res2
                if not trusted:
                    print("  servo: no confirmed view of the held cube "
                          "-- open loop")
                # Descend with the learned descent-walk compensation; check
                # once more before opening the gripper. Badly seated ->
                # raise and re-lower. The release read TRACKS the blob
                # confirmed at hover (tight radius) -- never a fresh
                # absolute search that could match a static cube.
                last_off = None
                prev_rel_px = None
                arm_prev = None
                for rtry in range(SERVO_RELEASE_RETRIES + 1):
                    tx_, ty_ = (ax - float(descent_comp[0]),
                                ay - float(descent_comp[1]))
                    seat_xy(tx_, ty_, z_tr, "pre-descent position")
                    _approach(client, calib, tx_, ty_, z_pl, "lower onto stack")
                    if not trusted:
                        break
                    if prev_rel_px is None:
                        exp = (np.array(tracked)
                               + model.u * (model.s_of_h(h_rel)
                                            - model.s_of_h(h_hover))
                               + px_shift((tx_ - ax, ty_ - ay), h_rel))
                    else:
                        exp = (np.array(prev_rel_px)
                               + px_shift((tx_ - arm_prev[0],
                                           ty_ - arm_prev[1]), h_rel))
                    res = measure_held(color, cube_xy, h_rel, guess_px=exp,
                                       radius=TRACK_RADIUS_PX)
                    arm_prev = (tx_, ty_)
                    if res is None:
                        print("  servo: cube not visible at release height "
                              "-- releasing as positioned")
                        last_off = None
                        break
                    off, gpx = res
                    prev_rel_px = gpx
                    last_off = off
                    print(f"  servo release: cube off "
                          f"({off[0]:+.1f},{off[1]:+.1f})mm  (descent comp "
                          f"({descent_comp[0]:+.1f},{descent_comp[1]:+.1f}))")
                    descent_comp = _clamp(
                        descent_comp + SERVO_COMP_GAIN * np.array(off), 12.0)
                    if (math.hypot(*off) <= SERVO_RELEASE_TOL_MM
                            or rtry == SERVO_RELEASE_RETRIES):
                        break
                    _approach(client, calib, ax, ay, z_tr, "raise to re-seat")
                if (last_off is None
                        or math.hypot(*last_off) <= SERVO_RELEASE_ABORT_MM):
                    break
                if cycle == SERVO_APPROACH_CYCLES - 1:
                    print(f"  servo: releasing {math.hypot(*last_off):.1f}mm "
                          f"off after all retries -- verification will "
                          f"handle it")
                    break
                print("  servo: still badly positioned -- full re-approach")
                _approach(client, calib, ax, ay, z_tr, "raise for re-approach")
            # Release happens at z_pl already 3mm under the nominal seat
            # (light contact; verified by the 2026-07-20 contact probe).
            # NO extra press before opening: releasing under a deliberate
            # 2mm press let the unloading fingers FLICK the cube 28mm
            # (attempt-10), while the plain contact release only drags
            # the usual few mm.
            client.gripper(calib.grip_open_s)
            # Rise DEAD VERTICAL from the exact release XY: the arm sits at
            # (ax - descent_comp), and clearing toward any other XY drags
            # the just-opened fingers through the cube -- attempt-5 shoved
            # a servo-perfect landing 33mm off the column this way.
            tcp2 = client.get_tcp()
            _approach(client, calib, tcp2.x, tcp2.y, z_tr, "clear stack")
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
            # Level 1 releases +1mm over the flat table (drops don't bounce
            # there). Level 2+ releases 3mm BELOW nominal seat: the arm's
            # true z runs a few mm high at the site, so "+1mm" was really a
            # several-mm drop onto the cube below -- which randomly bounces
            # 30-60mm (attempts 5-7 lost servo-perfect landings this way).
            # A light press is safe (the arm is springy); a drop is not.
            z_place = (calib.pick_z + (level - 1) * cube
                       + (1.0 if level == 1 else -3.0))
            z_travel = z_place + TRAVEL_ABOVE_MM
            if ik_position(sx, sy, z_travel, near=home_q) is None:
                print(f"level {level}: travel height {z_travel:.0f}mm not solvable -- stopping")
                break

            # Fresh scene each level: pick the planned color, verified grasp.
            # A cube that fails grip validation is parked on a free spot and
            # another candidate is tried (up to GRIP_CUBE_ATTEMPTS).
            goff = (0.0, 0.0)
            grip_reading = (0.0, 0.0)
            grip_good = False
            got_grip = False
            for cube_try in range(1, GRIP_CUBE_ATTEMPTS + 1):
                # Scene captures need the camera clear -- some paths (e.g.
                # parking a rejected cube) leave the arm over the desk.
                _travel(client, calib, *CAPTURE_POSE, "capture pose")
                time.sleep(0.8)
                scene = capture_scene(calib, cam.fresh())
                cands = [
                    c for c in stack_pickable(scene)
                    if c.color == color
                    and math.hypot(c.x - sx, c.y - sy) > SITE_KEEP_CLEAR_MM
                    and all(
                        math.hypot(c.x - ax, c.y - ay) >= AVOID_PARKED_MM
                        for ax, ay in avoid_xys
                    )
                ]
                if not cands:
                    print(f"level {level}: no pickable {color} cube left -- stopping")
                    break

                def clearance(c) -> float:
                    return min(
                        (math.hypot(c.x - o.x, c.y - o.y)
                         for o in scene.cubes if o is not c),
                        default=1e9,
                    )

                # Skip spots the arm cannot solve at safe height -- the
                # move would die mid-pick with `err mp joints` (attempt-9
                # crashed on a green at (120,-224)). Hard filter: an
                # unreachable cube is not a candidate at all.
                cands = [
                    c for c in cands
                    if ik_position(float(c.x), float(c.y), calib.safe_z,
                                   near=home_q) is not None
                ]
                if not cands:
                    print(f"level {level}: no reachable {color} cube left "
                          f"-- stopping")
                    break
                # Near-miss picks shove neighbours: take the biggest cube
                # with clear space around it; if none is clear, the most
                # isolated one.
                clear_cands = [
                    c for c in cands if clearance(c) >= PICK_ISOLATION_MM
                ]
                target = (max(clear_cands, key=lambda c: c.area)
                          if clear_cands else max(cands, key=clearance))
                print(f"level {level}: pick {color} at "
                      f"({target.x:.1f},{target.y:.1f}) "
                      f"clearance {clearance(target):.0f}mm "
                      f"(cube {cube_try}/{GRIP_CUBE_ATTEMPTS})")
                pick(
                    client, calib, target.x, target.y,
                    yaw_deg=target.yaw_deg,
                )
                # Straight to grip inspection: seeing the cube in the
                # gripper at the inspect pose proves the grasp AND measures
                # it -- the old separate capture-pose grasp check cost a
                # full round trip per pick.
                grip = validated_grip(color, (target.x, target.y))
                if grip is None:
                    # Nothing in view at inspect: grasp failed or lost.
                    _travel(client, calib, *CAPTURE_POSE, "capture pose")
                    frame = snap(0)
                    still = find_top_face(frame, color, (target.px, target.py))
                    if still is not None:
                        print("  grasp failed (cube still at pick spot) -- "
                              "rescanning")
                    else:
                        # Neither at the pick spot nor seen in the gripper:
                        # it may be held but occluded -- run a place cycle
                        # at a free spot before rescanning, never risk
                        # opening over thin air on a later pick. This
                        # cube/spot is hostile (attempt-8 looped on one
                        # green six times): avoid it for the whole session.
                        print("  cube neither at pick spot nor seen in "
                              "gripper -- safety park, then rescanning")
                        avoid_xys.append((float(target.x), float(target.y)))
                        if park_held("safety park", color) is None:
                            # NEVER proceed possibly-holding: the next
                            # pick opens the gripper at height (attempt-9
                            # risked a 340mm drop this way). Arm-known
                            # place at the original pick spot instead.
                            print("  no park spot -- placing back at the "
                                  "pick XY")
                            place(client, calib,
                                  float(target.x), float(target.y))
                    continue
                if isinstance(grip, tuple) and grip and grip[0] == "parked":
                    avoid_xys.append(grip[1])
                    if cube_try >= GRIP_CUBE_ATTEMPTS:
                        print(f"level {level}: no acceptable {color} grip "
                              f"after {GRIP_CUBE_ATTEMPTS} cubes -- stopping")
                    continue
                goff, grip_reading, grip_good = grip
                got_grip = True
                break
            if not got_grip:
                break

            h_expect = level * cube
            cmd = (np.array([sx, sy]) + carry
                   + np.array(args.shift_per_level) * (level - 1))
            level_ok = False
            for fix in range(FIX_ATTEMPTS + 1):
                deliver(float(cmd[0] - goff[0]), float(cmd[1] - goff[1]),
                        z_place, z_travel, color,
                        (float(cmd[0]), float(cmd[1])))
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
                    off1 = ground_offset_mm(
                        (found[0] - p_pred[0], found[1] - p_pred[1]),
                        jac, model.hc, h_expect,
                    )
                    print(f"level 1: top at ({found[0]:.1f},{found[1]:.1f}) "
                          f"-- anchor set (map-relative landing "
                          f"({off1[0]:+.1f},{off1[1]:+.1f})mm)")
                    prev_found = found
                    stack_top_px["px"] = found
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
                verdict = classify_level(h_err, drift, cube)
                if verdict == "misplaced":
                    print(f"level {level}: landed {drift:.1f}mm off the "
                          f"column -- NOT built; stopping honestly at "
                          f"{built} verified level(s)")
                    break
                if verdict == "abort" or (
                    verdict == "perched" and fix == FIX_ATTEMPTS
                ):
                    print(f"level {level}: not correctable "
                          f"(drift {drift:.1f}mm, h {h_est:.0f}mm) -- stopping")
                    break
                if verdict != "perched":
                    # Seated (possibly imperfectly). Moderate/large pairwise
                    # drift without an on-column height anomaly is XY offset
                    # (and/or along-axis coupling that fakes height) -- never
                    # table-pick; at most nudge the NEXT level's command.
                    if drift <= DRIFT_OK_MM:
                        # Only on-column reads train the hc fit: at larger
                        # drift `along` is XY contamination (the coupling
                        # above) and would skew every later height estimate.
                        model.add_observation(h_expect, along)
                    prev_found = found
                    stack_top_px["px"] = found
                    if drift > DRIFT_OK_MM:
                        nudge = ground_offset_mm(dp, jac, model.hc, h_expect,
                                                 cap_mm=SERVO_CAP_MM)
                        cmd = cmd - np.array(nudge)
                        print(f"  seated but offset: nudging next level by "
                              f"({-nudge[0]:+.1f},{-nudge[1]:+.1f})mm (no contact)")
                    level_ok = True
                    break
                # Perched only when on-column (drift<=OK) and height reads low.
                cube_xy = (float(cmd[0]) + off[0], float(cmd[1]) + off[1])
                print(f"  correction {fix + 1}: perched -- re-pick at "
                      f"({cube_xy[0]:.1f},{cube_xy[1]:.1f}) h~{h_est:.0f}mm, "
                      f"re-place shifted ({-off[0]:+.1f},{-off[1]:+.1f})mm")
                try:
                    grab_off_stack(cube_xy[0], cube_xy[1], max(h_est, cube), z_travel)
                except Mt4ClientError as exc:
                    print(f"  recovery grab failed ({exc}) -- stopping")
                    break
                err = np.array(off)
                n = float(np.linalg.norm(err))
                if n > 2 * SERVO_CAP_MM:
                    err *= 2 * SERVO_CAP_MM / n
                cmd = cmd - err
            if not level_ok:
                break
            if grip_good:
                # A grip that measured centered AND produced a verified
                # level becomes the color's new reference -- de-circularizes
                # a possibly-off first reference.
                grip_ref[color] = grip_reading
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
        cam.close()
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
