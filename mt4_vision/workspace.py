"""Work-surface model: calibrated markers, cube detections, occupancy, slots."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
import numpy as np

from mt4_vision.calib import Calibration
from mt4_vision.detect import CubeDetection, detect_cubes

# Cube centroid within this of a marker center counts the marker occupied
# when the ArUco tag does not decode. A decoded tag alone is *not*
# enough to call the marker free -- see PLACE_CLEARANCE_MM below.
# With the cube-top calibration fitted, on-marker cubes read 5-15mm from
# center while beside-the-paper cubes read 20mm+; 40mm classified adjacent
# cubes as occupants.
MARKER_OCCUPY_RADIUS_MM = 22.0
# Min center-to-center gap when folding a completed move into the state.
CUBE_CLEARANCE_MM = 35.0
# Min distance from any other cube for a *placement destination*: the
# fingers sweep outward when releasing, so they need more room than the
# cube footprint itself.
PLACE_CLEARANCE_MM = 45.0
# Open-table slots keep this far from marker papers (the printed sheet is
# wider than the occupancy radius).
MARKER_PAPER_CLEARANCE_MM = 40.0
# Min distance from any other cube for a *pick* target: the opened fingers
# straddle the cube, so a close neighbor gets clipped. Observed live: picks
# beside a ~35mm neighbor nudged both cubes instead of gripping.
PICK_CLEARANCE_MM = 45.0
# ArUco dictionary of the desk markers (same as calibrate_vision.py).
MARKER_DICT = "4x4_50"
# Conservative horizontal reach at pick height (mm).
MAX_REACH_MM = 320.0
# Firmware `mp` rejects TCP targets inside this cylinder (J1 axis, any Z).
KEEPOUT_RADIUS_MM = 170.0
KEEPOUT_TARGET_MARGIN_MM = 0.5  # mirrors start_absolute_move in motion.cpp

# Open-table placement candidates (robot frame, mm). Shared with
# calibrate_height.py probe grid.
PLACEMENT_SLOTS: list[tuple[float, float]] = [
    (200.0, -60.0),
    (200.0, 60.0),
    (150.0, 100.0),
    (240.0, -150.0),
    (240.0, 150.0),
    (150.0, -250.0),
    (150.0, 250.0),
    (280.0, 0.0),
]


@dataclass(frozen=True)
class MarkerSlot:
    marker_id: int
    x: float
    y: float


@dataclass
class WorkspaceState:
    cubes: list[CubeDetection]
    markers: list[MarkerSlot]
    occupied: list[tuple[MarkerSlot, CubeDetection]]
    # Placeable: tag decoded in the frame (provably empty) and clear of cubes.
    free_markers: list[MarkerSlot]
    # Neither provably empty nor occupied -- tag hidden (arm, shadow, a cube
    # on the paper's edge) with no cube inside the occupancy radius. Never a
    # placement target.
    unknown_markers: list[MarkerSlot]
    free_slots: list[tuple[float, float]]
    # Marker ids whose ArUco tag decoded in the source frame; None when the
    # state was built without decode information (legacy/test path).
    visible_marker_ids: set[int] | None = None


@dataclass(frozen=True)
class ShuffleMove:
    pick_x: float
    pick_y: float
    pick_color: str
    place_x: float
    place_y: float
    kind: str  # "to_marker" | "to_slot"
    place_marker_id: int | None = None


def dist_mm(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def is_mp_reachable_xy(x: float, y: float) -> bool:
    """True when firmware ``mp`` will accept (x, y) as a horizontal target."""
    return math.hypot(x, y) >= KEEPOUT_RADIUS_MM - KEEPOUT_TARGET_MARGIN_MM


def mp_reachable_cubes(cubes: list[CubeDetection]) -> list[CubeDetection]:
    return [c for c in cubes if is_mp_reachable_xy(c.x, c.y)]


def mp_reachable_markers(markers: list[MarkerSlot]) -> list[MarkerSlot]:
    return [m for m in markers if is_mp_reachable_xy(m.x, m.y)]


def pickable_cubes(cubes: list[CubeDetection]) -> list[CubeDetection]:
    """Cubes the gripper can actually go for: inside mp's reachable annulus
    and with finger clearance from every other detected cube."""
    return [
        c
        for c in mp_reachable_cubes(cubes)
        if all(
            dist_mm(c.x, c.y, o.x, o.y) >= PICK_CLEARANCE_MM
            for o in cubes
            if o is not c
        )
    ]


def marker_slots_from_calibration(calib: Calibration) -> list[MarkerSlot]:
    obs = calib.raw_marker_observations
    if not obs:
        return []
    slots: list[MarkerSlot] = []
    for key, data in obs.items():
        rx, ry = data["robot"]
        slots.append(MarkerSlot(int(key), float(rx), float(ry)))
    return sorted(slots, key=lambda m: m.marker_id)


def cubes_with_robot_coords(cubes: list[CubeDetection]) -> list[CubeDetection]:
    return [c for c in cubes if c.x is not None and c.y is not None]


def partition_cubes_on_markers(
    cubes: list[CubeDetection], markers: list[MarkerSlot]
) -> tuple[list[tuple[MarkerSlot, CubeDetection]], list[CubeDetection]]:
    """Return (occupied marker pairs, cubes not on any marker).

    Globally greedy nearest-pair matching: when two cubes contend for one
    marker, the loser can still claim its own second-nearest marker. The old
    per-cube nearest-only rule dropped the loser entirely, leaving a
    physically occupied marker "free" -- an invitation to stack.
    """
    pairs: list[tuple[float, int, MarkerSlot]] = []
    for index, cube in enumerate(cubes):
        for marker in markers:
            d = dist_mm(cube.x, cube.y, marker.x, marker.y)
            if d < MARKER_OCCUPY_RADIUS_MM:
                pairs.append((d, index, marker))
    pairs.sort(key=lambda p: p[0])

    on_marker: dict[int, CubeDetection] = {}
    assigned: set[int] = set()
    for _d, index, marker in pairs:
        if index in assigned or marker.marker_id in on_marker:
            continue
        on_marker[marker.marker_id] = cubes[index]
        assigned.add(index)

    occupied = [
        (m, on_marker[m.marker_id])
        for m in markers
        if m.marker_id in on_marker
    ]
    off_marker = [c for i, c in enumerate(cubes) if i not in assigned]
    return occupied, off_marker


def free_placement_slots(
    calib: Calibration,
    markers: list[MarkerSlot],
    cubes: list[CubeDetection],
    *,
    slots: list[tuple[float, float]] | None = None,
) -> list[tuple[float, float]]:
    candidates = slots if slots is not None else PLACEMENT_SLOTS
    free: list[tuple[float, float]] = []
    for sx, sy in candidates:
        if math.hypot(sx, sy) > MAX_REACH_MM:
            continue
        if not is_mp_reachable_xy(sx, sy):
            continue
        if any(dist_mm(sx, sy, m.x, m.y) < MARKER_PAPER_CLEARANCE_MM for m in markers):
            continue
        if any(dist_mm(sx, sy, c.x, c.y) < PLACE_CLEARANCE_MM for c in cubes):
            continue
        free.append((sx, sy))
    return free


def analyze_workspace(
    calib: Calibration,
    frame: np.ndarray,
) -> WorkspaceState:
    from mt4_vision.detect import detect_markers

    markers = marker_slots_from_calibration(calib)
    cubes = cubes_with_robot_coords(detect_cubes(frame, calib))
    visible = {m.marker_id for m in detect_markers(frame, MARKER_DICT)}
    return rebuild_workspace_state(calib, markers, cubes, visible_marker_ids=visible)


def rebuild_workspace_state(
    calib: Calibration,
    markers: list[MarkerSlot],
    cubes: list[CubeDetection],
    visible_marker_ids: set[int] | None = None,
) -> WorkspaceState:
    """Classify markers from detections plus tag visibility.

    Free requires *both* a decoded ArUco tag *and* no cube within
    PLACE_CLEARANCE_MM of the marker center. A visible tag alone is not
    enough -- a cube can sit beside or partly on the paper while the tag
    still decodes, and placing onto that marker would stack.

    Occupied: a cube sits inside MARKER_OCCUPY_RADIUS_MM (tag visibility
    does not override this -- we still check for blocks in the marker area).
    Unknown: neither free nor occupied (tag hidden with no nearby cube, or
    tag visible but a cube still inside place clearance). Never a place target.

    With visible_marker_ids=None (no frame available), falls back to
    proximity-only classification, still requiring place clearance for free.
    """
    if visible_marker_ids is None:
        occupied, _off = partition_cubes_on_markers(cubes, markers)
        occupied_ids = {m.marker_id for m, _ in occupied}
        free_markers = [
            m
            for m in markers
            if m.marker_id not in occupied_ids
            and all(dist_mm(m.x, m.y, c.x, c.y) >= PLACE_CLEARANCE_MM for c in cubes)
        ]
        free_ids = {m.marker_id for m in free_markers}
        unknown_markers = [
            m
            for m in markers
            if m.marker_id not in occupied_ids and m.marker_id not in free_ids
        ]
    else:
        # Cubes inside occupy radius claim their marker even if the tag still
        # decodes (partial occlusion / noisy decode). Visible tags without a
        # nearby cube still need place-clearance before counting as free.
        occupied, _off = partition_cubes_on_markers(cubes, markers)
        occupied_ids = {m.marker_id for m, _ in occupied}
        free_markers = [
            m
            for m in markers
            if m.marker_id in visible_marker_ids
            and m.marker_id not in occupied_ids
            and all(dist_mm(m.x, m.y, c.x, c.y) >= PLACE_CLEARANCE_MM for c in cubes)
        ]
        free_ids = {m.marker_id for m in free_markers}
        unknown_markers = [
            m
            for m in markers
            if m.marker_id not in occupied_ids and m.marker_id not in free_ids
        ]
    free_slots = free_placement_slots(calib, markers, cubes)
    return WorkspaceState(
        cubes=cubes,
        markers=markers,
        occupied=occupied,
        free_markers=free_markers,
        unknown_markers=unknown_markers,
        free_slots=free_slots,
        visible_marker_ids=visible_marker_ids,
    )


def apply_completed_move(
    state: WorkspaceState,
    move: ShuffleMove,
    calib: Calibration,
) -> WorkspaceState:
    """Fold a successful pick+place into the workspace (vision often lags here)."""
    place_x = move.place_x
    place_y = move.place_y
    if move.place_marker_id is not None:
        marker = next(m for m in state.markers if m.marker_id == move.place_marker_id)
        place_x = marker.x
        place_y = marker.y

    # Remove only the cube that was picked -- the single detection nearest the
    # pick point. Wiping everything within clearance erased innocent neighbors
    # too, which could flip an occupied marker to "free" and invite a stack.
    cubes = list(state.cubes)
    near_pick = [
        c for c in cubes
        if dist_mm(c.x, c.y, move.pick_x, move.pick_y) < CUBE_CLEARANCE_MM
    ]
    if near_pick:
        cubes.remove(
            min(near_pick, key=lambda c: dist_mm(c.x, c.y, move.pick_x, move.pick_y))
        )

    # The post-move frame usually already shows the cube at its destination.
    # Drop that detection -- for slot placements just like marker ones --
    # so the synthetic cube below doesn't duplicate it. (The destination was
    # verified clear at plan time, so anything inside clearance here is the
    # placed cube itself.)
    cubes = [
        c
        for c in cubes
        if dist_mm(c.x, c.y, place_x, place_y) >= CUBE_CLEARANCE_MM
    ]

    cubes.append(
        CubeDetection(
            color=move.pick_color,
            px=0.0,
            py=0.0,
            area=400.0,
            x=place_x,
            y=place_y,
        )
    )
    # The placed cube now covers its marker's tag even if the source frame
    # still showed it decoded.
    visible = state.visible_marker_ids
    if visible is not None and move.place_marker_id is not None:
        visible = visible - {move.place_marker_id}
    return rebuild_workspace_state(
        calib, state.markers, cubes, visible_marker_ids=visible
    )


class MarkerOccupancyTracker:
    """Carry marker occupancy across frames.

    The per-frame classification calls a hidden tag with no nearby cube
    "unknown" -- usually the arm occluding an occupied marker. Without
    memory the planner would starve on such markers (or, before the unknown
    state existed, stack onto them). The tracker keeps a marker occupied
    while it reads unknown if a recent frame saw a cube there, expiring the
    carry after MAX_CARRY consecutive unconfirmed frames (a human may have
    taken the cube). A decoded tag (frame says free) clears the carry
    immediately -- that signal is definitive.
    """

    MAX_CARRY = 4

    def __init__(self) -> None:
        # marker_id -> (last seen cube there, frames since last confirmation)
        self._occupied: dict[int, tuple[CubeDetection, int]] = {}

    def note_move(self, move: ShuffleMove, markers: list[MarkerSlot]) -> None:
        """Fold a completed pick+place into the carry map."""
        for m in markers:
            if dist_mm(m.x, m.y, move.pick_x, move.pick_y) < MARKER_OCCUPY_RADIUS_MM:
                self._occupied.pop(m.marker_id, None)
        if move.place_marker_id is not None:
            marker = next(
                m for m in markers if m.marker_id == move.place_marker_id
            )
            self._occupied[move.place_marker_id] = (
                CubeDetection(
                    color=move.pick_color,
                    px=0.0,
                    py=0.0,
                    area=400.0,
                    x=marker.x,
                    y=marker.y,
                ),
                0,
            )

    def update(self, state: WorkspaceState) -> WorkspaceState:
        """Merge frame classification with history; returns adjusted state."""
        if state.visible_marker_ids is None:
            return state  # no decode info -- nothing sound to carry

        carried: dict[int, tuple[CubeDetection, int]] = {
            m.marker_id: (c, 0) for m, c in state.occupied
        }
        unknown_ids = {m.marker_id for m in state.unknown_markers}
        for mid, (cube, age) in self._occupied.items():
            if mid in carried:
                continue
            if mid in unknown_ids and age + 1 <= self.MAX_CARRY:
                carried[mid] = (cube, age + 1)
        self._occupied = carried

        occupied = [
            (m, carried[m.marker_id][0])
            for m in state.markers
            if m.marker_id in carried
        ]
        occupied_ids = set(carried)
        unknown = [m for m in state.unknown_markers if m.marker_id not in occupied_ids]
        return WorkspaceState(
            cubes=state.cubes,
            markers=state.markers,
            occupied=occupied,
            free_markers=state.free_markers,
            unknown_markers=unknown,
            free_slots=state.free_slots,
            visible_marker_ids=state.visible_marker_ids,
        )


# A placed cube must be detected within this of the commanded destination
# (measured placement accuracy ~3-5mm; generous margin for blob noise).
VERIFY_PLACED_RADIUS_MM = 30.0
# A cube still within this of the pick point means the grasp failed -- the
# fingers nudge a near-missed cube a few mm rather than moving it away.
VERIFY_ORIGIN_RADIUS_MM = 25.0


def verify_move_outcome(state: WorkspaceState, move: ShuffleMove) -> str:
    """Judge a completed pick+place from an unoccluded post-move frame.

    Returns "ok" (cube of the right color seen at the destination),
    "grasp_failed" (still at the pick point), or "lost" (neither -- dropped
    in transit or knocked away). pick() has no force sensing, so a closed
    gripper proves nothing; this is the only ground truth we get.
    """
    dest_x, dest_y = move.place_x, move.place_y
    if move.place_marker_id is not None:
        marker = next(
            (m for m in state.markers if m.marker_id == move.place_marker_id),
            None,
        )
        if marker is not None:
            dest_x, dest_y = marker.x, marker.y
    same_color = [c for c in state.cubes if c.color == move.pick_color]
    if any(
        dist_mm(c.x, c.y, dest_x, dest_y) < VERIFY_PLACED_RADIUS_MM
        for c in same_color
    ):
        return "ok"
    if any(
        dist_mm(c.x, c.y, move.pick_x, move.pick_y) < VERIFY_ORIGIN_RADIUS_MM
        for c in same_color
    ):
        return "grasp_failed"
    return "lost"


def cubes_of_color(cubes: list[CubeDetection], color: str) -> list[CubeDetection]:
    return [c for c in cubes if c.color == color]


def pick_largest_cube(cubes: list[CubeDetection]) -> CubeDetection | None:
    if not cubes:
        return None
    return max(cubes, key=lambda c: c.area)


def plan_shuffle_move(state: WorkspaceState) -> ShuffleMove | None:
    """Pick a random cube and an empty marker, or relocate off a full marker."""
    pickable = pickable_cubes(state.cubes)
    place_markers = mp_reachable_markers(state.free_markers)
    if place_markers and pickable:
        cube = random.choice(pickable)
        marker = random.choice(place_markers)
        return ShuffleMove(
            cube.x,
            cube.y,
            cube.color,
            marker.x,
            marker.y,
            "to_marker",
            place_marker_id=marker.marker_id,
        )
    # Identity check against the pickable set: this also skips occupants the
    # tracker merely carries from an earlier frame (not in state.cubes) --
    # no blind picks at markers the arm is currently occluding.
    clear_pickable = set(map(id, pickable))
    reachable_occupied = [
        (marker, cube)
        for marker, cube in state.occupied
        if id(cube) in clear_pickable
    ]
    if reachable_occupied and state.free_slots:
        marker, cube = random.choice(reachable_occupied)
        sx, sy = random.choice(state.free_slots)
        return ShuffleMove(cube.x, cube.y, cube.color, sx, sy, "to_slot")
    return None
