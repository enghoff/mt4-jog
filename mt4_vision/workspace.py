"""Work-surface model: calibrated markers, cube detections, occupancy, slots."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
import numpy as np

from mt4_vision.calib import Calibration
from mt4_vision.detect import CubeDetection, detect_cubes

# Cube centroid within this of a marker center counts the marker occupied.
MARKER_OCCUPY_RADIUS_MM = 40.0
# Min center-to-center gap when placing on open table (one cube width).
CUBE_CLEARANCE_MM = 35.0
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
    free_markers: list[MarkerSlot]
    free_slots: list[tuple[float, float]]


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


def marker_slots_from_calibration(calib: Calibration) -> list[MarkerSlot]:
    obs = calib.raw_marker_observations
    if not obs:
        return []
    slots: list[MarkerSlot] = []
    for key, data in obs.items():
        rx, ry = data["robot"]
        slots.append(MarkerSlot(int(key), float(rx), float(ry)))
    return sorted(slots, key=lambda m: m.marker_id)


def nearest_marker(
    cube: CubeDetection,
    markers: list[MarkerSlot],
    *,
    max_dist: float = MARKER_OCCUPY_RADIUS_MM,
) -> MarkerSlot | None:
    best: MarkerSlot | None = None
    best_d = max_dist
    for marker in markers:
        d = dist_mm(cube.x, cube.y, marker.x, marker.y)
        if d < best_d:
            best_d = d
            best = marker
    return best


def cubes_with_robot_coords(cubes: list[CubeDetection]) -> list[CubeDetection]:
    return [c for c in cubes if c.x is not None and c.y is not None]


def partition_cubes_on_markers(
    cubes: list[CubeDetection], markers: list[MarkerSlot]
) -> tuple[list[tuple[MarkerSlot, CubeDetection]], list[CubeDetection]]:
    """Return (occupied marker pairs, cubes not on any marker)."""
    on_marker: dict[int, CubeDetection] = {}
    off_marker: list[CubeDetection] = []
    for cube in cubes:
        marker = nearest_marker(cube, markers)
        if marker is None:
            off_marker.append(cube)
        elif marker.marker_id in on_marker:
            # Two cubes claiming one marker -- keep the closer one.
            prev = on_marker[marker.marker_id]
            if dist_mm(cube.x, cube.y, marker.x, marker.y) < dist_mm(
                prev.x, prev.y, marker.x, marker.y
            ):
                off_marker.append(prev)
                on_marker[marker.marker_id] = cube
            else:
                off_marker.append(cube)
        else:
            on_marker[marker.marker_id] = cube
    occupied = [
        (m, on_marker[m.marker_id])
        for m in markers
        if m.marker_id in on_marker
    ]
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
        if any(dist_mm(sx, sy, m.x, m.y) < MARKER_OCCUPY_RADIUS_MM for m in markers):
            continue
        if any(dist_mm(sx, sy, c.x, c.y) < CUBE_CLEARANCE_MM for c in cubes):
            continue
        free.append((sx, sy))
    return free


def analyze_workspace(
    calib: Calibration,
    frame: np.ndarray,
) -> WorkspaceState:
    markers = marker_slots_from_calibration(calib)
    cubes = cubes_with_robot_coords(detect_cubes(frame, calib))
    return rebuild_workspace_state(calib, markers, cubes)


def rebuild_workspace_state(
    calib: Calibration,
    markers: list[MarkerSlot],
    cubes: list[CubeDetection],
) -> WorkspaceState:
    occupied, _off = partition_cubes_on_markers(cubes, markers)
    occupied_ids = {m.marker_id for m, _ in occupied}
    free_markers = [m for m in markers if m.marker_id not in occupied_ids]
    free_slots = free_placement_slots(calib, markers, cubes)
    return WorkspaceState(
        cubes=cubes,
        markers=markers,
        occupied=occupied,
        free_markers=free_markers,
        free_slots=free_slots,
    )


def apply_completed_move(
    state: WorkspaceState,
    move: ShuffleMove,
    calib: Calibration,
) -> WorkspaceState:
    """Fold a successful pick+place into the workspace (vision often lags here)."""
    cubes = [
        c
        for c in state.cubes
        if dist_mm(c.x, c.y, move.pick_x, move.pick_y) >= CUBE_CLEARANCE_MM
    ]

    place_x = move.place_x
    place_y = move.place_y
    if move.place_marker_id is not None:
        marker = next(m for m in state.markers if m.marker_id == move.place_marker_id)
        place_x = marker.x
        place_y = marker.y
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
    return rebuild_workspace_state(calib, state.markers, cubes)


def cubes_of_color(cubes: list[CubeDetection], color: str) -> list[CubeDetection]:
    return [c for c in cubes if c.color == color]


def pick_largest_cube(cubes: list[CubeDetection]) -> CubeDetection | None:
    if not cubes:
        return None
    return max(cubes, key=lambda c: c.area)


def plan_shuffle_move(state: WorkspaceState) -> ShuffleMove | None:
    """Pick a random cube and an empty marker, or relocate off a full marker."""
    pickable = mp_reachable_cubes(state.cubes)
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
    reachable_occupied = [
        (marker, cube)
        for marker, cube in state.occupied
        if is_mp_reachable_xy(cube.x, cube.y)
    ]
    if reachable_occupied and state.free_slots:
        marker, cube = random.choice(reachable_occupied)
        sx, sy = random.choice(state.free_slots)
        return ShuffleMove(cube.x, cube.y, cube.color, sx, sy, "to_slot")
    return None
