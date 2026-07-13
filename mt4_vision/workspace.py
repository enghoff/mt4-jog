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


def dist_mm(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


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


def cubes_of_color(cubes: list[CubeDetection], color: str) -> list[CubeDetection]:
    return [c for c in cubes if c.color == color]


def pick_largest_cube(cubes: list[CubeDetection]) -> CubeDetection | None:
    if not cubes:
        return None
    return max(cubes, key=lambda c: c.area)


def plan_shuffle_move(state: WorkspaceState) -> ShuffleMove | None:
    """Pick a random cube and an empty marker, or relocate off a full marker."""
    if state.free_markers and state.cubes:
        cube = random.choice(state.cubes)
        marker = random.choice(state.free_markers)
        return ShuffleMove(
            cube.x, cube.y, cube.color, marker.x, marker.y, "to_marker"
        )
    if state.occupied and state.free_slots:
        marker, cube = random.choice(state.occupied)
        sx, sy = random.choice(state.free_slots)
        return ShuffleMove(cube.x, cube.y, cube.color, sx, sy, "to_slot")
    return None
