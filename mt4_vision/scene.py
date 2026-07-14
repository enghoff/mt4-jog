"""Detection-as-state desk snapshot for the shuffle planner.

Each clear camera frame builds a fresh ``Scene`` from cube detections and
ArUco visibility. No persistent tracks -- vacated poses cannot linger.

Detections outside the calibrated marker hull (plus a small margin), near the
camera-park pose, or outside the typical cube-top area band are treated as
phantoms and dropped before planning -- arm paint, desk clutter, and FOV
noise otherwise become pick targets when sorted largest-first.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from mt4_vision.calib import Calibration
from mt4_vision.detect import CubeDetection, detect_cubes, detect_markers
from mt4_vision.pickplace import near_camera_park
from mt4_vision.workspace import (
    MARKER_DICT,
    MARKER_OCCUPY_RADIUS_MM,
    MAX_REACH_MM,
    PICK_CLEARANCE_MM,
    MarkerSlot,
    WorkspaceState,
    cubes_with_robot_coords,
    dist_mm,
    is_mp_reachable_xy,
    marker_slots_from_calibration,
    rebuild_workspace_state,
)

# Real cube top faces under this camera land ~200-650px^2 (see detect.py).
# Looser detect bounds still let phantoms into raw detections; planning uses
# this tighter band.
PICK_MIN_AREA = 200.0
PICK_MAX_AREA = 650.0
# Allow cubes a bit outside the marker convex hull (markers aren't at the
# desk edge). Measured phantoms run 60-90mm outside; real near-pad cubes
# sit within ~15mm.
HULL_OUTSIDE_MARGIN_MM = 40.0


@dataclass(frozen=True)
class Scene:
    """One clear-frame world: only what the camera sees right now."""

    cubes: list[CubeDetection]
    markers: list[MarkerSlot]
    occupied: list[tuple[MarkerSlot, CubeDetection]]
    free_markers: list[MarkerSlot]
    unknown_markers: list[MarkerSlot]
    free_slots: list[tuple[float, float]]
    visible_marker_ids: set[int]
    # Raw detections before phantom filtering (debug / logging).
    raw_cubes: list[CubeDetection] | None = None

    @classmethod
    def from_workspace(
        cls,
        state: WorkspaceState,
        *,
        raw_cubes: list[CubeDetection] | None = None,
    ) -> Scene:
        visible = state.visible_marker_ids or set()
        return cls(
            cubes=list(state.cubes),
            markers=list(state.markers),
            occupied=list(state.occupied),
            free_markers=list(state.free_markers),
            unknown_markers=list(state.unknown_markers),
            free_slots=list(state.free_slots),
            visible_marker_ids=set(visible),
            raw_cubes=list(raw_cubes) if raw_cubes is not None else None,
        )

    def summary_line(self) -> str:
        raw_n = len(self.raw_cubes) if self.raw_cubes is not None else len(self.cubes)
        dropped = max(0, raw_n - len(self.cubes))
        extra = f" phantoms_dropped={dropped}" if dropped else ""
        return (
            f"cubes={len(self.cubes)} blockers={len(self.blockers())} "
            f"free_markers={len(self.placeable_markers())} "
            f"occupied={len(self.occupied)} "
            f"unknown={len(self.unknown_markers)} "
            f"free_slots={len(self.free_slots)}{extra}"
        )

    def cube_lines(self) -> list[str]:
        lines = []
        for c in sorted(self.cubes, key=lambda x: (-(x.area or 0), x.color)):
            on = self.marker_for_cube(c)
            tag = f" marker {on}" if on is not None else " open"
            lines.append(
                f"  {c.color} ({c.x:.0f},{c.y:.0f}) area={c.area:.0f}{tag}"
            )
        return lines

    def marker_for_cube(self, cube: CubeDetection) -> int | None:
        best_id: int | None = None
        best_d = MARKER_OCCUPY_RADIUS_MM
        for m in self.markers:
            d = dist_mm(float(cube.x), float(cube.y), m.x, m.y)
            if d < best_d:
                best_d = d
                best_id = m.marker_id
        return best_id

    def blockers(self) -> list[CubeDetection]:
        """Cubes not sitting on any calibrated marker."""
        return [c for c in self.cubes if self.marker_for_cube(c) is None]

    def placeable_markers(self) -> list[MarkerSlot]:
        """Tag visible, place-clearance free, reachable, not near camera park."""
        return [
            m
            for m in self.free_markers
            if is_mp_reachable_xy(m.x, m.y) and not near_camera_park(m.x, m.y)
        ]

    def pickable(self, cubes: list[CubeDetection]) -> list[CubeDetection]:
        """Reachable cubes with finger clearance from every other detection."""
        out: list[CubeDetection] = []
        for c in cubes:
            if not is_mp_reachable_xy(float(c.x), float(c.y)):
                continue
            if math.hypot(float(c.x), float(c.y)) > MAX_REACH_MM:
                continue
            if any(
                dist_mm(float(c.x), float(c.y), float(o.x), float(o.y))
                < PICK_CLEARANCE_MM
                for o in self.cubes
                if o is not c
            ):
                continue
            out.append(c)
        # Prefer mid-size blobs closer to the workspace center -- largest
        # first kept promoting oversize outside-hull phantoms.
        cx = sum(m.x for m in self.markers) / max(len(self.markers), 1)
        cy = sum(m.y for m in self.markers) / max(len(self.markers), 1)
        out.sort(
            key=lambda c: (
                abs(c.area - 400.0),
                dist_mm(float(c.x), float(c.y), cx, cy),
            )
        )
        return out


def _marker_hull_robot(markers: list[MarkerSlot]) -> np.ndarray | None:
    if len(markers) < 3:
        return None
    pts = np.array([[m.x, m.y] for m in markers], dtype=np.float32)
    return cv2.convexHull(pts)


def is_phantom_detection(
    cube: CubeDetection,
    markers: list[MarkerSlot],
    *,
    hull: np.ndarray | None = None,
) -> bool:
    """True when a blob should not be treated as a real desk cube."""
    if cube.x is None or cube.y is None:
        return True
    if cube.area < PICK_MIN_AREA or cube.area > PICK_MAX_AREA:
        return True
    if near_camera_park(float(cube.x), float(cube.y)):
        return True
    if not is_mp_reachable_xy(float(cube.x), float(cube.y)):
        return True
    if math.hypot(float(cube.x), float(cube.y)) > MAX_REACH_MM:
        return True
    hull = hull if hull is not None else _marker_hull_robot(markers)
    if hull is not None:
        # OpenCV: positive = inside. Reject well outside the marker hull.
        inside = cv2.pointPolygonTest(
            hull, (float(cube.x), float(cube.y)), True
        )
        if inside < -HULL_OUTSIDE_MARGIN_MM:
            return True
    return False


def filter_phantoms(
    cubes: list[CubeDetection], markers: list[MarkerSlot]
) -> list[CubeDetection]:
    hull = _marker_hull_robot(markers)
    return [c for c in cubes if not is_phantom_detection(c, markers, hull=hull)]


def capture_scene(calib: Calibration, frame: np.ndarray) -> Scene:
    markers = marker_slots_from_calibration(calib)
    raw = cubes_with_robot_coords(detect_cubes(frame, calib))
    cubes = filter_phantoms(raw, markers)
    visible = {m.marker_id for m in detect_markers(frame, MARKER_DICT)}
    state = rebuild_workspace_state(
        calib, markers, cubes, visible_marker_ids=visible
    )
    return Scene.from_workspace(state, raw_cubes=raw)


# Post-move verification radii (mm).
VERIFY_PLACED_RADIUS_MM = 35.0
VERIFY_ORIGIN_RADIUS_MM = 30.0


def verify_pick_place(
    scene: Scene,
    *,
    pick_x: float,
    pick_y: float,
    pick_color: str,
    place_x: float,
    place_y: float,
) -> str:
    """Judge an atomic pick+place from the post-move scene.

    Returns ``placed``, ``grasp_failed``, or ``lost``.
    """
    same = [c for c in scene.cubes if c.color == pick_color]
    if any(
        dist_mm(float(c.x), float(c.y), place_x, place_y) < VERIFY_PLACED_RADIUS_MM
        for c in same
    ):
        return "placed"
    if any(
        dist_mm(float(c.x), float(c.y), pick_x, pick_y) < VERIFY_ORIGIN_RADIUS_MM
        for c in same
    ):
        return "grasp_failed"
    return "lost"
