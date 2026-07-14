"""Shuffle planner over a detection-as-state Scene.

Priority:
  1. Pick a blocker (open-table cube) → place on a free marker.
  2. Else pick a marker cube → place on a free marker (fill empty slots).
  3. Else if no free markers: pick a marker cube → place on a free table slot.

Within whichever tier applies, the cube/marker/slot is chosen *randomly*
among the valid candidates -- picking the first one every time (by area,
by marker id, by a fixed slot-list order) meant the same few cubes/markers
got shuffled over and over while others sat untouched.

Planning uses only the latest clear frame -- no synthetic cubes. Ghost/arm
blobs must be rejected in detection filtering, not papered over after they
become pick targets. The one deliberate bit of cross-cycle memory is
``avoid_xy`` (see plan_shuffle): a soft preference against repeating the
same cube twice in a row, not a hard stigma -- it always falls back to that
cube if nothing else is pickable.

``exclude_cube``/``exclude_marker_id``/``exclude_slot`` let a caller ask
"is there a *second*, independent move already visible in this same
frame?" -- shuffle.py uses this to chain two moves off one capture when the
first move's target doesn't interact with the second, skipping the
capture+settle pause in between.
"""

from __future__ import annotations

import random
from dataclasses import dataclass

from mt4_vision.detect import CubeDetection
from mt4_vision.pickplace import near_camera_park
from mt4_vision.scene import Scene
from mt4_vision.workspace import dist_mm

# A cube that just got placed is recognized next cycle by its current
# position matching where the last move dropped it. Skip re-picking it
# immediately -- unless it is the only pickable option, in which case
# repeating the same move is still better than sitting idle.
AVOID_LAST_RADIUS_MM = 40.0


@dataclass(frozen=True)
class Action:
    kind: str  # pick | wait
    reason: str
    cube: CubeDetection | None = None
    place_x: float | None = None
    place_y: float | None = None
    place_marker_id: int | None = None
    place_kind: str | None = None  # "to_marker" | "to_slot"


def _deprioritize_last(
    cubes: list[CubeDetection], avoid_xy: tuple[float, float] | None
) -> list[CubeDetection]:
    """Drop cubes sitting at ``avoid_xy`` (the last move's destination) --
    unless doing so would leave nothing pickable at all."""
    if avoid_xy is None:
        return cubes
    ax, ay = avoid_xy
    others = [
        c
        for c in cubes
        if dist_mm(float(c.x), float(c.y), ax, ay) >= AVOID_LAST_RADIUS_MM
    ]
    return others if others else cubes


def plan_shuffle(
    scene: Scene,
    *,
    avoid_xy: tuple[float, float] | None = None,
    exclude_cube: CubeDetection | None = None,
    exclude_marker_id: int | None = None,
    exclude_slot: tuple[float, float] | None = None,
) -> Action:
    """Choose the next shuffle action from this frame's detections.

    ``avoid_xy`` is the destination of the last completed move -- the cube
    now sitting there is deprioritized (not excluded outright) so the
    planner doesn't move the same cube twice in a row when another pickable
    cube is available.

    ``exclude_cube``/``exclude_marker_id``/``exclude_slot`` hard-exclude a
    cube/marker/slot already claimed by another action planned from this
    same scene -- used to look up a second, independent move without a new
    capture in between (see shuffle.py's lookahead).
    """
    free_markers = [
        m for m in scene.placeable_markers() if m.marker_id != exclude_marker_id
    ]
    blockers = _deprioritize_last(scene.pickable(scene.blockers()), avoid_xy)
    blockers = [c for c in blockers if c is not exclude_cube]
    occupied_cubes = _deprioritize_last(
        scene.pickable(scene.occupied_pick_cubes()), avoid_xy
    )
    occupied_cubes = [c for c in occupied_cubes if c is not exclude_cube]
    slots = [
        (sx, sy)
        for sx, sy in scene.free_slots
        if not near_camera_park(sx, sy) and (sx, sy) != exclude_slot
    ]

    if free_markers and blockers:
        cube = random.choice(blockers)
        marker = random.choice(free_markers)
        return Action(
            "pick",
            f"to_marker: pick blocker {cube.color} ({cube.x:.0f},{cube.y:.0f}) "
            f"-> marker {marker.marker_id}",
            cube=cube,
            place_x=marker.x,
            place_y=marker.y,
            place_marker_id=marker.marker_id,
            place_kind="to_marker",
        )

    if free_markers and occupied_cubes:
        cube = random.choice(occupied_cubes)
        src = next(m for m, c in scene.occupied if c is cube)
        marker = random.choice(free_markers)
        return Action(
            "pick",
            f"to_marker: pick {cube.color} from marker {src.marker_id} "
            f"({cube.x:.0f},{cube.y:.0f}) -> marker {marker.marker_id}",
            cube=cube,
            place_x=marker.x,
            place_y=marker.y,
            place_marker_id=marker.marker_id,
            place_kind="to_marker",
        )

    if free_markers:
        return Action(
            "wait",
            f"free markers {sorted(m.marker_id for m in free_markers)} "
            f"but no pickable cube "
            f"(blockers={len(scene.blockers())} occupied={len(scene.occupied)} "
            f"cubes={len(scene.cubes)})",
        )

    if occupied_cubes and slots:
        cube = random.choice(occupied_cubes)
        marker = next(m for m, c in scene.occupied if c is cube)
        sx, sy = random.choice(slots)
        return Action(
            "pick",
            f"to_slot: pick {cube.color} from marker {marker.marker_id} "
            f"({cube.x:.0f},{cube.y:.0f}) -> ({sx:.0f},{sy:.0f})",
            cube=cube,
            place_x=sx,
            place_y=sy,
            place_marker_id=None,
            place_kind="to_slot",
        )

    return Action("wait", f"no shuffle move ({scene.summary_line()})")
