"""Shuffle planner over a detection-as-state Scene.

Priority:
  1. Pick a blocker (open-table cube) → place on a free marker.
  2. If no free markers: pick a marker cube → place on a free table slot.

Every pick carries its place destination. Planning uses only the latest clear
frame -- no persistent tracks.
"""

from __future__ import annotations

from dataclasses import dataclass

from mt4_vision.detect import CubeDetection
from mt4_vision.pickplace import near_camera_park
from mt4_vision.scene import Scene


@dataclass(frozen=True)
class Action:
    kind: str  # pick | wait
    reason: str
    cube: CubeDetection | None = None
    place_x: float | None = None
    place_y: float | None = None
    place_marker_id: int | None = None
    place_kind: str | None = None  # "to_marker" | "to_slot"


def plan_shuffle(scene: Scene) -> Action:
    """Choose the next shuffle action from this frame's detections."""
    free_markers = scene.placeable_markers()
    blockers = scene.pickable(scene.blockers())

    if free_markers and blockers:
        cube = blockers[0]
        marker = free_markers[0]
        return Action(
            "pick",
            f"to_marker: pick {cube.color} ({cube.x:.0f},{cube.y:.0f}) "
            f"-> marker {marker.marker_id}",
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
            f"but no pickable blocker "
            f"(blockers={len(scene.blockers())} cubes={len(scene.cubes)})",
        )

    occupied_cubes = [c for _m, c in scene.occupied]
    pickable_on_markers = scene.pickable(occupied_cubes)
    slots = [
        (sx, sy) for sx, sy in scene.free_slots if not near_camera_park(sx, sy)
    ]
    if pickable_on_markers and slots:
        cube = pickable_on_markers[0]
        marker = next(m for m, c in scene.occupied if c is cube)
        sx, sy = slots[0]
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
