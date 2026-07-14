"""Unit tests for detection-as-state scene + shuffle planner (no hardware).

Run: python tests/test_scene.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_vision.detect import CubeDetection
from mt4_vision.policy import plan_shuffle
from mt4_vision.scene import Scene, verify_pick_place
from mt4_vision.workspace import MarkerSlot, rebuild_workspace_state


MARKERS = [
    MarkerSlot(0, 52.0, -258.6),
    MarkerSlot(1, 39.0, 318.9),
    MarkerSlot(2, 188.4, -161.3),
    MarkerSlot(3, 177.2, 181.5),
]


def cube(color: str, x: float, y: float, area: float = 450.0) -> CubeDetection:
    return CubeDetection(color=color, px=0.0, py=0.0, area=area, x=x, y=y)


def scene(
    cubes: list[CubeDetection],
    visible: set[int] | None = None,
) -> Scene:
    if visible is None:
        visible = {m.marker_id for m in MARKERS}
    state = rebuild_workspace_state(
        None, MARKERS, cubes, visible_marker_ids=visible
    )
    return Scene.from_workspace(state)


def test_blocker_to_free_marker():
    s = scene([cube("red", 240.0, 0.0)], visible={0, 1, 2, 3})
    action = plan_shuffle(s)
    assert action.kind == "pick"
    assert action.place_kind == "to_marker"
    assert action.place_marker_id is not None
    assert action.cube is not None
    assert action.cube.color == "red"


def test_cube_near_marker_blocks_place_even_if_tag_decodes():
    # 35mm from marker 3: outside occupy radius, inside place clearance.
    s = scene([cube("red", 177.2, 181.5 + 35.0)], visible={0, 1, 2, 3})
    assert 3 not in {m.marker_id for m in s.placeable_markers()}
    action = plan_shuffle(s)
    assert action.place_marker_id != 3


def test_occupied_marker_not_placeable():
    s = scene(
        [cube("green", 177.2, 181.5)],
        visible={0, 1, 2},  # marker 3 tag hidden
    )
    assert 3 not in {m.marker_id for m in s.placeable_markers()}
    assert any(m.marker_id == 3 for m, _ in s.occupied)


def test_prefer_blocker_over_marker_cube():
    s = scene(
        [cube("red", 240.0, 0.0), cube("blue", 177.2, 181.5)],
        visible={0, 1, 2},  # 3 occupied; others free
    )
    action = plan_shuffle(s)
    assert action.kind == "pick"
    assert action.place_kind == "to_marker"
    assert action.cube is not None
    assert action.cube.color == "red"


def test_marker_to_slot_when_full():
    cubes = [
        cube("red", 52.0, -258.6),
        cube("blue", 188.4, -161.3),
        cube("green", 177.2, 181.5),
        cube("yellow", 39.0, 300.0),
    ]
    s = scene(cubes, visible=set())  # no tags => occupied / unknown, not free
    action = plan_shuffle(s)
    assert action.kind == "pick", action
    assert action.place_kind == "to_slot", action
    assert action.place_marker_id is None


def test_idle_when_free_marker_but_no_blocker():
    s = scene(
        [cube("green", 177.2, 181.5)],
        visible={0, 1, 2},
    )
    action = plan_shuffle(s)
    assert action.kind == "wait"


def test_vacated_pose_not_planned_after_fresh_scene():
    """Detection-as-state: a gone cube simply isn't in the next scene."""
    before = scene([cube("red", 240.0, 0.0)], visible={0, 1, 2, 3})
    assert plan_shuffle(before).kind == "pick"
    after = scene([], visible={0, 1, 2, 3})
    assert plan_shuffle(after).kind == "wait"
    assert after.cubes == []


def test_outside_hull_blob_filtered_as_phantom():
    from mt4_vision.scene import filter_phantoms, is_phantom_detection

    # Far outside marker hull, oversize area -- the live "red (272,-188)" class.
    phantom = cube("red", 272.0, -188.0, area=733.0)
    real = cube("green", 177.2, 181.5, area=400.0)
    assert is_phantom_detection(phantom, MARKERS)
    assert not is_phantom_detection(real, MARKERS)
    kept = filter_phantoms([phantom, real], MARKERS)
    assert [c.color for c in kept] == ["green"]


def test_keepout_and_park_blobs_filtered():
    from mt4_vision.scene import is_phantom_detection

    assert is_phantom_detection(cube("blue", -19.0, 161.0, area=400.0), MARKERS)
    assert is_phantom_detection(cube("blue", 200.0, 0.0, area=400.0), MARKERS)


def test_verify_pick_place_outcomes():
    placed = scene([cube("blue", 177.0, 181.0)], visible={0, 1, 2})
    assert (
        verify_pick_place(
            placed,
            pick_x=240.0,
            pick_y=0.0,
            pick_color="blue",
            place_x=177.2,
            place_y=181.5,
        )
        == "placed"
    )
    failed = scene([cube("blue", 241.0, 1.0)], visible={0, 1, 2, 3})
    assert (
        verify_pick_place(
            failed,
            pick_x=240.0,
            pick_y=0.0,
            pick_color="blue",
            place_x=177.2,
            place_y=181.5,
        )
        == "grasp_failed"
    )
    lost = scene([cube("blue", 100.0, 100.0)], visible={0, 1, 2, 3})
    assert (
        verify_pick_place(
            lost,
            pick_x=240.0,
            pick_y=0.0,
            pick_color="blue",
            place_x=177.2,
            place_y=181.5,
        )
        == "lost"
    )


def run() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"ok  {fn.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL {fn.__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}: {exc}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(run())
