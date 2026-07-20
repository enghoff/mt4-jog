"""Pure-logic tests for stack_cubes helpers (no hardware)."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from stack_cubes import (
    ParallaxHeightModel,
    classify_level,
    color_sequence,
    park_spot_for_clear,
    site_occupant_color,
)


def test_color_sequence_alternates():
    seq = color_sequence({"red": 3, "green": 3, "blue": 2}, 8)
    assert len(seq) == 8
    assert all(a != b for a, b in zip(seq, seq[1:]))
    assert sorted(seq.count(c) for c in ("red", "green", "blue")) == [2, 3, 3]


def test_color_sequence_stops_when_alternation_impossible():
    assert color_sequence({"red": 3}, 5) == ["red"]


def test_parallax_height_model_round_trip():
    m = ParallaxHeightModel((0.0, 0.0), (10.0, 0.0), 20.0)
    s40 = m.s_of_h(40.0)
    assert s40 > 2 * m.s_of_h(20.0) * 0.98  # superlinear growth
    assert abs(m.h_of_s(s40) - 40.0) < 1e-6


def test_parallax_model_fits_camera_height():
    truth = ParallaxHeightModel((0.0, 0.0), (10.0, 0.0), 20.0)
    truth.hc = 800.0
    m = ParallaxHeightModel((0.0, 0.0), (10.0, 0.0), 20.0)
    p1 = truth.predict_px(20.0)
    m.set_anchor(p1[0], p1[1], 20.0)
    for h in (40.0, 60.0, 80.0):
        m.add_observation(h, truth.s_of_h(h) - truth.s_of_h(20.0))
    assert abs(m.hc - 800.0) < 15.0


def test_parallax_components():
    m = ParallaxHeightModel((100.0, 100.0), (110.0, 100.0), 20.0)
    along, perp = m.components(120.0, 103.0)
    assert abs(along - 20.0) < 1e-9
    assert abs(perp - 3.0) < 1e-9


def test_anchor_cancels_xy_error():
    """An XY placement/map offset shifts all levels' pixels equally; the
    anchored relative measurement must still read correct heights."""
    truth = ParallaxHeightModel((0.0, 0.0), (10.0, 0.0), 20.0)
    truth.hc = 700.0
    offset = (7.0, -3.0)  # site XY error in pixels, same for every level
    m = ParallaxHeightModel((0.0, 0.0), (10.0, 0.0), 20.0)
    p1 = truth.predict_px(20.0)
    m.set_anchor(p1[0] + offset[0], p1[1] + offset[1], 20.0)
    for level in (2, 3, 4):
        p = truth.predict_px(level * 20.0)
        along, perp = m.rel_components(p[0] + offset[0], p[1] + offset[1])
        assert abs(m.h_from_rel(along) - level * 20.0) < 3.0
        assert abs(perp) < 1e-9


def test_ground_offset_scales_with_height():
    import numpy as np

    from stack_cubes import ground_offset_mm

    jac = np.array([[1.0, 0.0], [0.0, 1.0]])  # 1mm/px, axis-aligned
    ox, oy = ground_offset_mm((10.0, 0.0), jac, 700.0, 0.0)
    assert abs(ox - 10.0) < 1e-9
    ox, oy = ground_offset_mm((10.0, 0.0), jac, 700.0, 140.0)
    assert abs(ox - 8.0) < 1e-9  # faces higher up move more px per mm
    ox, oy = ground_offset_mm((10.0, 0.0), jac, 700.0, 0.0, cap_mm=4.0)
    assert abs(ox - 4.0) < 1e-9


def test_classify_xy_coupling_is_never_a_table_miss():
    """Field: stacked L2 at dXY=(-16.5,+1) read h=67 -- coupling, not miss.
    Height overshoot must never table-pick; moderate drift is seated,
    drift beyond OFF_COLUMN_MM is an honest 'misplaced' (a 20mm cube
    offset >14mm cannot carry further levels)."""
    assert classify_level(15.0, 11.2, 20.0) == "seated"
    assert classify_level(27.0, 16.5, 20.0) == "misplaced"
    assert classify_level(68.0, 34.0, 20.0) == "misplaced"


def test_classify_perched_only_when_on_column():
    # Height trusted only when drift <= DRIFT_OK (6mm).
    assert classify_level(-12.0, 5.0, 20.0) == "perched"
    assert classify_level(-12.0, 10.0, 20.0) == "seated"  # off-column: ignore h
    assert classify_level(+12.0, 5.0, 20.0) == "seated"   # high never perched


def test_classify_abort_on_huge_drift():
    assert classify_level(0.0, 50.0, 20.0) == "abort"


def test_park_spot_prefers_far_marker():
    from types import SimpleNamespace

    near = SimpleNamespace(x=210.0, y=70.0, marker_id=1)
    far = SimpleNamespace(x=280.0, y=0.0, marker_id=2)
    scene = SimpleNamespace(
        placeable_markers=lambda: [near, far],
        free_slots=[],
    )
    spot = park_spot_for_clear(scene, 200.0, 60.0)
    assert spot is not None
    assert spot[2] == "marker 2"


def test_park_spot_rejects_shadow_zone_only():
    from types import SimpleNamespace

    shadow = SimpleNamespace(x=200.0, y=-60.0, marker_id=9)
    scene = SimpleNamespace(
        placeable_markers=lambda: [shadow],
        free_slots=[(200.0, -55.0)],
    )
    assert park_spot_for_clear(scene, 200.0, 60.0) is None


def test_held_cube_blob_gates_on_color_and_pixel():
    """The gripped cube at the capture pose is dropped only when the blob
    matches the held color near the predicted pixel (hardware-measured
    2026-07-19: held red read area 622 at 54px from prediction -- area
    does NOT separate it from table cubes, the gripper occludes the top
    face). A different-color cube at the same pixel and a same-color cube
    far away must both survive."""
    from types import SimpleNamespace

    from mt4_vision.scene import is_held_cube_blob

    held_px = (686.0, 339.0)
    phantom = SimpleNamespace(color="red", px=639.0, py=308.0, area=622.0)
    other_color = SimpleNamespace(color="blue", px=639.0, py=308.0, area=622.0)
    same_color_far = SimpleNamespace(color="red", px=400.0, py=440.0, area=520.0)
    assert is_held_cube_blob(phantom, held_px, "red")
    assert not is_held_cube_blob(other_color, held_px, "red")
    assert not is_held_cube_blob(same_color_far, held_px, "red")
