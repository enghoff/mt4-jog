"""Pure-logic tests for stack_cubes helpers (no hardware)."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from stack_cubes import ParallaxHeightModel, color_sequence


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
