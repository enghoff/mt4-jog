"""Pure-logic tests for unstack_cubes helpers (no hardware)."""

import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration
from mt4_vision.workspace import (
    MARKER_PAPER_CLEARANCE_MM,
    dist_mm,
    is_mp_reachable_xy,
    marker_slots_from_calibration,
)
from unstack_cubes import (
    SCATTER_MAX_RADIUS_MM,
    SCATTER_MIN_RADIUS_MM,
    SITE_AVOID_MM,
    find_landing,
    random_landing,
    random_place_j4,
)


def _calib_and_markers():
    calib = load_calibration(DEFAULT_CALIB_PATH)
    markers = marker_slots_from_calibration(calib)
    site = next(m for m in markers if m.marker_id == 4)
    return calib, markers, site


def test_random_landing_respects_reach_and_spacing():
    _calib, markers, site = _calib_and_markers()
    avoid = [(200.0, -60.0), (150.0, 100.0)]
    rng = random.Random(1)
    xy = random_landing(
        rng, sx=site.x, sy=site.y, markers=markers, avoid=avoid, spacing_mm=75.0,
    )
    assert xy is not None
    x, y = xy
    r = math.hypot(x, y)
    assert SCATTER_MIN_RADIUS_MM <= r <= SCATTER_MAX_RADIUS_MM
    assert is_mp_reachable_xy(x, y)
    assert dist_mm(x, y, site.x, site.y) >= SITE_AVOID_MM
    for ox, oy in avoid:
        assert dist_mm(x, y, ox, oy) >= 75.0
    for m in markers:
        assert dist_mm(x, y, m.x, m.y) >= MARKER_PAPER_CLEARANCE_MM


def test_random_landing_avoids_marker_papers_directly_on_a_marker():
    """A dense avoid ring right around one non-site marker must still steer
    the draw off that marker's own paper (a fixed check, not just spacing
    from prior drops)."""
    _calib, markers, site = _calib_and_markers()
    other = next(m for m in markers if m.marker_id != site.marker_id)
    rng = random.Random(2)
    for _ in range(20):
        xy = random_landing(
            rng, sx=site.x, sy=site.y, markers=markers, avoid=[], spacing_mm=75.0,
        )
        assert xy is not None
        x, y = xy
        assert dist_mm(x, y, other.x, other.y) >= MARKER_PAPER_CLEARANCE_MM


def test_find_landing_degrades_spacing_when_crowded():
    """When the preferred spacing can't be satisfied, find_landing must fall
    back to a tighter one rather than raising -- exercised here by pinning
    random_landing to only succeed at the tightest fallback."""
    import unstack_cubes as uc

    calls = []

    def fake_random_landing(rng, *, sx, sy, markers, avoid, spacing_mm, attempts=0):
        calls.append(spacing_mm)
        if spacing_mm == uc.DROP_SPACING_FALLBACKS_MM[-1]:
            return (250.0, 0.0)
        return None

    orig = uc.random_landing
    uc.random_landing = fake_random_landing
    try:
        landing, spacing = find_landing(
            random.Random(3), sx=0.0, sy=0.0, markers=[], avoid=[],
        )
    finally:
        uc.random_landing = orig
    assert landing == (250.0, 0.0)
    assert spacing == uc.DROP_SPACING_FALLBACKS_MM[-1]
    assert calls == list(uc.DROP_SPACING_FALLBACKS_MM)


def test_find_landing_raises_when_desk_has_no_room():
    import unstack_cubes as uc
    from mt4_jog.client import Mt4ClientError

    orig = uc.random_landing
    uc.random_landing = lambda *a, **k: None
    try:
        try:
            find_landing(random.Random(4), sx=0.0, sy=0.0, markers=[], avoid=[])
            assert False, "expected Mt4ClientError"
        except Mt4ClientError:
            pass
    finally:
        uc.random_landing = orig


def test_random_place_j4_delegates_to_face_align_with_the_drawn_angle():
    from mt4_vision.pickplace import j4_for_face_align

    x, y = 240.0, -150.0
    rng1 = random.Random(42)
    rng2 = random.Random(42)
    expected_angle = rng2.uniform(0.0, 360.0)
    expected = j4_for_face_align(expected_angle, current_j4_deg=None, x=x, y=y)
    assert random_place_j4(x, y, rng1) == expected


def test_pick_grip_height_matches_stack_release_line():
    """unstack_cubes.pick_from_stack grips level N at grip_top_z(N - 1);
    stack_cubes released it 4mm above that same line. If either script's
    height formula drifts, the two must disagree here first."""
    from mt4_vision.stackpath import StackPlanner
    from stack_cubes import release_z_for_level

    calib, _markers, site = _calib_and_markers()
    planner = StackPlanner(calib, site.x, site.y)
    for level in (1, 2, 3):
        grip_z = planner.grip_top_z(level - 1)
        assert abs(grip_z - (release_z_for_level(calib, level) - 4.0)) < 1e-9
