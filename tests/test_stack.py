"""Pure-logic tests for stack_cubes helpers (no hardware)."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from stack_cubes import (
    CLEAR_PARK_MM,
    SITE_CLEAR_MM,
    choose_park_slot,
    clear_aside_xy,
    cubes_near_site,
    in_stack_camera_shadow,
    release_z_for_level,
    stack_candidates,
    stack_shadow_behind_unit,
    travel_z_for_level,
)


def test_cubes_near_site_filters_by_radius():
    near = SimpleNamespace(x=200.0, y=60.0, color="red")
    far = SimpleNamespace(x=280.0, y=0.0, color="blue")
    scene = SimpleNamespace(raw_cubes=[near, far])
    found = cubes_near_site(scene, 200.0, 55.0, radius_mm=70.0)
    assert found == [near]


def test_clear_aside_pushes_past_keep_clear():
    # Marker 4-ish; cube slightly off-center toward +y.
    dest = clear_aside_xy(243.0, 5.0, 251.0, 27.0, occupied=[])
    assert dest is not None
    assert math.hypot(dest[0] - 243.0, dest[1] - 5.0) >= CLEAR_PARK_MM - 0.1
    # Must not land in the barely-outside free-slot ring that re-triggered clear.
    assert math.hypot(dest[0] - 243.0, dest[1] - 5.0) > SITE_CLEAR_MM + 20.0


def test_clear_aside_avoids_occupied():
    sx, sy = 243.0, 5.0
    cx, cy = 251.0, 27.0
    primary = clear_aside_xy(sx, sy, cx, cy, occupied=[])
    assert primary is not None
    alt = clear_aside_xy(sx, sy, cx, cy, occupied=[primary])
    assert alt is not None
    assert math.hypot(alt[0] - primary[0], alt[1] - primary[1]) >= 40.0


def test_clear_aside_stays_in_pick_hull():
    from mt4_vision.workspace import MarkerSlot

    # Tight triangle around the site; a long push along +y exits the hull.
    markers = [
        MarkerSlot(1, 200.0, 0.0),
        MarkerSlot(2, 280.0, 0.0),
        MarkerSlot(3, 240.0, 80.0),
    ]
    dest = clear_aside_xy(
        240.0, 20.0, 240.0, 40.0, occupied=[], markers=markers,
    )
    assert dest is not None
    from mt4_vision.scene import within_pick_hull

    assert within_pick_hull(dest[0], dest[1], markers)


def test_clear_aside_skips_stack_shadow_corridor():
    from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration

    calib = load_calibration(DEFAULT_CALIB_PATH)
    sx, sy = 178.7, 179.8
    behind = stack_shadow_behind_unit(calib, sx, sy)
    assert behind is not None
    # Cube already in the behind corridor; landing must not stay there.
    cx = sx + behind[0] * 40.0
    cy = sy + behind[1] * 40.0
    dest = clear_aside_xy(
        sx, sy, cx, cy, occupied=[], behind_u=behind, shadow_levels=8,
    )
    assert dest is not None
    assert not in_stack_camera_shadow(
        dest[0], dest[1], sx, sy, behind, stack_levels=8,
    )


def test_choose_park_slot_requires_clear_margin():
    # (200, 60) is ~70mm from marker 4 -- inside CLEAR_PARK_MM, must reject.
    scene = SimpleNamespace(
        free_slots=[(200.0, 60.0), (200.0, -60.0), (150.0, -250.0)],
    )
    spot = choose_park_slot(scene, 243.0, 5.0)
    assert spot == (150.0, -250.0)


def test_stack_candidates_exclude_site_and_use_pickable():
    near = SimpleNamespace(x=205.0, y=60.0, color="red", yaw_deg=0.0)
    far = SimpleNamespace(x=280.0, y=0.0, color="green", yaw_deg=10.0)
    scene = SimpleNamespace(
        cubes=[near, far],
        pickable=lambda cubes: [far],
    )
    assert stack_candidates(scene, 200.0, 60.0) == [far]


def test_stack_shadow_rejects_marker3_phantom():
    """Field case 2026-07-21: stack (179,180) → phantom ~(115,227)."""
    from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration

    calib = load_calibration(DEFAULT_CALIB_PATH)
    sx, sy = 178.7, 179.8
    behind = stack_shadow_behind_unit(calib, sx, sy)
    assert behind is not None
    assert in_stack_camera_shadow(
        115.0, 227.0, sx, sy, behind, stack_levels=4,
    )
    # A cube off to the side of the corridor must still be pickable.
    assert not in_stack_camera_shadow(
        280.0, 0.0, sx, sy, behind, stack_levels=4,
    )
    phantom = SimpleNamespace(x=115.0, y=227.0, color="green", yaw_deg=0.0)
    real = SimpleNamespace(x=250.0, y=96.0, color="red", yaw_deg=0.0)
    scene = SimpleNamespace(
        cubes=[phantom, real],
        pickable=lambda cubes: list(cubes),
    )
    cands = stack_candidates(
        scene, sx, sy, calib=calib, stack_levels=4,
    )
    assert phantom not in cands
    assert real in cands


def test_release_and_travel_z_step_by_cube_height():
    calib = SimpleNamespace(pick_z=150.0, safe_z=185.0, cube_height_mm=20.0)
    # 4mm above stack top: empty / 1-cube / 2-cube
    assert release_z_for_level(calib, 1) == 154.0
    assert release_z_for_level(calib, 2) == 174.0
    assert release_z_for_level(calib, 3) == 194.0
    assert travel_z_for_level(calib, 1) == 189.0  # 154+35
    assert travel_z_for_level(calib, 3) == 229.0  # 194+35 > safe_z


def test_stack_clear_xy_prefers_approach_ray():
    from mt4_vision.pickplace import STACK_AXIS_CLEAR_MM, stack_clear_xy

    sx, sy = 211.0, 7.0
    # Approached from +Y (cube side); clear point should stay near that ray.
    clear = stack_clear_xy(sx, sy, 211.0, 80.0, STACK_AXIS_CLEAR_MM)
    assert clear is not None
    assert abs(math.hypot(clear[0] - sx, clear[1] - sy) - STACK_AXIS_CLEAR_MM) < 0.1
    assert clear[1] > sy  # same half-plane as the approach
