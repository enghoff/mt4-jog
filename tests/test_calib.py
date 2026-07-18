"""Calibration mapping guards."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mt4_vision import calib as calib_mod
from mt4_vision.calib import Calibration

IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def make_calib(**overrides) -> Calibration:
    return Calibration(
        homography=IDENTITY, table_z=144.0, pick_z=154.0, safe_z=185.0, **overrides
    )


def _reset_warning(monkeypatch):
    monkeypatch.setattr(calib_mod, "_warned_no_cube_top_correction", False)


def test_missing_cube_top_warns_once(capsys, monkeypatch):
    _reset_warning(monkeypatch)
    c = make_calib()
    c.pixel_to_robot(10.0, 10.0, on_cube_top=True)
    c.pixel_to_robot(20.0, 20.0, on_cube_top=True)
    err = capsys.readouterr().err
    assert err.count("cube_top_homography") == 1
    assert "calibrate_height.py" in err


def test_table_plane_mapping_never_warns(capsys, monkeypatch):
    _reset_warning(monkeypatch)
    make_calib().pixel_to_robot(10.0, 10.0)
    assert capsys.readouterr().err == ""


def test_cube_top_homography_set_no_warning(capsys, monkeypatch):
    _reset_warning(monkeypatch)
    c = make_calib(cube_top_homography=IDENTITY)
    c.pixel_to_robot(10.0, 10.0, on_cube_top=True)
    assert capsys.readouterr().err == ""


def test_radial_fallback_no_warning(capsys, monkeypatch):
    _reset_warning(monkeypatch)
    c = make_calib(cam_xy_robot=[400.0, 0.0], cam_height_mm=600.0)
    c.pixel_to_robot(10.0, 10.0, on_cube_top=True)
    assert capsys.readouterr().err == ""


def test_color_xy_offset_applied_to_cube_detections():
    import cv2  # noqa: F401 -- ensures OpenCV present for detect_cubes
    import numpy as np

    from mt4_vision.detect import detect_cubes

    frame = np.zeros((100, 100, 3), np.uint8)
    frame[40:60, 30:50] = (0, 0, 255)  # red square, ~400px^2
    base = make_calib(cube_top_homography=IDENTITY)
    offset = make_calib(
        cube_top_homography=IDENTITY, color_xy_offset_mm={"red": [5.0, -3.0]}
    )
    a = detect_cubes(frame, base)[0]
    b = detect_cubes(frame, offset)[0]
    assert abs(b.x - a.x - 5.0) < 1e-6
    assert abs(b.y - a.y + 3.0) < 1e-6
