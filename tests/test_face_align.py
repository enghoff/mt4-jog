"""Unit tests for cube face-align J4 helpers (no hardware)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_vision.pickplace import (
    fold_square_yaw_deg,
    j4_for_face_align,
    j4_preserve_wrist,
)


def test_fold_square_yaw_period_90():
    assert abs(fold_square_yaw_deg(0.0)) < 1e-9
    assert abs(fold_square_yaw_deg(90.0)) < 1e-9
    assert abs(fold_square_yaw_deg(45.0) - (-45.0)) < 1e-9
    assert abs(fold_square_yaw_deg(30.0) - 30.0) < 1e-9
    assert abs(fold_square_yaw_deg(-30.0) - (-30.0)) < 1e-9


def test_j4_face_align_folds_without_current():
    assert abs(j4_for_face_align(10.0) - 10.0) < 1e-9
    assert abs(j4_for_face_align(100.0) - 10.0) < 1e-9


def test_j4_face_align_picks_nearest_90_to_current():
    # Cube edge at 10°, current wrist at 95° → prefer 100° over 10°.
    j4 = j4_for_face_align(10.0, current_j4_deg=95.0)
    assert abs(j4 - 100.0) < 1e-9


def test_j4_face_align_applies_offset():
    assert abs(j4_for_face_align(0.0, offset_deg=12.0) - 12.0) < 1e-9


def test_j4_preserve_wrist_holds_joint_across_j1_swing():
    # Park (200,0) world j4=77.8 → joint j4=77.8; at marker-0 bearing
    # j1≈−79.2, world j4 should be ≈−1.4 so joint stays 77.8.
    j4 = j4_preserve_wrist(
        49.1, -256.6, from_x=200.0, from_y=0.0, from_j4=77.8
    )
    assert abs(j4 - (-1.367)) < 0.05


if __name__ == "__main__":
    test_fold_square_yaw_period_90()
    test_j4_face_align_folds_without_current()
    test_j4_face_align_picks_nearest_90_to_current()
    test_j4_face_align_applies_offset()
    test_j4_preserve_wrist_holds_joint_across_j1_swing()
    print("ok")
