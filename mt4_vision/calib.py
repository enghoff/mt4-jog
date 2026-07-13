"""Camera-to-robot calibration: table-plane homography plus pick heights.

The homography maps camera pixels to robot-frame XY *on the table plane* (the
plane the ArUco markers lie on). It is fit from pixel/robot correspondences
gathered by jogging the arm's TCP to touch each marker center -- this absorbs
the entire camera pose, so no camera intrinsics are needed.

Cubes are detected by their top face, which sits cube_height_mm above the
table plane, so the raw homography output is displaced radially away from the
camera's nadir. When cam_xy_robot/cam_height_mm are set, detections are
corrected by shrinking toward the nadir point by (h_cam - h_cube) / h_cam.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

DEFAULT_CALIB_PATH = Path(
    os.environ.get("MT4_VISION_CALIB", Path(__file__).resolve().parent.parent / "vision_calibration.json")
)


class CalibrationError(Exception):
    """Raised when calibration data is missing, unloadable, or degenerate."""


@dataclass
class Calibration:
    # Row-major 3x3 pixel -> robot-XY homography (table plane).
    homography: list[list[float]]
    # Robot-frame Z of the table surface under the camera (mm).
    table_z: float
    # TCP Z for gripping a cube sitting on the table (mm).
    pick_z: float
    # Travel height between moves (mm). Keep modest -- the arm should stay
    # low over the desk, well inside its envelope.
    safe_z: float
    # Gripper S values for this cube size (firmware absolute, 120-285).
    grip_open_s: int = 140
    grip_close_s: int = 240
    # Cube edge length (mm), used for the parallax correction and place height.
    cube_height_mm: float = 30.0
    # Optional parallax correction: robot XY directly under the camera and the
    # camera lens height above the table (mm). None disables the correction.
    cam_xy_robot: list[float] | None = None
    cam_height_mm: float | None = None
    # Per-color HSV overrides merged over detect.COLOR_RANGES defaults.
    color_ranges: dict = field(default_factory=dict)
    # Pixel-space convex hull of the marker centers. Detections outside it
    # are rejected (the arm's orange body and off-desk clutter live there).
    workspace_hull_px: list[list[float]] | None = None

    def pixel_to_robot(self, px: float, py: float, *, on_cube_top: bool = False) -> tuple[float, float]:
        h = np.array(self.homography, dtype=np.float64)
        v = h @ np.array([px, py, 1.0])
        x, y = float(v[0] / v[2]), float(v[1] / v[2])
        if on_cube_top and self.cam_xy_robot and self.cam_height_mm:
            scale = (self.cam_height_mm - self.cube_height_mm) / self.cam_height_mm
            cx, cy = self.cam_xy_robot
            x = cx + (x - cx) * scale
            y = cy + (y - cy) * scale
        return x, y

    def save(self, path: Path = DEFAULT_CALIB_PATH) -> None:
        path.write_text(json.dumps(self.__dict__, indent=2), encoding="utf-8")


def load_calibration(path: Path = DEFAULT_CALIB_PATH) -> Calibration:
    if not Path(path).exists():
        raise CalibrationError(
            f"no calibration at {path} -- run: python calibrate_vision.py"
        )
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return Calibration(**data)


def fit_homography(
    pixel_pts: list[tuple[float, float]], robot_pts: list[tuple[float, float]]
) -> list[list[float]]:
    """Least-squares pixel->robot homography from >=4 correspondences."""
    if len(pixel_pts) < 4 or len(pixel_pts) != len(robot_pts):
        raise CalibrationError(
            f"need >=4 matched points, got {len(pixel_pts)} pixel / {len(robot_pts)} robot"
        )
    src = np.array(pixel_pts, dtype=np.float64)
    dst = np.array(robot_pts, dtype=np.float64)
    h, _mask = cv2.findHomography(src, dst, 0)
    if h is None:
        raise CalibrationError("homography fit failed (degenerate point set?)")
    return h.tolist()


def fit_affine(
    pixel_pts: list[tuple[float, float]], robot_pts: list[tuple[float, float]]
) -> list[list[float]]:
    """Affine pixel->robot fit from >=3 correspondences, embedded as a 3x3
    homography (bottom row [0, 0, 1]) so downstream code is unchanged.

    Only reach for this when just 3 markers are physically reachable: an
    affine map cannot model perspective foreshortening, so accuracy degrades
    away from the calibration triangle.
    """
    if len(pixel_pts) < 3 or len(pixel_pts) != len(robot_pts):
        raise CalibrationError(
            f"need >=3 matched points, got {len(pixel_pts)} pixel / {len(robot_pts)} robot"
        )
    src = np.hstack(
        [np.array(pixel_pts, dtype=np.float64), np.ones((len(pixel_pts), 1))]
    )
    dst = np.array(robot_pts, dtype=np.float64)
    coef, _res, rank, _sv = np.linalg.lstsq(src, dst, rcond=None)
    if rank < 3:
        raise CalibrationError("affine fit failed (collinear points?)")
    h = np.eye(3)
    h[0, :] = coef[:, 0]
    h[1, :] = coef[:, 1]
    return h.tolist()


def fit_transform(
    pixel_pts: list[tuple[float, float]], robot_pts: list[tuple[float, float]]
) -> tuple[list[list[float]], str]:
    """Fit the best available pixel->robot map: full homography with >=4
    points, affine with exactly 3. Returns (matrix, kind)."""
    if len(pixel_pts) >= 4:
        return fit_homography(pixel_pts, robot_pts), "homography"
    return fit_affine(pixel_pts, robot_pts), "affine"


def reprojection_errors(
    homography: list[list[float]],
    pixel_pts: list[tuple[float, float]],
    robot_pts: list[tuple[float, float]],
) -> list[float]:
    h = np.array(homography, dtype=np.float64)
    errors = []
    for (px, py), (rx, ry) in zip(pixel_pts, robot_pts):
        v = h @ np.array([px, py, 1.0])
        errors.append(float(np.hypot(v[0] / v[2] - rx, v[1] / v[2] - ry)))
    return errors
