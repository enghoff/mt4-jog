"""ArUco marker and colored-cube detection in overhead camera frames."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from mt4_vision.calib import Calibration

# HSV ranges (OpenCV scale: H 0-179, S/V 0-255) for solid-colored cubes under
# typical indoor lighting. Lighting-sensitive by nature -- override per-setup
# via Calibration.color_ranges rather than editing these. Red wraps the hue
# axis, hence two bands.
COLOR_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    # No "orange" by default: the wood table and red cubes' shaded side faces
    # both land in the orange hue band. Add it via Calibration.color_ranges
    # if the cube set actually has orange.
    "red": [((0, 100, 70), (9, 255, 255)), ((170, 100, 70), (179, 255, 255))],
    "yellow": [((23, 100, 100), (34, 255, 255))],
    # Green cubes sit darker than the rest under this lighting (V median ~84,
    # shadow side lower still), hence the low V floor.
    "green": [((36, 70, 45), (88, 255, 255))],
    "blue": [((90, 100, 60), (128, 255, 255))],
}
# Reject blobs smaller than this (px^2) -- noise, shadows, cable ties.
# Cube top faces are only ~150-600px^2 at this camera distance.
MIN_BLOB_AREA = 120.0
# Reject blobs larger than this -- the arm's own orange/red forearm reads as
# "red" and its blob (~1600px^2+) is much bigger than any real cube (topped
# out ~650px^2 across testing). Confirmed picking the wrong "cube" this way:
# detect_cubes sorts largest-first, so an uncapped arm-body blob outranks the
# real cube and gets treated as it by any caller taking the first result.
MAX_BLOB_AREA = 900.0
# Reject blobs whose bounding-box aspect is far from square (cubes are square
# from above; this drops elongated glare streaks and desk-edge artifacts).
MAX_ASPECT = 2.0

ARUCO_DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "4x4_100": cv2.aruco.DICT_4X4_100,
    "5x5_50": cv2.aruco.DICT_5X5_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_50": cv2.aruco.DICT_6X6_50,
    "6x6_250": cv2.aruco.DICT_6X6_250,
    "apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


@dataclass
class MarkerDetection:
    marker_id: int
    px: float  # center, pixels
    py: float
    # The 4 outline corners (pixels, detector order: TL, TR, BR, BL relative
    # to the marker's own printed orientation). Subpixel-refined. These carry
    # far more geometric information than the center alone -- 20 corners
    # across 5 identical printed squares are what make the table-plane
    # perspective actually observable (5 centers alone are not enough; see
    # table_fit.py).
    corners: list[list[float]] | None = None


@dataclass
class CubeDetection:
    color: str
    px: float  # centroid, pixels
    py: float
    area: float  # px^2
    x: float | None = None  # robot frame (mm), None when uncalibrated
    y: float | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "color": self.color,
            "pixel": [round(self.px, 1), round(self.py, 1)],
            "area_px": round(self.area),
            "x": None if self.x is None else round(self.x, 1),
            "y": None if self.y is None else round(self.y, 1),
        }


def detect_markers(
    frame: np.ndarray, dict_name: str = "4x4_50"
) -> list[MarkerDetection]:
    if dict_name not in ARUCO_DICTS:
        raise ValueError(f"unknown ArUco dict {dict_name!r}, one of {sorted(ARUCO_DICTS)}")
    params = cv2.aruco.DetectorParameters()
    # Subpixel corner refinement: the corners feed the table-plane fit, where
    # a half-pixel error is ~1mm on the table.
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name]), params
    )
    corners, ids, _rejected = detector.detectMarkers(frame)
    if ids is None:
        return []
    out = []
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        quad = marker_corners[0]
        center = quad.mean(axis=0)
        out.append(
            MarkerDetection(
                int(marker_id),
                float(center[0]),
                float(center[1]),
                corners=[[float(cx), float(cy)] for cx, cy in quad],
            )
        )
    return sorted(out, key=lambda m: m.marker_id)


def scan_marker_dicts(frame: np.ndarray) -> dict[str, int]:
    """Try every known dictionary; return {dict_name: markers_found} for hits."""
    hits = {}
    for name in ARUCO_DICTS:
        found = detect_markers(frame, name)
        if found:
            hits[name] = len(found)
    return hits


def detect_cubes(
    frame: np.ndarray,
    calibration: Calibration | None = None,
    colors: list[str] | None = None,
) -> list[CubeDetection]:
    """Detect colored cubes; robot XY filled in when a calibration is given."""
    ranges: dict[str, list] = dict(COLOR_RANGES)
    if calibration is not None:
        ranges.update(calibration.color_ranges)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Close first to heal ragged threshold edges, then a small open for
    # speckle -- a bigger open kernel eats the ~15-20px cube blobs whole.
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    hull = None
    if calibration is not None and calibration.workspace_hull_px:
        hull = np.array(calibration.workspace_hull_px, dtype=np.float32)

    detections: list[CubeDetection] = []
    for color, bands in ranges.items():
        if colors is not None and color not in colors:
            continue
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in bands:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA:
                continue
            _x, _y, w, h = cv2.boundingRect(contour)
            if max(w, h) / max(min(w, h), 1) > MAX_ASPECT:
                continue
            m = cv2.moments(contour)
            if m["m00"] == 0:
                continue
            px, py = m["m10"] / m["m00"], m["m01"] / m["m00"]
            # Workspace filter: off-desk clutter contributes stray blobs.
            # The margin is negative (allowing detections a bit OUTSIDE the
            # marker polygon) because the markers now sit inside the arm's
            # reach, not at the desk edges -- cubes can legitimately sit
            # outside the polygon. The arm's own red-reading body is handled
            # by MAX_BLOB_AREA, not this filter.
            if hull is not None and cv2.pointPolygonTest(hull, (px, py), True) < -80:
                continue
            det = CubeDetection(color, px, py, area)
            if calibration is not None:
                det.x, det.y = calibration.pixel_to_robot(px, py, on_cube_top=True)
                off = calibration.color_xy_offset_mm.get(color)
                if off:
                    det.x += float(off[0])
                    det.y += float(off[1])
            detections.append(det)
    return sorted(detections, key=lambda d: -d.area)
