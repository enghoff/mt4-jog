"""USB camera capture for the overhead work-surface camera."""

from __future__ import annotations

import os
import sys

import cv2
import numpy as np

# -1 = auto-detect: scan indices for the camera that sees ArUco markers
# (distinguishes the overhead work camera from e.g. a laptop's built-in one).
DEFAULT_CAMERA_INDEX = int(os.environ.get("MT4_CAMERA_INDEX", "-1"))
AUTO_SCAN_MAX_INDEX = 5
# The camera driver buffers several frames, so the first read() after a period
# of inactivity returns a stale image of the scene as it was seconds ago --
# fatal for pick-and-place, where we detect right before moving. Discard this
# many frames before keeping one.
FLUSH_FRAMES = 5


class CameraError(Exception):
    """Raised when the camera cannot be opened or a frame cannot be read."""


# Auto-detect result, cached because opening each candidate camera costs
# seconds. Reset by unplugging/replugging only across process restarts.
_detected_index: int | None = None


def _open_raw(index: int) -> cv2.VideoCapture:
    # CAP_DSHOW: the default MSMF backend on Windows takes several seconds to
    # open and sometimes refuses resolution changes on this Lenovo camera.
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    return cv2.VideoCapture(index, backend)


def _autodetect_index() -> int:
    """Find the work camera: the one that sees ArUco markers on the desk."""
    global _detected_index
    if _detected_index is not None:
        return _detected_index
    from mt4_vision.detect import scan_marker_dicts  # deferred: detect imports cv2 extras

    for index in range(AUTO_SCAN_MAX_INDEX + 1):
        cap = _open_raw(index)
        if not cap.isOpened():
            cap.release()
            continue
        try:
            frame = grab_frame(cap)
        except CameraError:
            cap.release()
            continue
        cap.release()
        if scan_marker_dicts(frame):
            _detected_index = index
            return index
    raise CameraError(
        f"no camera with visible ArUco markers found in indices 0-{AUTO_SCAN_MAX_INDEX}; "
        "set MT4_CAMERA_INDEX or pass --camera explicitly"
    )


def open_camera(index: int = DEFAULT_CAMERA_INDEX) -> cv2.VideoCapture:
    if index < 0:
        index = _autodetect_index()
    cap = _open_raw(index)
    if not cap.isOpened():
        raise CameraError(f"could not open camera index {index}")
    return cap


def grab_frame(cap: cv2.VideoCapture, flush: int = FLUSH_FRAMES) -> np.ndarray:
    """Read one fresh BGR frame, discarding buffered stale frames first."""
    for _ in range(flush):
        cap.grab()
    ok, frame = cap.read()
    if not ok or frame is None:
        raise CameraError("camera read failed")
    return frame


def capture_frame(index: int = DEFAULT_CAMERA_INDEX) -> np.ndarray:
    """One-shot open/grab/release for callers without a long-lived capture."""
    cap = open_camera(index)
    try:
        return grab_frame(cap)
    finally:
        cap.release()
