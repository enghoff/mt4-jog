"""Annotated live preview + video recording for pick/place scripts.

Draws the same cube/marker overlay as ``python -m mt4_vision scene`` on top
of every frame, so a human (or a saved recording) can watch the desk while
the planner works without touching the pick/place logic itself.
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from mt4_vision.calib import Calibration
from mt4_vision.camera import CameraError, FrameStream
from mt4_vision.detect import CubeDetection, MarkerDetection, detect_markers
from mt4_vision.scene import Scene, capture_scene
from mt4_vision.workspace import MARKER_DICT, dist_mm

# BGR overlay colors.
CUBE_BGR = (255, 255, 255)
PHANTOM_BGR = (90, 90, 90)
MARKER_FREE_BGR = (0, 200, 0)
MARKER_OCCUPIED_BGR = (0, 0, 255)
MARKER_UNKNOWN_BGR = (0, 200, 255)
TARGET_BGR = (255, 0, 255)


_OUTLINE_OFFSETS = [
    (-2, 0), (2, 0), (0, -2), (0, 2),
    (-1, -1), (-1, 1), (1, -1), (1, 1),
]


def draw_outlined_text(
    img: np.ndarray,
    text: str,
    org: tuple[int, int],
    *,
    scale: float,
    color: tuple[int, int, int],
    outline_color: tuple[int, int, int] = (0, 0, 0),
) -> None:
    """Draw text with a solid outline that can't drift from the fill.

    cv2.putText's Hershey font spaces characters differently per
    ``thickness`` argument (confirmed via cv2.getTextSize: the same string
    measures ~10px wider at thickness 3 than at thickness 1 for a ~200px
    line) -- a thick outline pass and a thin fill pass at the same origin
    silently drift apart character by character, worse the further into the
    string, rather than actually overlapping. Keeping every pass at
    thickness 1 and faking the stroke with small offset copies avoids that
    entirely.
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    for dx, dy in _OUTLINE_OFFSETS:
        cv2.putText(
            img, text, (org[0] + dx, org[1] + dy), font, scale,
            outline_color, 1, cv2.LINE_AA,
        )
    cv2.putText(img, text, org, font, scale, color, 1, cv2.LINE_AA)


def annotate_scene(
    frame: np.ndarray,
    scene: Scene,
    markers_px: list[MarkerDetection],
    *,
    target: CubeDetection | None = None,
    status_lines: list[str] | None = None,
) -> np.ndarray:
    """Return a copy of ``frame`` with cubes, markers, and a status header."""
    out = frame.copy()
    pick_ids = {id(c) for c in scene.cubes}
    for c in scene.raw_cubes if scene.raw_cubes is not None else scene.cubes:
        is_pick = id(c) in pick_ids
        color = CUBE_BGR if is_pick else PHANTOM_BGR
        label = c.color if is_pick else f"{c.color}?"
        cv2.circle(out, (int(c.px), int(c.py)), 8, color, 2)
        cv2.putText(
            out, label, (int(c.px) + 10, int(c.py)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA,
        )

    occupied_ids = {m.marker_id for m, _c in scene.occupied}
    free_ids = {m.marker_id for m in scene.free_markers}
    for m in markers_px:
        if m.marker_id in occupied_ids:
            color = MARKER_OCCUPIED_BGR
        elif m.marker_id in free_ids:
            color = MARKER_FREE_BGR
        else:
            color = MARKER_UNKNOWN_BGR
        cv2.drawMarker(out, (int(m.px), int(m.py)), color, cv2.MARKER_CROSS, 18, 2)
        cv2.putText(
            out, str(m.marker_id), (int(m.px) + 10, int(m.py) - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA,
        )

    if target is not None:
        cv2.circle(out, (int(target.px), int(target.py)), 16, TARGET_BGR, 3)

    for i, line in enumerate(status_lines or []):
        y = 24 + i * 22
        draw_outlined_text(out, line, (10, y), scale=0.6, color=(255, 255, 255))

    return out


class VideoRecorder:
    """Appends annotated frames to a video file at a fixed rate."""

    def __init__(self, *, video_path: str, fps: float = 10.0) -> None:
        self._video_path = video_path
        self._fps = fps
        self._writer: cv2.VideoWriter | None = None

    def _open_writer(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(
            *("mp4v" if str(self._video_path).lower().endswith(".mp4") else "MJPG")
        )
        writer = cv2.VideoWriter(str(self._video_path), fourcc, self._fps, (w, h))
        if not writer.isOpened():
            raise RuntimeError(f"could not open video writer for {self._video_path}")
        self._writer = writer

    def write(self, frame: np.ndarray) -> None:
        if self._writer is None:
            self._open_writer(frame)
        self._writer.write(frame)

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None


class PreviewStopped(Exception):
    """Raised when the user closes a ``LivePreview`` window (q or Esc)."""


class LivePreview:
    """Pop-up cv2 window showing whatever frame it's handed.

    Mirrors track_cube.py's preview window: non-blocking (``cv2.waitKey(1)``),
    closed with q/Esc.
    """

    def __init__(self, window_name: str = "stack_cubes preview (q or Esc to stop)") -> None:
        self._window = window_name

    def show(self, frame: np.ndarray) -> None:
        """Render one frame; raise ``PreviewStopped`` if the user hit q/Esc."""
        cv2.imshow(self._window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            raise PreviewStopped()

    def close(self) -> None:
        try:
            cv2.destroyWindow(self._window)
        except cv2.error:
            pass


# Gate for matching the cube a caller is currently acting on (``set_target``)
# against this tick's fresh detections -- generous enough to survive a frame
# or two of drift, tight enough not to ring the wrong same-color cube.
TARGET_MATCH_GATE_MM = 60.0


def _nearest_same_color(
    cubes: list[CubeDetection], color: str, x: float, y: float,
) -> CubeDetection | None:
    best: CubeDetection | None = None
    best_d = TARGET_MATCH_GATE_MM
    for c in cubes:
        if c.color != color or c.x is None or c.y is None:
            continue
        d = dist_mm(float(c.x), float(c.y), x, y)
        if d <= best_d:
            best = c
            best_d = d
    return best


class LiveFeed:
    """Continuously annotated preview/recording, decoupled from arm motion.

    Runs its own capture/detect/draw loop on a background thread against a
    caller-owned ``FrameStream``, so the feed keeps updating at ``fps`` while
    the main thread blocks on multi-second arm moves (pick/place). The arm is
    not retreated for this feed -- unlike the discrete captures a planner
    acts on, this is a monitor and shows whatever the camera currently sees,
    arm included.

    The caller opens and closes the ``FrameStream`` itself (typically shared
    with the planner's own "look now" captures, since only one consumer can
    hold the camera device); ``close()`` here only stops this feed's thread
    and output sinks, it never touches the stream.
    """

    def __init__(
        self,
        *,
        calib: Calibration,
        stream: FrameStream,
        fps: float = 10.0,
        video_path: str | None = None,
        show_preview: bool = False,
    ) -> None:
        self._calib = calib
        self._stream = stream
        self._period = 1.0 / fps if fps > 0 else 0.0
        self._recorder = (
            VideoRecorder(video_path=video_path, fps=fps) if video_path else None
        )
        self._live_preview = LivePreview() if show_preview else None
        self._status_lines: list[str] = []
        self._target: tuple[str, float, float] | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.stopped_by_user = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_status(self, lines: list[str]) -> None:
        with self._lock:
            self._status_lines = list(lines)

    def set_target(self, color: str, x: float, y: float) -> None:
        with self._lock:
            self._target = (color, x, y)

    def clear_target(self) -> None:
        with self._lock:
            self._target = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            tick_start = time.monotonic()
            try:
                frame = self._stream.fresh(min_advance=1)
            except CameraError:
                continue
            scene = capture_scene(self._calib, frame)
            markers_px = detect_markers(frame, MARKER_DICT)
            with self._lock:
                status_lines = list(self._status_lines)
                target_spec = self._target
            target = None
            if target_spec is not None:
                color, tx, ty = target_spec
                target = _nearest_same_color(
                    scene.raw_cubes if scene.raw_cubes is not None else scene.cubes,
                    color, tx, ty,
                )
            annotated = annotate_scene(
                frame, scene, markers_px, target=target, status_lines=status_lines,
            )
            if self._recorder is not None:
                self._recorder.write(annotated)
            if self._live_preview is not None:
                try:
                    self._live_preview.show(annotated)
                except PreviewStopped:
                    self.stopped_by_user.set()
                    break
            elapsed = time.monotonic() - tick_start
            remaining = self._period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._recorder is not None:
            self._recorder.close()
        if self._live_preview is not None:
            self._live_preview.close()
