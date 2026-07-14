"""Indefinite cube shuffle loop (see shuffle_blocks.py CLI)."""

from __future__ import annotations

import sys
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import Calibration
from mt4_vision.camera import grab_frame, open_camera
from mt4_vision.pickplace import home_arm, pick, place
from mt4_vision.workspace import (
    ShuffleMove,
    WorkspaceState,
    analyze_workspace,
    apply_completed_move,
    mp_reachable_cubes,
    plan_shuffle_move,
)


class _HomeKeyWatcher:
    """Detect a tap of H (same binding as jog_keyboard.py) without blocking."""

    def __init__(self, client: Mt4Client) -> None:
        self._client = client
        self._requested = threading.Event()
        self._h_down = False
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="shuffle-home-key", daemon=True
        )

    def start(self) -> None:
        if sys.platform == "win32":
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=0.3)

    def consume(self) -> bool:
        if self._requested.is_set():
            self._requested.clear()
            return True
        return False

    def _run(self) -> None:
        from jog_keyboard import key_down

        while not self._stop.is_set():
            down = key_down("h")
            if down and not self._h_down:
                self._requested.set()
                self._client.request_interrupt()
            self._h_down = down
            self._stop.wait(0.05)


class _VisionPrefetch:
    """Drain the camera during arm motion; capture once the arm is clear."""

    def __init__(
        self,
        calib: Calibration,
        camera: int,
        *,
        settle_s: float,
    ) -> None:
        self._calib = calib
        self._settle_s = settle_s
        self._cap = open_camera(camera)
        self._motion_done = threading.Event()
        self._motion_done.set()
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="shuffle-vision")
        self._future: Future[WorkspaceState] | None = None

    def close(self) -> None:
        self._motion_done.set()
        self._executor.shutdown(wait=False, cancel_futures=True)
        self._cap.release()

    def capture(self) -> WorkspaceState:
        frame = grab_frame(self._cap)
        return analyze_workspace(self._calib, frame)

    def begin_motion(self) -> None:
        """Discard buffered frames while the arm moves."""
        self._motion_done.clear()
        if self._future is None or self._future.done():
            self._future = self._executor.submit(self._run)

    def end_motion(self) -> None:
        """Arm move finished -- capture a fresh frame on the worker thread."""
        self._motion_done.set()

    def take(self) -> WorkspaceState:
        if self._future is None:
            return self.capture()
        return self._future.result()

    def _run(self) -> WorkspaceState:
        while not self._motion_done.wait(timeout=0.05):
            self._cap.grab()
        if self._settle_s > 0:
            time.sleep(self._settle_s)
        return self.capture()


def _print_no_move(state: WorkspaceState, retry_s: float) -> None:
    pickable = len(mp_reachable_cubes(state.cubes))
    print(
        f"no move: cubes={len(state.cubes)} ({pickable} mp-reachable) "
        f"free_markers={len(state.free_markers)} "
        f"occupied={len(state.occupied)} "
        f"free_slots={len(state.free_slots)} "
        f"-- retry in {retry_s:.0f}s"
    )


def _print_move(move: ShuffleMove) -> None:
    dest = (
        f"marker {move.place_marker_id}"
        if move.place_marker_id is not None
        else f"({move.place_x:.1f}, {move.place_y:.1f})"
    )
    print(
        f"{move.kind}: pick {move.pick_color} at "
        f"({move.pick_x:.1f}, {move.pick_y:.1f}) -> {dest}"
    )


def _plan_from_capture(prefetch: _VisionPrefetch) -> tuple[WorkspaceState, ShuffleMove | None]:
    state = prefetch.capture()
    return state, plan_shuffle_move(state)


def _plan_after_move(
    prefetch: _VisionPrefetch,
    calib: Calibration,
    completed: ShuffleMove,
) -> tuple[WorkspaceState, ShuffleMove | None]:
    """Refresh vision after the arm clears, then fold in the move we just made."""
    state = apply_completed_move(prefetch.take(), completed, calib)
    return state, plan_shuffle_move(state)


def _check_home_request(
    watcher: _HomeKeyWatcher,
    client: Mt4Client,
    prefetch: _VisionPrefetch,
    calib: Calibration,
) -> tuple[WorkspaceState, ShuffleMove | None] | None:
    if watcher.consume():
        return _run_home(client, prefetch, calib)
    return None


def _sleep_or_home(
    seconds: float,
    watcher: _HomeKeyWatcher,
    client: Mt4Client,
    prefetch: _VisionPrefetch,
    calib: Calibration,
) -> tuple[WorkspaceState, ShuffleMove | None] | None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if watcher.consume():
            return _run_home(client, prefetch, calib)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.05, remaining))
    return _check_home_request(watcher, client, prefetch, calib)


def _run_home(
    client: Mt4Client,
    prefetch: _VisionPrefetch,
    calib: Calibration,
) -> tuple[WorkspaceState, ShuffleMove | None]:
    print("homing (H)...")
    client.clear_interrupt()
    try:
        client.stop()
    except Mt4ClientError:
        pass
    prefetch.end_motion()
    home_arm(client)
    print("home ok")
    return _plan_from_capture(prefetch)


def _home_was_requested(watcher: _HomeKeyWatcher, exc: Mt4ClientError) -> bool:
    return watcher.consume() or "interrupted" in str(exc)


def run_shuffle_loop(
    client: Mt4Client,
    calib: Calibration,
    *,
    camera: int,
    pause_s: float = 2.0,
    retry_s: float = 5.0,
) -> None:
    """Home if needed, then shuffle cubes until interrupted."""
    if client.get_status().homed:
        print("already homed")
    else:
        print("homing...")
        home_arm(client)
        print("home ok")

    watcher = _HomeKeyWatcher(client)
    watcher.start()
    prefetch = _VisionPrefetch(calib, camera, settle_s=pause_s)
    try:
        state, move = _plan_from_capture(prefetch)
        while True:
            refreshed = _check_home_request(watcher, client, prefetch, calib)
            if refreshed is not None:
                state, move = refreshed
                continue

            if move is None:
                _print_no_move(state, retry_s)
                refreshed = _sleep_or_home(retry_s, watcher, client, prefetch, calib)
                if refreshed is not None:
                    state, move = refreshed
                else:
                    state, move = _plan_from_capture(prefetch)
                continue

            _print_move(move)
            completed = move
            prefetch.begin_motion()
            try:
                pick(client, calib, completed.pick_x, completed.pick_y)
                place(client, calib, completed.place_x, completed.place_y)
            except Mt4ClientError as exc:
                prefetch.end_motion()
                if _home_was_requested(watcher, exc):
                    state, move = _run_home(client, prefetch, calib)
                    continue
                print(f"move failed: {exc} -- retrying next cycle")
                refreshed = _sleep_or_home(retry_s, watcher, client, prefetch, calib)
                if refreshed is not None:
                    state, move = refreshed
                else:
                    prefetch.take()
                    state, move = _plan_from_capture(prefetch)
                continue

            prefetch.end_motion()
            state, move = _plan_after_move(prefetch, calib, completed)
    finally:
        watcher.close()
        prefetch.close()
