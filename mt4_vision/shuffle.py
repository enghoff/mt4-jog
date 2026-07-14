"""Indefinite cube shuffle loop (see shuffle_blocks.py CLI).

Plans only from the latest camera frame via detection-as-state Scene.
No synthetic cubes, no camera-park retreat, no post-move stigma: if the arm
obscures a cube or marker, that object simply isn't in the scene. Ghosts must
be filtered out of pick candidates; they are never "handled" as grips.
"""

from __future__ import annotations

import sys
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import Calibration
from mt4_vision.camera import grab_frame, open_camera
from mt4_vision.pickplace import home_arm, pick, place
from mt4_vision.policy import Action, plan_shuffle
from mt4_vision.scene import Scene, capture_scene, verify_pick_place

# A post-move capture taken right after the arm clears sometimes lags the
# real desk by a few hundred ms (camera driver frame backlog after the
# multi-second gap during arm motion) -- a genuinely completed move can
# briefly still look like "grasp_failed" (cube seen at the old spot, none at
# the new one). Re-check a couple of times before trusting that reading and
# repeating the same pick; a real grasp failure still reads the same way
# after these retries and proceeds exactly as before.
POST_MOVE_RECHECK_ATTEMPTS = 2
POST_MOVE_RECHECK_DELAY_S = 0.4


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
    """Drain the camera during arm motion; capture once motion ends."""

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
        self._executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="shuffle-vision"
        )
        self._future: Future[Scene] | None = None

    def close(self) -> None:
        self._motion_done.set()
        self._executor.shutdown(wait=False, cancel_futures=True)
        self._cap.release()

    def capture(self) -> Scene:
        frame = grab_frame(self._cap)
        return capture_scene(self._calib, frame)

    def begin_motion(self) -> None:
        """Discard buffered frames while the arm moves."""
        self._motion_done.clear()
        if self._future is None or self._future.done():
            self._future = self._executor.submit(self._run)

    def end_motion(self) -> None:
        """Arm move finished -- capture a fresh frame on the worker thread."""
        self._motion_done.set()

    def take(self) -> Scene:
        if self._future is None:
            return self.capture()
        return self._future.result()

    def _run(self) -> Scene:
        while not self._motion_done.wait(timeout=0.05):
            self._cap.grab()
        if self._settle_s > 0:
            time.sleep(self._settle_s)
        return self.capture()


def _print_scene(scene: Scene) -> None:
    print(f"scene: {scene.summary_line()}")
    for line in scene.cube_lines():
        print(line)


def _print_action(action: Action) -> None:
    print(f"action: {action.kind} -- {action.reason}")


def _action_targets(
    action: Action,
) -> tuple[float, float, str, float, float] | None:
    """Pick/place coords from one planned action, or None to wait."""
    if action.kind != "pick" or action.cube is None:
        return None
    if action.place_x is None or action.place_y is None:
        return None
    return (
        float(action.cube.x),
        float(action.cube.y),
        action.cube.color,
        float(action.place_x),
        float(action.place_y),
    )


def _plan_from_capture(prefetch: _VisionPrefetch) -> tuple[Scene, Action]:
    scene = prefetch.capture()
    return scene, plan_shuffle(scene)


def _plan_after_move(
    prefetch: _VisionPrefetch,
    *,
    pick_x: float,
    pick_y: float,
    pick_color: str,
    place_x: float,
    place_y: float,
) -> tuple[Scene, Action]:
    scene = prefetch.take()
    verdict = verify_pick_place(
        scene,
        pick_x=pick_x,
        pick_y=pick_y,
        pick_color=pick_color,
        place_x=place_x,
        place_y=place_y,
    )
    attempts = 0
    while verdict != "placed" and attempts < POST_MOVE_RECHECK_ATTEMPTS:
        attempts += 1
        time.sleep(POST_MOVE_RECHECK_DELAY_S)
        scene = prefetch.capture()
        verdict = verify_pick_place(
            scene,
            pick_x=pick_x,
            pick_y=pick_y,
            pick_color=pick_color,
            place_x=place_x,
            place_y=place_y,
        )
    print(f"post-move check: {verdict} (after {attempts} recheck(s))")
    return scene, plan_shuffle(scene)


def _check_home_request(
    watcher: _HomeKeyWatcher,
    client: Mt4Client,
    prefetch: _VisionPrefetch,
    calib: Calibration,
) -> tuple[Scene, Action] | None:
    if watcher.consume():
        return _run_home(client, prefetch, calib)
    return None


def _sleep_or_home(
    seconds: float,
    watcher: _HomeKeyWatcher,
    client: Mt4Client,
    prefetch: _VisionPrefetch,
    calib: Calibration,
) -> tuple[Scene, Action] | None:
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
) -> tuple[Scene, Action]:
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
        scene, action = _plan_from_capture(prefetch)
        while True:
            refreshed = _check_home_request(watcher, client, prefetch, calib)
            if refreshed is not None:
                scene, action = refreshed
                continue

            _print_scene(scene)
            _print_action(action)

            targets = _action_targets(action)
            if targets is None:
                print(f"waiting {retry_s:.0f}s for a clearer scene")
                refreshed = _sleep_or_home(
                    retry_s, watcher, client, prefetch, calib
                )
                if refreshed is not None:
                    scene, action = refreshed
                else:
                    scene, action = _plan_from_capture(prefetch)
                continue

            pick_x, pick_y, color, place_x, place_y = targets
            print(
                f"pick-and-place: {color} ({pick_x:.0f},{pick_y:.0f}) "
                f"-> ({place_x:.0f},{place_y:.0f})"
            )
            prefetch.begin_motion()
            try:
                pick(client, calib, pick_x, pick_y)
                place(client, calib, place_x, place_y)
            except Mt4ClientError as exc:
                prefetch.end_motion()
                if _home_was_requested(watcher, exc):
                    scene, action = _run_home(client, prefetch, calib)
                    continue
                print(f"move failed: {exc} -- retrying next cycle")
                refreshed = _sleep_or_home(
                    retry_s, watcher, client, prefetch, calib
                )
                if refreshed is not None:
                    scene, action = refreshed
                else:
                    prefetch.take()
                    scene, action = _plan_from_capture(prefetch)
                continue

            prefetch.end_motion()
            scene, action = _plan_after_move(
                prefetch,
                pick_x=pick_x,
                pick_y=pick_y,
                pick_color=color,
                place_x=place_x,
                place_y=place_y,
            )
    finally:
        watcher.close()
        prefetch.close()
