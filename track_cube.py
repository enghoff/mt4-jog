#!/usr/bin/env python3
"""Visually servo the gripper to hover over a single moving cube.

Holds the TCP a fixed height above the table (default 5cm) directly over
one cube and follows it as it's moved by hand. Only one cube is tracked at
a time -- if several are in view, whichever reachable one is detected first
is locked on and identity is then held via nearest-neighbor gating (not
color alone, since two cubes can share a color), never opportunistically
re-targeted.

Two phases:

  1. Setup (`Mt4Client`, blocking `mp`): home if needed, find an initial
     reachable cube, and move the TCP to hover directly above it. This
     reuses the well-tested absolute-move path for the one large motion in
     the whole run.
  2. Tracking (raw serial, streaming `cj`): `Mt4Client.move_to()` is a
     blocking point-to-point trajectory -- far too coarse-grained to chase a
     moving target. The firmware's Cartesian jog (`cj dx dy dz`) is
     direction-only but non-blocking and auto-refreshes on-device every
     40ms, so it's driven directly the way `jog.py` does: this script closes
     `Mt4Client` and opens its own serial connection for the loop, since
     only one process may own the COM port at a time.

Each tick (paced by the camera, ~10Hz measured on this setup):
  - grab the freshest frame and detect cubes
  - match the tracked cube by nearest position (same color, within a gate);
    the target is just that raw matched position, no velocity estimate or
    smoothing filter over it (deliberately dropped -- see git history if
    reintroducing lead-time prediction is ever worth revisiting)
  - request a fresh real TCP pose every tick (`pos`, fire-and-forget) and
    pick up the reply whenever it lands -- kept off the critical path so it
    never blocks the loop, but still only a tick or so stale
  - jog the direction-only `cj` command toward the error vector, sending a
    distance-scheduled target speed via `cjspeed` every tick -- the firmware
    (opted in once via `cjramp`, see below) ramps toward it on its own 40ms
    clock so the final approach doesn't overshoot the deadband at full speed
    (this is bang-bang + a gain schedule, not PID -- there's no integral or
    derivative term)

If the cube is lost from view for longer than a short grace period, the
arm stops and holds position, watching for the same cube to reappear
anywhere in view -- it never switches to a different cube.

Ctrl+C (or any error) always sends `stop` before exiting -- there is no
firmware watchdog that stops `cj` on its own if the host goes quiet.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from jog import console_focused, key_down, run_home
from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_jog.joints import (
    DEFAULT_BAUD,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
    JOG_SPEED_MIN_US,
)
from mt4_jog.serial import (
    FirmwareNotReadyError,
    SerialGoneError,
    await_firmware_alive,
    drain_lines,
    open_serial,
    send,
    send_quick,
)
from mt4_jog.status import TcpPose, parse_tcp_line
from mt4_vision.calib import (
    DEFAULT_CALIB_PATH,
    Calibration,
    CalibrationError,
    load_calibration,
)
from mt4_vision.camera import DEFAULT_CAMERA_INDEX, CameraError, FrameStream
from mt4_vision.detect import CubeDetection, detect_cubes
from mt4_vision.pickplace import ensure_homed
from mt4_vision.preview import VideoRecorder, draw_cube_marker, draw_lock_ring, draw_outlined_text
from mt4_vision.workspace import KEEPOUT_RADIUS_MM, MAX_REACH_MM

DEFAULT_HOVER_MM = 50.0
# Extra margin beyond the firmware's own keep-out / max-reach checks --
# `cj` (unlike `mp`) has no max-reach clamp, only a keep-out one, so the
# *target* fed into the error computation is clamped defensively every tick.
KEEPOUT_MARGIN_MM = 5.0
REACH_MARGIN_MM = 5.0

MAX_JUMP_MM = 40.0  # per-tick gate while actively tracking
# No position gate once actually lost (frozen, past the grace period): the
# cube can reappear anywhere in view (picked up and set down elsewhere) and
# still count as the same tracked cube -- color is the only identity signal
# available anyway, and match_detection() still prefers the nearest same-
# color candidate to last_known when more than one is in view.
REACQUIRE_GATE_MM = math.inf
# Brief coast on the last known position (no velocity estimate anymore, so
# this just holds last_known frozen) before stop + hold.
LOST_GRACE_S = 0.5
DEADBAND_MM = 3.0  # inside this, stop rather than jitter
# Real TCP feedback is requested every tick but never blocked on: a `pos`
# query is fired fire-and-forget (send_quick) and its reply is picked up
# opportunistically whenever it shows up in the buffer already being
# drained for firmware chatter (report_firmware_lines). This gives TCP
# feedback that's stale by about one tick (~40-60ms) instead of the several
# hundred ms a synchronous send(wait=...) would cost every tick.
#
# A synchronous per-tick poll was tried first and did fix a real bug (a
# multi-tick blind window where the arm kept jogging on a stale position
# estimate, overshot the target during it, and reversed hard once the next
# poll revealed the overshoot -- a feedback-deadtime limit cycle, confirmed
# live as TCP y oscillating ~40-70mm around a perfectly stationary target).
# But blocking ~150-200ms per tick throttled the whole loop from
# ~15-30Hz (camera-bound) to ~5-6Hz, and a bang-bang controller that looks
# smooth at high update rate looks like discrete steps at a low one --
# nothing about the motion logic changed, just how coarse each visible
# correction segment became. The async version keeps the fast loop rate
# and keeps the blind window short enough not to matter.
# Jog speed ramps with distance-to-target: fast (the firmware's quickest
# step rate) while far away, slower as it converges. This IS load-bearing,
# confirmed by removing it: even with fresh per-tick TCP feedback, a
# constant max-speed jog overshoots the 3mm deadband within a single ~100ms
# tick (real linear speed at max jog covers more than that easily), and the
# correction the next tick overshoots the other way -- a sustained
# oscillation with no feedback-staleness involved at all, just velocity too
# high relative to deadband width and tick period.
#
# `cj` jogging disables the firmware's own mp-style accel ramp (`speed <us>`
# is one direct timer-register write, no smoothing) -- but unlike an earlier
# version of this script, the ramp is no longer redone in Python. Smoothing
# a distance-scheduled target speed by capping its change *per host tick*
# ties the ramp rate to however fast (and however jittery) the camera loop
# happens to be, which is exactly backwards: a stationary target still
# produces a changing error vector as the arm itself converges, and batching
# that drift into infrequent, larger corrections reads as jerks regardless of
# the cap value (see track_cube's git history for the CJ_MIN_INTERVAL_S
# diagnostic that made this visible). `cjspeed <us>` instead just tells the
# firmware the current target speed every tick; `cjramp <us>` (sent once,
# below) opts the firmware into ramping `cjspeed` targets toward each other
# at a fixed rate on its own steady 40ms clock, immune to host loop jitter.
# `cjramp 0` (the firmware's power-on default) is the instant rollback: it
# makes `cjspeed` apply immediately, byte-for-byte the old un-ramped
# `speed` write, with no reflash needed.
SPEED_FAR_US = JOG_SPEED_MIN_US  # fastest, used beyond SPEED_FAR_MM
SPEED_NEAR_US = 2400  # gentle final approach, used within SPEED_NEAR_MM
SPEED_FAR_MM = 40.0
SPEED_NEAR_MM = 8.0  # just outside DEADBAND_MM, so it's already slow by the time it stops
# us per CJ_REFRESH_MS (40ms) firmware tick -- same ~800us/s net rate as the
# old Python-side cap (80us per ~100ms tick), just applied on the firmware's
# jitter-free clock instead of the camera-paced one.
CJ_RAMP_STEP_US = 32
# Skip resending `cj` unless the error vector has moved at least this much
# (mm, tip-to-tip) since the last cj actually sent -- see the resend gate in
# run_tracking_loop for why (every send re-arms the firmware's DDA and
# resets step-accumulator phase, which the firmware's own 40ms auto-refresh
# already does on its own for an unchanged direction).
#
# For a genuinely stationary target the arm closes on it in a straight
# line, so the recomputed direction shouldn't meaningfully change between
# ticks -- with this deadzone active, that means ~one cj send to start the
# approach and one `stop` at the deadband, nothing in between, letting the
# firmware's own unchanging 40ms auto-refresh carry the whole approach
# instead of us re-arming the DDA (and handing it a slightly different
# recomputed vector -- fresh vision + fresh `pos` + integer rounding, never
# bit-for-bit identical) on every tick.
CJ_UPDATE_DEADZONE_MM = 5.0
INITIAL_LOCK_TIMEOUT_S = 30.0
# `cj`'s firmware parser reads direction components as integers (sscanf
# %ld) then normalizes the vector on-device -- only the ratio matters, but
# a sub-1mm error would round straight to 0 and lose its direction without
# this scale-up.
CJ_INT_SCALE = 1000

PREVIEW_WINDOW = "track_cube preview (q or Esc to stop)"
COLOR_BGR = {
    "red": (0, 0, 255),
    "yellow": (0, 220, 220),
    "green": (0, 170, 0),
    "blue": (220, 100, 0),
}
UNKNOWN_COLOR_BGR = (200, 200, 200)
LOCK_BGR = (0, 255, 0)
LOST_BGR = (0, 0, 255)


def clamp_to_envelope(x: float, y: float) -> tuple[float, float]:
    """Pull (x, y) back inside the keep-out/reach annulus if it's outside."""
    r = math.hypot(x, y)
    r_min = KEEPOUT_RADIUS_MM + KEEPOUT_MARGIN_MM
    r_max = MAX_REACH_MM - REACH_MARGIN_MM
    if r < r_min and r > 1e-6:
        scale = r_min / r
        return x * scale, y * scale
    if r > r_max:
        scale = r_max / r
        return x * scale, y * scale
    return x, y


def is_reachable(x: float, y: float) -> bool:
    r = math.hypot(x, y)
    return (KEEPOUT_RADIUS_MM + KEEPOUT_MARGIN_MM) <= r <= (MAX_REACH_MM - REACH_MARGIN_MM)


def match_detection(
    detections: list[CubeDetection],
    color: str,
    last_xy: tuple[float, float],
    gate_mm: float,
) -> CubeDetection | None:
    best: CubeDetection | None = None
    best_d = gate_mm
    for det in detections:
        if det.color != color or det.x is None or det.y is None:
            continue
        d = math.hypot(det.x - last_xy[0], det.y - last_xy[1])
        if d <= best_d:
            best = det
            best_d = d
    return best


def find_initial_target(
    stream: FrameStream, calib: Calibration, timeout_s: float
) -> CubeDetection | None:
    """Lock onto the first reachable cube seen; None if none shows up in time."""
    deadline = time.monotonic() + timeout_s
    last_print = 0.0
    while time.monotonic() < deadline:
        frame = stream.fresh()
        detections = detect_cubes(frame, calib)
        candidates = [
            d for d in detections if d.x is not None and d.y is not None
            and is_reachable(d.x, d.y)
        ]
        if candidates:
            return candidates[0]
        now = time.monotonic()
        if now - last_print > 2.0:
            print(
                f"Waiting for a reachable cube in view... "
                f"({len(detections)} detected, none reachable)"
            )
            last_print = now
    return None


def speed_for_distance(dist_mm: float) -> int:
    if dist_mm >= SPEED_FAR_MM:
        return SPEED_FAR_US
    if dist_mm <= SPEED_NEAR_MM:
        return SPEED_NEAR_US
    t = (dist_mm - SPEED_NEAR_MM) / (SPEED_FAR_MM - SPEED_NEAR_MM)
    return int(round(SPEED_NEAR_US + t * (SPEED_FAR_US - SPEED_NEAR_US)))


def report_firmware_lines(ser, buf: list[str], verbose: bool) -> TcpPose | None:
    """Drain and print firmware chatter; return the newest TCP pose seen, if any.

    A `?` query's reply (a `tcp x=... y=... ...` line among others) may show
    up here on a later tick than the one that sent it -- that's the point:
    the query is fired async (send_quick) rather than blocked on, so this is
    where the answer gets picked up whenever it actually arrives.
    """
    latest_tcp: TcpPose | None = None
    for line in drain_lines(ser, buf):
        tcp = parse_tcp_line(line)
        if tcp is not None:
            latest_tcp = tcp
        if line.startswith("err ") or line.startswith("lim "):
            print(line)
        elif verbose:
            print(line, file=sys.stderr)
    return latest_tcp


def draw_preview(
    frame: np.ndarray,
    detections: list[CubeDetection],
    match: CubeDetection | None,
    *,
    color: str,
    lost_since: float | None,
    dist: float | None,
) -> np.ndarray:
    """Annotate a copy of `frame` for live troubleshooting.

    Draws every detected blob (so a flickering or wrong-color match is
    visible), highlights whichever one is currently locked, and overlays the
    tracker's own state (lost/tracking, distance-to-target) so it's obvious
    *why* the arm is or isn't moving, not just where the cube is.
    """
    out = frame.copy()
    for det in detections:
        bgr = COLOR_BGR.get(det.color, UNKNOWN_COLOR_BGR)
        draw_cube_marker(out, det.px, det.py, bgr, det.color)
    if match is not None:
        draw_lock_ring(out, match.px, match.py, LOCK_BGR)

    state = "LOST" if lost_since is not None else "tracking"
    state_bgr = LOST_BGR if lost_since is not None else LOCK_BGR
    lines = [
        f"{color}: {state}",
        f"dist={dist:.1f}mm" if dist is not None else "dist=n/a",
    ]
    for i, line in enumerate(lines):
        y = 24 + i * 22
        draw_outlined_text(out, line, (10, y), scale=0.6, color=state_bgr)
    return out


def run_tracking_loop(
    ser,
    stream: FrameStream,
    calib: Calibration,
    color: str,
    *,
    seed: tuple[float, float, float],
    seed_tcp_xyz: tuple[float, float, float],
    hover_z: float,
    deadband_mm: float,
    lost_grace_s: float,
    verbose: bool,
    deadline: float | None = None,
    preview: bool = False,
    recorder: VideoRecorder | None = None,
    j1_center: int = J1_HOME_CENTER_STEPS,
    j2_pull: int = J2_HOME_PULLOFF_STEPS,
) -> None:
    buf: list[str] = [""]
    _t0, x0, y0 = seed
    current_tcp_x, current_tcp_y, current_tcp_z = seed_tcp_xyz

    last_known = (x0, y0)
    lost_since: float | None = None
    jogging = False
    last_sent_error: tuple[float, float, float] | None = None
    last_telemetry = 0.0

    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                print("Reached --max-seconds, stopping.")
                return
            if console_focused() and key_down("h"):
                if jogging:
                    send_quick(ser, "stop")
                    jogging = False
                    last_sent_error = None
                print("Homing (h)...")
                run_home(ser, buf, j1_center, j2_pull, verbose)
                new_tcp = report_firmware_lines(ser, buf, verbose)
                if new_tcp is not None:
                    current_tcp_x, current_tcp_y, current_tcp_z = new_tcp.x, new_tcp.y, new_tcp.z
                lost_since = time.monotonic()
                continue
            # min_advance=1 (not FrameStream's default of 2): the camera
            # only delivers ~10fps here, and the default requires waiting
            # for 2 full new frames past the call, which halves our
            # achievable rate to ~5fps when called every tick like this.
            # min_advance=1 accepts a frame that may have already been
            # mid-capture at call time (up to ~1 frame period, ~100ms,
            # staler) in exchange for roughly doubling the loop rate to
            # ~10Hz -- a good trade for a slowly hand-moved cube.
            frame = stream.fresh(min_advance=1)
            now = time.monotonic()
            detections = detect_cubes(frame, calib)

            gate = MAX_JUMP_MM if lost_since is None else REACQUIRE_GATE_MM
            match = match_detection(detections, color, last_known, gate)
            if match is not None:
                assert match.x is not None and match.y is not None
                if lost_since is not None:
                    print(f"Reacquired {color} cube at ({match.x:.1f}, {match.y:.1f})")
                last_known = (match.x, match.y)
                lost_since = None
            elif lost_since is None:
                lost_since = now

            if lost_since is not None and now - lost_since > lost_grace_s:
                if jogging:
                    send_quick(ser, "stop")
                    jogging = False
                    print(
                        f"Lost {color} cube -- holding position, watching for "
                        f"reacquire near ({last_known[0]:.1f}, {last_known[1]:.1f})"
                    )
                new_tcp = report_firmware_lines(ser, buf, verbose)
                if new_tcp is not None:
                    current_tcp_x, current_tcp_y, current_tcp_z = new_tcp.x, new_tcp.y, new_tcp.z
                if _emit_preview(
                    frame, detections, match, color, lost_since, None,
                    preview=preview, recorder=recorder,
                ):
                    print("Preview window closed (q/Esc) -- stopping.")
                    return
                continue

            target_x, target_y = clamp_to_envelope(*last_known)

            # `pos` derives the same `tcp ...` line as `?` (both come from
            # print_joint_pos()'s live forward-kinematics computation) but
            # skips `?`'s multi-line status dump (mode, limits, gripper,
            # header/footer) that we don't need every tick.
            send_quick(ser, "pos")  # fire-and-forget; picked up below whenever it lands
            new_tcp = report_firmware_lines(ser, buf, verbose)
            if new_tcp is not None:
                current_tcp_x, current_tcp_y, current_tcp_z = new_tcp.x, new_tcp.y, new_tcp.z

            err_x = target_x - current_tcp_x
            err_y = target_y - current_tcp_y
            err_z = hover_z - current_tcp_z
            dist = math.hypot(err_x, err_y)

            if dist < deadband_mm:
                if jogging:
                    send_quick(ser, "stop")
                    jogging = False
                    last_sent_error = None
            else:
                # Raw distance-scheduled target every tick -- the firmware
                # (opted into ramping via `cjramp` above) smooths this toward
                # whatever it's currently doing on its own steady clock, so
                # there's nothing left to step/cap here in Python.
                send_quick(ser, f"cjspeed {speed_for_distance(dist)}")

                # Every `cj` -- including a resend of an unchanged direction
                # -- re-arms the firmware's DDA and resets its per-axis step
                # accumulator (motion.cpp setup_cartesian_jog -> dda_arm ->
                # dda_clear_axes), which the firmware's own 40ms auto-refresh
                # (refresh_cartesian_jog_if_due) already does on its own for
                # an unchanged direction, so resending here on top of that
                # just adds extra resets without changing what the arm is
                # actually doing.
                #
                # Gate resends on how far the error vector's tip has moved
                # (mm) since the last cj actually sent, rather than on the
                # angle between them: by the law of cosines a tip-distance
                # threshold catches both a direction swing and a magnitude
                # change in one test, and unlike an angular threshold it
                # doesn't get oversensitive as the error shrinks near the
                # deadband (a tiny absolute wobble there is a large angle at
                # small radius, which would otherwise trigger constant
                # resends exactly where we want the least twitchiness).
                if last_sent_error is None:
                    direction_changed = True
                else:
                    lsx, lsy, lsz = last_sent_error
                    tip_shift = math.sqrt(
                        (err_x - lsx) ** 2 + (err_y - lsy) ** 2 + (err_z - lsz) ** 2
                    )
                    direction_changed = tip_shift >= CJ_UPDATE_DEADZONE_MM
                if direction_changed:
                    # Firmware `cj` parses direction components with sscanf
                    # %ld -- integers only, no decimals -- then normalizes
                    # the vector on-device, so only the ratio between
                    # components matters. Scale the mm error up before
                    # rounding so small errors don't collapse to 0 and lose
                    # their direction.
                    ix, iy, iz = (int(round(v * CJ_INT_SCALE)) for v in (err_x, err_y, err_z))
                    send_quick(ser, f"cj {ix} {iy} {iz}")
                    last_sent_error = (err_x, err_y, err_z)
                jogging = True

            if _emit_preview(
                frame, detections, match, color, lost_since, dist,
                preview=preview, recorder=recorder,
            ):
                print("Preview window closed (q/Esc) -- stopping.")
                return

            if verbose and now - last_telemetry > 1.0:
                print(
                    f"track: target=({target_x:.1f},{target_y:.1f}) "
                    f"tcp=({current_tcp_x:.1f},{current_tcp_y:.1f},{current_tcp_z:.1f}) "
                    f"dist={dist:.1f}",
                    file=sys.stderr,
                )
                last_telemetry = now
    finally:
        if preview:
            try:
                cv2.destroyWindow(PREVIEW_WINDOW)
            except cv2.error:
                pass
        if recorder is not None:
            recorder.close()


def _emit_preview(
    frame: np.ndarray,
    detections: list[CubeDetection],
    match: CubeDetection | None,
    color: str,
    lost_since: float | None,
    dist: float | None,
    *,
    preview: bool,
    recorder: VideoRecorder | None,
) -> bool:
    """Draw+record/show one frame if requested; True if the user hit q/Esc.

    The tracking loop itself already runs at camera rate (~10Hz, paced by
    ``stream.fresh()``), so recording just writes whatever this tick already
    annotated -- no separate background feed is needed the way stack_cubes.py
    needed one to keep updating through its multi-second blocking moves.
    """
    if not preview and recorder is None:
        return False
    annotated = draw_preview(
        frame, detections, match,
        color=color, lost_since=lost_since, dist=dist,
    )
    if recorder is not None:
        recorder.write(annotated)
    if not preview:
        return False
    cv2.imshow(PREVIEW_WINDOW, annotated)
    key = cv2.waitKey(1) & 0xFF
    return key in (27, ord("q"))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Servo the gripper to hover over and track a single moving cube"
    )
    parser.add_argument("--port", default=None, help="serial port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--camera", type=int, default=None, help="camera index (auto-detect if omitted)")
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument(
        "--hover-mm", type=float, default=DEFAULT_HOVER_MM,
        help="height above the table to hover (mm); default 50 (5cm)",
    )
    parser.add_argument("--deadband-mm", type=float, default=DEADBAND_MM)
    parser.add_argument("--lost-grace-s", type=float, default=LOST_GRACE_S)
    parser.add_argument("--j1-center", type=int, default=J1_HOME_CENTER_STEPS)
    parser.add_argument("--j2-pull", type=int, default=J2_HOME_PULLOFF_STEPS)
    parser.add_argument(
        "--max-seconds", type=float, default=None,
        help="exit cleanly (stop + close) after this many seconds; default runs until Ctrl+C",
    )
    parser.add_argument(
        "--preview", action="store_true",
        help="show a live annotated camera preview window (q or Esc to stop)",
    )
    parser.add_argument(
        "--record",
        default=None,
        help="record the live annotated preview to this video path (e.g. track_run.mp4)",
    )
    parser.add_argument(
        "--record-fps", type=float, default=10.0,
        help="playback-rate metadata for --record (default 10, matching the "
        "camera-paced tracking loop; does not change the loop's actual pace)",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    try:
        calib = load_calibration(Path(args.calib))
    except CalibrationError as exc:
        print(exc, file=sys.stderr)
        return 1

    camera_index = args.camera if args.camera is not None else DEFAULT_CAMERA_INDEX
    print("Opening camera...")
    try:
        stream = FrameStream(camera_index)
    except CameraError as exc:
        print(exc, file=sys.stderr)
        return 1

    try:
        return _run(args, calib, stream)
    finally:
        stream.close()


def _run(args: argparse.Namespace, calib: Calibration, stream: FrameStream) -> int:
    hover_z = calib.table_z + args.hover_mm
    client = Mt4Client(port=args.port, baud=args.baud)
    target: CubeDetection | None = None
    seed_tcp_xyz: tuple[float, float, float] | None = None
    seed: tuple[float, float, float] | None = None
    try:
        client.ensure_connected()
        ensure_homed(client)
        print("Homed. Looking for a cube to track...")
        target = find_initial_target(stream, calib, INITIAL_LOCK_TIMEOUT_S)
        if target is None:
            print(
                "No reachable cube found within the timeout -- place a cube "
                "on the table within the arm's reach and in camera view.",
                file=sys.stderr,
            )
            return 1
        assert target.x is not None and target.y is not None
        seed = (time.monotonic(), target.x, target.y)
        tx, ty = clamp_to_envelope(target.x, target.y)
        print(
            f"Locked onto {target.color} cube at ({tx:.1f}, {ty:.1f}); "
            f"moving to hover at z={hover_z:.1f}"
        )
        client.move_to(tx, ty, hover_z, speed_us=calib.approach_speed_us)
        tcp = client.get_tcp()
        seed_tcp_xyz = (tcp.x, tcp.y, tcp.z)
    except Mt4ClientError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        client.close()

    assert target is not None and seed is not None and seed_tcp_xyz is not None
    port = args.port
    baud = args.baud

    print("Switching to direct jog control for tracking. Ctrl+C to stop, H (this window/terminal focused) to re-home.")
    try:
        with open_serial(port, baud) as ser:
            try:
                await_firmware_alive(ser, port_label=port or "auto-detected port")
                send(ser, "orient off", wait=0.2)
                send(ser, f"cjramp {CJ_RAMP_STEP_US}", wait=0.2)
                drain_lines(ser, [""])
            except FirmwareNotReadyError as exc:
                print(exc, file=sys.stderr)
                return 1
            recorder = (
                VideoRecorder(video_path=args.record, fps=args.record_fps)
                if args.record
                else None
            )
            try:
                deadline = time.monotonic() + args.max_seconds if args.max_seconds else None
                run_tracking_loop(
                    ser, stream, calib, target.color,
                    seed=seed, seed_tcp_xyz=seed_tcp_xyz, hover_z=hover_z,
                    deadband_mm=args.deadband_mm,
                    lost_grace_s=args.lost_grace_s, verbose=args.verbose,
                    deadline=deadline, preview=args.preview, recorder=recorder,
                    j1_center=args.j1_center, j2_pull=args.j2_pull,
                )
            except KeyboardInterrupt:
                print()
            finally:
                try:
                    send_quick(ser, "stop")
                    # Leave the firmware in its default (un-ramped) state for
                    # whatever runs next on this port -- see CJ_RAMP_STEP_US.
                    send_quick(ser, "cjramp 0")
                except SerialGoneError:
                    pass
    except SerialGoneError as exc:
        print(exc, file=sys.stderr)
        return 1

    print("Bye")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
