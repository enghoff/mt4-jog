#!/usr/bin/env python3
"""Measure the wrist's face-align reference angle (j4_face_offset_deg).

J4 has no home switch: its zero is wherever the wrist sat at boot, and the
constant between "J4 = 0" and the physical jaw direction in the robot frame
is unobservable from step counting alone. This script measures it visually,
using the gripper's own strongly asymmetric shape:

  1. Park the TCP on the arm's axis (j1 = 0 -> axis = robot x-axis). The
     overhead camera sits nearly on that axis, so the axis points almost
     straight at the lens.
  2. Sweep J4 across its range while processing the video stream live:
     one slow constant-velocity move per direction (firmware ramp off at
     speed_us >= 1800), each frame's angle recovered by timestamp
     interpolation between move start and the completion ack. Both
     directions are merged, cancelling serial-timing skew and centering
     J4 backlash. (--stepped falls back to move-settle-capture.)
  3. In a fixed image band where the fingers hang against the light desk,
     measure the dark silhouette's horizontal extent. The flat gripper is
     narrowest when its plane contains the viewing/arm axis (edge-on) --
     a 180-degree-periodic curve with two minima.
  4. Fold the sweep into mod-180 space (merging both minima), smooth, and
     take the argmin: the edge-on angle theta*.

At edge-on the jaw gap lies along the arm axis (robot x), i.e. commanding
world J4 = theta* grips a face-aligned (edge-yaw-0) cube on its +-x faces,
so j4_face_offset_deg = fold90(theta*). The result is written to the
calibration; face_align_picks is deliberately left untouched -- validate
with a real grip (held cube's visual yaw vs commanded J4) before enabling.
On completion the gripper is left at world J4 = theta*, i.e. aligned with
the arm axis (the wrist-preserving retreat keeps that yaw at the park).

Caveats:
  - The camera must sit approximately on the robot x-axis (it does for this
    mount); azimuth error adds directly to the measured angle.
  - The analysis band (--band) is camera-pose-specific. Run with
    --debug-frame first after any camera move: it saves a gridded overlay
    (j4_band_debug.png next to the calibration) to pick band coordinates
    where the fingers show against the desk, clear of the wrist bracket,
    cast shadows, and desk clutter.
  - The metric only needs the table-plane homography (for nothing, in
    fact, beyond operator sanity) -- it is robust to an invalid cube-top
    calibration, so it can run right after a camera move.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration
from mt4_vision.camera import FrameStream
from mt4_vision.pickplace import (
    _travel,
    ensure_homed,
    fold_square_yaw_deg,
    retreat_for_camera,
)

# Defaults measured for the 2026-07-20 closer front mount.
DEFAULT_POSE = (230.0, 0.0, 215.0)
DEFAULT_BAND = (450, 520, 480, 820)  # y0 y1 x0 x1
DARK_V_MAX = 70.0
MIN_DARK_PX = 200
SETTLE_S = 0.45


def band_width(frame: np.ndarray, band: tuple[int, int, int, int]) -> float | None:
    y0, y1, x0, x1 = band
    hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    dark = (hsv[:, :, 2] < DARK_V_MAX).astype(np.uint8)
    dark = cv2.morphologyEx(
        dark, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    xs = np.nonzero(dark)[1]
    if xs.size < MIN_DARK_PX:
        return None
    lo, hi = np.percentile(xs, [2.0, 98.0])
    return float(hi - lo)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="")
    parser.add_argument("--camera", type=int, default=-1)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument("--pose", type=float, nargs=3, default=DEFAULT_POSE,
                        metavar=("X", "Y", "Z"),
                        help="TCP pose for the sweep; y must be 0 so the arm "
                             "axis is the x-axis")
    parser.add_argument("--band", type=int, nargs=4, default=DEFAULT_BAND,
                        metavar=("Y0", "Y1", "X0", "X1"),
                        help="image band (px) where the fingers show against "
                             "the desk; re-derive with --debug-frame after "
                             "any camera move")
    parser.add_argument("--step", type=float, default=3.0)
    parser.add_argument("--range", type=float, default=100.0,
                        help="sweep +-range degrees (default 100 covers the "
                             "full 180-degree silhouette period with "
                             "margin; the wrist soft limit is ~141)")
    parser.add_argument("--stepped", action="store_true",
                        help="fallback: move-settle-capture at --step "
                             "increments instead of the continuous sweep")
    parser.add_argument("--sweep-speed-us", type=int, default=2400,
                        help="continuous-sweep step period; must be >=1800 "
                             "so the firmware ramp stays off and the "
                             "angular rate is constant")
    parser.add_argument("--debug-frame", action="store_true",
                        help="only park at the pose and save a gridded "
                             "dark-mask overlay for picking --band")
    parser.add_argument("--dry-run", action="store_true",
                        help="measure and report, but don't write the "
                             "calibration")
    args = parser.parse_args()

    if abs(args.pose[1]) > 1e-6:
        print("error: pose y must be 0 (the sweep needs j1 = 0 so the arm "
              "axis is the x-axis)", file=sys.stderr)
        return 1
    if not args.stepped and args.sweep_speed_us < 1800:
        print("error: --sweep-speed-us must be >=1800 for the continuous "
              "sweep -- below that the firmware accel ramp engages and the "
              "constant-angular-rate assumption behind the timestamp->angle "
              "mapping breaks", file=sys.stderr)
        return 1

    calib = load_calibration(Path(args.calib))
    client = Mt4Client() if not args.port else Mt4Client(port=args.port)
    cam = FrameStream(args.camera)
    try:
        client.ensure_connected()
        ensure_homed(client)
        _travel(client, calib, *args.pose, "sweep pose", j4=0.0)
        time.sleep(1.0)

        if args.debug_frame:
            f = cam.fresh()
            hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
            vis = f.copy()
            vis[hsv[:, :, 2] < DARK_V_MAX] = (0, 255, 255)
            for y in range(0, f.shape[0], 50):
                cv2.line(vis, (0, y), (f.shape[1] - 1, y), (0, 0, 255), 1)
                cv2.putText(vis, str(y), (5, y - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            for x in range(0, f.shape[1], 100):
                cv2.line(vis, (x, 0), (x, f.shape[0] - 1), (255, 0, 0), 1)
                cv2.putText(vis, str(x), (x + 2, f.shape[0] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            y0, y1, x0, x1 = args.band
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
            out = Path(args.calib).parent / "j4_band_debug.png"
            cv2.imwrite(str(out), vis)
            print(f"saved {out} -- pick --band so the fingers (dark) show "
                  "against the desk, clear of bracket/shadows/cubes")
            retreat_for_camera(client, calib)
            return 0

        band = tuple(args.band)
        rows: list[tuple[float, float]] = []
        if args.stepped:
            angles = np.arange(-args.range, args.range + 0.01, args.step)
            _travel(client, calib, *args.pose, "sweep start",
                    j4=float(angles[0]))
            time.sleep(1.0)
            for j4 in angles:
                _travel(client, calib, *args.pose, f"j4={j4:.0f}",
                        j4=float(j4))
                time.sleep(SETTLE_S)
                w = band_width(cam.fresh(), band)
                print(f"j4={j4:+7.1f}  width="
                      f"{'n/a' if w is None else format(w, '6.1f') + 'px'}")
                if w is not None:
                    rows.append((float(j4), w))
        else:
            # Continuous sweep: one slow constant-velocity J4 move per
            # direction while the video stream is processed live, each
            # frame's angle recovered by timestamp interpolation between
            # move start and the firmware's completion ack. speed_us must
            # stay >= 1800 so the firmware ramp is OFF and the angular rate
            # really is constant. Sweeping both directions and merging
            # cancels the residual timing skew (serial latency shifts the
            # two passes in opposite angular directions) and centers J4
            # backlash.
            x, y, z = args.pose
            for a0, a1 in ((-args.range, args.range),
                           (args.range, -args.range)):
                _travel(client, calib, x, y, z, "sweep start", j4=float(a0))
                time.sleep(1.0)
                span = a1 - a0
                est_s = abs(span) * 45.0 * args.sweep_speed_us * 1e-6
                done: list[float] = []
                err: list[Exception] = []

                def mover():
                    try:
                        client.move_to(x, y, z, j4=float(a1),
                                       speed_us=args.sweep_speed_us,
                                       timeout=est_s + 30.0)
                        done.append(time.time())
                    except Exception as exc:  # surfaced after join
                        err.append(exc)

                t0 = time.time()
                th = threading.Thread(target=mover, daemon=True)
                th.start()
                pass_rows: list[tuple[float, float]] = []
                while th.is_alive():
                    try:
                        f = cam.fresh(min_advance=1, timeout_s=2.0)
                    except Exception:
                        continue
                    t = time.time()
                    w = band_width(f, band)
                    if w is not None:
                        pass_rows.append((t, w))
                th.join()
                if err:
                    raise err[0]
                t1 = done[0] if done else time.time()
                # Trim samples outside the actual motion window.
                for t, w in pass_rows:
                    frac = (t - t0) / max(t1 - t0, 1e-6)
                    if 0.02 <= frac <= 0.98:
                        rows.append((a0 + span * frac, w))
                print(f"pass {a0:+.0f} -> {a1:+.0f}: "
                      f"{len(pass_rows)} frames in {t1 - t0:.1f}s")

        if len(rows) < 20:
            print("error: too few valid measurements -- check --band "
                  "(run --debug-frame)", file=sys.stderr)
            _travel(client, calib, *args.pose, "sweep end", j4=0.0)
            retreat_for_camera(client, calib)
            return 1

        # Fold into mod-180 space (the silhouette is 180-periodic, so any
        # twin minima merge into one well-sampled dip) and bin, so stepped
        # and continuous data feed the same estimator.
        bin_deg = max(args.step, 1.0) if args.stepped else 1.5
        folded: dict[float, list[float]] = {}
        for j4, w in rows:
            a = ((j4 + 90.0) % 180.0) - 90.0
            folded.setdefault(round(a / bin_deg) * bin_deg, []).append(w)
        fa = np.array(sorted(folded))
        fw = np.array([float(np.mean(folded[a])) for a in fa])
        k = 5
        pad = k // 2
        if fa[-1] - fa[0] >= 180.0 - 2 * bin_deg:
            # Full period sampled: smooth periodically across the wrap.
            fw_ext = np.concatenate([fw[-pad:], fw, fw[:pad]])
            fsm = np.convolve(fw_ext, np.ones(k) / k, mode="valid")
        else:
            # Partial window (e.g. the default +-60 sweep): no wrap --
            # smooth with edge replication instead.
            fw_ext = np.concatenate([np.full(pad, fw[0]), fw,
                                     np.full(pad, fw[-1])])
            fsm = np.convolve(fw_ext, np.ones(k) / k, mode="valid")
        i = int(np.argmin(fsm))
        theta = float(fa[i])
        offset = fold_square_yaw_deg(theta)
        print(f"\nedge-on (jaws along the arm axis) at theta* = {theta:+.1f} deg "
              f"(width {fw[i]:.1f}px)")
        print(f"j4_face_offset_deg = fold90(theta*) = {offset:+.1f}")
        if i <= 1 or i >= len(fa) - 2:
            print("WARNING: the minimum sits at the edge of the sweep "
                  "window -- the true edge-on angle may lie outside it. "
                  "Re-run with a larger --range.", file=sys.stderr)

        # Leave the gripper aligned with the arm axis (jaws along robot x).
        # The wrist-preserving retreat keeps this world yaw at the park.
        _travel(client, calib, *args.pose, "align with arm axis",
                j4=float(theta))
        retreat_for_camera(client, calib)
        print(f"gripper left aligned with the arm axis (world J4 = "
              f"{theta:+.1f} deg)")

        if args.dry_run:
            print("--dry-run: not writing")
            return 0
        calib.j4_face_offset_deg = float(round(offset, 1))
        calib.save(Path(args.calib))
        print(f"saved to {args.calib} (face_align_picks left "
              f"{'ON' if calib.face_align_picks else 'OFF'} -- validate with "
              "a real grip before changing it)")
        return 0
    except Mt4ClientError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        cam.close()
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
