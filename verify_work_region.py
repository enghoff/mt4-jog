#!/usr/bin/env python3
"""In-envelope I/K/U/O acceptance test for the MT4's intended workspace.

The earlier tests (verify_ik_jog.py) jogged from the home pose (z~256mm)
toward extremes: full extension r~330mm near the straight-arm singularity,
z~350mm up high. That is NOT where the arm is meant to operate. The intended
workspace is the end effector close to the base and low over the desk.

This test:
  1. Homes (the only physical reference available).
  2. Transits to a work-center pose (r=185mm, z=115mm) via bounded O/K jogs.
  3. Exercises I/K/U/O in small +/-15mm excursions around the work center,
     never leaving the intended envelope.
  4. Checks every telemetry sample against the independent closed-form IK
     (from verify_ik_jog).
  5. Ends by re-centering to the SAME counter position as the first center
     visit and photographing both. Open-loop step counters cannot see lost
     steps at all -- a skipped motor leaves the counters internally
     consistent while the physical arm drifts -- so the photo pair (same
     commanded pose, before vs after all excursions) is the only real
     lost-step canary. The test reports a pixel-difference score between the
     two frames; a large score at matched counters means physical drift.

Joint-angle guards abort any segment that leaves the envelope; every stop
path sends `stop` to the firmware.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

from mt4_jog.joints import (
    DEFAULT_BAUD,
    DEFAULT_PORT,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
)
from mt4_jog.serial import open_serial, read_lines, send, send_quick
from verify_ik_jog import _wrap_deg, fk, query_steps, run_home, solve_q2_q3, steps_to_pose

# Intended operating envelope (model degrees / mm). Work-region bounds are
# tight; transit bounds additionally allow the path down from the home pose.
WORK_GUARDS = {
    "q2": (25.0, 95.0),
    # J3's true mechanical limit is unknown; 70 deg is the steepest forearm
    # angle the arm has already demonstrably reached (conservative per user).
    # Range mirrored 2026-07-06: J3_STEP_SIGN was found inverted (photo-
    # confirmed), so what was reported as q3 in [-70,5] is physically q3 in
    # [-5,70] under the corrected sign.
    "q3": (-5.0, 70.0),
    "sep": (30.0, 165.0),  # q2-q3: away from straight (0) and folded (180)
    "r": (140.0, 245.0),
    "z": (60.0, 185.0),
}
TRANSIT_GUARDS = {**WORK_GUARDS, "z": (60.0, 270.0)}

WORK_R = 185.0
WORK_Z = 125.0  # keeps the O-down excursion ~3.5 deg above the q3 guard
EXCURSION_MM = 15.0
TOL_MM = 5.0
SAMPLE_DT = 0.12
SEG_TIMEOUT_S = 20.0
ANGLE_TOL_DEG = 4.0
REPEAT_TOL_DEG = 1.5


@dataclass
class Snap:
    t: float
    pose_q2: float
    pose_q3: float
    q1: float
    r: float
    z: float


def snapshot(ser, t0: float) -> Snap:
    j1, j2, j3, j4 = query_steps(ser)
    pose = steps_to_pose(j1, j2, j3)
    x, y, z = fk(pose)
    return Snap(time.monotonic() - t0, pose.q2, pose.q3, pose.q1, math.hypot(x, y), z)


def guard_violation(s: Snap, guards: dict) -> str | None:
    checks = [
        ("q2", s.pose_q2),
        ("q3", s.pose_q3),
        ("sep", s.pose_q2 - s.pose_q3),
        ("r", s.r),
        ("z", s.z),
    ]
    for name, val in checks:
        lo, hi = guards[name]
        if not (lo <= val <= hi):
            return f"{name}={val:.1f} outside [{lo}, {hi}]"
    return None


def jog_until(ser, cj_cmd: str, reached, guards: dict, label: str) -> tuple[list[Snap], str]:
    """Drive cj_cmd until `reached(r, z)`, a guard trips, or timeout.

    Querying status (`?`) halts the active jog in firmware, so motion happens
    in short bursts between samples; each cj resend re-solves the IK from the
    live joint state.
    """
    samples: list[Snap] = []
    t0 = time.monotonic()
    status = "timeout"
    try:
        while time.monotonic() - t0 < SEG_TIMEOUT_S:
            send_quick(ser, cj_cmd)
            time.sleep(SAMPLE_DT)
            s = snapshot(ser, t0)
            samples.append(s)
            viol = guard_violation(s, guards)
            if viol:
                status = f"guard: {viol}"
                break
            if reached(s.r, s.z):
                status = "target"
                break
    finally:
        send_quick(ser, "stop")
        time.sleep(0.05)
    return samples, status


CENTER_TOL_MM = 3.0


def fine_center(ser, label: str) -> Snap | None:
    """Converge the TCP to (WORK_R, WORK_Z) within CENTER_TOL_MM, one axis
    per burst. Returns the settled snapshot, or None if it fails to converge.
    """
    t0 = time.monotonic()
    for _ in range(40):
        s = snapshot(ser, t0)
        dr = s.r - WORK_R
        dz = s.z - WORK_Z
        if abs(dr) <= CENTER_TOL_MM and abs(dz) <= CENTER_TOL_MM:
            send_quick(ser, "stop")
            time.sleep(0.05)
            return s
        if abs(dr) >= abs(dz):
            send_quick(ser, "cj -1 0 0" if dr > 0 else "cj 1 0 0")
        else:
            send_quick(ser, "cj 0 0 -1" if dz > 0 else "cj 0 0 1")
        time.sleep(SAMPLE_DT / 2)
    send_quick(ser, "stop")
    print(f"  {label}: fine-centering did not converge", file=sys.stderr)
    return None


def photo_diff_score(path_a, path_b) -> float | None:
    """Mean absolute grayscale difference between two frames (0-255)."""
    try:
        import cv2
    except ImportError:
        return None
    a = cv2.imread(str(path_a), cv2.IMREAD_GRAYSCALE)
    b = cv2.imread(str(path_b), cv2.IMREAD_GRAYSCALE)
    if a is None or b is None or a.shape != b.shape:
        return None
    return float(cv2.absdiff(a, b).mean())


def ik_worst_error(samples: list[Snap]) -> tuple[float, int]:
    """Max |observed - closed-form-IK| joint angle over samples (deg)."""
    worst = 0.0
    unreachable = 0
    for s in samples:
        sol = solve_q2_q3(s.r, s.z, s.pose_q2, s.pose_q3)
        if sol is None:
            unreachable += 1
            continue
        worst = max(worst, abs(_wrap_deg(s.pose_q2 - sol[0])), abs(_wrap_deg(s.pose_q3 - sol[1])))
    return worst, unreachable


def main() -> int:
    parser = argparse.ArgumentParser(description="MT4 in-envelope work-region test")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--camera", type=int, default=None, help="camera index; omit to disable photos")
    args = parser.parse_args()

    cam = None
    photo_dir = None
    if args.camera is not None:
        from capture_camera import DEFAULT_OUTDIR, open_camera, save_frame

        photo_dir = DEFAULT_OUTDIR
        cam = open_camera(args.camera)
        if not cam.isOpened():
            print(f"Could not open camera {args.camera}", file=sys.stderr)
            return 1

    def photo(label: str) -> None:
        if cam is not None:
            from capture_camera import save_frame

            save_frame(cam, photo_dir, label)

    results: list[tuple[str, str, float]] = []  # (segment, status, worst IK err)
    center_first: Snap | None = None
    center_last: Snap | None = None

    try:
        with open_serial(args.port, args.baud) as ser:
            time.sleep(1.0)
            read_lines(ser, 1.0)
            send(ser, "all f", wait=0.3)
            run_home(ser, J1_HOME_CENTER_STEPS, J2_HOME_PULLOFF_STEPS)
            photo("wr_00_home")

            # --- Transit: down, then in, to the work center. ---
            plan = [
                ("transit-down", "cj 0 0 -1", lambda r, z: z <= WORK_Z + TOL_MM, TRANSIT_GUARDS),
                ("transit-in", "cj -1 0 0", lambda r, z: r <= WORK_R + TOL_MM, TRANSIT_GUARDS),
                # --- Work-region excursions (+/-15mm, always inside envelope). ---
                ("I-out", "cj 1 0 0", lambda r, z: r >= WORK_R + EXCURSION_MM - TOL_MM, WORK_GUARDS),
                ("K-in", "cj -1 0 0", lambda r, z: r <= WORK_R - EXCURSION_MM + TOL_MM, WORK_GUARDS),
                ("I-back", "cj 1 0 0", lambda r, z: r >= WORK_R - TOL_MM, WORK_GUARDS),
                ("U-up", "cj 0 0 1", lambda r, z: z >= WORK_Z + EXCURSION_MM - TOL_MM, WORK_GUARDS),
                ("O-down", "cj 0 0 -1", lambda r, z: z <= WORK_Z - EXCURSION_MM + TOL_MM, WORK_GUARDS),
                ("U-back", "cj 0 0 1", lambda r, z: z >= WORK_Z - TOL_MM, WORK_GUARDS),
            ]

            aborted = False
            for i, (label, cmd, reached, guards) in enumerate(plan):
                print(f"\n== {label} ({cmd}) ==")
                samples, status = jog_until(ser, cmd, reached, guards, label)
                worst, unreachable = ik_worst_error(samples)
                for s in samples:
                    print(
                        f"  t={s.t:5.2f} q2={s.pose_q2:7.2f} q3={s.pose_q3:7.2f} "
                        f"r={s.r:7.2f} z={s.z:7.2f}"
                    )
                print(f"  -> {status}, worst IK err {worst:.2f} deg"
                      + (f", {unreachable} unreachable" if unreachable else ""))
                results.append((label, status, worst))
                photo(f"wr_{i + 1:02d}_{label}")
                if status.startswith("guard"):
                    print("  guard tripped -- stopping test here for safety")
                    aborted = True
                    break
                # Lost-step canary reference: settle exactly on the work
                # center once transit finishes, and again after the last
                # excursion, photographing both at matched counters.
                if label == "transit-in":
                    center_first = fine_center(ser, "center-ref")
                    if center_first:
                        photo("wr_center_ref")
                if label == "U-back":
                    center_last = fine_center(ser, "center-return")
                    if center_last:
                        photo("wr_center_return")

            send_quick(ser, "stop")
            send(ser, "all f", wait=0.3)
    finally:
        if cam is not None:
            cam.release()

    print("\n== Results ==")
    failed = 0
    for label, status, worst in results:
        ok = status == "target" and worst <= ANGLE_TOL_DEG
        if not ok:
            failed += 1
        print(f"[{'PASS' if ok else 'FAIL'}] {label}: {status}, worst IK err {worst:.2f} deg")

    if center_first and center_last and not aborted:
        # The two settles may sit anywhere within CENTER_TOL_MM of the
        # target, and a few mm legitimately shifts q2/q3 by degrees -- so
        # compare the RESIDUAL after subtracting the IK-predicted angles for
        # each settle's own (r, z). The PHYSICAL drift check is the photo
        # pair; counters cannot see lost steps.
        conv_ok = False
        sol_a = solve_q2_q3(center_first.r, center_first.z, center_first.pose_q2, center_first.pose_q3)
        sol_b = solve_q2_q3(center_last.r, center_last.z, center_last.pose_q2, center_last.pose_q3)
        if sol_a and sol_b:
            rq2 = abs(_wrap_deg((center_last.pose_q2 - center_first.pose_q2) - (sol_b[0] - sol_a[0])))
            rq3 = abs(_wrap_deg((center_last.pose_q3 - center_first.pose_q3) - (sol_b[1] - sol_a[1])))
            conv_ok = rq2 <= REPEAT_TOL_DEG and rq3 <= REPEAT_TOL_DEG
            line = f"center re-visit consistent: residual dq2={rq2:.2f} deg dq3={rq3:.2f} deg (counters)"
        else:
            line = "center re-visit: IK unreachable at a settle point"
        if not conv_ok:
            failed += 1
        print(f"[{'PASS' if conv_ok else 'FAIL'}] {line}")
        if photo_dir is not None:
            score = photo_diff_score(
                photo_dir / "wr_center_ref.jpg", photo_dir / "wr_center_return.jpg"
            )
            if score is not None:
                print(
                    f"[INFO] lost-step canary: photo diff score {score:.2f} "
                    f"(mean gray delta 0-255; same counters before/after all "
                    f"excursions -- a high score means the arm physically "
                    f"drifted, i.e. skipped steps)"
                )
    else:
        failed += 1
        print("[FAIL] center repeatability: test did not complete both center visits")

    if photo_dir is not None:
        print(f"Photos: {photo_dir}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
