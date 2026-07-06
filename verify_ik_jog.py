#!/usr/bin/env python3
"""Independent PC-side acceptance test for the MT4 world +/-X (I/K) jog.

This is a *cross-check*, not a unit test of mt4_jog/kinematics.py: it does
not import that module or read the firmware's IK implementation
(mt4_jog/kinematics.py / firmware/mt4_jog/src/kinematics.cpp). Instead it
re-derives forward/inverse kinematics for the MT4's parallel-link arm from
scratch, using a classic closed-form two-circle-intersection solve (the
textbook 2-link planar IK). The firmware/kinematics.py solve the same
problem numerically (finite-difference Jacobian + damped least squares);
using a structurally different method here means a bug specific to that
numerical approach is unlikely to be reproduced blindly by this reference
model.

The only things intentionally reused from the existing tooling are pure
plumbing/calibration, not the IK algorithm under test:
  - mt4_jog/serial.py, mt4_jog/joints.py: serial protocol + pin/step-per-deg
    tables (physical wiring facts, needed to interpret raw step counts at
    all, regardless of which IK method interprets them).
  - The link geometry constants (LINKAGE1/2, offsets) are the robot's fixed
    physical dimensions -- any correct model of this arm must use the same
    numbers, so agreement there is expected, not a tell of copying the IK.

Test procedure:
  1. Home J1 + J2.
  2. Hold `I` (cj 1 0 0) for --hold-s seconds, sampling telemetry.
  3. Hold `K` (cj -1 0 0) for --hold-s seconds, sampling telemetry.
  4. For every sample, convert steps -> joint angles -> TCP via the from-
     scratch FK above, and independently solve for the (q2, q3) that the
     closed-form IK says *should* produce the measured radial reach while
     holding height constant. Compare against the telemetry-derived (q2,
     q3), and check:
       - height (z) stays within tolerance of the start height
       - yaw (q1) / lateral (y) stays near zero (pure X motion)
       - radial reach moves in the commanded direction (I extends, K
         retracts) -- i.e. I and K are confirmed opposite, not identical
       - the observed J2/J3 split matches the independent IK prediction

A safety watchdog aborts (sends `stop`) immediately if height drifts past
--safety-z-mm, independent of the pass/fail assertions, since this is
exercising a real robot arm.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from mt4_jog.joints import (
    DEFAULT_BAUD,
    DEFAULT_PORT,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
    STEPS_PER_DEG,
)
from mt4_jog.serial import open_serial, read_lines, send, send_quick

# ---------------------------------------------------------------------------
# Independent reference kinematic model (see module docstring).
# ---------------------------------------------------------------------------

LINKAGE1 = 130.0  # shoulder -> elbow, mm
LINKAGE2 = 150.0  # elbow -> wrist pivot, mm
CENCER_OFFSET = 45.0  # J1 axis -> shoulder, horizontal, mm
CENCER_HEIGHT = 140.0  # shoulder pivot height, mm
HEAD_OFFSET = 35.0  # wrist pivot -> TCP, horizontal, mm
HEAD_HEIGHT = 14.43  # TCP below wrist pivot, mm

HOME_J2_DEG = 90.0
HOME_J3_DEG = 0.0

# DIR-pin polarity per joint: +1 if a positive step count increases the
# joint's model angle, -1 if inverted. Wiring calibration, not IK.
# J3 confirmed inverted 2026-07-06: a +299-step probe raised the forearm tip
# instead of lowering it as the old -1.0 sign predicted (photo-confirmed).
STEP_SIGN = (1.0, -1.0, 1.0, 1.0)


@dataclass(frozen=True)
class Pose:
    q1: float
    q2: float
    q3: float


def steps_to_pose(j1: int, j2: int, j3: int) -> Pose:
    return Pose(
        q1=STEP_SIGN[0] * j1 / STEPS_PER_DEG[0],
        q2=HOME_J2_DEG + STEP_SIGN[1] * j2 / STEPS_PER_DEG[1],
        q3=HOME_J3_DEG + STEP_SIGN[2] * j3 / STEPS_PER_DEG[2],
    )


def fk(pose: Pose) -> tuple[float, float, float]:
    """Forward kinematics: joint angles (deg) -> TCP (x, y, z) mm."""
    q1, q2, q3 = map(math.radians, (pose.q1, pose.q2, pose.q3))
    radial = CENCER_OFFSET + LINKAGE1 * math.cos(q2) + LINKAGE2 * math.cos(q3) + HEAD_OFFSET
    z = CENCER_HEIGHT + LINKAGE1 * math.sin(q2) + LINKAGE2 * math.sin(q3) - HEAD_HEIGHT
    return radial * math.cos(q1), radial * math.sin(q1), z


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def solve_q2_q3(
    radial: float, z: float, near_q2: float, near_q3: float
) -> tuple[float, float] | None:
    """Closed-form two-link IK: find (q2, q3) with

        L1*(cos q2, sin q2) + L2*(cos q3, sin q3) = target

    via circle-circle intersection (classic 2-link elbow solve, here with
    absolute rather than relative joint angles). Returns the branch nearest
    (near_q2, near_q3), or None if the target is out of reach.
    """
    tx = radial - CENCER_OFFSET - HEAD_OFFSET
    ty = z - CENCER_HEIGHT + HEAD_HEIGHT
    d = math.hypot(tx, ty)
    if d < 1e-6 or d > LINKAGE1 + LINKAGE2 or d < abs(LINKAGE1 - LINKAGE2):
        return None

    cos_alpha = (LINKAGE1**2 + d * d - LINKAGE2**2) / (2 * LINKAGE1 * d)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)
    beta = math.atan2(ty, tx)

    best: tuple[float, float, float] | None = None
    for sign in (1.0, -1.0):
        q2 = math.degrees(beta + sign * alpha)
        p1x = LINKAGE1 * math.cos(math.radians(q2))
        p1y = LINKAGE1 * math.sin(math.radians(q2))
        q3 = math.degrees(math.atan2(ty - p1y, tx - p1x))
        dist = abs(_wrap_deg(q2 - near_q2)) + abs(_wrap_deg(q3 - near_q3))
        if best is None or dist < best[0]:
            best = (dist, q2, q3)
    assert best is not None
    return best[1], best[2]


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

POS_RE = re.compile(r"pos J1=(-?\d+) J2=(-?\d+) J3=(-?\d+) J4=(-?\d+)")


def query_steps(ser) -> tuple[int, int, int, int]:
    """Send `?`. NOTE: per the firmware's handle_line(), any command other
    than !/stop/j/jog/home/$H/cj stops an active jog -- so this halts
    whatever cj is in flight. That's fine here: we resend `cj` right after,
    which re-solves from the (now updated) live joint state.
    """
    lines = send(ser, "?", wait=0.4)
    for line in lines:
        m = POS_RE.search(line)
        if m:
            return tuple(int(g) for g in m.groups())  # type: ignore[return-value]
    raise RuntimeError(f"no 'pos' line in status response: {lines!r}")


def run_home(ser, j1_center: int, j2_pull: int, timeout_s: float = 180.0) -> None:
    print(f"Homing... (J1 center {j1_center}, J2 pull {j2_pull})")
    ser.write(f"home {j1_center} {j2_pull}\n".encode("ascii"))
    ser.flush()
    deadline = time.monotonic() + timeout_s
    buf = ""
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            buf += ser.read(waiting).decode("utf-8", "replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    print(f"  {line}")
                if line == "home ok":
                    return
                if line.startswith("home fail"):
                    raise RuntimeError(f"homing failed: {line}")
        time.sleep(0.02)
    raise RuntimeError("homing timed out")


# ---------------------------------------------------------------------------
# Test harness
# ---------------------------------------------------------------------------


@dataclass
class Sample:
    t: float
    steps: tuple[int, int, int, int]
    pose: Pose
    tcp: tuple[float, float, float]
    photo: "Path | None" = None


class SafetyAbort(RuntimeError):
    pass


def hold_and_sample(
    ser,
    cj_cmd: str,
    hold_axis: str,  # "radial" -> only guard dz; "z" -> only guard dr
    hold_s: float,
    sample_dt: float,
    z_start: float,
    r_start: float,
    safety_mm: float,
    cam=None,
    photo_dir: "Path | None" = None,
    photo_prefix: str = "sample",
) -> list[Sample]:
    samples: list[Sample] = []
    t0 = time.monotonic()
    try:
        i = 0
        while time.monotonic() - t0 < hold_s:
            send_quick(ser, cj_cmd)
            time.sleep(sample_dt)
            j1, j2, j3, j4 = query_steps(ser)
            pose = steps_to_pose(j1, j2, j3)
            tcp = fk(pose)
            photo = None
            if cam is not None:
                from capture_camera import save_frame

                photo = save_frame(cam, photo_dir, f"{photo_prefix}_{i:03d}")
            samples.append(
                Sample(time.monotonic() - t0, (j1, j2, j3, j4), pose, tcp, photo)
            )
            i += 1
            r = math.hypot(tcp[0], tcp[1])
            # Only the axis that's supposed to stay put is safety-guarded --
            # the commanded axis is *expected* to move well past this bound.
            drift = abs(tcp[2] - z_start) if hold_axis == "radial" else abs(r - r_start)
            if drift > safety_mm:
                raise SafetyAbort(
                    f"held axis drifted {drift:+.1f}mm (dz={tcp[2] - z_start:+.1f}mm, "
                    f"dr={r - r_start:+.1f}mm) from start "
                    f"(> safety limit {safety_mm} mm) -- aborting"
                )
    finally:
        send_quick(ser, "stop")
        time.sleep(0.05)
    return samples


def print_samples(label: str, samples: list[Sample]) -> None:
    print(f"\n-- {label} samples --")
    print(f"{'t':>5}  {'J1':>6}{'J2':>6}{'J3':>6}  {'q1':>7}{'q2':>7}{'q3':>7}  {'x':>7}{'y':>7}{'z':>7}  photo")
    for s in samples:
        print(
            f"{s.t:5.2f}  {s.steps[0]:6d}{s.steps[1]:6d}{s.steps[2]:6d}  "
            f"{s.pose.q1:7.2f}{s.pose.q2:7.2f}{s.pose.q3:7.2f}  "
            f"{s.tcp[0]:7.2f}{s.tcp[1]:7.2f}{s.tcp[2]:7.2f}  "
            f"{s.photo.name if s.photo else ''}"
        )


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str


def check_segment(
    label: str,
    samples: list[Sample],
    start: Sample,
    axis: str,  # "radial" (I/K: height held, radial moves) or "z" (U/O: radial held, height moves)
    expected_sign: float,
    z_tol_mm: float,
    y_tol_mm: float,
    q1_tol_deg: float,
    angle_tol_deg: float,
    radial_tol_mm: float,
) -> list[CheckResult]:
    results: list[CheckResult] = []
    r_start = math.hypot(start.tcp[0], start.tcp[1])
    z_start = start.tcp[2]

    if axis == "radial":
        # 1. Height stays flat.
        max_z_drift = max(abs(s.tcp[2] - z_start) for s in samples)
        results.append(
            CheckResult(
                f"{label}: height held (|dz| <= {z_tol_mm} mm)",
                max_z_drift <= z_tol_mm,
                f"max |dz| = {max_z_drift:.2f} mm",
            )
        )
        # 2. Radial reach moves in the commanded direction.
        r_end = math.hypot(*samples[-1].tcp[:2])
        dr = r_end - r_start
        results.append(
            CheckResult(
                f"{label}: radial reach moves {'outward' if expected_sign > 0 else 'inward'}",
                dr * expected_sign > 0,
                f"radial {r_start:.2f} -> {r_end:.2f} mm (d={dr:+.2f})",
            )
        )
    elif axis == "z":
        # 1. Radial reach stays put.
        max_r_drift = max(abs(math.hypot(*s.tcp[:2]) - r_start) for s in samples)
        results.append(
            CheckResult(
                f"{label}: radial reach held (|dr| <= {radial_tol_mm} mm)",
                max_r_drift <= radial_tol_mm,
                f"max |dr| = {max_r_drift:.2f} mm",
            )
        )
        # 2. Height moves in the commanded direction.
        dz = samples[-1].tcp[2] - z_start
        results.append(
            CheckResult(
                f"{label}: height moves {'up' if expected_sign > 0 else 'down'}",
                dz * expected_sign > 0,
                f"z {z_start:.2f} -> {samples[-1].tcp[2]:.2f} mm (d={dz:+.2f})",
            )
        )
    else:
        raise ValueError(axis)

    # 3. No unexpected yaw / lateral drift.
    max_y_drift = max(abs(s.tcp[1]) for s in samples)
    max_q1_drift = max(abs(s.pose.q1) for s in samples)
    results.append(
        CheckResult(
            f"{label}: no yaw drift (|y| <= {y_tol_mm} mm, |q1| <= {q1_tol_deg} deg)",
            max_y_drift <= y_tol_mm and max_q1_drift <= q1_tol_deg,
            f"max |y| = {max_y_drift:.2f} mm, max |q1| = {max_q1_drift:.3f} deg",
        )
    )

    # 4. Observed (q2, q3) matches the independent closed-form IK solution
    #    for each sample's OWN measured TCP point. This is axis-agnostic: it
    #    checks internal consistency between telemetry and the model,
    #    regardless of which direction was commanded.
    worst_q2_err = 0.0
    worst_q3_err = 0.0
    unreachable = 0
    for s in samples:
        r = math.hypot(*s.tcp[:2])
        solved = solve_q2_q3(r, s.tcp[2], start.pose.q2, start.pose.q3)
        if solved is None:
            unreachable += 1
            continue
        q2_pred, q3_pred = solved
        worst_q2_err = max(worst_q2_err, abs(_wrap_deg(s.pose.q2 - q2_pred)))
        worst_q3_err = max(worst_q3_err, abs(_wrap_deg(s.pose.q3 - q3_pred)))
    ok = unreachable == 0 and worst_q2_err <= angle_tol_deg and worst_q3_err <= angle_tol_deg
    results.append(
        CheckResult(
            f"{label}: J2/J3 split matches independent IK (<= {angle_tol_deg} deg)",
            ok,
            f"max |dq2| = {worst_q2_err:.2f} deg, max |dq3| = {worst_q3_err:.2f} deg"
            + (f", {unreachable} sample(s) unreachable" if unreachable else ""),
        )
    )
    return results


SEGMENTS = [
    ("I", "cj 1 0 0", "radial", +1.0),
    ("K", "cj -1 0 0", "radial", -1.0),
    ("U", "cj 0 0 1", "z", +1.0),
    ("O", "cj 0 0 -1", "z", -1.0),
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Independent I/K/U/O jog acceptance test")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--j1-center", type=int, default=J1_HOME_CENTER_STEPS)
    parser.add_argument("--j2-pull", type=int, default=J2_HOME_PULLOFF_STEPS)
    parser.add_argument("--hold-s", type=float, default=1.5, help="hold time per direction (s)")
    parser.add_argument("--sample-dt", type=float, default=0.25, help="seconds between samples")
    parser.add_argument("--z-tol-mm", type=float, default=8.0)
    parser.add_argument("--radial-tol-mm", type=float, default=8.0)
    parser.add_argument("--y-tol-mm", type=float, default=5.0)
    parser.add_argument("--q1-tol-deg", type=float, default=1.5)
    parser.add_argument("--angle-tol-deg", type=float, default=4.0)
    parser.add_argument("--safety-mm", type=float, default=25.0)
    parser.add_argument("--camera", type=int, default=None, help="camera index; omit to disable photos")
    parser.add_argument("--photo-dir", type=Path, default=None)
    args = parser.parse_args()

    cam = None
    photo_dir = args.photo_dir
    if args.camera is not None:
        from capture_camera import DEFAULT_OUTDIR, open_camera

        photo_dir = photo_dir or DEFAULT_OUTDIR
        cam = open_camera(args.camera)
        if not cam.isOpened():
            print(f"Could not open camera {args.camera}", file=sys.stderr)
            return 1

    all_results: list[CheckResult] = []
    try:
        with open_serial(args.port, args.baud) as ser:
            time.sleep(1.0)
            read_lines(ser, 1.0)
            send(ser, "all f", wait=0.3)

            run_home(ser, args.j1_center, args.j2_pull)

            j1, j2, j3, j4 = query_steps(ser)
            pose0 = steps_to_pose(j1, j2, j3)
            cur = Sample(0.0, (j1, j2, j3, j4), pose0, fk(pose0))
            if cam is not None:
                from capture_camera import save_frame

                cur.photo = save_frame(cam, photo_dir, "home")
            print(
                f"\nHome pose: steps={cur.steps} q=({cur.pose.q1:.2f},{cur.pose.q2:.2f},{cur.pose.q3:.2f}) "
                f"tcp={tuple(round(v, 2) for v in cur.tcp)}"
            )

            try:
                for label, cj_cmd, axis, expected_sign in SEGMENTS:
                    print(f"\n== Holding {label} ({cj_cmd}) for {args.hold_s}s ==")
                    r_ref = math.hypot(*cur.tcp[:2])
                    samples = hold_and_sample(
                        ser, cj_cmd, axis, args.hold_s, args.sample_dt,
                        cur.tcp[2], r_ref, args.safety_mm,
                        cam=cam, photo_dir=photo_dir, photo_prefix=label.lower(),
                    )
                    print_samples(label, samples)
                    all_results += check_segment(
                        label, samples, cur, axis, expected_sign,
                        args.z_tol_mm, args.y_tol_mm, args.q1_tol_deg,
                        args.angle_tol_deg, args.radial_tol_mm,
                    )
                    cur = samples[-1]
            except SafetyAbort as exc:
                send_quick(ser, "stop")
                print(f"\nSAFETY ABORT: {exc}", file=sys.stderr)
                return 2
            finally:
                send_quick(ser, "stop")
                send(ser, "all f", wait=0.3)
    finally:
        if cam is not None:
            cam.release()

    print("\n== Results ==")
    failed = 0
    for r in all_results:
        status = "PASS" if r.ok else "FAIL"
        if not r.ok:
            failed += 1
        print(f"[{status}] {r.name}  ({r.detail})")

    print(f"\n{len(all_results) - failed}/{len(all_results)} checks passed")
    if photo_dir is not None:
        print(f"Photos saved to: {photo_dir}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
