#!/usr/bin/env python3
"""Camera-calibration sweep: visit known poses in the validated safe region,
photographing each with its telemetry pose. The photo<->pose pairs (all in
the CURRENT camera orientation) are the input for fitting the plane
homography that converts hand-placement photos into model (r, z).

All targets sit well inside the load-validated region (r 170-240, z 110-255)
-- light gravity moments, fresh homing, so the counters can be trusted.
"""

from __future__ import annotations

import sys
import time

from mt4_jog.joints import (
    DEFAULT_BAUD,
    DEFAULT_PORT,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
)
from mt4_jog.serial import open_serial, read_lines, send, send_quick
from verify_ik_jog import run_home
from square_snap import goto, snapshot

CAL_GUARDS = {
    "q2": (24.0, 95.0),
    "q3": (-5.0, 70.0),  # mirrored 2026-07-06: J3_STEP_SIGN was inverted
    "sep": (30.0, 165.0),
    "r": (150.0, 255.0),
    "z": (95.0, 270.0),
}

CAL_POINTS = [
    (225.0, 205.0),
    (240.0, 160.0),
    (225.0, 110.0),
    (200.0, 125.0),
    (190.0, 155.0),
    (170.0, 125.0),
]


def main() -> int:
    from capture_camera import DEFAULT_OUTDIR, open_camera, save_frame

    cam = open_camera(0)
    if not cam.isOpened():
        print("Could not open camera", file=sys.stderr)
        return 1

    records = []
    try:
        with open_serial(DEFAULT_PORT, DEFAULT_BAUD) as ser:
            time.sleep(1.0)
            read_lines(ser, 1.0)
            send(ser, "all f", wait=0.3)
            run_home(ser, J1_HOME_CENTER_STEPS, J2_HOME_PULLOFF_STEPS)

            # home itself is the first calibration point (exactly known pose)
            steps, pose, r, z = snapshot(ser)
            save_frame(cam, DEFAULT_OUTDIR, "cal_home")
            records.append(("cal_home", r, z, pose.q2, pose.q3))
            print(f"cal_home: r={r:.1f} z={z:.1f} q2={pose.q2:.1f} q3={pose.q3:.1f}")

            for tr, tz in CAL_POINTS:
                label = f"cal_r{int(tr)}_z{int(tz)}"
                steps, pose, r, z = goto(ser, tr, tz, CAL_GUARDS, label)
                save_frame(cam, DEFAULT_OUTDIR, label)
                records.append((label, r, z, pose.q2, pose.q3))
                print(f"{label}: r={r:.1f} z={z:.1f} q2={pose.q2:.1f} q3={pose.q3:.1f}")

            send_quick(ser, "stop")
            send(ser, "e0", wait=0.2)
            send(ser, "all f", wait=0.3)
    except RuntimeError as exc:
        send_quick(ser, "stop")
        print(f"ABORT: {exc}", file=sys.stderr)
        return 2
    finally:
        cam.release()

    print("\n# label, r_mm, z_mm, q2_deg, q3_deg  (paste into the fit step)")
    for rec in records:
        print(f"{rec[0]}, {rec[1]:.2f}, {rec[2]:.2f}, {rec[3]:.2f}, {rec[4]:.2f}")
    print("\nNOTE: drivers floated at exit -- the arm may sag; re-home before"
          " any verification moves.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
