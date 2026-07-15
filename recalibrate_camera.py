#!/usr/bin/env python3
"""Refit the table-plane camera calibration after the camera moves --
*without* touching the arm, when the robot base and markers are known not
to have moved.

calibrate_vision.py's touch step exists to discover each marker's robot-
frame XY by jogging the TCP onto it -- but if the robot and markers are
unchanged, those XYs (already stored in vision_calibration.json's
raw_marker_observations) are still correct. All the camera move actually
invalidated is the *pixel* side of each correspondence. So this script:

  1. Loads the existing calibration's per-marker robot XYs (no arm motion).
  2. Captures a fresh frame and auto-detects each marker's *current* pixel
     center + corners.
  3. Refits the table-plane homography from (new pixel, old robot) pairs,
     via the same table_fit.fit_table_map() calibrate_vision.py uses.
  4. Backs up the old calibration, then saves the new one -- clearing
     cube_top_homography, since that correction was fit at the *old*
     camera pose and does not carry over. Run calibrate_height.py next
     (that step is already fully automatic -- no human input needed).

If the "robot and markers didn't move" assumption is wrong, this produces a
confidently wrong calibration with no independent way to catch it from
pixels alone -- the touch-residual check below is an internal consistency
check (does one homography explain every marker's old robot XY from its
new pixel position), not proof the assumption holds. A residual outlier
usually *does* mean a marker moved (or the wrong marker id matched) and is
worth investigating before trusting the result.

Usage:
  python recalibrate_camera.py --camera 1
  python recalibrate_camera.py --camera 1 --dry-run   # fit and report only
"""
from __future__ import annotations

import argparse
import shutil
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from mt4_vision.calib import (
    Calibration,
    CalibrationError,
    DEFAULT_CALIB_PATH,
    load_calibration,
)
from mt4_vision.camera import DEFAULT_CAMERA_INDEX, grab_frame, open_camera
from mt4_vision.detect import detect_markers
from mt4_vision.table_fit import fit_table_map
from mt4_vision.workspace import MARKER_DICT

# A refit marker whose old robot XY disagrees with the new camera geometry
# by more than this is more likely a moved marker / id mismatch than noise
# -- same threshold calibrate_vision.py uses for the equivalent touch check.
SUSPECT_RESIDUAL_MM = 25.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA_INDEX)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument(
        "--output",
        default=None,
        help="where to save the refit calibration (default: overwrite --calib, after backing it up)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="fit and print the report, but don't write anything",
    )
    args = parser.parse_args()

    calib_path = Path(args.calib)
    output_path = Path(args.output) if args.output else calib_path

    try:
        prev = load_calibration(calib_path)
    except CalibrationError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    old_obs = prev.raw_marker_observations or {}
    old_robot = {int(mid): tuple(data["robot"]) for mid, data in old_obs.items()}
    if len(old_robot) < 3:
        print(
            f"error: existing calibration only has {len(old_robot)} marker "
            "robot position(s) recorded -- need >=3. This script refits from "
            "the *existing* touch data; it can't help if there isn't any.",
            file=sys.stderr,
        )
        return 1

    print(f"loaded {calib_path}: {len(old_robot)} marker(s) with known robot XY: {sorted(old_robot)}")

    cap = open_camera(args.camera)
    try:
        frame = grab_frame(cap)
    finally:
        cap.release()

    detected = {m.marker_id: m for m in detect_markers(frame, MARKER_DICT)}
    matched_ids = sorted(set(old_robot) & set(detected))
    missing = sorted(set(old_robot) - set(detected))
    if missing:
        print(f"note: marker(s) {missing} not visible in this frame (occluded/out of view?) -- skipped")
    if len(matched_ids) < 3:
        print(
            f"error: only {len(matched_ids)} marker(s) both visible now and in the "
            "old calibration -- need >=3 to refit. Clear whatever's occluding the "
            "others, or reposition the camera so more are visible.",
            file=sys.stderr,
        )
        return 1
    if len(matched_ids) < len(old_robot):
        print(
            f"warning: refitting from {len(matched_ids)} of {len(old_robot)} previously "
            "calibrated markers -- fewer correspondences means a weaker fit, "
            "especially for perspective."
        )

    touch_px = {mid: (detected[mid].px, detected[mid].py) for mid in matched_ids}
    touch_robot = {mid: old_robot[mid] for mid in matched_ids}
    marker_corners = {
        mid: detected[mid].corners for mid in matched_ids if detected[mid].corners is not None
    }

    matrix, report = fit_table_map(marker_corners, touch_px, touch_robot)
    print(f"\n{report.kind} fit from markers {matched_ids}")
    if report.corner_rms_px is not None:
        print(
            f"corner-bundle RMS: {report.corner_rms_px}px (~{report.corner_rms_mm}mm; "
            ">1px suggests lens distortion)"
        )
    print(f"per-marker residual vs. stored robot XY (mm): {report.touch_residuals_mm}")
    if report.touch_loo_mm:
        print(f"per-marker leave-one-out error (mm): {report.touch_loo_mm}")
    for note in report.notes:
        print(f"NOTE: {note}")

    suspects = [m for m, e in report.touch_residuals_mm.items() if e > SUSPECT_RESIDUAL_MM]
    if suspects:
        print(
            f"\nWARNING: marker(s) {suspects} disagree with the new camera geometry "
            f"by >{SUSPECT_RESIDUAL_MM}mm."
        )
        print(
            "  This usually means that marker (or the robot) actually moved, not "
            "just the camera -- inspect before trusting this calibration. If it "
            "genuinely moved, run calibrate_vision.py's full touch process instead."
        )

    if args.dry_run:
        print("\n--dry-run: not writing anything.")
        return 1 if suspects else 0

    if prev.cube_top_homography is not None:
        print(
            "\ncube_top_homography cleared -- it was fit at the old camera pose "
            "and does not carry over. Run calibrate_height.py next (fully "
            "automatic, no touching required) before picking cubes; until then, "
            "cube picks fall back to the less accurate table-plane map."
        )

    backup_dir = calib_path.parent / "backups"
    backup_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = backup_dir / f"{calib_path.stem}_pre_recalibrate_{stamp}{calib_path.suffix}"
    if calib_path.exists():
        shutil.copy2(calib_path, backup_path)
        print(f"\nbacked up previous calibration to {backup_path}")

    new_calib = Calibration(
        homography=matrix,
        table_z=prev.table_z,
        pick_z=prev.pick_z,
        safe_z=prev.safe_z,
        travel_speed_us=prev.travel_speed_us,
        approach_speed_us=prev.approach_speed_us,
        grip_open_s=prev.grip_open_s,
        grip_close_s=prev.grip_close_s,
        cube_height_mm=prev.cube_height_mm,
        cube_top_homography=None,
        bundle_homography=report.bundle_h,
        raw_marker_observations={
            str(mid): {
                "pixel": list(touch_px[mid]),
                "corners": marker_corners.get(mid),
                "robot": list(touch_robot[mid]),
            }
            for mid in matched_ids
        },
        cam_xy_robot=prev.cam_xy_robot,
        cam_height_mm=prev.cam_height_mm,
        color_ranges=prev.color_ranges,
        workspace_hull_px=cv2.convexHull(
            np.array([[detected[mid].px, detected[mid].py] for mid in matched_ids], dtype=np.float32)
        ).reshape(-1, 2).tolist(),
    )
    new_calib.save(output_path)
    print(f"saved refit calibration to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
