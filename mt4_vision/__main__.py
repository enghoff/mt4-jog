"""CLI for camera calibration and vision-driven pick/place.

Workflow:
  1. python -m mt4_vision markers      -- verify the ArUco markers are seen
  2. python -m mt4_vision calibrate    -- fit pixel->robot homography + heights
  3. python -m mt4_vision scene        -- sanity-check cube detections
  4. python -m mt4_vision pick red     -- hardware test of one pick
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from mt4_vision.calib import (
    DEFAULT_CALIB_PATH,
    Calibration,
    fit_homography,
    load_calibration,
    reprojection_errors,
)
from mt4_vision.camera import DEFAULT_CAMERA_INDEX, capture_frame
from mt4_vision.detect import detect_cubes, detect_markers, scan_marker_dicts


def _save_annotated(frame, path: str) -> None:
    cv2.imwrite(path, frame)
    print(f"annotated frame saved to {path}")


def cmd_markers(args: argparse.Namespace) -> int:
    frame = capture_frame(args.camera)
    if args.dict == "scan":
        hits = scan_marker_dicts(frame)
        if not hits:
            print("no ArUco markers found with any known dictionary")
            _save_annotated(frame, "markers_frame.jpg")
            return 1
        for name, count in sorted(hits.items(), key=lambda kv: -kv[1]):
            print(f"{name}: {count} markers")
        best = max(hits, key=hits.get)  # type: ignore[arg-type]
        print(f"\nusing --dict {best} for detail:")
        args.dict = best
    markers = detect_markers(frame, args.dict)
    for m in markers:
        print(f"  id {m.marker_id}: pixel ({m.px:.1f}, {m.py:.1f})")
        cv2.circle(frame, (int(m.px), int(m.py)), 6, (0, 0, 255), 2)
        cv2.putText(
            frame, str(m.marker_id), (int(m.px) + 8, int(m.py) - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
        )
    _save_annotated(frame, "markers_frame.jpg")
    return 0 if markers else 1


def _prompt_float(label: str, default: float | None = None) -> float:
    suffix = f" [{default}]" if default is not None else ""
    while True:
        raw = input(f"{label}{suffix}: ").strip()
        if not raw and default is not None:
            return default
        try:
            return float(raw)
        except ValueError:
            print("  enter a number")


def cmd_calibrate(args: argparse.Namespace) -> int:
    frame = capture_frame(args.camera)
    markers = detect_markers(frame, args.dict)
    if len(markers) < 4:
        print(f"only {len(markers)} markers found; need >=4 (try --dict scan via `markers`)")
        return 1
    print(f"found {len(markers)} markers: {[m.marker_id for m in markers]}")
    print(
        "\nFor each marker, jog the arm's TCP to touch the marker center\n"
        "(jog_keyboard.py, read X/Y from the status line), then enter the\n"
        "robot-frame coordinates here. Blank X skips a marker.\n"
    )
    pixel_pts, robot_pts = [], []
    for m in markers:
        raw = input(f"marker {m.marker_id} robot X (blank=skip): ").strip()
        if not raw:
            continue
        x = float(raw)
        y = _prompt_float(f"marker {m.marker_id} robot Y")
        pixel_pts.append((m.px, m.py))
        robot_pts.append((x, y))
    homography = fit_homography(pixel_pts, robot_pts)
    errors = reprojection_errors(homography, pixel_pts, robot_pts)
    print(f"\nfit ok -- reprojection error per point (mm): {[round(e, 2) for e in errors]}")
    if max(errors) > 5.0:
        print("WARNING: worst point is >5mm off; re-measure that marker")

    print("\nHeights (robot-frame Z, mm). Touch a marker/table with the TCP to read table Z.")
    table_z = _prompt_float("table_z (TCP touching table)")
    cube = _prompt_float("cube edge length (mm)", 30.0)
    pick_z = _prompt_float("pick_z (TCP gripping a cube on the table)", table_z + cube / 2)
    safe_z = _prompt_float("safe_z (travel height; keep low over the desk)", table_z + cube + 40.0)
    grip_open = int(_prompt_float("grip_open_s (gripper S, clears cube)", 140))
    grip_close = int(_prompt_float("grip_close_s (gripper S, firm on cube)", 240))

    hull = cv2.convexHull(
        np.array([[m.px, m.py] for m in markers], dtype=np.float32)
    ).reshape(-1, 2)
    calib = Calibration(
        homography=homography,
        table_z=table_z,
        pick_z=pick_z,
        safe_z=safe_z,
        grip_open_s=grip_open,
        grip_close_s=grip_close,
        cube_height_mm=cube,
        workspace_hull_px=hull.tolist(),
    )
    calib.save(Path(args.output))
    print(f"\nsaved to {args.output}")
    print("optional: set cam_xy_robot/cam_height_mm in the JSON for cube-top parallax correction")
    return 0


def cmd_scene(args: argparse.Namespace) -> int:
    try:
        calib = load_calibration(Path(args.calib))
    except Exception as exc:  # noqa: BLE001 -- scene is usable pre-calibration
        print(f"(no calibration: {exc})")
        calib = None
    frame = capture_frame(args.camera)
    cubes = detect_cubes(frame, calib)
    if not cubes:
        print("no cubes detected")
    for c in cubes:
        robot = f" robot ({c.x:.1f}, {c.y:.1f})" if c.x is not None else ""
        print(f"  {c.color}: pixel ({c.px:.0f}, {c.py:.0f}) area {c.area:.0f}px^2{robot}")
        cv2.circle(frame, (int(c.px), int(c.py)), 8, (255, 255, 255), 2)
        cv2.putText(
            frame, c.color, (int(c.px) + 10, int(c.py)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
        )
    _save_annotated(frame, "scene_frame.jpg")
    return 0


def _pick_place_client(args: argparse.Namespace):
    from mt4_jog.client import Mt4Client

    return Mt4Client() if not args.port else Mt4Client(port=args.port)


def cmd_pick(args: argparse.Namespace) -> int:
    from mt4_vision.pickplace import pick

    calib = load_calibration(Path(args.calib))
    frame = capture_frame(args.camera)
    matches = [c for c in detect_cubes(frame, calib) if c.color == args.color]
    if not matches:
        print(f"no {args.color} cube in view")
        return 1
    target = matches[0]
    print(f"picking {args.color} at robot ({target.x:.1f}, {target.y:.1f})")
    client = _pick_place_client(args)
    try:
        pick(client, calib, target.x, target.y)
    finally:
        client.close()
    print("done")
    return 0


def cmd_place(args: argparse.Namespace) -> int:
    from mt4_vision.pickplace import place

    calib = load_calibration(Path(args.calib))
    client = _pick_place_client(args)
    try:
        place(client, calib, args.x, args.y)
    finally:
        client.close()
    print("done")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(prog="mt4_vision")
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA_INDEX)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("markers", help="detect ArUco markers, save annotated frame")
    p.add_argument("--dict", default="scan", help="ArUco dict name, or 'scan' to try all")
    p.set_defaults(func=cmd_markers)

    p = sub.add_parser("calibrate", help="fit pixel->robot homography interactively")
    p.add_argument("--dict", default="4x4_50")
    p.add_argument("--output", default=str(DEFAULT_CALIB_PATH))
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("scene", help="detect cubes, print robot coords")
    p.set_defaults(func=cmd_scene)

    p = sub.add_parser("pick", help="pick a cube by color (moves the arm)")
    p.add_argument("color")
    p.add_argument("--port", default="")
    p.set_defaults(func=cmd_pick)

    p = sub.add_parser("place", help="place held cube at robot X Y (moves the arm)")
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("--port", default="")
    p.set_defaults(func=cmd_place)

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
