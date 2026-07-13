"""CLI for vision diagnostics and vision-driven pick/place.

Workflow:
  1. python -m mt4_vision markers      -- verify the ArUco markers are seen
  2. python calibrate_vision.py        -- jog-to-marker interactive calibration
  3. python -m mt4_vision scene        -- sanity-check cube detections
  4. python -m mt4_vision pick red     -- hardware test of one pick
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration
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
