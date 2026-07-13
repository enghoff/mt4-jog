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
from mt4_vision.workspace import (
    analyze_workspace,
    cubes_of_color,
    cubes_with_robot_coords,
    pick_largest_cube,
)


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
    if calib is not None:
        state = analyze_workspace(calib, frame)
        cubes = state.cubes
        print(
            f"markers: {len(state.free_markers)} free, "
            f"{len(state.occupied)} occupied, "
            f"{len(state.free_slots)} open slots"
        )
        for marker, cube in state.occupied:
            print(
                f"  marker {marker.marker_id} ({marker.x:.1f}, {marker.y:.1f}): "
                f"{cube.color}"
            )
        for marker in state.free_markers:
            print(f"  marker {marker.marker_id} ({marker.x:.1f}, {marker.y:.1f}): empty")
    else:
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
    target = pick_largest_cube(
        cubes_of_color(cubes_with_robot_coords(detect_cubes(frame, calib)), args.color)
    )
    if target is None:
        print(f"no {args.color} cube in view")
        return 1
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


def cmd_place_here(args: argparse.Namespace) -> int:
    from mt4_vision.pickplace import place_here

    calib = load_calibration(Path(args.calib))
    client = _pick_place_client(args)
    try:
        tcp = client.get_tcp()
        print(f"placing at current position ({tcp.x:.1f}, {tcp.y:.1f})")
        place_here(client, calib)
    finally:
        client.close()
    print("done")
    return 0


def cmd_shuffle(args: argparse.Namespace) -> int:
    import time

    from mt4_vision.shuffle import run_shuffle_loop

    calib = load_calibration(Path(args.calib))
    client = _pick_place_client(args)
    try:
        time.sleep(1.0)
        print("shuffle loop started (Ctrl+C to stop)")
        run_shuffle_loop(
            client,
            calib,
            camera=args.camera,
            pause_s=args.pause,
            retry_s=args.retry,
        )
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        client.close()
    return 0


def cmd_goto_marker(args: argparse.Namespace) -> int:
    from mt4_vision.pickplace import goto_marker

    calib = load_calibration(Path(args.calib))
    frame = capture_frame(args.camera)
    markers = detect_markers(frame, args.dict)
    match = next((m for m in markers if m.marker_id == args.marker_id), None)
    if match is None:
        print(f"marker {args.marker_id} not in view (visible: "
              f"{sorted(m.marker_id for m in markers)})")
        return 1
    x, y = calib.pixel_to_robot(match.px, match.py)
    print(f"marker {args.marker_id} at pixel ({match.px:.0f}, {match.py:.0f}) "
          f"-> robot ({x:.1f}, {y:.1f}){' -- touching table' if args.touch else ' -- hovering at safe_z'}")
    client = _pick_place_client(args)
    try:
        goto_marker(client, calib, x, y, touch=args.touch)
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

    p = sub.add_parser(
        "place-here",
        help="place held cube at the current TCP xy (moves the arm)",
    )
    p.add_argument("--port", default="")
    p.set_defaults(func=cmd_place_here)

    p = sub.add_parser(
        "goto-marker",
        help="move the arm to a detected marker -- calibration accuracy check",
    )
    p.add_argument("marker_id", type=int)
    p.add_argument("--dict", default="4x4_50")
    p.add_argument(
        "--touch", action="store_true",
        help="descend to table_z instead of hovering at safe_z",
    )
    p.add_argument("--port", default="")
    p.set_defaults(func=cmd_goto_marker)

    p = sub.add_parser(
        "shuffle",
        help="home then shuffle cubes between markers and open table (Ctrl+C to stop)",
    )
    p.add_argument("--port", default="")
    p.add_argument("--pause", type=float, default=2.0)
    p.add_argument("--retry", type=float, default=5.0)
    p.set_defaults(func=cmd_shuffle)

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
