"""Live validation: camera → Scene → plan_shuffle.

Usage:
  python scripts/validate_scene_live.py --camera 1
  python scripts/validate_scene_live.py --camera 1 --move
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration
from mt4_vision.camera import DEFAULT_CAMERA_INDEX, grab_frame, open_camera
from mt4_vision.pickplace import home_arm, retreat_for_camera
from mt4_vision.policy import plan_shuffle
from mt4_vision.scene import capture_scene


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA_INDEX)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument("--port", default=None)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument(
        "--move",
        action="store_true",
        help="park the arm at camera-clear pose before each capture",
    )
    args = parser.parse_args()

    calib = load_calibration(args.calib)
    cap = open_camera(args.camera)
    client: Mt4Client | None = None
    try:
        if args.move:
            client = Mt4Client() if args.port is None else Mt4Client(port=args.port)
            time.sleep(0.5)
            status = client.get_status()
            print(f"arm: homed={status.homed} tcp={status.tcp}")
            if not status.homed:
                print("homing...")
                home_arm(client)
            print("retreating to camera park...")
            retreat_for_camera(client, calib)

        for i in range(args.cycles):
            if args.move and client is not None and i > 0:
                retreat_for_camera(client, calib)
                time.sleep(0.3)
            scene = capture_scene(calib, grab_frame(cap))
            action = plan_shuffle(scene)
            print(f"\n--- cycle {i} ---")
            print(f"scene: {scene.summary_line()}")
            for line in scene.cube_lines():
                print(line)
            print(
                f"placeable markers: "
                f"{sorted(m.marker_id for m in scene.placeable_markers())}"
            )
            print(f"action: {action.kind} -- {action.reason}")
        print("\nvalidation ok")
        return 0
    except Mt4ClientError as exc:
        print(f"arm error: {exc}")
        return 1
    finally:
        cap.release()
        if client is not None:
            client.close()


if __name__ == "__main__":
    raise SystemExit(main())
