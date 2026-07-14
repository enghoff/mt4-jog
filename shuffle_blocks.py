#!/usr/bin/env python3
"""Shuffle cubes between the open work surface and calibrated marker slots.

Homes once, then in a loop:
  - If any marker is empty: pick a random visible cube and place it on a random
    empty marker.
  - Else if every marker is occupied: pick a cube from a random occupied marker
    and place it on a random clear open-table slot.

Requires vision_calibration.json (see calibrate_vision.py).
"""

from __future__ import annotations

import argparse
import sys
import time

from mt4_jog.client import Mt4Client
from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration
from mt4_vision.camera import DEFAULT_CAMERA_INDEX
from mt4_vision.shuffle import run_shuffle_loop


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="serial port (auto-detect if omitted)")
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA_INDEX)
    parser.add_argument(
        "--pause",
        type=float,
        default=0.5,
        help="seconds to let the scene settle before capture after release "
        "(overlaps with the post-place lift; default 0.5)",
    )
    parser.add_argument(
        "--retry",
        type=float,
        default=5.0,
        help="seconds to wait when no valid move is visible (default 5)",
    )
    args = parser.parse_args()

    calib = load_calibration(args.calib)
    client = Mt4Client() if args.port is None else Mt4Client(port=args.port)
    try:
        time.sleep(1.0)
        print("shuffle loop started (Ctrl+C to stop, H to re-home)")
        run_shuffle_loop(
            client,
            calib,
            camera=args.camera,
            pause_s=args.pause,
            retry_s=args.retry,
        )
    except KeyboardInterrupt:
        print("\nstopped")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
