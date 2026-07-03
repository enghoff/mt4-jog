#!/usr/bin/env python3
"""Flash MT4 gripper exercise firmware via PlatformIO."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FIRMWARE = ROOT / "firmware" / "mt4_gripper"
DEFAULT_PORT = "COM6"


def main() -> int:
    parser = argparse.ArgumentParser(description="Flash MT4 gripper test firmware")
    parser.add_argument("--port", default=DEFAULT_PORT)
    args = parser.parse_args()

    cmd = [
        sys.executable,
        "-m",
        "platformio",
        "run",
        "-t",
        "upload",
        "-d",
        str(FIRMWARE),
        f"--upload-port={args.port}",
    ]
    print("Running:", " ".join(cmd))
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
