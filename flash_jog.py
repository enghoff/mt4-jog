#!/usr/bin/env python3
"""Flash custom MT4 jog firmware via PlatformIO."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from mt4_jog.ports import Mt4PortError, port_display, resolve_port

ROOT = Path(__file__).resolve().parent
FIRMWARE = ROOT / "firmware" / "mt4_jog"


def main() -> int:
    parser = argparse.ArgumentParser(description="Flash MT4 jog firmware")
    parser.add_argument(
        "--port",
        default=None,
        help="upload port (auto-detect MT4 if omitted)",
    )
    args = parser.parse_args()

    try:
        port = resolve_port(args.port, probe=False)
    except Mt4PortError as exc:
        print(exc, file=sys.stderr)
        return 1

    print(port_display(port, explicit=args.port is not None))

    cmd = [
        sys.executable,
        "-m",
        "platformio",
        "run",
        "-t",
        "upload",
        "-d",
        str(FIRMWARE),
        f"--upload-port={port}",
    ]
    print("Running:", " ".join(cmd))
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
