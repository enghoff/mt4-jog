#!/usr/bin/env python3
"""Restore WLKATA MT4 extender box ESP32 flash from a full backup image."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 921600
DEFAULT_BIN = ROOT / "backups" / "extender" / "extender_esp32_flash_2026-07-03.bin"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Restore extender box ESP32 firmware from a full flash dump"
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--bin", type=Path, default=DEFAULT_BIN)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="Optional manifest JSON from backup_extender.py (validates SHA256)",
    )
    parser.add_argument(
        "--erase-all",
        action="store_true",
        help="Call erase-flash before writing (recommended for full images)",
    )
    parser.add_argument("--yes", action="store_true", help="Skip confirmation")
    args = parser.parse_args()

    if not args.bin.is_file():
        print(f"Missing flash image: {args.bin}", file=sys.stderr)
        return 1

    flash_size = args.bin.stat().st_size
    if flash_size not in (0x400000, 0x800000, 0x1000000, 0x2000000):
        print(
            f"Warning: unusual flash image size {flash_size} bytes; "
            "confirm this is a full-chip dump.",
            file=sys.stderr,
        )

    if args.manifest is not None:
        if not args.manifest.is_file():
            print(f"Missing manifest: {args.manifest}", file=sys.stderr)
            return 1
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        expected = manifest.get("flash_sha256")
        if expected:
            import hashlib

            digest = hashlib.sha256()
            with args.bin.open("rb") as fh:
                for chunk in iter(lambda: fh.read(1024 * 1024), b""):
                    digest.update(chunk)
            actual = digest.hexdigest()
            if actual != expected:
                print(
                    f"SHA256 mismatch:\n  expected {expected}\n  actual   {actual}",
                    file=sys.stderr,
                )
                return 1

    print("This will OVERWRITE the extender box ESP32 flash.")
    print(f"Image: {args.bin} ({flash_size} bytes)")
    print(f"Target: {args.port} @ {args.baud}")
    if not args.yes:
        print("Press Enter to continue, Ctrl+C to abort.")
        try:
            input()
        except KeyboardInterrupt:
            print("\nAborted.")
            return 1

    common = [sys.executable, "-m", "esptool", "--port", args.port, "--baud", str(args.baud), "--chip", "esp32"]

    if args.erase_all:
        erase_cmd = [*common, "erase-flash"]
        print("Running:", " ".join(erase_cmd))
        result = subprocess.run(erase_cmd)
        if result.returncode != 0:
            return result.returncode

    write_cmd = [
        *common,
        "write-flash",
        "--flash-mode",
        "dio",
        "--flash-freq",
        "40m",
        "--flash-size",
        f"{flash_size // (1024 * 1024)}MB",
        "0",
        str(args.bin),
    ]
    print("Running:", " ".join(write_cmd))
    result = subprocess.run(write_cmd)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
