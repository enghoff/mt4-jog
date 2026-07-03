#!/usr/bin/env python3
"""Restore stock WLKATA MT4 flash image over USB (wiring bootloader)."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_HEX = ROOT / "backups" / "mt4_flash_2026-07-02.hex"
DEFAULT_EEPROM = ROOT / "backups" / "mt4_eeprom_2026-07-02.hex"
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 115200


def main() -> int:
    parser = argparse.ArgumentParser(description="Restore stock MT4 firmware")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--hex", dest="hex_path", type=Path, default=DEFAULT_HEX)
    parser.add_argument(
        "--eeprom",
        dest="eeprom_path",
        type=Path,
        default=None,
        help="Also restore EEPROM from this .hex (e.g. backups/mt4_eeprom_2026-07-02.hex)",
    )
    parser.add_argument("--yes", action="store_true", help="Skip confirmation")
    args = parser.parse_args()

    if not args.hex_path.is_file():
        print(f"Missing firmware image: {args.hex_path}", file=sys.stderr)
        return 1

    print(f"This will flash STOCK firmware from:\n  {args.hex_path}")
    print(f"Target: {args.port} @ {args.baud} (avrdude -c wiring)")
    if not args.yes:
        print("Press Enter to continue, Ctrl+C to abort.")
        try:
            input()
        except KeyboardInterrupt:
            print("\nAborted.")
            return 1

    cmd = [
        "avrdude",
        "-p",
        "atmega2560",
        "-c",
        "wiring",
        "-P",
        args.port,
        "-b",
        str(args.baud),
        "-D",
        "-U",
        f"flash:w:{args.hex_path}:i",
    ]
    print("Running:", " ".join(cmd))
    result = subprocess.run(cmd)
    if result.returncode != 0:
        return result.returncode

    if args.eeprom_path is not None:
        if not args.eeprom_path.is_file():
            print(f"Missing EEPROM image: {args.eeprom_path}", file=sys.stderr)
            return 1
        eeprom_cmd = [
            "avrdude",
            "-p",
            "atmega2560",
            "-c",
            "wiring",
            "-P",
            args.port,
            "-b",
            str(args.baud),
            "-D",
            "-U",
            f"eeprom:w:{args.eeprom_path}:i",
        ]
        print("Running:", " ".join(eeprom_cmd))
        result = subprocess.run(eeprom_cmd)

    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
