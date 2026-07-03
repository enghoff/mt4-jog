#!/usr/bin/env python3
"""Read full ESP32 flash + metadata from the WLKATA MT4 extender box."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 921600
DEFAULT_OUT_DIR = ROOT / "backups" / "extender"
DEFAULT_FLASH_SIZE = 0x400000  # ESP32-WROOM-32E modules are usually 4 MB


def run_esptool(args: list[str]) -> subprocess.CompletedProcess[str]:
    cmd = [sys.executable, "-m", "esptool", *args]
    print("Running:", " ".join(cmd))
    return subprocess.run(cmd, text=True, capture_output=True)


def run_espefuse(args: list[str]) -> subprocess.CompletedProcess[str]:
    cmd = [sys.executable, "-m", "espefuse", *args]
    print("Running:", " ".join(cmd))
    return subprocess.run(cmd, text=True, capture_output=True)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Backup WLKATA MT4 extender box ESP32 flash and chip metadata"
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="Directory for backup artifacts",
    )
    parser.add_argument(
        "--flash-size",
        default=f"0x{DEFAULT_FLASH_SIZE:X}",
        help="Bytes to read (default: 4 MB). Use esptool flash-id to confirm.",
    )
    parser.add_argument(
        "--tag",
        default=date.today().isoformat(),
        help="Date tag for output filenames (default: today)",
    )
    parser.add_argument("--yes", action="store_true", help="Skip confirmation")
    args = parser.parse_args()

    flash_size = int(args.flash_size, 0)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    flash_path = args.out_dir / f"extender_esp32_flash_{args.tag}.bin"
    efuse_txt = args.out_dir / f"extender_esp32_efuse_{args.tag}.txt"
    manifest_path = args.out_dir / f"extender_esp32_manifest_{args.tag}.json"

    print("This will read the full ESP32 flash over USB (esptool read-flash).")
    print(f"Port: {args.port} @ {args.baud}")
    print(f"Flash dump: {flash_path} ({flash_size} bytes)")
    print(f"eFuse summary: {efuse_txt}")
    if not args.yes:
        print("Press Enter to continue, Ctrl+C to abort.")
        try:
            input()
        except KeyboardInterrupt:
            print("\nAborted.")
            return 1

    common = ["--port", args.port, "--baud", str(args.baud), "--chip", "esp32"]

    chip_id = run_esptool([*common, "chip-id"])
    flash_id = run_esptool([*common, "flash-id"])
    for result in (chip_id, flash_id):
        if result.stdout:
            print(result.stdout, end="")
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        if result.returncode != 0:
            return result.returncode

    read_flash = run_esptool(
        [
            *common,
            "read-flash",
            "0",
            f"0x{flash_size:X}",
            str(flash_path),
        ]
    )
    if read_flash.stdout:
        print(read_flash.stdout, end="")
    if read_flash.stderr:
        print(read_flash.stderr, end="", file=sys.stderr)
    if read_flash.returncode != 0:
        return read_flash.returncode

    if not flash_path.is_file() or flash_path.stat().st_size != flash_size:
        print(
            f"Unexpected flash dump size: {flash_path.stat().st_size if flash_path.is_file() else 'missing'}",
            file=sys.stderr,
        )
        return 1

    efuse = run_espefuse([*common, "summary"])
    efuse_txt.write_text(efuse.stdout + efuse.stderr, encoding="utf-8")
    if efuse.returncode != 0:
        print(efuse.stderr, file=sys.stderr)
        return efuse.returncode

    manifest = {
        "device": "WLKATA MT4 extender box",
        "mcu": "ESP32-WROOM-32E",
        "tag": args.tag,
        "port": args.port,
        "baud": args.baud,
        "flash_size_bytes": flash_size,
        "flash_file": flash_path.name,
        "flash_sha256": sha256_file(flash_path),
        "efuse_summary": efuse_txt.name,
        "chip_id_output": chip_id.stdout.strip(),
        "flash_id_output": flash_id.stdout.strip(),
        "restore_command": (
            f"python restore_extender.py --port {args.port} "
            f"--bin {flash_path.relative_to(ROOT)} --yes"
        ),
        "notes": [
            "Full raw flash image includes bootloader, partition table, app, SPIFFS, and NVS.",
            "eFuses are chip-specific and are not rewritten by restore_extender.py.",
            "UART download mode must be available (CH340 DTR/RTS auto-reset, or hold BOOT + tap RESET).",
        ],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"\nBackup complete.")
    print(f"  Flash: {flash_path}")
    print(f"  SHA256: {manifest['flash_sha256']}")
    print(f"  Manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
