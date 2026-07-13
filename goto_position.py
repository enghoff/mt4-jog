#!/usr/bin/env python3
"""Send the MT4 to an absolute position via the firmware `mp` command.

Prompts for X/Y/Z (mm, origin at the base under J1's pivot), J4 gripper yaw
(deg, world frame), and gripper S (120-285, absolute) -- each defaulting to the
arm's current reported position if left blank. When J4 is left at its current
value, the firmware holds gripper orientation fixed in world space during the
move (J4 counters base yaw 1:1, like Cartesian jog `orient on`).

The firmware's homed flag lives in MCU RAM, and whether it survives from a
prior script's connection to this one is unreliable (this board's serial
connect sometimes resets the MCU, sometimes doesn't) -- so if the arm isn't
homed in *this* connection, this script homes it itself before prompting
for a position.
"""

from __future__ import annotations

import argparse
import sys
import time

from mt4_jog.joints import (
    DEFAULT_BAUD,
    DEFAULT_PORT,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
    JOG_SPEED_MAX_US,
    JOG_SPEED_MIN_US,
)
from mt4_jog.serial import open_serial, read_lines, send
from mt4_jog.status import parse_tcp_line
MOVE_TIMEOUT_S = 30.0
# Matches jog_keyboard.py's HOME_WAIT_S -- the limit-switch seeks inside
# do_home() can each run up to HOME_SEEK_MAX steps (~20s) if a limit isn't
# found quickly, on top of the widen/backoff steps.
HOME_TIMEOUT_S = 180.0


def query_status(ser) -> tuple[bool, dict[str, float] | None]:
    homed = False
    tcp: dict[str, float] | None = None
    for line in send(ser, "?", wait=1.0):
        # HOMED= sits mid-line ("MODE=joint  ORIENT=hold  HOMED=yes"), not
        # at the start -- check by substring, not startswith.
        if "HOMED=yes" in line:
            homed = True
        pose = parse_tcp_line(line)
        if pose is not None:
            tcp = pose.as_dict()
    return homed, tcp


def prompt_float(label: str, unit: str, default: float) -> float:
    raw = input(f"{label} ({unit}) [{default:.1f}]: ").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        print(f"Not a number, using default {default:.1f}.", file=sys.stderr)
        return default


def prompt_int(label: str, unit: str, default: int) -> int:
    raw = input(f"{label} ({unit}) [{default}]: ").strip()
    if not raw:
        return default
    try:
        return int(float(raw))
    except ValueError:
        print(f"Not a number, using default {default}.", file=sys.stderr)
        return default


def run_home(ser, j1: int, j2: int) -> bool:
    cmd = f"home {j1} {j2}"
    ser.write(f"{cmd}\n".encode("ascii"))
    ser.flush()
    deadline = time.monotonic() + HOME_TIMEOUT_S
    while time.monotonic() < deadline:
        for line in read_lines(ser, 0.3):
            print(line)
            if line == "home ok":
                return True
            if line.startswith("home fail"):
                return False
    print("Homing timed out.", file=sys.stderr)
    return False


def send_move(
    ser, x: float, y: float, z: float, j4: float, grip: int, speed_us: int = 0
) -> None:
    cmd = f"mp {x} {y} {z} {j4} {grip}"
    if speed_us > 0:
        cmd += f" {speed_us}"
    print(f">>> {cmd}")
    for line in send(ser, cmd, wait=1.0):
        print(line)

    # "ok mp" comes back immediately; "mp done pos ..." (or an "err ...")
    # arrives async once the coordinated move finishes.
    deadline = time.monotonic() + MOVE_TIMEOUT_S
    while time.monotonic() < deadline:
        lines = read_lines(ser, 0.3)
        for line in lines:
            print(line)
            if line.startswith("mp done") or line.startswith("err"):
                return
    print("Timed out waiting for the move to finish.", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Prompt for an absolute MT4 TCP position and send it (firmware `mp`)"
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--j1-center", type=int, default=J1_HOME_CENTER_STEPS)
    parser.add_argument("--j2-pull", type=int, default=J2_HOME_PULLOFF_STEPS)
    parser.add_argument(
        "--speed",
        type=int,
        default=0,
        metavar="US",
        help=(
            "move step period in microseconds (700-4000, same as firmware "
            "`speed`; 0 = leave the current period unchanged)"
        ),
    )
    args = parser.parse_args()
    if args.speed != 0 and not JOG_SPEED_MIN_US <= args.speed <= JOG_SPEED_MAX_US:
        print(
            f"--speed must be 0 or {JOG_SPEED_MIN_US}-{JOG_SPEED_MAX_US} us.",
            file=sys.stderr,
        )
        return 2

    with open_serial(args.port, args.baud) as ser:
        time.sleep(1.0)
        read_lines(ser, 0.5)  # discard the boot banner

        homed, tcp = query_status(ser)
        if not homed:
            print("Arm has not homed in this connection yet -- homing now.")
            print("WARNING: homing drives J1/J2 to their limit switches -- clear the workspace.")
            if not run_home(ser, args.j1_center, args.j2_pull):
                print("Homing failed -- exiting.", file=sys.stderr)
                return 1
            homed, tcp = query_status(ser)
        if not homed:
            print("Still not homed after the home command -- exiting.", file=sys.stderr)
            return 1
        if tcp is None:
            print("Could not read the arm's current position.", file=sys.stderr)
            return 1
        if args.speed > 0:
            for line in send(ser, f"speed {args.speed}", wait=0.5):
                print(line)
            homed, tcp = query_status(ser)
            if tcp is None:
                print("Could not read speed after --speed.", file=sys.stderr)
                return 1

        print("Enter blank to keep the current value. Ctrl-C to quit.\n")
        try:
            while True:
                print(
                    f"Current position: x={tcp['x']:.1f} y={tcp['y']:.1f} "
                    f"z={tcp['z']:.1f} j4={tcp['j4']:.1f} grip={tcp['grip']:.0f} "
                    f"speed={int(tcp['speed'])}"
                )
                x = prompt_float("X", "mm", tcp["x"])
                y = prompt_float("Y", "mm", tcp["y"])
                z = prompt_float("Z", "mm", tcp["z"])
                j4 = prompt_float("J4 yaw", "deg WS", tcp["j4"])
                grip = prompt_int("Gripper", "S120-285", int(tcp["grip"]))
                speed = prompt_int(
                    "Speed",
                    f"us {JOG_SPEED_MIN_US}-{JOG_SPEED_MAX_US}, 0=unchanged",
                    int(tcp["speed"]),
                )
                if speed != 0 and not JOG_SPEED_MIN_US <= speed <= JOG_SPEED_MAX_US:
                    speed = int(tcp["speed"])
                    print(
                        f"Speed out of range, using current {speed}.",
                        file=sys.stderr,
                    )
                print()

                send_move(ser, x, y, z, j4, grip, speed)
                print()

                homed, tcp = query_status(ser)
                if not homed or tcp is None:
                    print("Lost homed/position state -- stopping.", file=sys.stderr)
                    return 1
        except (KeyboardInterrupt, EOFError):
            print("\nBye.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
