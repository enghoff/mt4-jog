#!/usr/bin/env python3
"""Keyboard jog for MT4 custom jog firmware — hold key(s) to move, H to home.

Two motion modes, both live at once (Cartesian takes priority when any of
its keys are held):
  - Joint jog: Q/A W/S E/D R/F drive J1-J4 directly (raw step/dir pins).
  - Cartesian jog: I/K J/L U/O drive world-frame TCP motion (firmware `cj`,
    on-device Jacobian/DLS resolved-rate). Requires Cartesian-capable jog
    firmware. R/F also work here as J4 wrist roll when no Cartesian key is
    held (same underlying joint jog as the R/F binding above).
"""

from __future__ import annotations

import argparse
import sys
import time

from mt4_jog.joints import (
    DEFAULT_BAUD,
    DEFAULT_PORT,
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
    KEYBOARD_JOINTS,
    LIMIT_JOINTS,
    Joint,
)
from mt4_jog.kinematics import DEFAULT_ORIENT_GAIN
from mt4_jog.serial import drain_lines, open_serial, read_lines, send, send_quick

POLL_MS = 10
HOME_WAIT_S = 180.0
CJ_RESEND_S = 0.05
ORIENT_GAIN_STEP = 0.05

KEY_BINDINGS: dict[str, tuple[int, bool]] = {
    "q": (0, False),
    "a": (0, True),
    "w": (1, False),
    "s": (1, True),
    "e": (2, False),
    "d": (2, True),
    "r": (3, False),
    "f": (3, True),
}

# World-frame TCP jog (mm/s direction; firmware normalizes).
CART_BINDINGS: dict[str, tuple[int, int, int]] = {
    "i": (0, 0, 1),
    "k": (0, 0, -1),
    "j": (0, 1, 0),
    "l": (0, -1, 0),
    "u": (1, 0, 0),
    "o": (-1, 0, 0),
}

VK = {
    "q": 0x51,
    "a": 0x41,
    "w": 0x57,
    "s": 0x53,
    "e": 0x45,
    "d": 0x44,
    "r": 0x52,
    "f": 0x46,
    "i": 0x49,
    "k": 0x4B,
    "j": 0x4A,
    "l": 0x4C,
    "u": 0x55,
    "o": 0x4F,
    "esc": 0x1B,
    "space": 0x20,
    "0": 0x30,
    "h": 0x48,
    "t": 0x54,
    "g": 0x47,
    "minus": 0xBD,
    "plus": 0xBB,
}


def print_help() -> None:
    print("MT4 keyboard jog — hold one or more keys to move, release to stop")
    print()
    for key_neg, key_pos, joint in zip("qwer", "asdf", KEYBOARD_JOINTS):
        print(
            f"  {key_neg.upper()} / {key_pos.upper()}  "
            f"{joint.gcode} {joint.label}  "
            f"(drive D{joint.drive}, dir D{joint.direction})"
        )
    print()
    print("  I / K     world +Z / -Z          (Cartesian, priority over joint jog)")
    print("  J / L     world +Y / -Y")
    print("  U / O     world +X / -X")
    print("  -  / =    nudge J4 wrist-unwind orient gain down / up (live tuning)")
    print()
    print("  H       home J1 + J2 (on-device)")
    print(
        f"  T / G   gripper sweep open / close "
        f"(S{GRIPPER_S_OPEN}–S{GRIPPER_S_CLOSED} on MT4; release = stop)"
    )
    print("  SPACE   status")
    print("  0       stop, drivers off")
    print("  ESC     quit")
    print()
    limits = ", ".join(f"{io}={label}" for io, label in sorted(LIMIT_JOINTS.items()))
    print(f"Limits: {limits}")
    print()


def format_limit(line: str) -> str:
    parts = line.split()
    if len(parts) < 2 or not parts[0].startswith("I"):
        return line
    pin, _, raw = parts[0].partition("=")
    joint = LIMIT_JOINTS.get(pin, "")
    label = f"{pin} {joint}" if joint else pin
    if parts[1] == "TRIG":
        return f"{label} TRIGGERED (raw={raw})"
    return f"{label} released (raw={raw})"


def drain_async(ser, buf: list[str], verbose: bool) -> None:
    for line in drain_lines(ser, buf):
        if line.startswith("lim "):
            print(f"LIMIT: {format_limit(line[4:])}")
        elif line.startswith("home ") or line.startswith("pos "):
            print(line)
        elif line.startswith("err cj"):
            print(f"CART: {line}")
        elif verbose:
            print(line, file=sys.stderr)


def key_down(key: str) -> bool:
    if sys.platform != "win32":
        return False
    import ctypes

    vk = VK.get(key)
    return vk is not None and bool(ctypes.windll.user32.GetAsyncKeyState(vk) & 0x8000)


def stop_jog(ser) -> None:
    send_quick(ser, "stop")
    time.sleep(0.02)


def sync_jog(ser, keys: set[str]) -> None:
    """Configure firmware for all currently held jog keys."""
    if not keys:
        stop_jog(ser)
        return
    send_quick(ser, "stop")
    send_quick(ser, "e1")
    send_quick(ser, "xc")
    for key in sorted(keys):
        idx, dir_high = KEY_BINDINGS[key]
        joint = KEYBOARD_JOINTS[idx]
        level = "h" if dir_high else "l"
        send_quick(ser, f"d{joint.direction} {level}")
        send_quick(ser, f"x+{joint.drive}")
        time.sleep(0.01)
    send_quick(ser, "j")


def start_jog(ser, joint: Joint, dir_high: bool) -> None:
    """Single-axis jog (legacy helper)."""
    level = "h" if dir_high else "l"
    for cmd in ("stop", "e1", "xc", f"d{joint.direction} {level}", f"x{joint.drive}", "j"):
        send_quick(ser, cmd)
        time.sleep(0.02)


def pressed_cart_vector() -> tuple[int, int, int] | None:
    vec = [0, 0, 0]
    for key, delta in CART_BINDINGS.items():
        if key_down(key):
            for i in range(3):
                vec[i] += delta[i]
    if vec == [0, 0, 0]:
        return None
    return vec[0], vec[1], vec[2]


def sync_cart_jog(ser, vector: tuple[int, int, int] | None) -> None:
    if vector is None:
        stop_jog(ser)
        return
    send_quick(ser, f"cj {vector[0]} {vector[1]} {vector[2]}")


def run_home(ser, buf: list[str], j1: int, j2: int, verbose: bool) -> None:
    cmd = f"home {j1} {j2}"
    print(f"Homing… (J1 center {j1}, J2 pull {j2})")
    if verbose:
        print(f">>> {cmd}", file=sys.stderr)
    ser.write(f"{cmd}\n".encode("ascii"))
    ser.flush()
    deadline = time.monotonic() + HOME_WAIT_S
    while time.monotonic() < deadline:
        for line in drain_lines(ser, buf):
            if line.startswith("home ") or line.startswith("lim "):
                if line.startswith("lim "):
                    print(f"LIMIT: {format_limit(line[4:])}")
                else:
                    print(line)
            if line == "home ok" or line.startswith("home fail"):
                return
        time.sleep(0.02)
    print("Homing timed out.", file=sys.stderr)


def gripper_sweep_open(ser) -> None:
    send_quick(ser, "g o")


def gripper_sweep_close(ser) -> None:
    send_quick(ser, "g c")


def gripper_sweep_stop(ser) -> None:
    send_quick(ser, "g stop")


def gripper_key_state() -> str | None:
    """Return 'open', 'close', or None when T/G not held (or both held)."""
    t = key_down("t")
    g = key_down("g")
    if t and not g:
        return "open"
    if g and not t:
        return "close"
    return None


def sync_gripper_state(ser, state: str | None, prev: str | None) -> str | None:
    """Send one sweep command when T/G state changes."""
    if state == prev:
        return prev
    if state == "open":
        gripper_sweep_open(ser)
    elif state == "close":
        gripper_sweep_close(ser)
    else:
        gripper_sweep_stop(ser)
    return state


def pressed_jog_keys() -> set[str]:
    chosen: set[str] = set()
    for idx in range(len(KEYBOARD_JOINTS)):
        held = [
            key
            for key, (joint_idx, _) in KEY_BINDINGS.items()
            if joint_idx == idx and key_down(key)
        ]
        if len(held) == 1:
            chosen.add(held[0])
    return chosen


def main() -> int:
    parser = argparse.ArgumentParser(description="MT4 keyboard jog")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--poll-ms", type=int, default=POLL_MS)
    parser.add_argument("--j1-center", type=int, default=J1_HOME_CENTER_STEPS)
    parser.add_argument("--j2-pull", type=int, default=J2_HOME_PULLOFF_STEPS)
    parser.add_argument(
        "--no-orient",
        action="store_true",
        help="disable J4 wrist unwind during Cartesian jog",
    )
    parser.add_argument(
        "--orient-gain",
        type=float,
        default=None,
        help="initial J4 wrist-unwind gain (default: firmware default 0.82); "
        "nudge live with -/= keys",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    if sys.platform != "win32":
        print("Requires Windows (GetAsyncKeyState).", file=sys.stderr)
        return 1

    print_help()
    print(f"Port {args.port} @ {args.baud} — focus this window.")
    print("WARNING: drivers energize while any jog key is held.")

    poll_s = args.poll_ms / 1000.0
    buf: list[str] = [""]
    active_keys: set[str] = set()
    active_cart: tuple[int, int, int] | None = None
    grip_state: str | None = None
    last_cj_send = 0.0
    orient_gain = DEFAULT_ORIENT_GAIN if args.orient_gain is None else args.orient_gain
    prev_minus = False
    prev_plus = False

    with open_serial(args.port, args.baud) as ser:
        time.sleep(1.0)
        if args.verbose:
            for line in read_lines(ser, 1.0):
                print(line, file=sys.stderr)
        else:
            drain_async(ser, buf, False)
        send(ser, "all f", wait=0.5)
        if args.no_orient:
            send(ser, "orient off", wait=0.3)
        elif args.orient_gain is not None:
            for line in send(ser, f"orient {orient_gain}", wait=0.3):
                print(line)
        else:
            send(ser, "orient on", wait=0.3)

        try:
            while True:
                drain_async(ser, buf, args.verbose)

                if key_down("esc"):
                    break

                if key_down("space"):
                    stop_jog(ser)
                    active_keys.clear()
                    active_cart = None
                    grip_state = sync_gripper_state(ser, None, grip_state)
                    for line in send(ser, "?", wait=1.0):
                        print(line)
                    while key_down("space"):
                        time.sleep(poll_s)
                    continue

                if key_down("0"):
                    stop_jog(ser)
                    active_keys.clear()
                    active_cart = None
                    grip_state = sync_gripper_state(ser, None, grip_state)
                    send(ser, "e0", wait=0.2)
                    send(ser, "all f", wait=0.3)
                    while key_down("0"):
                        time.sleep(poll_s)
                    continue

                if key_down("h"):
                    stop_jog(ser)
                    active_keys.clear()
                    active_cart = None
                    grip_state = sync_gripper_state(ser, None, grip_state)
                    run_home(ser, buf, args.j1_center, args.j2_pull, args.verbose)
                    while key_down("h"):
                        time.sleep(poll_s)
                    continue

                minus = key_down("minus")
                plus = key_down("plus")
                if (minus and not prev_minus) or (plus and not prev_plus):
                    orient_gain += ORIENT_GAIN_STEP if plus else -ORIENT_GAIN_STEP
                    orient_gain = round(max(0.0, orient_gain), 3)
                    for line in send(ser, f"orient {orient_gain}", wait=0.1):
                        print(line)
                prev_minus, prev_plus = minus, plus

                # Cartesian keys take priority over joint jog when held.
                desired_cart = pressed_cart_vector()
                now = time.monotonic()
                if desired_cart is not None:
                    if active_keys:
                        sync_jog(ser, set())
                        active_keys = set()
                    if desired_cart != active_cart or now - last_cj_send >= CJ_RESEND_S:
                        sync_cart_jog(ser, desired_cart)
                        active_cart = desired_cart
                        last_cj_send = now
                else:
                    if active_cart is not None:
                        stop_jog(ser)
                        active_cart = None
                    desired_keys = pressed_jog_keys()
                    if desired_keys != active_keys:
                        sync_jog(ser, desired_keys)
                        active_keys = desired_keys

                grip_state = sync_gripper_state(ser, gripper_key_state(), grip_state)

                time.sleep(poll_s)
        except KeyboardInterrupt:
            print()
        finally:
            stop_jog(ser)
            gripper_sweep_stop(ser)
            send(ser, "e0", wait=0.2)
            send(ser, "all f", wait=0.3)

    print("Bye.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
