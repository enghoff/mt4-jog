#!/usr/bin/env python3
"""Keyboard jog for MT4 custom jog firmware — hold key(s) to move, H to home.

Cartesian-only jog (world-frame TCP motion via firmware `cj`, on-device
Jacobian/DLS resolved-rate) plus J4 wrist roll and the gripper. Direct
per-joint jog (J1-J3) has been dropped -- Cartesian jog is the only motion
mode now.
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
)
from mt4_jog.kinematics import DEFAULT_ORIENT_GAIN
from mt4_jog.serial import drain_lines, open_serial, read_lines, send, send_quick

POLL_MS = 10
HOME_WAIT_S = 180.0
CJ_RESEND_S = 0.05
# Gripper/J4 commands used to be sent once per key-transition only; a single
# dropped serial line then left them stuck until the next transition. Resend
# on a timer while held, same fix already applied to Cartesian jog above.
GRIP_RESEND_S = 0.05
J4_RESEND_S = 0.05
SPEED_STEP_US = 100
SPEED_MIN_US = 700
SPEED_MAX_US = 4000
DEFAULT_SPEED_US = 1524
# `speed <us>` is a plain idempotent set command (no start/stop state), so
# holding -/= just re-sends it on a repeat timer -- no protocol change needed.
SPEED_REPEAT_S = 0.08

J4_JOINT = KEYBOARD_JOINTS[3]

# World-frame TCP jog (mm/s direction; firmware normalizes).
CART_BINDINGS: dict[str, tuple[int, int, int]] = {
    "i": (0, 0, 1),
    "k": (0, 0, -1),
    "s": (0, 1, 0),
    "w": (0, -1, 0),
    "a": (1, 0, 0),
    "d": (-1, 0, 0),
}

VK = {
    "i": 0x49,
    "k": 0x4B,
    "s": 0x53,
    "w": 0x57,
    "a": 0x41,
    "d": 0x44,
    "j": 0x4A,
    "l": 0x4C,
    "q": 0x51,
    "e": 0x45,
    "esc": 0x1B,
    "space": 0x20,
    "0": 0x30,
    "h": 0x48,
    "minus": 0xBD,
    "plus": 0xBB,
}


def print_help() -> None:
    print("MT4 keyboard jog — hold one or more keys to move, release to stop")
    print()
    print("  I / K     world +Z / -Z")
    print("  S / W     world +Y / -Y")
    print("  A / D     world +X / -X")
    print("  J / L     J4 wrist roll (when no Cartesian key held)")
    print("  -  / =    nudge jog speed slower / faster (live)")
    print()
    print("  H       home J1 + J2 (on-device)")
    print(
        f"  Q / E   gripper sweep open / close "
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
        elif line.startswith("home ") or line.startswith("pos ") or line.startswith("ok speed "):
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


def sync_j4_jog(ser, dir_high: bool | None) -> None:
    if dir_high is None:
        return
    level = "h" if dir_high else "l"
    for cmd in ("stop", "e1", "xc", f"d{J4_JOINT.direction} {level}", f"x+{J4_JOINT.drive}", "j"):
        send_quick(ser, cmd)
        time.sleep(0.01)


def resend_j4_jog(ser) -> None:
    """Cheap keep-alive while J/L stays held: re-arms the jog ISR without the
    stop/dir/axis-select preamble, so a dropped `j` can't strand it mid-jog."""
    send_quick(ser, "j")


def j4_key_state() -> bool | None:
    """Return dir_high for J4 roll, or None when J/L not held (or both held)."""
    j = key_down("j")
    l = key_down("l")
    if j and not l:
        return False
    if l and not j:
        return True
    return None


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
    """Return 'open', 'close', or None when Q/E not held (or both held)."""
    q = key_down("q")
    e = key_down("e")
    if q and not e:
        return "open"
    if e and not q:
        return "close"
    return None


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
        help="initial J4 wrist-unwind gain (default: firmware default 1.0); "
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
    active_j4: bool | None = None
    active_cart: tuple[int, int, int] | None = None
    grip_state: str | None = None
    last_cj_send = 0.0
    last_j4_send = 0.0
    last_grip_send = 0.0
    orient_gain = DEFAULT_ORIENT_GAIN if args.orient_gain is None else args.orient_gain
    speed_us = DEFAULT_SPEED_US
    last_speed_adjust = 0.0

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
                    active_j4 = None
                    active_cart = None
                    gripper_sweep_stop(ser)
                    grip_state = None
                    for line in send(ser, "?", wait=1.0):
                        print(line)
                    while key_down("space"):
                        time.sleep(poll_s)
                    continue

                if key_down("0"):
                    stop_jog(ser)
                    active_j4 = None
                    active_cart = None
                    gripper_sweep_stop(ser)
                    grip_state = None
                    send(ser, "e0", wait=0.2)
                    send(ser, "all f", wait=0.3)
                    while key_down("0"):
                        time.sleep(poll_s)
                    continue

                if key_down("h"):
                    stop_jog(ser)
                    active_j4 = None
                    active_cart = None
                    gripper_sweep_stop(ser)
                    grip_state = None
                    run_home(ser, buf, args.j1_center, args.j2_pull, args.verbose)
                    while key_down("h"):
                        time.sleep(poll_s)
                    continue

                minus = key_down("minus")
                plus = key_down("plus")
                now_t = time.monotonic()
                if (minus or plus) and now_t - last_speed_adjust >= SPEED_REPEAT_S:
                    # Lower period = faster; "=" (plus) speeds up. Keeps
                    # repeating for as long as the key is held.
                    speed_us += -SPEED_STEP_US if plus else SPEED_STEP_US
                    speed_us = max(SPEED_MIN_US, min(SPEED_MAX_US, speed_us))
                    send_quick(ser, f"speed {speed_us}")
                    last_speed_adjust = now_t

                # Cartesian keys take priority over J4 roll when held.
                desired_cart = pressed_cart_vector()
                now = time.monotonic()
                if desired_cart is not None:
                    if active_j4 is not None:
                        stop_jog(ser)
                        active_j4 = None
                    if desired_cart != active_cart or now - last_cj_send >= CJ_RESEND_S:
                        sync_cart_jog(ser, desired_cart)
                        active_cart = desired_cart
                        last_cj_send = now
                else:
                    if active_cart is not None:
                        stop_jog(ser)
                        active_cart = None
                    j4 = j4_key_state()
                    if j4 is None:
                        if active_j4 is not None:
                            stop_jog(ser)
                            active_j4 = None
                    elif j4 != active_j4:
                        sync_j4_jog(ser, j4)
                        active_j4 = j4
                        last_j4_send = now
                    elif now - last_j4_send >= J4_RESEND_S:
                        resend_j4_jog(ser)
                        last_j4_send = now

                desired_grip = gripper_key_state()
                if desired_grip is None:
                    if grip_state is not None:
                        gripper_sweep_stop(ser)
                        grip_state = None
                elif desired_grip != grip_state or now - last_grip_send >= GRIP_RESEND_S:
                    if desired_grip == "open":
                        gripper_sweep_open(ser)
                    else:
                        gripper_sweep_close(ser)
                    grip_state = desired_grip
                    last_grip_send = now

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
