#!/usr/bin/env python3
"""Keyboard / Xbox gamepad jog for MT4 custom jog firmware — hold to move, H to home.

Cartesian-only jog (world-frame TCP motion via firmware `cj`, on-device
Jacobian/DLS resolved-rate) plus J4 wrist roll and the gripper. Direct
per-joint jog (J1-J3) has been dropped -- Cartesian jog is the only motion
mode now.
"""

from __future__ import annotations

import argparse
import sys
import time

from mt4_jog.gamepad import A, B, START, X, XboxGamepad
from mt4_jog.joints import (
    DEFAULT_BAUD,
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
    LIMIT_JOINTS,
)
from mt4_jog.ports import Mt4PortError, port_display, resolve_port
from mt4_jog.serial import drain_lines, open_serial, read_lines, send, send_quick

POLL_MS = 10
HOME_WAIT_S = 180.0
CJ_RESEND_S = 0.05
# Gripper commands used to be sent once per key-transition only; a single
# dropped serial line then left them stuck until the next transition. Resend
# on a timer while held, same fix already applied to Cartesian jog above.
GRIP_RESEND_S = 0.05
SPEED_STEP_US = 100
SPEED_MIN_US = 700
SPEED_MAX_US = 4000
DEFAULT_SPEED_US = 1524
# `speed <us>` is a plain idempotent set command (no start/stop state), so
# holding -/= just re-sends it on a repeat timer -- no protocol change needed.
SPEED_REPEAT_S = 0.08

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
    "grave": 0xC0,
    "minus": 0xBD,
    "plus": 0xBB,
}


def print_help(*, gamepad: bool) -> None:
    print("MT4 jog — hold keys or sticks to move, release to stop")
    print()
    print("Keyboard:")
    print("  I / K     world +Z / -Z")
    print("  S / W     world +Y / -Y")
    print("  A / D     world +X / -X")
    print("  J / L     J4 wrist roll (also while moving XYZ)")
    print("  -  / =    nudge jog speed slower / faster (live)")
    print("  H       home J1 + J2 (on-device)")
    print(
        f"  Q / E   gripper sweep open / close "
        f"(S{GRIPPER_S_OPEN}–S{GRIPPER_S_CLOSED} on MT4; release = stop)"
    )
    print("  SPACE   status")
    print("  `       toggle invert world X + Y")
    print("  0       stop, drivers off")
    print("  ESC     quit")
    if gamepad:
        print()
        print("Xbox controller (player 1):")
        print("  Left stick        world X / Y")
        print("  Right stick Y     world Z")
        print("  Right stick X     J4 wrist roll (also while moving XYZ)")
        print("  LT / RT           gripper open / close")
        print("  LB / RB or D-pad  jog speed slower / faster")
        print("  A                 home")
        print("  B                 stop, drivers off")
        print("  X                 status")
        print("  Start             toggle invert world X + Y")
        print("  Back              quit")
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


def gamepad_button_held(gamepad: XboxGamepad | None, mask: int) -> bool:
    """Poll fresh controller state before testing a held button."""
    if gamepad is None or not mask:
        return False
    gamepad.poll()
    return gamepad.is_pressed(mask)


def wait_until_released(
    ser,
    buf: list[str],
    verbose: bool,
    poll_s: float,
    *,
    gamepad: XboxGamepad | None,
    keyboard_keys: tuple[str, ...] = (),
    gamepad_mask: int = 0,
) -> None:
    """Block until keyboard keys and/or a gamepad button are released."""
    while any(key_down(key) for key in keyboard_keys) or gamepad_button_held(
        gamepad, gamepad_mask
    ):
        drain_async(ser, buf, verbose)
        time.sleep(poll_s)


def stop_jog(ser) -> None:
    send_quick(ser, "stop")
    time.sleep(0.02)


def clear_active_motion(ser) -> tuple[None, None]:
    """Stop jog/gripper and return cleared motion state."""
    stop_jog(ser)
    gripper_sweep_stop(ser)
    return None, None


def apply_xy_invert(
    vector: tuple[int, int, int] | None,
    invert_xy: bool,
) -> tuple[int, int, int] | None:
    if vector is None or not invert_xy:
        return vector
    x, y, z = vector
    return -x, -y, z


def pressed_cart_vector(
    gamepad_cart: tuple[int, int, int] | None = None,
    *,
    invert_xy: bool = False,
) -> tuple[int, int, int] | None:
    vec = [0, 0, 0]
    for key, delta in CART_BINDINGS.items():
        if key_down(key):
            for i in range(3):
                vec[i] += delta[i]
    if gamepad_cart is not None:
        for i in range(3):
            vec[i] += gamepad_cart[i]
    vec = [max(-1, min(1, v)) for v in vec]
    if vec == [0, 0, 0]:
        return None
    return apply_xy_invert((vec[0], vec[1], vec[2]), invert_xy)


def sync_cart_jog(
    ser, vector: tuple[int, int, int] | None, j4_roll: int = 0
) -> None:
    """Send the unified jog command: Cartesian direction and/or J4 roll.
    The firmware layers the roll onto the resolved-rate solution (`cj dx dy
    dz [j4]`), so the wrist can rotate while the TCP moves; a zero vector
    with nonzero roll is a pure wrist roll through the same path (the old
    separate single-axis J4 jog is gone)."""
    if vector is None and j4_roll == 0:
        stop_jog(ser)
        return
    x, y, z = vector if vector is not None else (0, 0, 0)
    send_quick(ser, f"cj {x} {y} {z} {j4_roll}")


def j4_roll_state(gamepad_j4: bool | None = None) -> int:
    """J4 roll direction from J/L keys or right stick X: +1 / -1 in joint
    step sign (J = positive), 0 when idle or both held."""
    j = key_down("j")
    l = key_down("l")
    dir_high: bool | None = None
    if j and not l:
        dir_high = False
    elif l and not j:
        dir_high = True
    elif gamepad_j4 is not None:
        dir_high = gamepad_j4
    if dir_high is None:
        return 0
    # DIR-high on J4 is the negative joint direction (J_DIR_POS_HIGH is
    # false), matching the retired single-axis jog's physical direction.
    return -1 if dir_high else 1


def run_home(ser, buf: list[str], j1: int, j2: int, verbose: bool) -> None:
    cmd = f"home {j1} {j2}"
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


def gripper_key_state(gamepad_grip: str | None = None) -> str | None:
    """Return 'open', 'close', or None when Q/E not held (or both held)."""
    q = key_down("q")
    e = key_down("e")
    if q and not e:
        return "open"
    if e and not q:
        return "close"
    return gamepad_grip


def main() -> int:
    parser = argparse.ArgumentParser(description="MT4 keyboard jog")
    parser.add_argument(
        "--port",
        default=None,
        help="serial port (auto-detect MT4 if omitted)",
    )
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
        "--no-gamepad",
        action="store_true",
        help="disable Xbox controller input (keyboard only)",
    )
    parser.add_argument(
        "--gamepad-deadzone",
        type=int,
        default=9000,
        help="stick deadzone for Xbox controller (default: 9000)",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    if sys.platform != "win32":
        print("Requires Windows (GetAsyncKeyState / XInput).", file=sys.stderr)
        return 1

    gamepad = None
    if not args.no_gamepad:
        gamepad = XboxGamepad(deadzone=args.gamepad_deadzone)
        if not gamepad.available:
            print("XInput not available — keyboard only.", file=sys.stderr)
            gamepad = None

    try:
        port = resolve_port(args.port, baud=args.baud)
    except Mt4PortError as exc:
        print(exc, file=sys.stderr)
        return 1

    print_help(gamepad=gamepad is not None)
    print(f"{port_display(port, baud=args.baud, explicit=args.port is not None)} — focus this window for keyboard.")
    if gamepad is not None:
        print("Xbox controller: plug in before start; sticks work without focus.")
    print("WARNING: drivers energize while any jog key is held.")

    poll_s = args.poll_ms / 1000.0
    buf: list[str] = [""]
    # Active jog command: (cart vector or None, j4 roll) once anything is
    # held, else None.
    active_cart: tuple[tuple[int, int, int] | None, int] | None = None
    grip_state: str | None = None
    last_cj_send = 0.0
    last_grip_send = 0.0
    speed_us = DEFAULT_SPEED_US
    last_speed_adjust = 0.0
    invert_xy = False
    grave_was_down = False

    with open_serial(port, args.baud) as ser:
        time.sleep(1.0)
        if args.verbose:
            for line in read_lines(ser, 1.0):
                print(line, file=sys.stderr)
        else:
            drain_async(ser, buf, False)
        send(ser, "all f", wait=0.5)
        send(ser, "orient off" if args.no_orient else "orient on", wait=0.3)
        status_lines = send(ser, "?", wait=1.5)
        for line in status_lines:
            if line.startswith("MODE=") or line.startswith("pos ") or line.startswith("EN="):
                print(line)
        if not any(line.startswith("MODE=") for line in status_lines):
            print(
                "WARNING: no firmware status reply — check USB cable/hub and "
                "that custom jog firmware is flashed.",
                file=sys.stderr,
            )

        try:
            while True:
                drain_async(ser, buf, args.verbose)

                pad = gamepad.poll() if gamepad is not None else None
                pad_cart = pad.cart if pad and pad.connected else None
                pad_j4 = pad.j4 if pad and pad.connected else None
                pad_grip = pad.grip if pad and pad.connected else None
                now = time.monotonic()

                if key_down("esc") or (pad is not None and pad.quit):
                    break

                grave_down = key_down("grave")
                toggle_invert = (grave_down and not grave_was_down) or (
                    pad is not None and pad.toggle_invert_xy
                )
                grave_was_down = grave_down
                if toggle_invert:
                    invert_xy = not invert_xy
                    print(f"XY invert: {'on' if invert_xy else 'off'}")
                    if active_cart is not None:
                        stop_jog(ser)
                        active_cart = None
                    wait_until_released(
                        ser,
                        buf,
                        args.verbose,
                        poll_s,
                        gamepad=gamepad,
                        keyboard_keys=("grave",),
                        gamepad_mask=START,
                    )
                    grave_was_down = key_down("grave")
                    continue

                if key_down("space") or (pad is not None and pad.status):
                    active_cart, grip_state = clear_active_motion(ser)
                    for line in send(ser, "?", wait=1.0):
                        print(line)
                    wait_until_released(
                        ser, buf, args.verbose, poll_s, gamepad=gamepad,
                        keyboard_keys=("space",), gamepad_mask=X,
                    )
                    continue

                if key_down("0") or (pad is not None and pad.stop_all):
                    active_cart, grip_state = clear_active_motion(ser)
                    send(ser, "e0", wait=0.2)
                    send(ser, "all f", wait=0.3)
                    wait_until_released(
                        ser, buf, args.verbose, poll_s, gamepad=gamepad,
                        keyboard_keys=("0",), gamepad_mask=B,
                    )
                    continue

                if key_down("h") or (pad is not None and pad.home):
                    active_cart, grip_state = clear_active_motion(ser)
                    run_home(ser, buf, args.j1_center, args.j2_pull, args.verbose)
                    wait_until_released(
                        ser, buf, args.verbose, poll_s, gamepad=gamepad,
                        keyboard_keys=("h",), gamepad_mask=A,
                    )
                    continue

                minus = key_down("minus")
                plus = key_down("plus")
                if pad is not None:
                    minus = minus or pad.speed_down
                    plus = plus or pad.speed_up
                if (minus or plus) and now - last_speed_adjust >= SPEED_REPEAT_S:
                    # Lower period = faster; "=" (plus) speeds up. Keeps
                    # repeating for as long as the key is held.
                    speed_us += -SPEED_STEP_US if plus else SPEED_STEP_US
                    speed_us = max(SPEED_MIN_US, min(SPEED_MAX_US, speed_us))
                    send_quick(ser, f"speed {speed_us}")
                    last_speed_adjust = now

                # Unified jog: Cartesian direction and J4 roll are one `cj`
                # command, so the wrist rotates while the TCP moves.
                desired_cart = pressed_cart_vector(pad_cart, invert_xy=invert_xy)
                desired_roll = j4_roll_state(pad_j4)
                if desired_cart is not None or desired_roll != 0:
                    desired = (desired_cart, desired_roll)
                    if desired != active_cart or now - last_cj_send >= CJ_RESEND_S:
                        sync_cart_jog(ser, desired_cart, desired_roll)
                        active_cart = desired
                        last_cj_send = now
                elif active_cart is not None:
                    stop_jog(ser)
                    active_cart = None

                desired_grip = gripper_key_state(pad_grip)
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
