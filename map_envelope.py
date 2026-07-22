#!/usr/bin/env python3
"""Map the MT4 operating envelope by jogging and tagging poses.

Same jog controls as jog.py. Tag the current pose:

  D-pad Up / ]     in-range (allowed)
  D-pad Down / [   out-of-range (forbidden / past limit)
  Backspace / LB   undo last sample

Samples (TCP + joint steps/deg) are written to envelope_samples.json with a
live summary of joint min/max and Cartesian ground-plane / reach from in
samples. Does not patch MAX_REACH_MM or firmware.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from jog import (
    CJ_RESEND_S,
    DEFAULT_SPEED_US,
    GRIP_RESEND_S,
    POLL_MS,
    SPEED_MAX_US,
    SPEED_MIN_US,
    SPEED_REPEAT_S,
    SPEED_STEP_US,
    VK,
    clear_active_motion,
    drain_async,
    flush_console_input,
    gripper_key_state,
    gripper_sweep_close,
    gripper_sweep_open,
    gripper_sweep_stop,
    j4_roll_state,
    key_down,
    pressed_cart_vector,
    run_home,
    speed_us_from_stick_factor,
    stop_jog,
    sync_cart_jog,
    wait_until_released,
)
from mt4_jog.console import print_status
from mt4_jog.envelope import (
    DEFAULT_ENVELOPE_PATH,
    Label,
    append_sample,
    format_counts,
    format_sample_line,
    load_doc,
    sample_from_status,
    save_doc,
    undo_last_sample,
)
from mt4_jog.gamepad import (
    A,
    B,
    BACK,
    DPAD_DOWN,
    DPAD_UP,
    LEFT_SHOULDER,
    START,
    X,
    XboxGamepad,
)
from mt4_jog.joints import DEFAULT_BAUD, J1_HOME_CENTER_STEPS, J2_HOME_PULLOFF_STEPS
from mt4_jog.ports import Mt4PortError, port_display, resolve_port
from mt4_jog.serial import (
    FirmwareNotReadyError,
    SerialGoneError,
    await_firmware_alive,
    open_serial,
    send,
    send_quick,
)
from mt4_jog.status import Mt4Status, parse_status_lines

# Envelope-only keys (] = in, [ = out, backspace = undo, enter = save).
VK.update(
    {
        "rbrack": 0xDD,
        "lbrack": 0xDB,
        "backspace": 0x08,
        "enter": 0x0D,
    }
)

STATUS_WAIT_S = 1.0


def print_help(*, gamepad: bool, output: Path) -> None:
    print("MT4 envelope map — jog, then tag in/out poses")
    print(f"Output: {output}")
    print()
    print("Jog (as jog.py):")
    print("  I/K  A/D  S/W   world Z / X / Y      J / L   J4 roll")
    print("  Q / E           gripper open / close  - / =  keyboard speed")
    print("  `               toggle invert world X + Y")
    print()
    print("Envelope:")
    print("  ] / D-pad Up      record in-range")
    print("  [ / D-pad Down    record out-of-range")
    print("  Backspace / LB    undo last sample")
    print("  ENTER / Start     save JSON (keep jogging)")
    print("  H / A             home")
    print("  SPACE / X         status")
    print("  0 / B             stop, drivers off")
    print("  ESC / Back        save and quit")
    if not gamepad:
        print()
        print("(no gamepad — keyboard only)")
    print()


def query_status(ser, buf: list[str], verbose: bool) -> Mt4Status:
    lines = send(ser, "?", wait=STATUS_WAIT_S)
    drain_async(ser, buf, verbose)
    return parse_status_lines(lines)


def record_pose(
    ser,
    buf: list[str],
    verbose: bool,
    doc: dict,
    path: Path,
    label: Label,
) -> None:
    status = query_status(ser, buf, verbose)
    if not status.homed:
        print("Record failed: arm reports not homed -- press H to home first")
        return
    sample = sample_from_status(status, label)
    if sample is None:
        print("Record failed: no TCP/joint pose in status reply, try again")
        return
    sample = append_sample(doc, sample)
    save_doc(path, doc)
    print(f"{format_sample_line(sample)}  ({format_counts(doc)})")


def undo_pose(doc: dict, path: Path) -> None:
    removed = undo_last_sample(doc)
    if removed is None:
        print("Nothing to undo")
        return
    save_doc(path, doc)
    print(f"{format_sample_line(removed, verb='undid')}  ({format_counts(doc)})")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Jog and record in/out-of-range envelope samples to JSON"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_ENVELOPE_PATH,
        help=f"JSON path (default: {DEFAULT_ENVELOPE_PATH})",
    )
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--poll-ms", type=int, default=POLL_MS)
    parser.add_argument("--j1-center", type=int, default=J1_HOME_CENTER_STEPS)
    parser.add_argument("--j2-pull", type=int, default=J2_HOME_PULLOFF_STEPS)
    parser.add_argument("--no-orient", action="store_true")
    parser.add_argument("--no-gamepad", action="store_true")
    parser.add_argument("--gamepad-deadzone", type=int, default=9000)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    if sys.platform != "win32":
        print("Requires Windows (GetAsyncKeyState / XInput)", file=sys.stderr)
        return 1

    output: Path = args.output
    doc = load_doc(output)
    if doc["samples"]:
        print(f"Loaded {len(doc['samples'])} existing sample(s) from {output}")
        print(f"  {format_counts(doc)}")

    gamepad = None
    if not args.no_gamepad:
        gamepad = XboxGamepad(deadzone=args.gamepad_deadzone)
        if not gamepad.available:
            print("XInput not available — keyboard only", file=sys.stderr)
            gamepad = None

    try:
        port = resolve_port(args.port, baud=args.baud)
    except Mt4PortError as exc:
        print(exc, file=sys.stderr)
        return 1

    print_help(gamepad=gamepad is not None, output=output)
    print(port_display(port, baud=args.baud, explicit=args.port is not None))

    poll_s = args.poll_ms / 1000.0
    buf: list[str] = [""]
    active_cart: tuple[tuple[int, int, int] | None, int] | None = None
    grip_state: str | None = None
    last_cj_send = 0.0
    last_grip_send = 0.0
    keyboard_speed_us = DEFAULT_SPEED_US
    applied_speed_us = keyboard_speed_us
    last_speed_adjust = 0.0
    invert_xy = False
    grave_was_down = False
    rbrack_was_down = False
    lbrack_was_down = False
    backspace_was_down = False

    with open_serial(port, args.baud) as ser:
        try:
            await_firmware_alive(ser, port_label=port)
            send(ser, "all f", wait=0.5)
            send(ser, "orient off" if args.no_orient else "orient on", wait=0.3)
            status = query_status(ser, buf, args.verbose)
            if status.homed:
                print("Already homed (press H anytime to re-home)")
            else:
                print("Homing first (clear the workspace)...")
                run_home(ser, buf, args.j1_center, args.j2_pull, args.verbose)
            print("Ready — tag in/out poses with D-pad Up/Down or ] / [")
        except (FirmwareNotReadyError, SerialGoneError) as exc:
            print(exc, file=sys.stderr)
            return 1

        try:
            while True:
                drain_async(ser, buf, args.verbose)

                pad = gamepad.poll() if gamepad is not None else None
                pad_cart = pad.cart if pad and pad.connected else None
                pad_j4 = pad.j4 if pad and pad.connected else None
                pad_grip = pad.grip if pad and pad.connected else None
                pad_edges = pad.edges if pad and pad.connected else 0
                now = time.monotonic()

                if key_down("esc") or pad_edges & BACK:
                    save_doc(output, doc)
                    print(f"Saved {output} ({format_counts(doc)})")
                    break

                if key_down("enter") or pad_edges & START:
                    active_cart, grip_state = clear_active_motion(ser)
                    save_doc(output, doc)
                    print(f"Saved {output} ({format_counts(doc)})")
                    wait_until_released(
                        ser,
                        buf,
                        args.verbose,
                        poll_s,
                        gamepad=gamepad,
                        keyboard_keys=("enter",),
                        gamepad_mask=START,
                    )
                    continue

                # Record in / out
                rbrack_down = key_down("rbrack")
                if (rbrack_down and not rbrack_was_down) or (pad_edges & DPAD_UP):
                    active_cart, grip_state = clear_active_motion(ser)
                    record_pose(ser, buf, args.verbose, doc, output, "in")
                    if pad_edges & DPAD_UP:
                        wait_until_released(
                            ser, buf, args.verbose, poll_s,
                            gamepad=gamepad, gamepad_mask=DPAD_UP,
                        )
                rbrack_was_down = rbrack_down

                lbrack_down = key_down("lbrack")
                if (lbrack_down and not lbrack_was_down) or (pad_edges & DPAD_DOWN):
                    active_cart, grip_state = clear_active_motion(ser)
                    record_pose(ser, buf, args.verbose, doc, output, "out")
                    if pad_edges & DPAD_DOWN:
                        wait_until_released(
                            ser, buf, args.verbose, poll_s,
                            gamepad=gamepad, gamepad_mask=DPAD_DOWN,
                        )
                lbrack_was_down = lbrack_down

                backspace_down = key_down("backspace")
                if (backspace_down and not backspace_was_down) or (
                    pad_edges & LEFT_SHOULDER
                ):
                    undo_pose(doc, output)
                    if pad_edges & LEFT_SHOULDER:
                        wait_until_released(
                            ser, buf, args.verbose, poll_s,
                            gamepad=gamepad, gamepad_mask=LEFT_SHOULDER,
                        )
                backspace_was_down = backspace_down

                grave_down = key_down("grave")
                if grave_down and not grave_was_down:
                    invert_xy = not invert_xy
                    print(f"XY invert: {'on' if invert_xy else 'off'}")
                    if active_cart is not None:
                        stop_jog(ser)
                        active_cart = None
                grave_was_down = grave_down

                if key_down("space") or pad_edges & X:
                    active_cart, grip_state = clear_active_motion(ser)
                    print_status(send(ser, "?", wait=1.0))
                    wait_until_released(
                        ser, buf, args.verbose, poll_s, gamepad=gamepad,
                        keyboard_keys=("space",), gamepad_mask=X,
                    )
                    continue

                if key_down("0") or pad_edges & B:
                    active_cart, grip_state = clear_active_motion(ser)
                    send(ser, "e0", wait=0.2)
                    send(ser, "all f", wait=0.3)
                    wait_until_released(
                        ser, buf, args.verbose, poll_s, gamepad=gamepad,
                        keyboard_keys=("0",), gamepad_mask=B,
                    )
                    continue

                if key_down("h") or pad_edges & A:
                    active_cart, grip_state = clear_active_motion(ser)
                    run_home(ser, buf, args.j1_center, args.j2_pull, args.verbose)
                    wait_until_released(
                        ser, buf, args.verbose, poll_s, gamepad=gamepad,
                        keyboard_keys=("h",), gamepad_mask=A,
                    )
                    continue

                minus = key_down("minus")
                plus = key_down("plus")
                if (minus or plus) and now - last_speed_adjust >= SPEED_REPEAT_S:
                    keyboard_speed_us += -SPEED_STEP_US if plus else SPEED_STEP_US
                    keyboard_speed_us = max(
                        SPEED_MIN_US, min(SPEED_MAX_US, keyboard_speed_us)
                    )
                    last_speed_adjust = now
                    print(f"Speed: {keyboard_speed_us}")

                stick_factor = (
                    pad.speed_factor
                    if pad is not None and pad.connected
                    else None
                )
                if stick_factor is not None:
                    stick_speed = speed_us_from_stick_factor(stick_factor)
                    if stick_speed != applied_speed_us:
                        send_quick(ser, f"speed {stick_speed}")
                        applied_speed_us = stick_speed
                elif applied_speed_us != keyboard_speed_us:
                    send_quick(ser, f"speed {keyboard_speed_us}")
                    applied_speed_us = keyboard_speed_us

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
        except SerialGoneError as exc:
            print(exc, file=sys.stderr)
            save_doc(output, doc)
            return 1
        except KeyboardInterrupt:
            print()
            save_doc(output, doc)
            print(f"Saved {output} ({format_counts(doc)})")
        finally:
            try:
                stop_jog(ser)
                gripper_sweep_stop(ser)
                send(ser, "e0", wait=0.2)
                send(ser, "all f", wait=0.3)
            except SerialGoneError:
                pass

    print("Bye")
    return 0


if __name__ == "__main__":
    try:
        _exit_code = main()
    finally:
        flush_console_input()
    raise SystemExit(_exit_code)
