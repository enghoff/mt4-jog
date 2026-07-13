"""Indefinite cube shuffle loop (see shuffle_blocks.py CLI)."""

from __future__ import annotations

import time

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_vision.calib import Calibration
from mt4_vision.camera import capture_frame
from mt4_vision.pickplace import ensure_homed, pick, place
from mt4_vision.workspace import analyze_workspace, plan_shuffle_move


def run_shuffle_loop(
    client: Mt4Client,
    calib: Calibration,
    *,
    camera: int,
    pause_s: float = 2.0,
    retry_s: float = 5.0,
) -> None:
    """Home if needed, then shuffle cubes until interrupted."""
    if client.get_status().homed:
        print("already homed")
    else:
        print("homing...")
        ensure_homed(client)
        print("home ok")
    while True:
        frame = capture_frame(camera)
        state = analyze_workspace(calib, frame)
        move = plan_shuffle_move(state)
        if move is None:
            print(
                f"no move: cubes={len(state.cubes)} "
                f"free_markers={len(state.free_markers)} "
                f"occupied={len(state.occupied)} "
                f"free_slots={len(state.free_slots)} "
                f"-- retry in {retry_s:.0f}s"
            )
            time.sleep(retry_s)
            continue

        print(
            f"{move.kind}: pick {move.pick_color} at "
            f"({move.pick_x:.1f}, {move.pick_y:.1f}) -> "
            f"({move.place_x:.1f}, {move.place_y:.1f})"
        )
        try:
            pick(client, calib, move.pick_x, move.pick_y)
            place(client, calib, move.place_x, move.place_y)
        except Mt4ClientError as exc:
            print(f"move failed: {exc} -- retrying next cycle")
            time.sleep(retry_s)
            continue

        time.sleep(pause_s)
