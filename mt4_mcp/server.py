"""Local HTTP MCP server for MT4 Cartesian control and status."""

from __future__ import annotations

import os
from contextlib import asynccontextmanager
from typing import Any

from dotenv import load_dotenv
from fastmcp import FastMCP

from mt4_jog.client import Mt4Client, Mt4ClientError
from mt4_jog.joints import DEFAULT_BAUD
from mt4_jog.joints import DEFAULT_PORT as DEFAULT_SERIAL_PORT
from mt4_mcp.auth import build_auth_provider, oauth_enabled

load_dotenv()

DEFAULT_HOST = "127.0.0.1"
DEFAULT_MCP_PORT = 8787

_client: Mt4Client | None = None


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return int(raw)


def get_client() -> Mt4Client:
    if _client is None:
        raise RuntimeError("MT4 client is not initialized")
    _client.ensure_connected()
    return _client


@asynccontextmanager
async def lifespan(_app: FastMCP):
    global _client
    serial_port = os.environ.get("MT4_SERIAL_PORT", DEFAULT_SERIAL_PORT)
    baud = _env_int("MT4_BAUD", DEFAULT_BAUD)
    _client = Mt4Client(port=serial_port, baud=baud)
    try:
        yield
    finally:
        if _client is not None:
            _client.close()
            _client = None


def create_mcp(*, auth: Any | None = None) -> FastMCP:
    server = FastMCP(
        name="MT4 Robot",
        instructions=(
            "Control and read status from a WLKATA MT4 arm over serial. "
            "TCP x/y/z are in mm with origin at the base under J1's pivot. "
            "j4 is world-frame gripper yaw in degrees. "
            "Execute motion commands directly when asked -- never ask the "
            "user to confirm before calling a tool. mt4_move_to and "
            "mt4_move_relative move the arm immediately. Check mt4_status "
            "first when you need the current pose or homed flag. "
            "mt4_move_to requires homing this session (mt4_home) first; "
            "mt4_home returns homed and tcp on success. "
            "For pick-and-place of colored cubes on the work surface, use "
            "mt4_scene to see cube positions, mt4_pick_cube to grab one by "
            "color, and mt4_place_at to set it down at robot-frame x/y."
        ),
        auth=auth,
        lifespan=lifespan,
    )

    @server.tool
    def mt4_status() -> dict[str, Any]:
        """Full arm status: homed flag, mode, joints, TCP pose, drivers, jog."""
        try:
            return get_client().get_status().as_dict()
        except Mt4ClientError as exc:
            return {"error": str(exc)}

    @server.tool
    def mt4_tcp() -> dict[str, Any]:
        """Current Cartesian TCP pose (x/y/z mm, world-frame j4 deg, grip, speed)."""
        try:
            return get_client().get_tcp().as_dict()
        except Mt4ClientError as exc:
            return {"error": str(exc)}

    @server.tool
    def mt4_stop() -> dict[str, Any]:
        """Stop jog and cancel any in-progress coordinated move."""
        try:
            lines = get_client().stop()
            return {"ok": True, "lines": lines}
        except Mt4ClientError as exc:
            return {"ok": False, "error": str(exc)}

    @server.tool
    def mt4_home() -> dict[str, Any]:
        """Home J1 and J2 by driving them into their limit switches, then
        reference J3 indirectly through J2's switch (J3 has no switch of its
        own). Required once per power cycle/session before mt4_move_to will
        accept absolute moves -- check mt4_status's `homed` field first to
        see if this is even necessary.

        Runs immediately, no confirmation or workspace check required --
        call directly. The arm moves on its own, and both J1 and J2 travel
        to their hard limit switches during the seek. Takes up to ~30s;
        can take longer (up to 180s) if a limit switch isn't found on the
        first pass. On success, returns `homed` and `tcp` from a fresh
        status query so callers don't need a separate mt4_status round-trip.
        """
        try:
            return get_client().home()
        except Mt4ClientError as exc:
            return {"ok": False, "error": str(exc)}

    @server.tool
    def mt4_move_to(
        x: float,
        y: float,
        z: float,
        j4: float | None = None,
        grip: int = 0,
        speed_us: int = 0,
    ) -> dict[str, Any]:
        """Move the TCP to an absolute Cartesian position in a straight
        world-frame line (firmware `mp`). Requires the arm to have homed
        this session (mt4_status's `homed` field) -- call mt4_home first if
        not. Blocks until the move completes or times out (~30s), then
        returns the arm's final pose.

        Args:
            x: Target TCP X in mm, origin at the base under J1's pivot.
            y: Target TCP Y in mm.
            z: Target TCP Z in mm.
            j4: Target gripper yaw in world-frame degrees. If omitted, the
                current yaw is reused, which makes the firmware hold gripper
                orientation fixed in world space during the move (like
                `orient on`) rather than rotating it.
            grip: Absolute gripper position, 120 (open) to 285 (closed).
                0 (default) leaves the gripper wherever it currently is.
            speed_us: Step period in microseconds, 700 (fast) to 4000
                (slow). 0 (default) leaves the current speed unchanged.
        """
        try:
            return get_client().move_to(
                x, y, z, j4=j4, grip=grip, speed_us=speed_us
            )
        except Mt4ClientError as exc:
            return {"ok": False, "error": str(exc)}

    @server.tool
    def mt4_move_relative(
        dj1: int,
        dj2: int,
        dj3: int,
        dj4: int,
        dgrip: int = 0,
    ) -> dict[str, Any]:
        """Nudge each joint by a relative step count, all axes finishing
        together (firmware `m`). Does not require homing -- deltas are
        relative to whatever the current step counters are. Prefer
        mt4_move_to for absolute Cartesian targets once homed; use this for
        small joint-space nudges or before homing has run. Blocks until the
        move completes or times out (~30s).

        Args:
            dj1: J1 (base) step delta, signed.
            dj2: J2 (shoulder) step delta, signed.
            dj3: J3 (elbow) step delta, signed.
            dj4: J4 (wrist) step delta, signed.
            dgrip: Gripper S-value delta, signed (gripper spans 120-285).
                0 (default) leaves the gripper unchanged.
        """
        try:
            return get_client().move_relative(dj1, dj2, dj3, dj4, dgrip=dgrip)
        except Mt4ClientError as exc:
            return {"ok": False, "error": str(exc)}

    @server.tool
    def mt4_gripper(action: str | int) -> dict[str, Any]:
        """Open, close, stop, or set the gripper (firmware `g`).
        Args:
            action: One of the strings "open", "close", "stop" (start/stop
                a sweep between the gripper's travel limits), or an absolute
                integer S-value from 120 (fully open) to 285 (fully closed).
        """
        try:
            return get_client().gripper(action)
        except Mt4ClientError as exc:
            return {"ok": False, "error": str(exc)}

    # Vision tools import cv2 lazily so the motion tools keep working on
    # hosts without the camera stack installed.
    @server.tool
    def mt4_scene() -> dict[str, Any]:
        """Detect colored cubes on the work surface via the overhead camera.
        Returns each cube's color and robot-frame x/y (mm) -- pass those
        straight to mt4_pick_cube/mt4_place_at/mt4_move_to. Detections are
        from a fresh frame, so re-call this after anything moves. Requires
        the camera calibration produced by `python calibrate_vision.py`.
        """
        try:
            from mt4_vision.calib import load_calibration
            from mt4_vision.camera import capture_frame
            from mt4_vision.workspace import analyze_workspace

            calib = load_calibration()
            state = analyze_workspace(calib, capture_frame())
            return {
                "ok": True,
                "cubes": [c.as_dict() for c in state.cubes],
                "markers": {
                    "free": [
                        {"id": m.marker_id, "x": m.x, "y": m.y}
                        for m in state.free_markers
                    ],
                    "occupied": [
                        {
                            "id": m.marker_id,
                            "x": m.x,
                            "y": m.y,
                            "color": c.color,
                        }
                        for m, c in state.occupied
                    ],
                },
                "free_slots": [{"x": x, "y": y} for x, y in state.free_slots],
            }
        except Exception as exc:  # noqa: BLE001 -- surface camera/calib errors to the model
            return {"ok": False, "error": str(exc)}

    @server.tool
    def mt4_pick_cube(color: str) -> dict[str, Any]:
        """Pick up a cube by color: re-detects it on a fresh camera frame,
        then opens the gripper, descends over the cube, grips, and lifts to
        the calibrated safe height. Homes first if needed. If several cubes
        of the color are in view, picks the largest detection; use mt4_scene
        + mt4_move_to for finer control.

        Args:
            color: Cube color name, e.g. "red", "green", "blue", "yellow".
        """
        try:
            from mt4_vision.calib import load_calibration
            from mt4_vision.camera import capture_frame
            from mt4_vision.detect import detect_cubes
            from mt4_vision.pickplace import pick
            from mt4_vision.scene import filter_phantoms
            from mt4_vision.workspace import (
                cubes_of_color,
                cubes_with_robot_coords,
                marker_slots_from_calibration,
                pick_largest_cube,
            )

            calib = load_calibration()
            # Phantom-filter so arm-base blobs (which can out-area a real
            # cube of the same color) never win the largest-detection pick.
            candidates = filter_phantoms(
                cubes_with_robot_coords(detect_cubes(capture_frame(), calib)),
                marker_slots_from_calibration(calib),
            )
            target = pick_largest_cube(cubes_of_color(candidates, color))
            if target is None:
                return {"ok": False, "error": f"no {color} cube in view"}
            result = pick(get_client(), calib, target.x, target.y)
            result["color"] = color
            return result
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

    @server.tool
    def mt4_place_at(x: float, y: float) -> dict[str, Any]:
        """Place the currently-held cube at robot-frame (x, y): moves there
        at the calibrated safe height, descends, releases, and lifts clear.
        Get target coordinates from mt4_scene (e.g. next to another cube --
        offset by at least one cube width, ~35mm, to avoid collision).

        Args:
            x: Target X in mm (robot frame).
            y: Target Y in mm (robot frame).
        """
        try:
            from mt4_vision.calib import load_calibration
            from mt4_vision.pickplace import place

            calib = load_calibration()
            return place(get_client(), calib, x, y)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

    @server.tool
    def mt4_goto_marker(marker_id: int, touch: bool = False) -> dict[str, Any]:
        """Move the TCP to a calibration ArUco marker's position -- a
        calibration accuracy check, not a normal operation. Re-detects the
        marker on a fresh frame and converts its pixel position through the
        calibration. Hovers at the calibrated safe height by default (won't
        crash into the table even if the calibration is off); pass
        touch=true to descend and physically touch the table at that spot.

        Args:
            marker_id: ArUco marker id to move to (see mt4_scene's calibration
                or the physical markers on the work surface).
            touch: If true, descend to table height instead of hovering.
        """
        try:
            from mt4_vision.calib import load_calibration
            from mt4_vision.camera import capture_frame
            from mt4_vision.detect import detect_markers
            from mt4_vision.pickplace import goto_marker

            calib = load_calibration()
            markers = detect_markers(capture_frame())
            match = next((m for m in markers if m.marker_id == marker_id), None)
            if match is None:
                return {
                    "ok": False,
                    "error": f"marker {marker_id} not in view "
                    f"(visible: {sorted(m.marker_id for m in markers)})",
                }
            x, y = calib.pixel_to_robot(match.px, match.py)
            result = goto_marker(get_client(), calib, x, y, touch=touch)
            result["marker_id"] = marker_id
            return result
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

    return server


# Default module-level server for imports/tests (no OAuth).
mcp = create_mcp()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="MT4 MCP server")
    parser.add_argument(
        "--stdio",
        action="store_true",
        help="stdio transport for Cursor/Claude Desktop (default: HTTP)",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("MT4_MCP_HOST", DEFAULT_HOST),
        help="HTTP bind host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=_env_int("MT4_MCP_PORT", DEFAULT_MCP_PORT),
        help="HTTP port (default: 8787)",
    )
    args = parser.parse_args()

    if args.stdio:
        create_mcp().run(transport="stdio")
        return

    auth = build_auth_provider() if oauth_enabled() else None
    public = os.environ.get("MT4_MCP_PUBLIC", "").lower() in ("1", "true", "yes")
    http_kwargs: dict[str, object] = {
        "transport": "http",
        "host": args.host,
        "port": args.port,
        "path": os.environ.get("MT4_MCP_PATH", "/mcp"),
    }
    if public or auth is not None:
        # Allow ngrok / reverse-proxy Host headers through to the MCP endpoint.
        http_kwargs["host_origin_protection"] = False

    create_mcp(auth=auth).run(**http_kwargs)


if __name__ == "__main__":
    main()
