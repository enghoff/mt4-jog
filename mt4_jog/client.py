"""Long-lived serial client for MT4 jog firmware."""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

from mt4_jog.joints import (
    DEFAULT_BAUD,
    DEFAULT_PORT,
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    GROUND_Z_MM,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
    JOG_SPEED_MAX_US,
    JOG_SPEED_MIN_US,
)
from mt4_jog.serial import (
    FirmwareNotReadyError,
    STATUS_WAIT_S,
    SerialGoneError,
    await_firmware_alive,
    close_quiet,
    open_serial,
    read_lines,
    send,
    write_raw,
)
from mt4_jog.status import Mt4Status, TcpPose, parse_status_lines, parse_tcp_line

if TYPE_CHECKING:
    import serial as serial_module

COMMAND_WAIT_S = 0.5
GRIP_WAIT_S = 1.0
# Firmware acks an absolute `g <value>` as soon as it parses the command, not
# once the servo physically arrives -- and read_lines() (see serial.py) exits
# on serial silence, which follows that immediate ack by ~100ms, long before
# the ~700ms the gripper actually takes to reach its target. Without this,
# gripper() returns while the servo is still mid-travel, so pick()/place()'s
# next move_to() starts while the grip hasn't closed (or opened) yet.
GRIPPER_SETTLE_S = 0.8
# Homing seeks limit switches with no position feedback beforehand, so it
# needs the same generous ceiling as jog.py's HOME_WAIT_S.
HOME_TIMEOUT_S = 180.0
# If firmware hasn't acked a `home` command (home start/fail/err) within
# this long, assume the line was swallowed (e.g. an MCU reset between
# connect and the write) and re-send once. Harmless if homing actually is
# in flight: do_home()'s serial_abort() reads and discards any line that
# isn't "!"/"stop".
HOME_ACK_RESEND_S = 5.0
# Matches goto_position.py's MOVE_TIMEOUT_S for coordinated m/mp moves.
MOVE_TIMEOUT_S = 30.0
# Cap line collection so a chatty firmware stream cannot block forever.
READ_HARD_LIMIT_S = 1.0
# Ceiling on how long a single read_lines() poll waits while draining an
# async move/home completion line, so the overall timeout loop stays responsive.
POLL_WAIT_S = 0.3


class Mt4ClientError(Exception):
    """Raised when the arm is unreachable or returns an unexpected response."""


class Mt4Client:
    """Thread-safe owner of the MT4 serial connection."""

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
    ) -> None:
        self.port = port
        self.baud = baud
        self._ser: serial_module.Serial | None = None
        # Reentrant so methods like move_to() can call _get_status_unlocked()
        # (to default J4 / check homed) while already holding the lock.
        self._lock = threading.RLock()
        self._interrupt = threading.Event()

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def ensure_connected(self) -> None:
        with self._lock:
            self._ensure_connected_unlocked()

    def _ensure_connected_unlocked(self) -> None:
        if self.connected:
            return
        try:
            self._ser = open_serial(self.port, self.baud)
        except Exception as exc:
            raise Mt4ClientError(
                f"Could not open {self.port} @ {self.baud} baud: {exc}"
            ) from exc
        try:
            await_firmware_alive(self._ser, port_label=self.port or "auto-detected port")
        except FirmwareNotReadyError as exc:
            close_quiet(self._ser)
            self._ser = None
            raise Mt4ClientError(str(exc)) from exc
        except SerialGoneError as exc:
            close_quiet(self._ser)
            self._ser = None
            raise Mt4ClientError(str(exc)) from exc
        except Exception:
            close_quiet(self._ser)
            self._ser = None
            raise

    def close(self) -> None:
        with self._lock:
            close_quiet(self._ser)
            self._ser = None

    def _require_serial(self) -> serial_module.Serial:
        if not self.connected:
            raise Mt4ClientError("Not connected to the arm")
        assert self._ser is not None
        return self._ser

    def _mark_gone_unlocked(self, exc: SerialGoneError) -> None:
        close_quiet(self._ser)
        self._ser = None
        raise Mt4ClientError(str(exc)) from exc

    def _send_and_collect(self, cmd: str, wait: float) -> list[str]:
        ser = self._require_serial()
        try:
            return send(
                ser,
                cmd,
                wait=wait,
                hard_limit=wait + READ_HARD_LIMIT_S,
            )
        except SerialGoneError as exc:
            self._mark_gone_unlocked(exc)
            raise  # unreachable; keeps type-checkers happy

    def _drain_serial_unlocked(self) -> list[str]:
        """Discard any unread firmware lines (stale status, prior home ok, etc.)."""
        ser = self._require_serial()
        drained: list[str] = []
        try:
            while True:
                batch = read_lines(ser, 0.05, hard_limit=0.12)
                if not batch:
                    break
                drained.extend(batch)
        except SerialGoneError as exc:
            self._mark_gone_unlocked(exc)
            raise
        return drained

    def _get_status_unlocked(self, *, retries: int = 1) -> Mt4Status:
        self._ensure_connected_unlocked()
        status = Mt4Status()
        for attempt in range(retries):
            if attempt > 0:
                time.sleep(0.15)
            lines = self._send_and_collect("?", wait=STATUS_WAIT_S)

            # `?` and `pos` both print the same joint-position + derived
            # `tcp x=...` line (see firmware print_joint_pos()), so retrying
            # with the lighter `pos` query is a real fallback if `?`'s fuller
            # reply got cut off. There's no separate firmware "tcp" command.
            if not lines:
                lines = self._send_and_collect("pos", wait=1.5)

            status = parse_status_lines(lines)
            status.parse_failed = status.tcp is None and not status.joints
            if not status.parse_failed:
                return status
        return status

    def get_status(self) -> Mt4Status:
        with self._lock:
            return self._get_status_unlocked(retries=3)

    def get_tcp(self) -> TcpPose:
        status = self.get_status()
        if status.tcp is None:
            raise Mt4ClientError("Could not read TCP pose")
        return status.tcp

    def stop(self) -> list[str]:
        with self._lock:
            self._ensure_connected_unlocked()
            return self._send_and_collect("stop", wait=COMMAND_WAIT_S)

    def request_interrupt(self) -> None:
        """Ask an in-flight move/gripper settle to abort (e.g. shuffle home key)."""
        self._interrupt.set()

    def clear_interrupt(self) -> None:
        self._interrupt.clear()

    def _sleep_interruptible(self, seconds: float) -> bool:
        """Sleep in short slices; return True if interrupted."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._interrupt.is_set():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.05, remaining))
        return False

    def _abort_if_interrupted_unlocked(self, collected: list[str]) -> dict[str, object] | None:
        if not self._interrupt.is_set():
            return None
        self._send_and_collect("stop", wait=COMMAND_WAIT_S)
        return {"ok": False, "error": "interrupted", "lines": collected}

    def _await_completion(
        self,
        *,
        done_prefix: str,
        timeout: float,
        collected: list[str],
    ) -> dict[str, object]:
        """Poll for an async `<cmd> done ...` / `err ...` line after an `ok`
        ack, matching the pattern goto_position.py's send_move() already uses
        for `m`/`mp`. Returns a dict with "ok", "lines", and (on success) the
        parsed final "tcp" pose if one was printed.

        `collected` may already contain the terminal line -- a no-op move
        (e.g. all-zero relative deltas) gets "ok m" + "m done ..." back
        synchronously in the same initial read, with nothing further to
        poll for -- so every batch (including the one passed in) is scanned
        before waiting for more.
        """

        def scan(batch: list[str]) -> dict[str, object] | None:
            for line in batch:
                if line.startswith("err"):
                    return {"ok": False, "error": line, "lines": collected}
                if line.startswith(done_prefix):
                    tcp = next(
                        (parse_tcp_line(candidate) for candidate in batch),
                        None,
                    )
                    return {
                        "ok": True,
                        "lines": collected,
                        "tcp": tcp.as_dict() if tcp else None,
                    }
            return None

        result = scan(collected)
        if result is not None:
            return result

        ser = self._require_serial()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            aborted = self._abort_if_interrupted_unlocked(collected)
            if aborted is not None:
                return aborted
            try:
                lines = read_lines(
                    ser, POLL_WAIT_S, hard_limit=POLL_WAIT_S + READ_HARD_LIMIT_S
                )
            except SerialGoneError as exc:
                self._mark_gone_unlocked(exc)
                raise
            collected.extend(lines)
            result = scan(lines)
            if result is not None:
                return result
        return {"ok": False, "error": f"Timed out waiting for {done_prefix!r}", "lines": collected}

    def home(
        self,
        j1_center: int = J1_HOME_CENTER_STEPS,
        j2_pull: int = J2_HOME_PULLOFF_STEPS,
        timeout: float = HOME_TIMEOUT_S,
    ) -> dict[str, object]:
        """Run on-device homing (`home <j1> <j2>`) and wait for completion."""
        with self._lock:
            self.clear_interrupt()
            self._ensure_connected_unlocked()
            ser = self._require_serial()
            self._drain_serial_unlocked()
            cmd = f"home {j1_center} {j2_pull}\n".encode("ascii")
            try:
                write_raw(ser, cmd)
            except SerialGoneError as exc:
                self._mark_gone_unlocked(exc)
                raise
            collected: list[str] = []
            saw_home_start = False
            resent = False
            ack_deadline = time.monotonic() + HOME_ACK_RESEND_S
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if (
                    not saw_home_start
                    and not resent
                    and time.monotonic() > ack_deadline
                ):
                    # No ack yet -- the command was likely swallowed (MCU
                    # reset since connect). Re-send once; if homing actually
                    # is running, the firmware discards this line unread.
                    try:
                        write_raw(ser, cmd)
                    except SerialGoneError as exc:
                        self._mark_gone_unlocked(exc)
                        raise
                    resent = True
                try:
                    lines = read_lines(
                        ser, POLL_WAIT_S, hard_limit=POLL_WAIT_S + READ_HARD_LIMIT_S
                    )
                except SerialGoneError as exc:
                    self._mark_gone_unlocked(exc)
                    raise
                collected.extend(lines)
                for line in lines:
                    if line == "home start":
                        saw_home_start = True
                    if line == "home ok" and saw_home_start:
                        self._drain_serial_unlocked()
                        status = self._get_status_unlocked(retries=3)
                        return {
                            "ok": True,
                            "lines": collected,
                            "homed": status.homed,
                            "tcp": status.tcp.as_dict() if status.tcp else None,
                            "parse_failed": status.parse_failed,
                        }
                    if line.startswith("home fail") or line.startswith("err"):
                        return {"ok": False, "error": line, "lines": collected}
            return {"ok": False, "error": "Homing timed out", "lines": collected}

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        j4: float | None = None,
        grip: int = 0,
        speed_us: int = 0,
        timeout: float = MOVE_TIMEOUT_S,
    ) -> dict[str, object]:
        """Absolute Cartesian move (`mp`). Requires homing this session.

        `grip=0` leaves the gripper unchanged (firmware convention). When
        `j4` is left as None, it defaults to the arm's *current* world-frame
        yaw, which makes the firmware hold gripper orientation fixed during
        the move (same as `orient on`).
        """
        if speed_us and not JOG_SPEED_MIN_US <= speed_us <= JOG_SPEED_MAX_US:
            raise Mt4ClientError(
                f"speed_us must be 0 or {JOG_SPEED_MIN_US}-{JOG_SPEED_MAX_US}"
            )
        if grip and not GRIPPER_S_OPEN <= grip <= GRIPPER_S_CLOSED:
            raise Mt4ClientError(
                f"grip must be 0 (unchanged) or {GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED}"
            )
        if z < GROUND_Z_MM - 0.05:
            raise Mt4ClientError(
                f"z={z:.1f} is below ground plane ({GROUND_Z_MM:.1f}mm)"
            )

        with self._lock:
            status = self._get_status_unlocked(retries=3)
            if not status.homed:
                raise Mt4ClientError(
                    "Arm has not homed this session -- call home() first"
                )
            if j4 is None:
                if status.tcp is None:
                    raise Mt4ClientError(
                        "Could not read current TCP pose to default j4"
                    )
                j4 = status.tcp.j4

            # Firmware line_buf is 64 bytes; full Python float repr of
            # detection XY + j4 + speed easily overflows and truncates mid-
            # token (parse fails with the usage string). Match the firmware's
            # own 0.1mm / 0.1deg reporting precision.
            cmd = f"mp {x:.2f} {y:.2f} {z:.2f} {j4:.2f} {int(grip)}"
            if speed_us:
                cmd += f" {int(speed_us)}"
            if len(cmd) >= 64:
                raise Mt4ClientError(
                    f"mp command exceeds 64-byte firmware line buffer: {cmd!r}"
                )
            collected = self._send_and_collect(cmd, wait=1.0)
            return self._await_completion(
                done_prefix="mp done", timeout=timeout, collected=collected
            )

    def move_relative(
        self,
        dj1: int,
        dj2: int,
        dj3: int,
        dj4: int,
        dgrip: int = 0,
        timeout: float = MOVE_TIMEOUT_S,
    ) -> dict[str, object]:
        """Bounded relative joint-step move (`m`). Does not require homing --
        deltas are relative to the current step counters, whatever they are.
        """
        with self._lock:
            self._ensure_connected_unlocked()
            cmd = f"m {dj1} {dj2} {dj3} {dj4} {dgrip}"
            collected = self._send_and_collect(cmd, wait=1.0)
            return self._await_completion(
                done_prefix="m done", timeout=timeout, collected=collected
            )

    def gripper(self, action: str | int) -> dict[str, object]:
        """Gripper sweep/set (`g o|c|stop|<120-285>`).

        An absolute target (int) blocks until the servo has had time to
        physically arrive (see GRIPPER_SETTLE_S) -- the firmware only acks
        that it parsed the command, not that motion finished, so callers
        that move the arm right after (pick()/place()) would otherwise race
        a gripper that's still mid-travel. Sweep actions ('open'/'close'/
        'stop') are open-ended/continuous, so they return as soon as sent.
        """
        settle = False
        if isinstance(action, str):
            normalized = action.strip().lower()
            arg = {"open": "o", "close": "c", "stop": "stop"}.get(normalized)
            if arg is None:
                raise Mt4ClientError(
                    "action must be 'open', 'close', 'stop', or an int "
                    f"{GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED}"
                )
        else:
            if not GRIPPER_S_OPEN <= action <= GRIPPER_S_CLOSED:
                raise Mt4ClientError(
                    f"action must be {GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED}"
                )
            arg = str(int(action))
            settle = True

        with self._lock:
            self._ensure_connected_unlocked()
            lines = self._send_and_collect(f"g {arg}", wait=GRIP_WAIT_S)
        for line in lines:
            if line.startswith("err"):
                return {"ok": False, "error": line, "lines": lines}
        if settle:
            with self._lock:
                if self._sleep_interruptible(GRIPPER_SETTLE_S):
                    self._send_and_collect("stop", wait=COMMAND_WAIT_S)
                    return {"ok": False, "error": "interrupted", "lines": lines}
        return {"ok": True, "lines": lines}
