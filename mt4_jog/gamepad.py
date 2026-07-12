"""Xbox controller input for MT4 jog (Windows XInput)."""

from __future__ import annotations

import sys
from dataclasses import dataclass

# XInput button masks (XINPUT_GAMEPAD_*).
DPAD_UP = 0x0001
DPAD_DOWN = 0x0002
DPAD_LEFT = 0x0004
DPAD_RIGHT = 0x0008
START = 0x0010
BACK = 0x0020
LEFT_THUMB = 0x0040
RIGHT_THUMB = 0x0080
LEFT_SHOULDER = 0x0100
RIGHT_SHOULDER = 0x0200
A = 0x1000
B = 0x2000
X = 0x4000
Y = 0x8000

ERROR_DEVICE_NOT_CONNECTED = 1167


@dataclass
class GamepadSnapshot:
    """Instantaneous controller state for one poll."""

    cart: tuple[int, int, int] | None
    j4: bool | None
    grip: str | None
    home: bool = False
    stop_all: bool = False
    status: bool = False
    quit: bool = False
    speed_up: bool = False
    speed_down: bool = False
    connected: bool = False


def _axis_sign(value: int, deadzone: int) -> int:
    if value > deadzone:
        return 1
    if value < -deadzone:
        return -1
    return 0


class XboxGamepad:
    """Poll player 1 Xbox / XInput-compatible gamepad."""

    def __init__(self, *, deadzone: int = 9000, trigger_threshold: int = 64) -> None:
        self.deadzone = deadzone
        self.trigger_threshold = trigger_threshold
        self._prev_buttons = 0
        self._last_buttons = 0
        self._available = False
        self._xinput = None
        self._state_type = None
        if sys.platform == "win32":
            self._init_xinput()

    @property
    def available(self) -> bool:
        return self._available

    def _init_xinput(self) -> None:
        import ctypes
        from ctypes import wintypes

        class XINPUT_GAMEPAD(ctypes.Structure):
            _fields_ = [
                ("wButtons", wintypes.WORD),
                ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short),
                ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short),
            ]

        class XINPUT_STATE(ctypes.Structure):
            _fields_ = [("dwPacketNumber", wintypes.DWORD), ("Gamepad", XINPUT_GAMEPAD)]

        for lib_name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
            try:
                lib = ctypes.WinDLL(lib_name)
            except OSError:
                continue
            get_state = lib.XInputGetState
            get_state.argtypes = [wintypes.DWORD, ctypes.POINTER(XINPUT_STATE)]
            get_state.restype = wintypes.DWORD
            self._xinput = get_state
            self._state_type = XINPUT_STATE
            self._available = True
            return

    def _button_edge(self, buttons: int, mask: int) -> bool:
        return bool(buttons & mask and not (self._prev_buttons & mask))

    def is_pressed(self, mask: int) -> bool:
        return bool(self._last_buttons & mask)

    def poll(self) -> GamepadSnapshot:
        if not self._available or self._xinput is None or self._state_type is None:
            return GamepadSnapshot(cart=None, j4=None, grip=None)

        import ctypes

        state = self._state_type()
        err = self._xinput(0, ctypes.byref(state))
        if err == ERROR_DEVICE_NOT_CONNECTED:
            self._prev_buttons = 0
            self._last_buttons = 0
            return GamepadSnapshot(cart=None, j4=None, grip=None)

        pad = state.Gamepad
        buttons = int(pad.wButtons)
        dz = self.deadzone

        # Left stick: world X/Y (both axes inverted). Right stick Y: world Z;
        # right stick X: J4 roll.
        x = _axis_sign(-int(pad.sThumbLX), dz)
        y = _axis_sign(-int(pad.sThumbLY), dz)
        z = _axis_sign(int(pad.sThumbRY), dz)
        cart = None
        if (x, y, z) != (0, 0, 0):
            cart = (x, y, z)

        j4: bool | None = None
        if cart is None:
            rx = _axis_sign(int(pad.sThumbRX), dz)
            if rx < 0:
                j4 = False
            elif rx > 0:
                j4 = True

        lt = int(pad.bLeftTrigger)
        rt = int(pad.bRightTrigger)
        grip: str | None = None
        if lt >= self.trigger_threshold and rt < self.trigger_threshold:
            grip = "open"
        elif rt >= self.trigger_threshold and lt < self.trigger_threshold:
            grip = "close"

        snap = GamepadSnapshot(
            cart=cart,
            j4=j4,
            grip=grip,
            home=self._button_edge(buttons, A),
            stop_all=self._button_edge(buttons, B),
            status=self._button_edge(buttons, X),
            quit=self._button_edge(buttons, BACK),
            speed_down=self._button_edge(buttons, LEFT_SHOULDER)
            or self._button_edge(buttons, DPAD_DOWN),
            speed_up=self._button_edge(buttons, RIGHT_SHOULDER)
            or self._button_edge(buttons, DPAD_UP),
            connected=True,
        )
        self._prev_buttons = buttons
        self._last_buttons = buttons
        return snap
