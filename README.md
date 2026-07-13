# MT4 custom jog

Keyboard jog and on-device homing for the WLKATA MT4 arm (ATmega2560 @ COM6, 115200).

Custom firmware (`firmware/mt4_jog/`) replaces the stock Grbl-derived firmware with a
4-axis step/dir jog engine plus an on-device Cartesian (world-frame) resolved-rate
jog. `jog_keyboard.py` is the main client: Cartesian jog is the sole motion mode
(direct per-joint jog was dropped), plus J4 wrist roll and the gripper. `goto_position.py`
is a second, prompt-driven client for one-shot absolute moves.

## Requirements

- Python 3.10+
- `pip install -r requirements.txt`
- [PlatformIO](https://platformio.org/) + avrdude (to flash firmware)
- Windows (keyboard client uses `GetAsyncKeyState`)

## Quick start

```powershell
cd d:\mt4
pip install -r requirements.txt

# Flash custom jog firmware
python flash_jog.py --port COM6

# Run keyboard jog (focus terminal, hold one or more keys)
python jog_keyboard.py --port COM6

# Or move to an absolute position (prompts for x/y/z/j4/gripper; requires
# having homed this session)
python goto_position.py --port COM6

# Local HTTP MCP server (Phase 1: status + stop only)
python -m mt4_mcp
# Then open MCP Inspector at http://127.0.0.1:8787/mcp
```

### MCP server (`mt4_mcp`)

Phase 1 exposes read-only status tools plus emergency stop over local HTTP
(Streamable HTTP at `/mcp`). Motion tools (`mt4_home`, `mt4_move`) come in a
later phase.

```powershell
# Default: serial COM6, MCP on http://127.0.0.1:8787/mcp
python -m mt4_mcp

# Override ports
$env:MT4_SERIAL_PORT = "COM6"
$env:MT4_MCP_PORT = "8787"
python -m mt4_mcp
```

| Tool | Purpose |
|------|---------|
| `mt4_status` | Full `?` status as JSON |
| `mt4_tcp` | Current TCP pose only |
| `mt4_stop` | Stop jog / cancel move |

Test with [MCP Inspector](https://github.com/modelcontextprotocol/inspector):
connect to `http://127.0.0.1:8787/mcp`, transport **Streamable HTTP**.

### Cursor

This repo includes `.cursor/mcp.json`, which registers the MT4 server for this
workspace. Cursor launches it over stdio (`python -m mt4_mcp --stdio`) when you
enable the **MT4** MCP server in settings.

1. Open **Cursor Settings → MCP** (or reload the window after pulling).
2. Enable the **MT4** server.
3. Stop `jog_keyboard.py` first — only one process can use COM6.

For HTTP mode instead (e.g. MCP Inspector), run `python -m mt4_mcp` without
`--stdio`.

### Keys (`jog_keyboard.py`)

| Key | Action |
|-----|--------|
| I/K | World +Z / -Z |
| S/W | World +Y / -Y |
| A/D | World +X / -X |
| J/L | J4 wrist roll (when no Cartesian key held) |
| Q/E | Gripper sweep open / close (S120–S285 on MT4; release = stop) |
| -/= | Jog speed slower / faster (live, repeats while held) |
| H | Home (on-device) |
| SPACE | Status |
| 0 | Stop, drivers off |
| ESC | Quit |

**Xbox controller** (player 1, via Windows XInput; works without terminal focus):

| Control | Action |
|---------|--------|
| Left stick | World X / Y |
| Right stick Y | World Z |
| Right stick X | J4 wrist roll (when not moving XYZ) |
| LT / RT | Gripper open / close |
| LB / RB or D-pad up/down | Jog speed faster / slower |
| A | Home |
| B | Stop, drivers off |
| X | Status |
| Back | Quit |

Use `--no-gamepad` for keyboard only. `--gamepad-deadzone` adjusts stick deadzone (default 9000).

Use `--no-orient` to disable J4 wrist unwind during Cartesian moves (also
live-toggleable via serial `orient on|off`). When on, J4 counters J1's yaw 1:1.

Gripper and J4-roll commands resend on a ~50ms timer while their key is held, so a
single dropped serial line can't strand them mid-motion — same fix already applied
to Cartesian jog's `cj` resend.

Firmware serial commands: `cj +x|-x|+y|-y|+z|-z|<dx> <dy> <dz>`, `orient on|off`,
`speed <us>` (live jog step period, 700-4000us, session state), `pos` (joint steps + derived TCP
mm/world-frame J4 deg/gripper S/move speed us), `setpos <j1> <j2> <j3> <j4>`,
`m <dj1> <dj2> <dj3> <dj4> [dg]` (bounded relative move, all axes finish together),
`mp <x> <y> <z> <j4> <g> [speed_us]` (absolute move to a TCP position in mm + world-frame J4 deg +
absolute gripper S + optional step period 700-4000us; TCP xyz interpolated along straight world-frame lines in short
segments with closed-form IK per segment; when the commanded J4 matches the current
world-frame yaw, gripper orientation is held fixed in world space; rejected with `err not homed` unless homed this
session), `home [j1 j2]`, `g o|c|stop|<120-285>`, `?`/`s`. Full reference in the header
comment of `firmware/mt4_jog/src/main.cpp`.

`goto_position.py` queries `pos` for the current TCP/J4/gripper state, prompts for
each (blank = keep current), then sends `mp` and prints the async completion line.

Kinematic model: the MT4 is a parallel-link (palletizing) arm — J2 sets the upper-arm
absolute angle, J3 sets the forearm absolute angle through the link rods (independent of
J2), and the head platform stays level, using EEPROM link/offset geometry (L1 130, L2 150,
base 45/140, head 35/14.43). The homed pose (step counters = 0) is **q2 = 103°, q3 =
4.7°** — measured directly (J2-J4 straight-line distance + J4 height above the base), not
the upper-arm-vertical/forearm-horizontal (90°, 0°) previously assumed. This custom
firmware's homing pull-off distances don't reach the same physical pose the factory
firmware's own homing does, so the model no longer matches the factory-reported home TCP
(230, 0, 255.57) — it reports (200.2, 0, 264.6) instead.

Per-joint calibration (`MT4_STEPS_PER_DEG` / `J_STEP_SIGN`, duplicated in
`firmware/mt4_jog/src/kinematics.{h,cpp}`, `mt4_jog/joints.py`, and
`mt4_jog/kinematics.py` — no shared config file, so edit all three together) is from
direct measurement (phone clinometer for J2-J4, direct yaw measurement for J1): J1/J2/J3
= 35 steps/deg, J4 = 45 steps/deg.

Homing seeks J1/J2's limit switches directly; J3 has no limit switch of its own, so it's
homed indirectly by driving it into mechanical interference with J2 until that
displaces J2 enough to release J2's own limit switch. J1 center **4580** steps, J2/J3
pull-off **1000** steps by default (override with `--j1-center` / `--j2-pull`).

## Restore stock firmware

Original factory images are in `backups/`:

```powershell
python restore_stock.py --port COM6 --yes
python restore_stock.py --port COM6 --yes --eeprom backups/mt4_eeprom_2026-07-02.hex
```

Optional EEPROM restore:

```powershell
avrdude -p atmega2560 -c wiring -P COM6 -b 115200 -U eeprom:w:backups\mt4_eeprom_2026-07-02.hex:i
```

## Layout

| Path | Purpose |
|------|---------|
| `jog_keyboard.py` | Keyboard + Xbox gamepad jog client (Cartesian + J4 roll + gripper) |
| `goto_position.py` | Prompt-driven absolute-position client (firmware `mp`) |
| `mt4_mcp/` | Local HTTP MCP server for arm status, control, and vision pick/place |
| `mt4_vision/` | Overhead-camera vision: ArUco calibration, cube detection, pick/place |
| `flash_jog.py` | Flash custom firmware |
| `restore_stock.py` | Flash stock firmware backup |
| `mt4_jog/` | Python joint map, kinematics, serial helpers |
| `firmware/mt4_jog/` | Arduino firmware: `config`/`pins`/`gripper`/`dda`/`motion`/`homing`/`commands`/`kinematics` modules |
| `backups/` | Stock flash/EEPROM images |
| `docs/` | Hardware and pin map reference (`MT4_ARCHITECTURE.md`) |

## Vision pick-and-place

An overhead USB camera watches the work surface, which carries four ArUco
markers (DICT_4X4_50, ids 0-3). The camera is auto-detected by scanning for
the one that sees the markers (override with `MT4_CAMERA_INDEX` or
`--camera`).

One-time calibration maps camera pixels to robot-frame XY on the table plane
-- no camera intrinsics needed. For each marker, jog the TCP to touch the
marker center (`jog_keyboard.py`, read X/Y off the status line), then:

```powershell
python -m mt4_vision markers    # verify all 4 markers are seen
python -m mt4_vision calibrate  # enter each marker's robot X/Y + pick heights
python -m mt4_vision scene      # sanity-check cube detections in robot coords
python -m mt4_vision pick red   # hardware test: pick a cube by color
```

Calibration lands in `vision_calibration.json` (homography, pick/safe
heights, gripper S values, HSV overrides). Colored cubes are detected by HSV
threshold inside the marker quadrilateral; detections outside it (the arm's
orange body, off-desk clutter) are rejected.

Natural-language control comes via the MCP server: `mt4_scene` reports cube
colors and robot-frame positions from a fresh frame, `mt4_pick_cube` grabs a
cube by color, `mt4_place_at` sets it down at x/y. Connect Claude (or any MCP
client) to the server and say "put the red cube next to the blue one".

## Safety

- Clear workspace before jogging or homing.
- Drivers energize while a key is held.
- After a stall/jerk, **power-cycle motor supply ~10 s** before retrying (TMC2209 latch).
- Only J1 (I21) and J2 (I20) have hardware limit switches; J3/J4 have none (J3 is
  homed indirectly via J2's switch; J4 is unreferenced and relies on power-on step
  counters staying valid).

## Pin map (custom firmware)

| Joint | G-code | Drive | DIR | Limit |
|-------|--------|-------|-----|-------|
| J1 base | X | D23 | D22 | I21 |
| J2 shoulder | Y | D25 | D24 | I20 |
| J3 elbow | Z | D27 | D26 | — |
| J4 wrist | A | D35 | D36 | — |

Shared enable: **D40** (active low).

Gripper PWM: **D7** (Timer4 OC4B). Limits and sweep run **on the MT4** (S120–S285). Client sends **`g o`** / **`g c`** on key down (resent while held), **`g stop`** on release. Manual: **`g <120-285>`** or query with **`g`**.
