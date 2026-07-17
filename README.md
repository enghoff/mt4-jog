# WLKATA MT4 — custom control stack

A full replacement software stack for the WLKATA MT4 desktop robot arm
(ATmega2560, 115200 baud serial): custom firmware with on-device Cartesian
motion, interactive jog clients (keyboard + Xbox gamepad), overhead-camera
vision pick-and-place for colored cubes, and an MCP server that lets an LLM
drive the arm — "put the red cube next to the blue one".

The stock Grbl-derived firmware is replaced entirely (original images are
backed up and restorable, see [Restoring stock firmware](#restoring-stock-firmware)).

## Repo layout

| Path | Purpose |
|------|---------|
| `firmware/mt4_jog/` | Custom Arduino firmware: `config`/`pins`/`gripper`/`dda`/`motion`/`homing`/`commands`/`kinematics` modules |
| `mt4_jog/` | Python client library: serial protocol, joint map, kinematics, gamepad |
| `mt4_vision/` | Overhead-camera vision: ArUco calibration, cube detection, pick/place, shuffle |
| `mt4_mcp/` | MCP server (HTTP or stdio) exposing status, motion, and vision pick/place tools |
| `jog_keyboard.py` | Keyboard + Xbox gamepad jog client (Cartesian + J4 roll + gripper) |
| `goto_position.py` | Prompt-driven absolute-position client (firmware `mp`) |
| `calibrate_vision.py` | Interactive jog-to-marker camera calibration |
| `calibrate_height.py` | Auto probe-fit cube-top / pick-height correction after vision calibration |
| `recalibrate_camera.py` | Camera-only homography refit when the camera moved but markers/base did not |
| `shuffle_blocks.py` | Live loop: detect cubes and shuffle them between markers / open table |
| `flash_jog.py` | Flash the custom firmware |
| `restore_stock.py` | Flash the stock firmware backup |
| `backups/` | Stock flash/EEPROM images |
| `scripts/` | Diagnostics (`diagnose_pick_accuracy.py`, `validate_scene_live.py`), ngrok launcher |
| `tests/` | Unit tests |
| `docs/` | Hardware reference, OAuth setup, sort-behavior spec, printable ArUco sheet |

## Requirements

- Python 3.10+ — `pip install -r requirements.txt`
- [PlatformIO](https://platformio.org/) + avrdude (only to flash firmware)
- Windows (jog client uses `GetAsyncKeyState` / XInput)
- For vision: an overhead USB camera and printed ArUco markers
  (DICT_4X4_50; sheet in `docs/ArUco Markers A4 5x5cm.pdf`)

Serial ports auto-detect the CH340 USB-UART when `--port` / `MT4_SERIAL_PORT`
are omitted (COM numbers often change after a re-plug). The camera is
auto-detected by scanning for the one that sees the markers (override with
`MT4_CAMERA_INDEX` or `--camera`).

## Quick start

```powershell
pip install -r requirements.txt

# Flash the custom firmware (one-time, or after firmware changes)
python flash_jog.py --port COM6

# Jog interactively (focus the terminal, hold keys; gamepad works unfocused)
python jog_keyboard.py

# One-shot absolute moves (prompts for x/y/z/j4/gripper; requires homing first)
python goto_position.py

# Vision: calibrate once, then pick/place/shuffle
python -m mt4_vision markers        # verify the markers are seen
python calibrate_vision.py          # jog-to-marker interactive calibration
python calibrate_height.py          # optional probe-fit for pick height
python -m mt4_vision scene          # sanity-check cube detections in robot coords
python -m mt4_vision pick red       # hardware test: pick a cube by color
python shuffle_blocks.py            # live shuffle loop (Ctrl+C stop, H re-home)

# MCP server for LLM control
python -m mt4_mcp                   # HTTP at http://127.0.0.1:8787/mcp
```

## Jog clients

`jog_keyboard.py` drives the arm in world-frame Cartesian jog (the sole
motion mode — direct per-joint jog was dropped), plus J4 wrist roll and the
gripper.

### Keyboard

| Key | Action |
|-----|--------|
| I/K | World +Z / -Z |
| S/W | World +Y / -Y |
| A/D | World +X / -X |
| J/L | J4 wrist roll (also while moving XYZ) |
| Q/E | Gripper sweep open / close (S120–S285; release = stop) |
| -/= | Jog speed slower / faster (live, repeats while held) |
| H | Home (on-device) |
| SPACE | Status |
| 0 | Stop, drivers off |
| ESC | Quit |

### Xbox controller

Player 1, via Windows XInput; works without terminal focus.

| Control | Action |
|---------|--------|
| Left stick | World X / Y |
| Right stick Y | World Z |
| Right stick X | J4 wrist roll (also while moving XYZ) |
| LT / RT | Gripper open / close |
| LB / RB or D-pad up/down | Jog speed faster / slower |
| A | Home |
| B | Stop, drivers off |
| X | Status |
| Back | Quit |

Use `--no-gamepad` for keyboard only; `--gamepad-deadzone` adjusts the stick
deadzone (default 9000).

### Behavior notes

- `--no-orient` disables J4 wrist unwind during Cartesian moves (also
  live-toggleable via serial `orient on|off`). When on, J4 counters J1's yaw
  1:1 so the gripper holds its world-frame orientation.
- Gripper and J4-roll commands resend on a ~50 ms timer while their key is
  held, so a single dropped serial line can't strand them mid-motion — the
  same fix applied to Cartesian jog's `cj` resend.
- `goto_position.py` queries `pos` for the current TCP/J4/gripper state,
  prompts for each field (blank = keep current), then sends `mp` and prints
  the async completion line.

## Firmware

`firmware/mt4_jog/` is a 4-axis step/dir jog engine with an on-device
world-frame resolved-rate jog and closed-form IK for straight-line moves.
Build/flash with PlatformIO via `flash_jog.py`.

### Serial protocol

Full reference lives in the header comment of
`firmware/mt4_jog/src/main.cpp`. Summary:

| Command | Effect |
|---------|--------|
| `cj +x\|-x\|+y\|-y\|+z\|-z\|<dx> <dy> <dz> [j4]` | Cartesian jog. Optional J4 roll `-1\|0\|1` layers onto the solved rates so the wrist rotates during the move; zero direction + nonzero j4 = pure wrist roll |
| `orient on\|off` | J4 wrist unwind during Cartesian moves |
| `speed <us>` | Live jog step period, 700–4000 µs (session state) |
| `pos` | Joint steps + derived TCP mm, world-frame J4 deg, gripper S, move speed |
| `setpos <j1> <j2> <j3> <j4>` | Overwrite step counters |
| `m <dj1> <dj2> <dj3> <dj4> [dg]` | Bounded relative move; all axes finish together |
| `mp <x> <y> <z> <j4> <g> [speed_us]` | Absolute move: TCP position (mm) + world-frame J4 (deg) + gripper S + optional step period. XYZ interpolated along straight world-frame lines in short segments with closed-form IK per segment; when the commanded J4 matches the current world-frame yaw, gripper orientation is held fixed in world space. Rejected with `err not homed` unless homed this session |
| `home [j1 j2]` | On-device homing (see below) |
| `g o\|c\|stop\|<120-285>` | Gripper open / close / stop / set; bare `g` queries |
| `?` / `s` | Status |

### Kinematics and calibration

The MT4 is a parallel-link (palletizing) arm: J2 sets the upper-arm absolute
angle, J3 sets the forearm absolute angle through the link rods (independent
of J2), and the head platform stays level. The model uses EEPROM link/offset
geometry (L1 130, L2 150, base 45/140, head 35/14.43).

The homed pose (step counters = 0) is **q2 = 103°, q3 = 4.7°** — measured
directly (J2–J4 straight-line distance + J4 height above the base), not the
upper-arm-vertical / forearm-horizontal (90°, 0°) previously assumed. This
firmware's homing pull-off distances don't reach the same physical pose the
factory firmware's homing does, so the home TCP is (200.2, 0, 264.6) rather
than the factory-reported (230, 0, 255.57).

Per-joint calibration is from direct measurement (phone clinometer for
J2–J4, direct yaw measurement for J1): J1/J2/J3 = 35 steps/deg, J4 = 45
steps/deg. `MT4_STEPS_PER_DEG` / `J_STEP_SIGN` are duplicated in
`firmware/mt4_jog/src/kinematics.{h,cpp}`, `mt4_jog/joints.py`, and
`mt4_jog/kinematics.py` — there is no shared config file, so edit all three
together.

### Homing

Homing seeks J1/J2's limit switches directly. J3 has no switch of its own,
so it's homed indirectly by driving it into mechanical interference with J2
until that displaces J2 enough to release J2's own limit switch. Defaults:
J1 center **4580** steps, J2/J3 pull-off **1000** steps (override with
`--j1-center` / `--j2-pull` on the clients).

### Pin map

| Joint | G-code | Drive | DIR | Limit |
|-------|--------|-------|-----|-------|
| J1 base | X | D23 | D22 | I21 |
| J2 shoulder | Y | D25 | D24 | I20 |
| J3 elbow | Z | D27 | D26 | — |
| J4 wrist | A | D35 | D36 | — |

Shared enable: **D40** (active low). Gripper PWM: **D7** (Timer4 OC4B);
limits and sweep run on the MCU (S120–S285).

Full hardware detail (board, drivers, flash path) is in
`docs/MT4_ARCHITECTURE.md`.

## Vision pick-and-place

An overhead USB camera watches the work surface, which carries ArUco
markers. A one-time calibration maps camera pixels to robot-frame XY on the
table plane — no camera intrinsics needed.

### Calibration

```powershell
python -m mt4_vision markers    # verify the markers are seen
python calibrate_vision.py      # jog-to-marker interactive calibration
python calibrate_height.py      # optional probe-fit for cube-top / pick height
python -m mt4_vision scene      # sanity-check cube detections in robot coords
```

`calibrate_vision.py` homes the arm, then drops into the jog controls from
`jog_keyboard.py`. Jog the TCP onto any reachable marker (any order;
unreachable markers are simply skipped) and record it with its digit key —
or with gamepad **A**, which identifies the marker automatically as the one
the arm is hiding from the camera. **G** records the pick height and gripper
S while physically gripping a cube; **Enter**/**Start** fits and saves.
Because digits and A are taken, drivers-off moves to **X** and home to
gamepad **Y**. Three recorded markers give an affine fit (accurate within
the marker triangle); four or more give a full perspective homography.

If the **camera** moves but the arm base and markers do not, skip
re-touching markers:

```powershell
python recalibrate_camera.py
python calibrate_height.py    # refit cube-top map (cleared by recalibrate)
```

Calibration lands in `vision_calibration.json` (transform, pick/safe
heights, gripper S values, HSV overrides — tuning fields carry over when
re-calibrating). Colored cubes are detected by HSV threshold inside the
marker quadrilateral; detections outside it (the arm's orange body, off-desk
clutter) are rejected.

### CLI

`python -m mt4_vision <subcommand>`:

| Subcommand | Purpose |
|------------|---------|
| `markers` | Detect ArUco markers, save an annotated frame |
| `scene` | Detect cubes, print robot-frame coordinates |
| `pick <color>` | Pick a cube by color (moves the arm) |
| `place <x> <y>` | Place the held cube at robot-frame x/y (moves the arm) |
| `place-here` | Place the held cube at the current TCP xy (moves the arm) |
| `goto-marker <id>` | Move the TCP to a marker — calibration accuracy check (`--touch` descends to table height) |
| `shuffle` | Home, then shuffle cubes between markers and open table |

### Shuffle loop

`shuffle_blocks.py` runs an interruptible detect→plan→pick/place loop that
moves cubes between free markers and open-table slots (Ctrl+C to stop, H to
re-home). Sort-into-rows behavior (S key) is specified in
`docs/SORT_OCCUPANCY_REQUIREMENTS.md` and is not implemented yet.

## MCP server

`mt4_mcp` exposes the arm to any MCP client over Streamable HTTP or stdio.
Natural-language pick-and-place: connect an LLM and say "put the red cube
next to the blue one".

| Tool | Purpose |
|------|---------|
| `mt4_status` | Full arm status as JSON (homed flag, mode, joints, TCP, drivers, jog) |
| `mt4_tcp` | Current TCP pose only |
| `mt4_stop` | Stop jog / cancel any in-progress move |
| `mt4_home` | On-device homing |
| `mt4_move_to` | Absolute TCP move (requires homing this session) |
| `mt4_move_relative` | Bounded relative per-joint move |
| `mt4_gripper` | Open / close / stop / set the gripper |
| `mt4_scene` | Detect cubes on a fresh camera frame; returns color + robot-frame x/y |
| `mt4_pick_cube` | Pick a cube by color |
| `mt4_place_at` | Place the held cube at robot-frame x/y |
| `mt4_goto_marker` | Move the TCP to a calibration marker — accuracy check |

Only one process can own the serial port — stop `jog_keyboard.py` (or any
other client) before starting the server.

### Local HTTP

```powershell
python -m mt4_mcp     # http://127.0.0.1:8787/mcp (Streamable HTTP)
```

Configuration via flags or environment (a `.env` file is loaded
automatically — copy `.env.example` to get started): `MT4_SERIAL_PORT`,
`MT4_BAUD`, `MT4_MCP_HOST`, `MT4_MCP_PORT`, `MT4_MCP_PATH`. Test with
[MCP Inspector](https://github.com/modelcontextprotocol/inspector):
connect to `http://127.0.0.1:8787/mcp`, transport **Streamable HTTP**.

### Cursor (stdio)

`.cursor/mcp.json` registers the server for this workspace; Cursor launches
it as `python -m mt4_mcp --stdio`. Open **Cursor Settings → MCP**, enable
the **MT4** server.

### Public access (ChatGPT / remote clients)

Set `MT4_MCP_PUBLIC=1` to bind publicly, tunnel with ngrok
(`scripts/start_ngrok.ps1`), and enable the OAuth 2.1 flow (FastMCP's Google
provider) for ChatGPT-compatible auth. Full setup: `docs/OAUTH_CHATGPT.md`.

## Restoring stock firmware

Original factory images are in `backups/`:

```powershell
python restore_stock.py --port COM6 --yes
```

Optional EEPROM restore:

```powershell
python restore_stock.py --port COM6 --yes --eeprom backups/mt4_eeprom_2026-07-02.hex
# or directly:
avrdude -p atmega2560 -c wiring -P COM6 -b 115200 -U eeprom:w:backups\mt4_eeprom_2026-07-02.hex:i
```

## Further docs

| Doc | Contents |
|-----|----------|
| `docs/MT4_ARCHITECTURE.md` | Hardware and pin-map reference, ATmega2560 flash path |
| `docs/OAUTH_CHATGPT.md` | OAuth 2.1 via Google + ngrok for public MCP access |
| `docs/SORT_OCCUPANCY_REQUIREMENTS.md` | Spec for the (unimplemented) sort-into-rows shuffle behavior |
| `docs/ArUco Markers A4 5x5cm.pdf` | Printable marker sheet (DICT_4X4_50) |
| `firmware/mt4_jog/src/main.cpp` | Full serial protocol reference (header comment) |
