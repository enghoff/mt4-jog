# MT4 custom jog

Keyboard jog and on-device homing for the WLKATA MT4 arm (ATmega2560 @ COM6, 115200).

Custom firmware (`firmware/mt4_jog/`) replaces the stock Grbl-derived firmware with a
4-axis step/dir jog engine plus an on-device Cartesian (world-frame) resolved-rate
jog. `jog_keyboard.py` is the only client: Cartesian jog is the sole motion mode
(direct per-joint jog was dropped), plus J4 wrist roll and the gripper.

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
```

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
`speed <us>` (live jog step period, 700-4000us), `pos`, `setpos <j1> <j2> <j3> <j4>`,
`m <dj1> <dj2> <dj3> <dj4> [dg]` (bounded relative move, all axes finish together),
`home [j1 j2]`, `g o|c|stop|<120-285>`, `?`/`s`. Full reference in the header comment
of `firmware/mt4_jog/src/main.cpp`.

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
| `flash_jog.py` | Flash custom firmware |
| `restore_stock.py` | Flash stock firmware backup |
| `mt4_jog/` | Python joint map, kinematics, serial helpers |
| `firmware/mt4_jog/` | Arduino firmware: `config`/`pins`/`gripper`/`dda`/`motion`/`homing`/`commands`/`kinematics` modules |
| `backups/` | Stock flash/EEPROM images |
| `docs/` | Hardware and pin map reference (`MT4_ARCHITECTURE.md`) |

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
