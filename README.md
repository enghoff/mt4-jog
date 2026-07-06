# MT4 custom jog

Keyboard jog and on-device J1/J2 homing for the WLKATA MT4 arm (ATmega2560 @ COM6, 115200).

Joint-space jog (`jog_keyboard.py`) and **Cartesian world-frame jog** (`jog_keyboard_cartesian.py`) share the same firmware.

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

# Run joint-space keyboard jog (focus terminal, hold one or more keys)
python jog_keyboard.py --port COM6

# Run Cartesian keyboard jog (world X/Y/Z — requires firmware below)
python jog_keyboard_cartesian.py --port COM6
```

### Joint jog keys (`jog_keyboard.py`)

| Key | Action |
|-----|--------|
| Q/A | J1 base |
| W/S | J2 shoulder |
| E/D | J3 elbow |
| R/F | J4 wrist |
| T/G | Gripper ramp open / closed (hold key) |
| H | Home J1 + J2 (seek limits, pull off) |
| SPACE | Status |
| 0 | Stop, drivers off |
| ESC | Quit |

### Cartesian jog keys (`jog_keyboard_cartesian.py`)

| Key | Action |
|-----|--------|
| I/K | World +X / -X |
| J/L | World +Y / -Y |
| U/O | World +Z / -Z |
| R/F | J4 wrist (joint jog when no XYZ key held) |
| T/G | Gripper open / close |
| H | Home J1 + J2 |
| SPACE | Status (shows `MODE=cart`, joint step counters) |
| 0 | Stop, drivers off |
| ESC | Quit |

Use `--no-orient` to disable J4 wrist unwind during Cartesian moves.

Firmware Cartesian commands (serial): `cj +x`, `cj -y`, `cj 1 0 1`, `orient on|off`, `pos`.

Kinematic model: the MT4 is a parallel-link (palletizing) arm — J2 sets the upper-arm
absolute angle, J3 sets the forearm absolute angle through the link rods (independent of
J2), and the head platform stays level. At the homed pose (upper arm vertical, forearm
horizontal) the model reproduces the factory-reported TCP **(230.000, 0, 255.570)** exactly
from the EEPROM geometry (L1 130, L2 150, base 45/140, head 35/14.43).

Homing defaults: J1 center **4580** steps, J2 pull **1000** steps (override with `--j1-center` / `--j2-pull`).

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
| `jog_keyboard.py` | Joint-space keyboard jog client |
| `jog_keyboard_cartesian.py` | Cartesian world X/Y/Z keyboard jog |
| `flash_jog.py` | Flash custom firmware |
| `restore_stock.py` | Flash stock firmware backup |
| `mt4_jog/` | Python joint map, kinematics, serial helpers |
| `firmware/mt4_jog/` | Arduino jog + homing + Cartesian firmware |
| `backups/` | Stock flash/EEPROM images |

## Safety

- Clear workspace before jogging or homing.
- Drivers energize while a key is held.
- After a stall/jerk, **power-cycle motor supply ~10 s** before retrying (TMC2209 latch).
- Only J1 (I21) and J2 (I20) have hardware limit switches; J3/J4 use soft limits in stock firmware.

## Pin map (custom firmware)

| Joint | G-code | Drive | DIR | Limit |
|-------|--------|-------|-----|-------|
| J1 base | X | D23 | D22 | I21 |
| J2 shoulder | Y | D25 | D24 | I20 |
| J3 elbow | Z | D27 | D26 | — |
| J4 wrist | A | D35 | D36 | — |

Shared enable: **D40** (active low).

Gripper PWM: **D7** (Timer4 OC4B). Limits and sweep run **on the MT4** (S120–S285). Client sends **`g o`** / **`g c`** on key down, **`g stop`** on release. Manual: **`g <120-285>`** or query with **`g`**.
