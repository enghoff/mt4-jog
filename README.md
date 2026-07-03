# MT4 custom jog

Keyboard jog and on-device J1/J2 homing for the WLKATA MT4 arm (ATmega2560 @ COM6, 115200).

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

### Keys

| Key | Action |
|-----|--------|
| Q/A | J1 base |
| W/S | J2 shoulder |
| E/D | J3 elbow |
| R/F | J4 wrist |
| H | Home J1 + J2 (seek limits, pull off) |
| SPACE | Status |
| 0 | Stop, drivers off |
| ESC | Quit |

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
| `jog_keyboard.py` | Keyboard jog client |
| `flash_jog.py` | Flash custom firmware |
| `restore_stock.py` | Flash stock firmware backup |
| `mt4_jog/` | Python joint map + serial helpers |
| `firmware/mt4_jog/` | Arduino jog + homing firmware |
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

Investigation notes and disassembly artifacts are in `docs/archive/`.
