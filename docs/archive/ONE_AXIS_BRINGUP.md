# One-axis bring-up walkthrough

> **SUPERSEDED (2026-07-03).** This walkthrough targeted an early one-axis custom
> firmware that no longer exists. Use **[MT4_BRINGUP.md](MT4_BRINGUP.md)** and
> **[MT4_PIN_MAP.md](MT4_PIN_MAP.md)** instead. Kept for historical reference only.

Step-by-step path to validate axis **X / J1** on stock firmware, then flash
minimal custom firmware and iterate settings over USB only.

**Safety:** clear workspace, keep power accessible, always have stock backup.

---

## Prerequisites

- MT4 powered and on USB (`COM6`, 115200)
- Python deps: `pip install -r requirements.txt`
- **Phase B only:** [PlatformIO](https://platformio.org/) + `avrdude` (from AVR-GCC winget)
- Close any other program using COM6 (Cursor terminal, other Python scripts)

---

## Phase A — Stock firmware axis test (do this first)

Confirms that commanding **X** moves **J1** (`angle_x`) before you replace firmware.

```powershell
cd d:\mt4
python test_one_axis.py --port COM6 --axis X --delta 3 --feed 500 --save
```

**What it does:** `M50` unlock → move X by +3° → wait Idle → print which joints changed.

**Pass:** only `X` delta ~+3°, state returns to `Idle`.

**If Alarm persists:** try homing first:

```powershell
python -c "from mt4_client import Mt4Client; c=Mt4Client('COM6',115200); c.home_blocking(); c.close()"
```

Then re-run `test_one_axis.py`.

**Try other axes (optional):**

```powershell
python test_one_axis.py --axis Y --delta 3 --save
python test_one_axis.py --axis Z --delta 3 --save
python test_one_axis.py --axis A --delta 3 --save
```

Record which `angle_*` field moves for each — that's your joint map.

---

## Phase B — Build minimal firmware

```powershell
cd d:\mt4\firmware\minimal_x
pio run
```

Edit `platformio.ini` if your port is not `COM6`.

**Pins baked in** (axis X from RE):

| Signal | Pin |
|--------|-----|
| STEP | D24 |
| DIR | D36 |
| LIMIT | D5 |
| ENABLE | D40 |

---

## Phase C — Flash custom firmware

**Warning:** motors may energize on boot. Keep the arm clear.

```powershell
cd d:\mt4\firmware\minimal_x
pio run -t upload
```

If upload fails with "access denied", close anything holding COM6 and retry.

**Restore stock anytime:**

```powershell
cd d:\mt4
python restore_stock.py --port COM6 --yes
```

---

## Phase D — Exercise custom firmware

Use a serial monitor @ 115200, or the helper script:

```powershell
cd d:\mt4\firmware\minimal_x
python test_custom_axis.py --port COM6
```

### Suggested iteration sequence

| Step | Command | What to look for |
|------|---------|------------------|
| 1 | `?` | Banner + `limit=open` at rest |
| 2 | `e1` then `?` | Drivers enabled (`en=on`) |
| 3 | `+200` | J1 should twitch/move; `pos_steps` changes |
| 4 | `-200` | Returns roughly to start |
| 5 | `d1` then `+200` | If motion reversed, keep `d1` |
| 6 | `s` | Raw limit: `1` = open, `0` = triggered |
| 7 | `h-` or `h+` | Seeks until limit; try both directions |

**If no motion:**

- Try `e0` / `e1` (enable polarity may be wrong — edit `ENABLE_ACTIVE_LOW` in `src/main.cpp`)
- Try smaller/larger step counts: `+50`, `+2000`
- Restore stock, confirm arm still moves with `test_one_axis.py`

**If wrong joint moves:** update pin defines in `platformio.ini` / `main.cpp` using the map from Phase A.

---

## Phase E — Tune homing (serial iteration)

On **stock** firmware, note from `captures/settings_2026-07-02_150731.txt`:

- `$23=121` → axis X homing dir invert **bit 0 set** → try `h-` first on custom FW
- `$27=6` mm pull-off — convert to steps once steps/deg is known (~44 steps/deg for axis `a`)
- `$150=95` deg reset distance — apply after homing in later firmware versions

Iterate on custom FW:

1. `h-` hits limit → good direction
2. Adjust `HOMING_PULLOFF_STEPS` in `main.cpp` if switch stays triggered
3. When stable, set `position_steps = 0` at homed pose

---

## Phase F — Back to stock + full arm

```powershell
cd d:\mt4
python restore_stock.py --port COM6 --yes
python test_one_axis.py --axis X --delta 3 --save
```

---

## Files

| File | Purpose |
|------|---------|
| `test_one_axis.py` | Phase A stock firmware axis test |
| `firmware/minimal_x/` | Phase B–D minimal one-axis firmware |
| `firmware/minimal_x/test_custom_axis.py` | Serial console for custom FW |
| `restore_stock.py` | Flash stock backup |
| `captures/` | Saved test logs |
| `dump_settings.py` | Re-capture `$$` anytime |

---

## Next after one axis works

1. Copy `minimal_x` → add Y/Z/A pin tables from RE
2. Add steps/deg (`$100` = 44.001 for axis `a`)
3. Add simple `G01 X` parser or reuse Grbl-Mega
4. EEPROM for homed offset + calibration
