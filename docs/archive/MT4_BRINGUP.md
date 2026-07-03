# MT4 bring-up workflow

Current path for validating and experimenting with the MT4 arm after the pin hunt.

**Safety:** clear workspace, motor power accessible, stock firmware backed up in
`backups/`.

---

## 1. Stock firmware check (optional but recommended)

Confirms USB, G-code axis naming, and that joints move before custom firmware.

```powershell
cd d:\mt4
python test_one_axis.py --port COM6 --axis X --delta 3 --feed 500 --save
```

**Pass:** only axis X (J1 base) moves ~3°. If Alarm persists, home first:

```powershell
python -c "from mt4_client import Mt4Client; c=Mt4Client('COM6',115200); c.home_blocking(); c.close()"
```

Repeat for Y/Z/A to verify G-code → J2/J3/J4 mapping.

---

## 2. Flash pin lab firmware

Close any program holding COM6.

```powershell
python -m platformio run -t upload -d d:\mt4\firmware\minimal_x
```

All lab pins start **floating** (high-Z). Limits D20/D21 have pull-ups.

---

## 3. Pin lab session

```powershell
python d:\mt4\firmware\minimal_x\pin_lab.py --port COM6
```

Or keyboard jog:

```powershell
python d:\mt4\firmware\minimal_x\pin_keyboard.py --port COM6
```

Pin map and commands: **[MT4_PIN_MAP.md](MT4_PIN_MAP.md)**.

**Rules:**
- Only drive STEP + DIR + ENABLE for the joint under test
- Float all other lab pins
- After any jerk/stall → **power cycle motor supply ~10 s** before continuing
- Limit events print automatically in `pin_keyboard.py`

---

## 4. Restore stock firmware

Always return to stock when done experimenting:

```powershell
python d:\mt4\restore_stock.py --port COM6 --yes
python -c "from mt4_client import Mt4Client; c=Mt4Client('COM6',115200); c.home_blocking(); c.close()"
```

---

## Script reference

| Script | Purpose |
|--------|---------|
| `test_one_axis.py` | Stock G-code axis validation |
| `mt4_client.py` | Stock serial client library |
| `restore_stock.py` | Flash backed-up stock firmware |
| `firmware/minimal_x/pin_lab.py` | Pin lab interactive client |
| `firmware/minimal_x/pin_keyboard.py` | Hold-to-jog keyboard |
| `firmware/minimal_x/pin_home.py` | J1/J2 homing |
| `firmware/minimal_x/pin_sweep.py` | Drive / DIR sweep |
| `firmware/minimal_x/pin_assess_sweep.py` | Non-drive pin assess |
| `firmware/minimal_x/pin_map.py` | Joint/pin constants (shared) |

---

## Superseded

**[ONE_AXIS_BRINGUP.md](ONE_AXIS_BRINGUP.md)** described an early one-axis custom
firmware (STEP D24, DIR D36, LIMIT D5) that no longer exists. Do not use
`test_custom_axis.py` or `run_homing.py` — removed 2026-07-03.
