# MT4 pin map

**Status: pin hunt complete (2026-07-03).** Limit hunt **J3/J4: no hardware switches found**
(2026-07-03). Empirical mapping of STEP/DIR, limits, and shared enable.

Python source of truth: `firmware/minimal_x/pin_map.py`

---

## Pin map (canonical)

| Joint | Name | G-code | Drive (STEP) | DIR | Limit |
|-------|------|--------|--------------|-----|-------|
| **J1** | Base | X | **D23** (PA1) | **D22** (PA0) | **I21** (D21) |
| **J2** | Shoulder | Y | **D25** (PA3) | **D24** (PA2) | **I20** (D20) |
| **J3** | Elbow | Z | **D27** (PA5) | **D26** (PA4) | — (soft limit only) |
| **J4** | Wrist | A | **D35** (PC2) | **D36** (PC3) | — (soft limit only) |

| Signal | Pin | Notes |
|--------|-----|-------|
| Shared driver **ENABLE** | **D40** (PG1) | Active low (`e1` = on) |
| Limit inputs | **D20**, **D21** | Active-low, pull-up; **not** Grbl D10–D12 / PCINT0 |

**MCU channel order** D23 → D25 → D27 → D35 does **not** follow J1 → J4. Always
map by joint using this table.

**Disproved:** D34 as J4 DIR (J4 DIR is **D36**).

---

## Joint numbering

J-numbers follow the kinematic chain base → tool: **J1 = base**, J2 shoulder,
J3 elbow, J4 wrist. Stock firmware, `Angle(ABCDXYZ)` status, and G-code agree:

| G-code | Joint |
|--------|-------|
| X | J1 base |
| Y | J2 shoulder |
| Z | J3 elbow |
| A | J4 wrist |

---

## Limit switches

| Pin | MCU | Interrupt | Joint |
|-----|-----|-----------|-------|
| **I20** (D20) | PD1 | INT1 | **J2 shoulder** |
| **I21** (D21) | PD0 | INT0 | **J1 base** |

Limits are active-low with internal pull-ups. Pin-lab firmware pushes async
events on edge: `lim I<pin>=<raw> open|TRIG`.

### J3 / J4 limit hunt (2026-07-03) — **no hardware switches found**

| Test | Result |
|------|--------|
| Pin lab: jog J3/J4 toward stops, monitor I2–I19, I38–I53 | No stable `TRIG` on unknown pins |
| Pin lab: I17 | **Noise only** (rapid 0/1 bounce) — not a limit |
| Stock FW: `$H` homing | Completes (uses **I20/I21** only) |
| Stock FW: Z/A moves ±30° and ±60° | **Idle**, no Alarm / A101 / A102 |

**Conclusion:** this MT4 has **two physical limit switches** (J1/J2). J3/J4 envelope
is enforced by **firmware soft limits** (`$130`–`$146` travel), not MCU GPIO.

If a future PCB revision adds switches, re-run:

```powershell
python -m platformio run -t upload -d d:\mt4\firmware\minimal_x
python d:\mt4\firmware\minimal_x\pin_limit_scan.py --port COM6
```

Stock cross-check:

```powershell
python d:\mt4\firmware\minimal_x\stock_limit_probe.py --port COM6
```

Monitored hunt pins (wide scan): I20/I21 known; candidates I18/I19, I10–I12,
I2/I3, I14–I16, I4–I9, I38–I53 (I17 excluded as noise).

Stock Grbl Mega docs cite PCINT0 on D10–D12; that ISR exists in MT4 flash but
is not wired to J3/J4 limits on this arm.

---

## Safety (learned during pin hunt)

1. **Only drive pins you understand** — STEP, one DIR, ENABLE. Float everything else.
   A floating DIR can lock a motor to one direction regardless of software.

2. **TMC2209 stall latch** — after a jerk/stall, drivers may stop responding until
   a **full motor power cycle (~10 s off)**. USB reflash does not clear it.

3. **Serial success ≠ motion** — pulse counts are software-only; watch the arm.

4. **Drive sweep hazard** — holding unknown pins HIGH (`pin_sweep.py` default) can
   disturb drivers. Prefer float-all + single-pin tests when in doubt.

---

## Pin lab firmware

Location: `firmware/minimal_x/`. Interactive GPIO/step tester over USB @ 115200
(DTR/RTS off on host).

```powershell
# Flash
python -m platformio run -t upload -d d:\mt4\firmware\minimal_x

# Interactive
python d:\mt4\firmware\minimal_x\pin_lab.py --port COM6

# Keyboard jog (Q/A=Y, W/S=Z, E/D=X, R/F=A; H=home)
python d:\mt4\firmware\minimal_x\pin_keyboard.py --port COM6
python d:\mt4\firmware\minimal_x\pin_limit_scan.py --port COM6
```

### Commands

| Command | Effect |
|---------|--------|
| `?` / `d` | Dashboard (all lab pins: mode + readback) |
| `d<pin> f\|l\|h` | Float / output LOW / HIGH |
| `x<pin>` / `step <pin>` | Select drive pin for pulses |
| `n<count>` | Pulse count for `+` (default 200) |
| `+` / `go` | Run pulses (requires `x<pin>`) |
| `j` / `jog` | Continuous step until `stop` |
| `!` / `stop` | Stop jog |
| `all f\|l\|h` | All lab pins float / low / high |
| `e1` / `e0` | Shared enable D40 |
| `s` / `lims` | All monitored limit pins |
| `rf` | Float lab pin reads + limits |
| `home` | **J1/J2 homing** — seek limits, pull off (see below) |
| `home <j1> <j2>` | Homing with custom J1 center / J2 pull-off step counts |
| *(async)* | `lim I<pin>=<raw> open\|TRIG` on limit change |

Lab pins: **D22–D29, D30–D37, D40**.

### Example — J1 base

```
e1
x23
d22 l
+
d22 h
+
all f
```

### Homing (J1 + J2)

Seeks **J1** toward its limit in keyboard **D** direction (DIR high), then **J2**
in keyboard **A** direction, then pulls off:

- **J1:** reverse **4580** steps
- **J2:** reverse **1000** steps

```powershell
python d:\mt4\firmware\minimal_x\pin_home.py --port COM6
# or from pin_lab:  home
# custom counts:     home 4580 1000
```

Abort during homing: send `stop` or `!` on serial.

Constants in `pin_map.py` (`J1_HOME_CENTER_STEPS`, `J2_HOME_PULLOFF_STEPS`).

### Helper scripts

```powershell
python d:\mt4\firmware\minimal_x\pin_sweep.py --port COM6
python d:\mt4\firmware\minimal_x\pin_sweep.py --port COM6 --dir-for 25
python d:\mt4\firmware\minimal_x\pin_assess_sweep.py --port COM6
```

---

## Restore stock firmware

```powershell
python d:\mt4\restore_stock.py --port COM6 --yes
python -c "from mt4_client import Mt4Client; c=Mt4Client('COM6',115200); c.home_blocking(); c.close()"
```

Run from `d:\mt4`. Stock homing verified working after pin-lab sessions.

---

## Open items

1. Confirm **J2/J3 DIR** (D24, D26 guesses) via `pin_sweep.py --dir-for 25` / `27`.
2. Steps/degree per joint (stock ~44 steps/deg on axis A).
3. Custom 4-axis motion firmware using this pin map (J3/J4: soft limits only).

---

## Related docs

| Doc | Purpose |
|-----|---------|
| `MT4_BRINGUP.md` | Stock test → pin lab → restore workflow |
| `MT4_ARCHITECTURE.md` | Host protocol, firmware identity |
| `MT4_DISASM_STEPPER.md` | Static RE (ISR, PORTA/PORTC) |
| `docs/archive/superseded_pin_hunt.md` | Superseded hypotheses from early hunt |

---

## Appendix: corrected assumptions

| Old belief | Reality |
|------------|---------|
| Limits on D10–D12 (Grbl PCINT0) | **D20/D21** (INT1/INT0) |
| DIR for axis X = D36 | D36 is **J4 wrist** DIR only |
| DIR = D30 (Grbl PC7) | No effect on MT4 |
| STEP=PORTA, DIR=PORTC split | ISR writes both ports; bits interleaved |
| D23 = DIR, D24 = STEP (motor 1) | **D23 = drive, D22 = DIR** |
