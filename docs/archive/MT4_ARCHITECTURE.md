# WLKATA MT4 — Architecture & Connectivity Reference

Document compiled from USB investigation, firmware backup analysis, PCB observations,
and control experiments in this repository. **Last updated:** 2026-07-02.

---

## 1. Executive summary

The WLKATA MT4 arm controller is an **ATmega2560-based motion controller** with a
**CH340 USB–serial bridge**, running a **Grbl-derived firmware** branded
`MT4,20240820`. A PC talks G-code-like commands over **115200 baud** serial; the MCU
generates **STEP/DIR/ENABLE** signals to **TMC2209-class** stepper driver modules.

| Layer | Component | Status |
|-------|-----------|--------|
| PC host | Python + pyserial (`mt4_client.py`) | Verified |
| USB bridge | CH340C (`VID:PID 1A86:7523`) | Verified on COM6 |
| MCU | ATmega2560 (`signature 0x1E9801`) | Verified via avrdude |
| Bootloader | Wiring / STK500v2 (`Arduino explorer stk500V2 by MLS`) | Verified (`-c wiring`) |
| Application | Grbl 0.9j fork + WLKATA extensions | Confirmed in flash image |
| Motor drivers | StepStick TMC2209-LA (STEP/DIR/EN) | Observed on PCB (not traced in FW yet) |

---

## 2. System block diagram

```
┌─────────────┐    USB      ┌──────────┐   UART    ┌──────────────────┐
│  PC / Host  │────────────►│  CH340C  │──────────►│   ATmega2560     │
│  (Python)   │   serial    │  bridge  │  115200   │   16 MHz         │
└─────────────┘             └──────────┘           └────────┬─────────┘
                                                            │
                         G-code parser / motion planner     │ GPIO
                         (Grbl-derived firmware)            │
                                                            ▼
                                              ┌─────────────────────────┐
                                              │ StepStick driver sockets │
                                              │ (TMC2209-LA × N)         │
                                              │  STEP / DIR / EN         │
                                              └────────────┬────────────┘
                                                           │
                                                           ▼
                                              Joint stepper motors (4+ axes)
```

**Not controlled by ATmega flash image:**

- CH340 USB enumeration and drivers (separate chip firmware)
- TMC2209 chopper/microstep config if set by module jumpers (standalone mode)

---

## 3. Hardware (PCB observations + verification)

### 3.1 Main components

| Part | Marking / role | Notes |
|------|----------------|-------|
| MCU | ATmega2560-16U | Same family as Arduino Mega 2560; 256 KB flash, 4 KB EEPROM |
| Clock | 16 MHz crystal | Standard Mega-class timing |
| USB bridge | CH340C | Near USB connector; presents as virtual COM port |
| Stepper drivers | TMC2209-LA on StepStick modules | Labels: VMOT, GND, 2B, 2A, 1A, 1B, VDD, STEP, DIR, EN |
| Test pads | R / T near USB | Likely UART RX/TX test points (CH340 or MCU side — unconfirmed) |

### 3.2 Verified USB identity (this unit)

| Field | Value |
|-------|-------|
| OS | Windows 10 |
| Port | `COM6` |
| VID:PID | `1A86:7523` |
| Description | USB-SERIAL CH340 (COM6) |
| Manufacturer | wch.cn |
| Working baud | **115200** |

### 3.3 Electrical / mechanical interfaces (inferred)

| Interface | Evidence |
|-----------|----------|
| 4 primary joint axes (J1–J4) | Angle-mode G-code uses X/Y/Z/A; status `Angle(ABCDXYZ)` |
| Additional axes (5–7) | Firmware strings: “7th axis home”, axis 6 calibration `M42`/`M43` |
| Gripper | `M3S<pwm>` — PWM range ~40 (open) – 60 (closed) in project code |
| Suction / pump | Status fields `Pump PWM`, `Valve PWM` |
| Color sensor | `M60` query RGB; enable via `$52 = 1` |
| Limits / homing | Grbl-style `$H`, hard/soft limit errors; **hardware limits on D20/D21** (see `MT4_PIN_MAP.md`) |

**MCU pin map (empirical, 2026-07-03)** — see **`MT4_PIN_MAP.md`** and
`firmware/minimal_x/pin_map.py`. Four drive lines D23/D25/D27/D35, DIR
D22/D24/D26/D36, shared ENABLE D40, limits I20 (J2) / I21 (J1). Static RE
details in `MT4_DISASM_STEPPER.md`.

---

## 4. Firmware identity

### 4.1 Backup artifacts (`backups/`)

Captured 2026-07-02 with read-only avrdude:

| File | Size | SHA-256 |
|------|------|---------|
| `mt4_flash_2026-07-02.hex` | 629,073 B | `9FE579BA…DF58B` |
| `mt4_flash_2026-07-02.bin` | 261,406 B | `672C9F37…25EA1` |
| `mt4_eeprom_2026-07-02.hex` | 9,869 B | `E2F11098…5F28` |

Restore (when needed):

```powershell
avrdude -p atmega2560 -c wiring -P COM6 -b 115200 -U flash:w:backups\mt4_flash_2026-07-02.hex:i
avrdude -p atmega2560 -c wiring -P COM6 -b 115200 -U eeprom:w:backups\mt4_eeprom_2026-07-02.hex:i
```

### 4.2 Version strings (from flash)

| String | Meaning |
|--------|---------|
| `WLKATA Robot started successfully.Firmware version:MT4,20240820` | Boot banner on serial connect |
| `[0.9j.20160303:` | Base **Grbl 0.9j** lineage (March 2016) |
| `Mirobot,` / `Mirobot SETTINGS_VERSION,` | Shared codebase with Mirobot product line |
| `ATmega2560` | Build target |
| `Arduino explorer stk500V2 by MLS` | Bootloader identification |
| `GCC Version = 4.3.5` / `1.6.8` | Legacy Arduino/AVR toolchain era |

### 4.3 Memory map (approximate)

| Region | Size | Content |
|--------|------|---------|
| Application flash | ~256 KB | Grbl-derived planner, WLKATA kinematics, strings |
| EEPROM | 4 KB | `$` settings, calibration, tool offsets |
| Bootloader | Tail of flash | STK500v2-compatible serial bootloader |

First flash bytes (`0c940608…`) are consistent with AVR interrupt vector table (RJMP to init).

---

## 5. USB programming path

Verified read-only on this hardware:

```powershell
avrdude -p atmega2560 -c wiring -P COM6 -b 115200 -v
# → Device signature = 1E 98 01 (ATmega2560)
```

| Programmer | Result |
|------------|--------|
| `-c wiring` | **Works** — use this for flash/EEPROM read/write |
| `-c stk500v2` | Hung / unreliable on this board — prefer `wiring` |

Close any serial client (`exercise_arm.py`, etc.) before avrdude. Bootloader may share the same CH340 COM port as runtime firmware.

---

## 6. Host communication protocol

### 6.1 Transport

| Parameter | Value |
|-----------|-------|
| Physical | USB → CH340 → UART |
| Baud | **115200** 8N1 (default) |
| Framing | Lines terminated with `\n` (CRLF also accepted) |
| Encoding | ASCII |

### 6.2 Startup banner

On connect / reset, firmware may emit:

```text
WLKATA Robot started successfully.Firmware version:MT4,20240820
<Alarm,Angle(ABCDXYZ):...,Cartesian coordinate(XYZ RxRyRz):...,Pump PWM:0,Valve PWM:0,Motion_MODE:0>
```

Blank line or `?` often returns `ok`.

### 6.3 Status query (`?`)

Primary real-time status mechanism. **Marlin commands `M114`/`M115`/`M119` are not supported** (`Error,E112,Unsupported command`).

**Status line format** (parsed in `mt4_client.py`):

```text
<State,Angle(ABCDXYZ):a,b,c,d,x,y,z,Cartesian coordinate(XYZ RxRyRz):...,Pump PWM:n,Valve PWM:n,Motion_MODE:n>
```

| Field | Meaning |
|-------|---------|
| `State` | `Idle`, `Run`, `Hold`, `Home`, `Alarm`, `Check`, `Door`, … |
| `Angle(ABCDXYZ)` | Joint-related angles (degrees); see axis mapping below |
| `Cartesian coordinate(XYZ RxRyRz)` | TCP pose when Cartesian mode active |
| `Pump PWM` / `Valve PWM` | End-effector pneumatic outputs |
| `Motion_MODE` | Active motion/coordinate mode indicator |

### 6.4 Machine states

```
                    ┌──────────┐
         power-up   │  Alarm   │◄──── limit fault, lock, error
            ───────►│ (locked) │
                    └────┬─────┘
                         │ $X or $H|M50 per unlock flow
                         ▼
                    ┌──────────┐     ~      ┌──────────┐
                    │   Idle   │───────────►│   Run    │
                    └────┬─────┘            └────┬─────┘
                         │                       │ !
                         │ $H                    ▼
                         ▼                  ┌──────────┐
                    ┌──────────┐            │   Hold   │
                    │   Home   │            └──────────┘
                    └──────────┘
```

| State | Host behavior |
|-------|---------------|
| `Alarm` | Motion locked; `$X` or homing/unlock sequence required |
| `Idle` | Ready after unlock (`M50`) |
| `Run` | Executing queued motion |
| `Hold` | Feed hold (`!`); recover with `M50` + `~` |
| `Home` | Homing cycle in progress (`$H`) |

---

## 7. Motion & coordinate systems

### 7.1 Coordinate modes

| Command | Mode |
|---------|------|
| `M20` | **Cartesian** — TCP position (XYZ + orientation) |
| `M21` | **Angle** — joint-space degrees (primary mode used in this repo) |

Arc commands (`G2`/`G3`) apply **only in Cartesian mode** (`Error,E115`).

### 7.2 Axis mapping (angle mode — verified in project)

Firmware G-code letters map to arm joints as used by `mt4_client.py`:

| G-code axis | Arm joint | Status index |
|-------------|-----------|--------------|
| `X` | **J1 base** | `angle_x` in `?` response |
| `Y` | **J2 shoulder** | `angle_y` |
| `Z` | **J3 elbow** | `angle_z` |
| `A` | **J4 wrist** | `angle_a` (field `d` in `ABCDXYZ`) |

Fields `a,b,c` in `ABCDXYZ` are additional angle channels (likely auxiliary / internal axes).

### 7.3 Typical move sequence (verified)

Firmware queues G-code; host must **unlock** and **cycle-start** each motion:

```text
M50                              ; unlock axes
M21 G90 G01 X.. Y.. Z.. A.. F..  ; absolute angle-mode linear move
~                                ; cycle start / execute
```

Relative jog:

```text
M21 G91 G01 X5.000 F3000
~
```

| Parameter | Practical range (project) |
|-----------|-------------------------|
| Feed `F` | 100 – 3000 (client clamps to this) |
| Gripper `M3S` | ~40 open, ~60 closed |

### 7.4 Homing & unlock

| Command | Action |
|---------|--------|
| `$H` | Run homing cycle |
| `M50` | Unlock each axis (required before motion after alarm/hold) |
| `$X` | Kill alarm lock (Grbl heritage) |
| `!` | Feed hold |
| `~` | Cycle start / resume |

After alarm: firmware may report `Error,A106,Locked status of each axis` until unlock/homing.

### 7.5 Kinematics parameters (firmware strings — EEPROM `$` settings)

Linkage-style robot model exposed in settings help text:

| Setting | Description (from firmware) |
|---------|----------------------------|
| `$31` | LINKAGE1 link length |
| `$32` | LINKAGE2 link length |
| `$33` – `$36` | CENCER/HEAD offset and height lengths |
| `$37` – `$38` | Interpolation enable / count |
| `$39` – `$40` | Compensation enable / count |
| `$48` – `$50` | X/Y/Z tool offsets |
| `$53` – `$54` | Max angle between axis 2 and axis 3 (coupling constraint) |

Workspace violations: `Error,E118` (Cartesian), `Error,E119` (angle out of range),
`Error,A107` (axes 2 and 3 conflicting angles).

---

## 8. Command reference

### 8.1 Grbl-standard commands (supported)

| Command | Description |
|---------|-------------|
| `?` | Status report |
| `$H` | Homing cycle |
| `$X` | Clear alarm lock |
| `~` | Cycle start |
| `!` | Feed hold |
| `$$` | View all settings |
| `$#` | View parameters |
| `$G` | Parser state |
| `$I` | Build info |
| `$N` | Startup blocks |
| `$x=value` | Save setting |
| `ctrl-x` | Soft reset |

### 8.2 WLKATA-specific M-codes (from firmware strings + project use)

| M-code | Description |
|--------|-------------|
| `M20` | Enter Cartesian mode |
| `M21` | Enter angle (joint) mode |
| `M3S<n>` | Gripper / tool PWM |
| `M40` | Start calibration; clear reset parameters |
| `M41` | Write reset parameters to EEPROM |
| `M42` | Start **axis 6** calibration |
| `M43` | Write axis 6 reset parameters to EEPROM |
| `M50` | Unlock axes |
| `M60` | Query color sensor RGB (`Data, Color:`) |
| `G4 P0` | Dwell / planner flush (used in `retarget_test.py`) |
| `G38.x` | Probe cycle (Grbl heritage) |

### 8.3 Unsupported / different from Marlin

| Command | Response |
|---------|----------|
| `M115` | Partial/garbled; not standard Marlin firmware info |
| `M114` | `Error,E112,Unsupported command` |
| `M119` | `Error,E112,Unsupported command` |

---

## 9. Configuration (`$` settings)

Firmware extends Grbl `$0`–`$30` with robot-specific `$31`–`$54`. View live values:

```text
$$
```

### 9.1 Standard Grbl-like settings (labels from flash)

| Area | Examples |
|------|----------|
| Step timing | step pulse (µs), step idle delay, step port invert mask |
| Direction | dir port invert mask |
| Limits | soft/hard limits, limit pins invert, probe pin invert |
| Homing | homing cycle, dir invert masks, feed/seek rates, pull-off, debounce |
| Planner | junction deviation, arc tolerance, report inches |
| Axis mechanics | max rate, accel, min/max travel, step/deg, backlash |

### 9.2 WLKATA extension settings (`$31`–`$54`)

| Setting | Purpose |
|---------|---------|
| `$31`–`$32` | Linkage lengths |
| `$33`–`$36` | Head/sensor geometry |
| `$37`–`$38` | Interpolation |
| `$39`–`$40` | Compensation |
| `$41`–`$42` | reset_pos / back-to-text flags |
| `$43`–`$45` | XYZ offsets |
| `$46`–`$47` | Rail vs conveyor mode |
| `$48`–`$50` | Tool offsets |
| `$51` | Tool type: 0=none, 1=suction, 2=grip, 3–4=soft claw, 5=custom |
| `$52` | Enable serial color sensor (`$52=1` for `M60`) |
| `$53` | Enable 7th axis home |
| `$54` | Axis 2–3 angle coupling limit |

EEPROM failure falls back to defaults: `Info,E106,EEPROM read fail. Using defaults`.

---

## 10. Error & info codes

### 10.1 Application errors (`Error,Axxx`)

| Code | Message |
|------|---------|
| A101 | Hard limit |
| A102 | Soft limit |
| A103 | Abort during cycle |
| A104 | Probe fail |
| A105 | Homing fail |
| A106 | Locked status of each axis |
| A107 | Axis 2 and 3 conflicting angles |
| E112 | Unsupported command |

### 10.2 Parser / planner errors (`Error,Exxx`)

| Code | Message |
|------|---------|
| E100–E103 | G-code syntax / value errors |
| E108 | Alarm lock |
| E109 | Homing not enabled |
| E113 | Undefined feed rate |
| E114 | Door command — Cartesian only |
| E115 | Arc — Cartesian only |
| E118 | Outside workspace |
| E119 | Axis angle out of range |
| E120 | Color sensor mode off (`$52=1` required) |

---

## 11. Peripherals & end effectors

| Peripheral | Control | Notes |
|------------|---------|-------|
| Gripper | `M3S<pwm>` | PWM 40–60 typical; immediate (not queued like some moves) |
| Vacuum pump | `Pump PWM` in status | Pneumatic tool path |
| Valve | `Valve PWM` in status | Paired with suction mode (`$51=1`) |
| Color sensor | `M60`; `$52=1` | Returns RGB; checksum errors if sensor fault |
| Conveyor / rail | `$46`–`$47` | Motion platform mode selection |

---

## 12. Motion driver layer (PCB → firmware gap)

### What we know

- Drivers are **TMC2209-LA** in **STEP/DIR/ENABLE** mode (labels on modules).
- Firmware contains Grbl **step port invert mask**, **dir port invert mask**, **step pulse** width — classic GPIO bit-bang or timer-driven stepping.
- Settings use **step/deg** — rotary joint axes, not linear mm-only machine.

### What we do not yet know

| Item | How to determine |
|------|------------------|
| PORT/BIT per STEP/DIR/EN | Ghidra disassembly of stepper ISR |
| Which driver socket = which axis | Cross-reference pin map with PCB photos + continuity |
| TMC2209 UART config | Check MS1/MS2/PDN pins and driver jumper config |
| Limit switch pins | Limit ISR + `$` invert settings + meter |
| 5th–7th axis driver channels | Settings + calibration M-codes + PCB trace |

---

## 13. Repository tooling map

| File | Role |
|------|------|
| `mt4_probe.py` | Read-only port discovery and firmware query |
| `mt4_client.py` | Serial client: status parse, move, homing, gripper |
| `mt4_common.py` | Shared connect / confirm / keyboard helpers |
| `exercise_arm.py` | Interactive preset pose exerciser |
| `jog_arm.py` | Keyboard joint jog + gripper |
| `retarget_test.py` | Mid-motion retarget / hold experiments |
| `backups/` | Firmware + EEPROM images (do not commit to public repos) |
| `RESULTS_TEMPLATE.md` | Investigation worksheet |

---

## 14. Investigation checklist (completed vs open)

| Item | Status |
|------|--------|
| USB serial enumeration | ✅ COM6, CH340 `1A86:7523` |
| Firmware serial at 115200 | ✅ Banner + `?` status |
| ATmega2560 signature | ✅ `0x1E9801` |
| Bootloader over USB | ✅ `avrdude -c wiring` |
| Flash + EEPROM backup | ✅ `backups/mt4_flash_2026-07-02.*` |
| Grbl lineage identified | ✅ 0.9j + WLKATA extensions |
| Motion commands (host) | ✅ M21/G01/M50/~ pattern |
| MCU pin map (STEP/DIR ports, ISR) | ✅ `MT4_DISASM_STEPPER.md` + empirical **`MT4_PIN_MAP.md`** |
| J1–J4 drive/DIR/limit pins | ✅ Drive + 2 limits mapped; J2/J3 DIR guesses remain |
| Full 7-axis driver map | ❌ J3/J4 limits unknown; axes 5–7 not mapped |
| TMC2209 configuration method | ❌ Unknown (stand-alone vs UART) |

---

## 15. Recommended next steps

1. **Confirm J2/J3 DIR** — `pin_sweep.py --dir-for 25` / `27` (see `MT4_PIN_MAP.md`).
2. **Custom 4-axis firmware** — motion planner using confirmed pin map in `pin_map.py`.
3. **Live `$` dump** — document factory tuning (steps/deg, homing masks) if not already captured.
4. **EEPROM decode** — compare EEPROM image before/after `$$` changes.

---

## 16. Safety notes

- **Alarm state** does not necessarily mean hardware fault — may indicate power-on lock until `$H` / `M50` / `$X`.
- Always **backup flash + EEPROM** before experimental firmware writes.
- Test firmware can energize motors on boot — keep workspace clear and power removable.
- Do not modify **fuse** or **lock** bits without an ISP recovery plan.
- Only one host may open the COM port at a time (Python or avrdude, not both).

---

## Appendix A — Verified probe transcript (summary)

```
Port: COM6 @ 115200
Banner: WLKATA Robot started successfully. Firmware version:MT4,20240820
?: <Alarm,Angle(ABCDXYZ):0.000,...,Cartesian coordinate(...),Pump PWM:0,Valve PWM:0,Motion_MODE:0>
M114/M119: Error,E112,Unsupported command
avrdude -c wiring: Device signature = 1E 98 01 (ATmega2560)
```

## Appendix B — Bootloader explorer menu (embedded strings)

The tail of flash contains an interactive **Arduino Explorer** monitor (MLS):

```
H=Help  L=List I/O Ports  R=Dump RAM  F=Dump FLASH  E=Dump EEPROM
B=Blink LED  Y=Port blink  V=show interrupt Vectors  Q=Quit
```

This is part of the bootloader/monitor image, not the normal WLKATA runtime command set.

## Appendix C — References

- [Grbl v0.9 documentation](https://github.com/gnea/grbl/wiki) — baseline protocol (MT4 diverges)
- [ATmega2560 datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega2560-Data-Sheet-DS40002211A.pdf)
- [avrdude manual](https://www.nongnu.org/avrdude/)
- WCH CH340 driver: vendor package for `1A86:7523`
