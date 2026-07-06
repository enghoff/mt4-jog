# WLKATA MT4 — Architecture & Hardware Reference

Hardware and pin map for the WLKATA MT4 arm, plus the ATmega2560 flash path shared by
any firmware image. For the current custom jog firmware, its serial protocol, and the
client, see the top-level `README.md`.

---

## 1. Executive summary

The WLKATA MT4 arm controller is an **ATmega2560-based motion controller** with a
**CH340 USB–serial bridge**, communicating over **115200 baud** serial; the MCU
generates **STEP/DIR/ENABLE** signals to **TMC2209-class** stepper driver modules.

| Layer | Component |
|-------|-----------|
| USB bridge | CH340C (`VID:PID 1A86:7523`) |
| MCU | ATmega2560 (`signature 0x1E9801`) |
| Bootloader | Wiring / STK500v2 (`Arduino explorer stk500V2 by MLS`) |
| Motor drivers | StepStick TMC2209-LA (STEP/DIR/EN) |

---

## 2. System block diagram

```
┌─────────────┐    USB      ┌──────────┐   UART    ┌──────────────────┐
│  PC / Host  │────────────►│  CH340C  │──────────►│   ATmega2560     │
│  (Python)   │   serial    │  bridge  │  115200   │   16 MHz         │
└─────────────┘             └──────────┘           └────────┬─────────┘
                                                            │
                                   Firmware (see README)     │ GPIO
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

**Not controlled by the ATmega application flash:**

- CH340 USB enumeration and drivers (separate chip firmware)
- TMC2209 chopper/microstep config if set by module jumpers (standalone mode)

---

## 3. Hardware

### 3.1 Main components

| Part | Marking / role | Notes |
|------|----------------|-------|
| MCU | ATmega2560-16U | Same family as Arduino Mega 2560; 256 KB flash, 4 KB EEPROM |
| Clock | 16 MHz crystal | Standard Mega-class timing |
| USB bridge | CH340C | Near USB connector; presents as virtual COM port |
| Stepper drivers | TMC2209-LA on StepStick modules | Labels: VMOT, GND, 2B, 2A, 1A, 1B, VDD, STEP, DIR, EN |

### 3.2 USB identity (this unit)

| Field | Value |
|-------|-------|
| OS | Windows |
| Port | `COM6` |
| VID:PID | `1A86:7523` |
| Description | USB-SERIAL CH340 (COM6) |
| Manufacturer | wch.cn |
| Baud | **115200** |

### 3.3 Electrical / mechanical interfaces

| Interface | Notes |
|-----------|-------|
| 4 primary joint axes (J1–J4) | Driven directly by this project's firmware; see README |
| Additional axes (5–7) | Present on the controller but not driven by this project |
| Gripper | PWM-driven (Timer4 OC4B on D7); see README for command set |
| Limits / homing | Hardware limit switches on D20/D21 only (see §3.4) |

### 3.4 Pin map (canonical)

| Joint | Name | G-code | Drive (STEP) | DIR | Limit |
|-------|------|--------|--------------|-----|-------|
| **J1** | Base | X | **D23** (PA1) | **D22** (PA0) | **I21** (D21) |
| **J2** | Shoulder | Y | **D25** (PA3) | **D24** (PA2) | **I20** (D20) |
| **J3** | Elbow | Z | **D27** (PA5) | **D26** (PA4) | — (none) |
| **J4** | Wrist | A | **D35** (PC2) | **D36** (PC3) | — (none) |

| Signal | Pin | Notes |
|--------|-----|-------|
| Shared driver **ENABLE** | **D40** (PG1) | Active low |
| Limit inputs | **D20**, **D21** | Active-low, pull-up; not Grbl's default D10–D12 / PCINT0 |

The MCU channel order D23 → D25 → D27 → D35 does **not** follow J1 → J4 — always map
by joint using this table, not by pin-number order. This matches
`firmware/mt4_jog/src/config.h` in the current custom firmware.

**Limit switches:**

| Pin | MCU | Interrupt | Joint |
|-----|-----|-----------|-------|
| **I20** (D20) | PD1 | INT1 | **J2 shoulder** |
| **I21** (D21) | PD0 | INT0 | **J1 base** |

This MT4 unit has **two physical limit switches** (J1/J2) and none on J3/J4. J3 is
homed indirectly by driving it into interference with J2 until that displaces J2
enough to release J2's own limit switch (`firmware/mt4_jog/src/homing.cpp`); J4 has no
homing reference at all and relies on step counters staying valid since power-on.

**Pin-driving safety:**

- Only drive pins you understand — STEP, one DIR, ENABLE. Float everything else; a
  floating DIR can lock a motor to one direction regardless of software.
- Holding an unrecognized pin HIGH/LOW can disturb driver modules — prefer
  float-all + single-pin tests when probing new wiring.

---

## 4. Firmware identity

### 4.1 Backup artifacts (`backups/`)

Read-only captures via avrdude:

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

(Or `python restore_stock.py --port COM6 --yes` — see `README.md`.)

### 4.2 Memory map (approximate)

| Region | Size | Content |
|--------|------|---------|
| Application flash | ~256 KB | Firmware code, strings |
| EEPROM | 4 KB | Settings, calibration, tool offsets |
| Bootloader | Tail of flash | STK500v2-compatible serial bootloader |

First flash bytes (`0c940608…`) are consistent with an AVR interrupt vector table (RJMP to init).

---

## 5. USB programming path

```powershell
avrdude -p atmega2560 -c wiring -P COM6 -b 115200 -v
# → Device signature = 1E 98 01 (ATmega2560)
```

| Programmer | Result |
|------------|--------|
| `-c wiring` | **Works** — use this for flash/EEPROM read/write |
| `-c stk500v2` | Hung / unreliable on this board — prefer `wiring` |

Close any other serial client before running avrdude. The bootloader shares the same
CH340 COM port as the runtime firmware.

---

## 6. Motion driver layer — open unknowns

- PORT/BIT assignments in §3.4 are empirical, from direct pin manipulation, not from
  reading any firmware source.
- TMC2209 UART configuration (standalone vs UART-configured; MS1/MS2/PDN pin states).
- Full 5th–7th axis driver channel mapping (not used by this project).

---

## 7. Safety notes

- Clear workspace before jogging or homing; drivers energize while a key/command is active.
- After a stall/jerk, **power-cycle the motor supply ~10 s** before retrying — TMC2209
  drivers can latch into a non-responsive state that a USB reflash does not clear.
- Serial command success ≠ motion — pulse counts are software-only; watch the arm.
- Always back up flash + EEPROM before experimental firmware writes.
- Do not modify **fuse** or **lock** bits without an ISP recovery plan.
- Only one host may open the COM port at a time (Python or avrdude, not both).

---

## Appendix A — Bootloader explorer menu (embedded strings)

The tail of flash contains an interactive **Arduino Explorer** monitor (MLS):

```
H=Help  L=List I/O Ports  R=Dump RAM  F=Dump FLASH  E=Dump EEPROM
B=Blink LED  Y=Port blink  V=show interrupt Vectors  Q=Quit
```

This is part of the bootloader/monitor image, independent of whatever application
firmware is flashed.

## Appendix B — References

- [ATmega2560 datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega2560-Data-Sheet-DS40002211A.pdf)
- [avrdude manual](https://www.nongnu.org/avrdude/)
- WCH CH340 driver: vendor package for `1A86:7523`
