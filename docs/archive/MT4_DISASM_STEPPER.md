# MT4 stepper ISR — disassembly notes

Analysis of `backups/mt4_flash_2026-07-02.bin` (WLKATA MT4, `20240820`) using
**avr-objdump** (AVR-GCC 14.1). Ghidra was not required; results below are from
the flash image only.

**Tools:** `avr-objcopy` → ELF, `avr-objdump -m avr6 -D`, helper script `mt4_disasm.py`

**Artifacts:**
- `backups/mt4_stepper_isr.asm` — full `TIMER1_COMPA` handler disassembly
- `backups/mt4_flash.elf` — ELF wrapper for objdump

---

## 1. Interrupt vector

| Vector | Flash addr | Handler | Role |
|--------|------------|---------|------|
| `TIMER1_COMPA_vect` | `0x0044` | **`0x008E92`** | Grbl-style stepper tick (Bresenham + step pulse) |
| `PCINT0_vect` | `0x0024` | `0x09956` | Pin-change on PORTB (Grbl legacy); **MT4 limits use INT0/INT1 on D21/D20** |
| `USART0_RX_vect` | `0x0064` | `0x097D8` | Serial receive |
| `USART0_UDRE_vect` | `0x0060` | `0x098FA` | Serial transmit |

Reset vector: `0x0000` → `0x0100C` (startup).

---

## 2. ISR structure (`0x008E92`)

Classic **Grbl 0.9j** stepper interrupt layout:

```
TIMER1_COMPA @ 0x8E92
├── Save all registers
├── If step_phase (0x0A9C) == 0:
│   ├── Toggle STEP on PORTA (0x02) with 0x55/0xAA edge pattern
│   ├── Update DIR on PORTC (0x08)
│   ├── Start TIMER0 one-shot for pulse width (TCCR0A/TCCR0B)
│   └── Set step_phase = 1
├── Else (phase 1): jump to 0x9620 → clear phase, merge axis step bits
├── Bresenham: up to **6 axes** (counter 0x13F3 wraps at 6)
├── Build step bitmask in 0x0ABB (per-axis flags)
├── XOR with port invert mask 0x0A9B → runtime step mask 0x0ABC
└── Restore + RETI (tail helpers at 0x8E80)
```

**Step pulse timing:** `TCCR0A` (`0x26`) / `TCCR0B` (`0x25`) — variable pulse width from setting at `0x0ABA` (Grbl `$0` step pulse time).

---

## 3. MCU port usage (confirmed in ISR)

| AVR I/O | Symbol | Function in ISR |
|---------|--------|-----------------|
| `0x02` | **PORTA** | **STEP** outputs (read/modify/write each tick) |
| `0x08` | **PORTC** | **DIRECTION** outputs |
| `0x25` | **TCCR0B** | Step pulse width timer |
| `0x26` | **TCCR0A** | Step pulse width timer |

### PORTA / PORTC — Arduino Mega 2560 pin map

| PORT bit | Arduino D# | Typical Grbl Mega use |
|----------|------------|------------------------|
| PA0–PA7 | D22–D29 | Step bits (stock: X/Y/Z on PA2–PA4) |
| PC0–PC7 | D37–D30 | Dir bits (stock: X/Y/Z on PC7–PC5) |

**Important:** The ISR uses **runtime masks** (`0x0ABC`, `0x0ABB`, invert `0x0A9B`) loaded from EEPROM/Grbl `$` settings — not hard-coded pin constants. Factory defaults are *likely* stock Grbl Mega 2560, but **must be confirmed** with a live `$$` dump.

### Direction mask pattern in ISR

The first-phase DIR update uses `andi` masks `0x2A` / `0x54` / `0xD5` / `0xAB` on PORTC — an alternating-bit pattern across **six** port bits, consistent with **up to six direction lines** on one port (not only three).

---

## 4. Six-axis Bresenham

Evidence in ISR (`0x8F36`–`0x95F4`):

| Item | SRAM addr | Notes |
|------|-----------|-------|
| Axis index / block step | `0x13F3` | Wraps at **6** (`cpi 24, 6`) |
| Segment buffer pointer | `0x0ADE`/`0x0ADF` | Current planner block |
| Per-axis position accumulators | `0x0A9D`–`0x0AB0` | 32-bit × 6 |
| Per-axis step counters | `0x0ABD`–`0x0AD8` | 32-bit × 6 |
| Axis step request flags | `0x0ABB` | Bit flags OR'd per axis |

**Axis flag bits in `0x0ABB` (from ISR):**

| Bit mask | Set near | Likely axis index |
|----------|----------|-------------------|
| `0x04` | `0x91D2` | Axis 0 |
| `0x10` | `0x9268` | Axis 1 |
| `0x40` | `0x9300` | Axis 2 |
| `0x80` | `0x9398` | Axis 3 |
| `0x02` | `0x9430` | Axis 4 |
| `0x08` | `0x94C8` | Axis 5 |
| `0x20` | `0x9560` | Axis 6? / extension |

Planner supports **at least six stepper channels** in software. The MT4 arm uses four joints in angle mode (X/Y/Z/A); extra channels may drive gripper rotation, conveyor, or seventh axis per firmware strings (`$53`).

---

## 5. Comparison with stock Grbl Mega 2560

From upstream `cpu_map_atmega2560.h` (Grbl 0.9j):

| Signal | Stock Grbl Mega | Seen in MT4 ISR |
|--------|-----------------|-----------------|
| STEP port | PORTA | **PORTA** ✓ |
| DIR port | PORTC | **PORTC** ✓ |
| Step enable | PORTB bit 7 (D13) | Not toggled in this ISR slice; likely elsewhere |
| Limits | **D20 (INT1), D21 (INT0)** | **I20 = J2 shoulder, I21 = J1 base**; Grbl D10–D12 / `PCINT0` not used for MT4 limits |
| Spindle PWM | PORTH / TIMER4 | Not in stepper ISR |

**Conclusion:** The stepper ISR is **structurally identical to Grbl Mega 2560** with
**extended multi-axis** (6+) Bresenham. PORTA/PORTC assignment matches upstream.
Empirical **bit→joint** mapping for J1–J4: see **`MT4_PIN_MAP.md`**.

---

## 6. Other port activity in firmware (not all stepper ISR)

| Port | Access count (scan) | Notes |
|------|---------------------|-------|
| PORTD | 32 | General I/O, serial-related |
| PORTE | 29 | Includes D0/D1 UART pins |
| PORTC | 12 | DIR + other |
| PORTA | 11 | STEP + other |
| PORTB | 7 | Limits / enable candidates |
| PORTF | 2 | Aux I/O |
| PORTL | STS `0x010B` | Extended port; bootloader tests + rare app use |

Bootloader “Arduino explorer” menu (`0x3F350+`) can blink any port for hardware test — not runtime pin config.

---

## 7. How to reproduce

```powershell
# Convert backup to ELF
avr-objcopy -I binary -O elf32-avr --change-addresses 0x0 `
  backups\mt4_flash_2026-07-02.bin backups\mt4_flash.elf

# Disassemble stepper ISR
avr-objdump -m avr6 -D --start-address=0x8e92 --stop-address=0x9620 `
  backups\mt4_flash.elf > backups\mt4_stepper_isr.asm

# Helper (vectors + port scan)
python mt4_disasm.py --objdump
```

### Live validation (recommended)

With the arm connected and workspace safe:

```text
?
$$
```

Record `$0` (step pulse), step port invert mask, dir port invert mask, and homing/limit settings. Compare to disassembly SRAM layout above.

### Logic analyzer

Jog one joint (`jog_arm.py`), probe PORTA/PORTC pins on the Mega header — correlate STEP/DIR with joint motion faster than full static RE.

---

## 8. Open items

| Item | Status |
|------|--------|
| TIMER1_COMPA handler address | **Done** — `0x8E92` |
| STEP = PORTA, DIR = PORTC | **Done** |
| 6-axis planner in ISR | **Done** |
| Bit → J1–J4 → driver socket | **`MT4_PIN_MAP.md`** (empirical); J2/J3 DIR guesses |
| Stepper ENABLE pin | **Not found in ISR** — check PORTB D13 or per-driver EN |
| PORTL / PORTF for extra axes | **Minimal** in motion path |
| Ghidra project | Optional; avr-objdump sufficient for ISR |

---

## 9. Key disassembly excerpt

Step pulse edge on PORTA + direction on PORTC (`0x8ECC`–`0x8EFE`):

```asm
; phase 0: assert step + dir
8ecc:  in  r18, PORTA        ; 0x02
8ece:  lds r24, 0x0ABC       ; runtime step mask
       ; ... 0x55/0xAA edge merge ...
8eda:  out PORTA, r25
8edc:  in  r25, PORTC        ; 0x08
       ; ... direction mask merge ...
8ee4:  out PORTC, r24
; phase 0 second half: de-assert step
8ee6:  in  r18, PORTA
8ee8:  lds r24, 0x0ABB
       ...
8ef4:  out PORTA, r25
8ef6:  in  r25, PORTC
       ...
8efe:  out PORTC, r24
; pulse width timer
8f04:  out TCCR0A, r24       ; 0x26
8f08:  out TCCR0B, r24       ; 0x25  (ldi 0x02)
8f0c:  sts 0x0A9C, r1        ; step_phase = 1
```

---

*Generated from firmware backup 2026-07-02. See also `MT4_ARCHITECTURE.md`.*
