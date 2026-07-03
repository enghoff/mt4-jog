# MT4 USB investigation results

Date:
Investigator:
Robot / controller ID (if any):

## Environment

| Field | Value |
|-------|-------|
| OS | |
| Python version | |
| pyserial version | |
| avrdude version | |
| CH340 driver installed? | yes / no / N/A |

## Serial enumeration

| Field | Value |
|-------|-------|
| Detected port | e.g. `COM6` / `/dev/ttyUSB0` |
| VID:PID | e.g. `1A86:7523` |
| Port description | |
| CH340-like (probe flag) | yes / no |
| Other ports seen | |

## Firmware probe (`mt4_probe.py`)

| Field | Value |
|-------|-------|
| Command run | e.g. `python mt4_probe.py --port COM6` |
| Port opened successfully? | yes / no |
| Error if open failed | |
| Baud rate(s) with any response | e.g. `115200` |
| Startup/banner text | paste or “none” |
| Commands that produced responses | e.g. `?`, `M114` |
| MT4 status line seen? | yes / no |
| Example status line | e.g. `<Idle,Angle(ABCDXYZ):0.0,...` |
| Raw notes | |

### Per-baud notes (optional)

| Baud | Banner | Responding commands | MT4 `?` status |
|------|--------|---------------------|----------------|
| 115200 | | | |
| 250000 | | | |
| 57600 | | | |
| 38400 | | | |
| 9600 | | | |

## AVR signature (`avrdude`, read-only)

| Field | Value |
|-------|-------|
| avrdude command used | |
| Programmer (`-c`) | wiring / stk500v2 / other |
| Baud (`-b`) | |
| Reset method | auto-DTR / manual button / none |
| Device signature | e.g. `0x1e9801` or “failed” |
| Full avrdude output (paste) | |

## Conclusions

| Question | Answer |
|----------|--------|
| 1. MT4 appears as USB serial? | yes / no |
| 2. Existing firmware responds to read-only queries? | yes / no |
| 3. ATmega2560 bootloader accessible over USB? | yes / no / inconclusive |
| 4. AVR signature read without write/erase? | yes / no |

### Summary (1–3 sentences)



### Recommended next steps

- [ ] Re-run probe with only working baud
- [ ] Try avrdude with manual reset timing
- [ ] Document motion-test prerequisites (power, clearance, E-stop)
- [ ] Other:

## Attachments

- Probe log / screenshot paths:
- PCB photo references:
