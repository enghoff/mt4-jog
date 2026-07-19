# Envelope & soft-limits handover

Status as of **2026-07-19**. Firmware with these limits was built and flashed to COM9.

## Why this exists

`MAX_REACH_MM = 335` was too conservative for real desk work. We mapped the physical envelope by jogging and tagging poses, then encoded soft limits in firmware (jog + `mp`) and mirrored them in Python.

## How to re-map

```text
python map_envelope.py                  # default → envelope_samples.json
```

| Control | Action |
|---------|--------|
| D-pad Up / `]` | Record **in-range** |
| D-pad Down / `[` | Record **out-of-range** |
| Backspace / LB | Undo |
| Enter / Start | Save (keep jogging) |
| ESC / Back | Save and quit |

Stop anything else on the serial port first (`jog.py`, `mt4_mcp`). Only one process may own COM9.

Helpers: [`mt4_jog/envelope.py`](../mt4_jog/envelope.py). Schema includes TCP + joint steps/deg + live `summary`. File is gitignored.

## Measured envelope (from `envelope_samples.json`)

27 **in** / 11 **out** samples.

| Quantity | In-range | Notes |
|----------|----------|--------|
| Ground Z | min **135.7 mm** | Outs go to ~130 mm |
| Max reach XY | **352.1 mm** | Out at 353.6 mm |
| J1 steps | −4795‥5296 | Switch side ~−4580 after home |
| J2 steps | −485‥2922 | Switch side −1000 after home |
| J3 steps | −2021‥1122 | No hard switch |
| J4 steps | −6594‥6319 | No switch |
| **j2−j3 (deg)** at full stretch | min **15.2°** | Over-extend outs ~9–14° |
| **j2_steps + j3_steps** at full stretch | max **2910** | Equivalent to min j2−j3 (opposite step signs) |

### What actually limits full extension

Not J3 max alone. When the arm is stretched from the base, the links are nearly aligned and **j2_deg − j3_deg** bottoms out. With J2/J3 opposite step signs:

```text
j2_deg − j3_deg ≈ 98.3 − (j2_steps + j3_steps) / 35
```

So a **minimum** angle difference ↔ a **maximum** on `j2_steps + j3_steps`. Axis boxes alone **miss** some over-extension outs (e.g. sample #3); the sum catches them.

Folded / desk-scraping poses are still gated mainly by **J3 min** + **ground Z**, not by the sum max.

## Limits now in software

Keep firmware [`config.h`](../firmware/mt4_jog/src/config.h) and Python [`mt4_jog/joints.py`](../mt4_jog/joints.py) / [`mt4_vision/workspace.py`](../mt4_vision/workspace.py) in sync.

| Bound | Value | Source |
|-------|------:|--------|
| `GROUND_Z_MM` | **136** | In min Z + small clearance |
| `MAX_REACH_MM` | **350** | Vision/planning only (not jog-enforced) |
| J1 soft steps | **−4800‥4580** | After home: min = `−j1_center` (4580) |
| J2 soft steps | **−1000‥2950** | After home: min = `−j2_pull` (1000) |
| J3 soft steps | **−2050‥1150** | Envelope (both ends) |
| J4 soft steps | **−6600‥6350** | Envelope |
| **J2+J3 sum** | **−1700‥2910** | Stretch couple; min is a loose backstop |

## Firmware behaviour

On successful **home**:

1. Step counters reset to 0 at the pull-off pose.
2. J1/J2 soft mins set from pull-off distances.
3. Prints e.g. `home limits J1=... J2=... J3=... J4=... J2+J3=-1700..2910 ground_z=136.0`.

**Cartesian jog (`cj`)**:

- Soft joint limits + J2+J3 sum checked per step and in the rate planner.
- Hitting a limit **stops the whole jog** (no sliding on free axes).
- Ground plane: clamps downward Z once TCP Z ≤ ~136.5 mm.
- Keep-out cylinder (~170 mm) still projects away inward XY.

**`mp`**: rejects `err mp ground`, `err mp joints` (includes sum), `err mp keepout`.

DDA ISR is a safety net: if any axis would step out of bounds, abort all axes.

## Python behaviour

- [`mt4_jog/client.py`](../mt4_jog/client.py): `move_to` refuses z below `GROUND_Z_MM`.
- [`mt4_vision/workspace.py`](../mt4_vision/workspace.py): `MAX_REACH_MM`, `is_within_envelope`, `joints_within_soft_limits` (includes sum).
- Unit tests: [`tests/test_envelope_bounds.py`](../tests/test_envelope_bounds.py), [`tests/test_envelope_summary.py`](../tests/test_envelope_summary.py).

## Tools / files touched

| Path | Role |
|------|------|
| `map_envelope.py` | Interactive recorder |
| `mt4_jog/envelope.py` | JSON load/save + summary |
| `mt4_jog/gamepad.py` / `jog.py` | Stick speed, Y bookmark, etc. (separate jog UX work) |
| `firmware/mt4_jog/src/config.h` | Soft-limit constants |
| `firmware/mt4_jog/src/motion.cpp` | Enforce + status/`home limits` |
| `firmware/mt4_jog/src/dda.cpp` | ISR abort on limit |
| `firmware/mt4_jog/src/homing.cpp` | Apply limits after home |
| `envelope_samples.json` | Local data (gitignored) |

Flash: `python flash_jog.py` (stop jog/MCP first). Always OK to flash without asking.

## Sanity check (box vs sum)

On the 11 out samples, with current box + sum max 2910 (no false-reject of ins):

- **Caught:** over-extension (#3, #8, #12), high J3 (#5, #14), deep J3 (#21).
- **Still leaked** (need other rules / more samples): #17, #19 (near-desk mid-reach — ground helps when descending), #28 / #37 / #38 (mostly J1 yaw past the tightened J1 max of 4580).

So sum correctly models the **stretch** edge; it is not a full envelope substitute.

## Better models (next steps)

1. **Convex hull in (j2°, j3°)** from in-samples — tighter than box+sum, still cheap.
2. **`r_max(z)`** curve from samples — Cartesian desk + stretch without joint coupling math.
3. **FK / IK gate** — reject if unreachable or near-singular (condition number).
4. Re-run `map_envelope.py` after mechanical changes; recompute constants and keep `config.h` ↔ `joints.py` paired.

## Verify after flash

1. `python jog.py` → home → confirm `home limits ... J2+J3=-1700..2910 ground_z=136.0`.
2. SPACE status shows `GROUND_Z` / `SOFT` / `J2+J3`.
3. Stretch toward max reach: motion should stop cleanly (whole jog), not slide.
4. Descend to desk: downward jog should clamp near 136 mm.
