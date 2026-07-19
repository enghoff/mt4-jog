# Cube stacking — state of progress (2026-07-19)

Goal: `stack_cubes.py` builds the tallest possible cube stack using vision-based
picking, dead-reckoned level heights (20 mm cubes), and vision-based verification
of each placed level.

**Record so far: 2 verified levels.** The system now self-diagnoses every failure
mode correctly instead of knocking stacks over; the remaining gap is mechanical
repeatability, not logic.

## Where the stack site is and why

- Site: **(200, 60)** robot frame (a calibrated cube-top probe point, torque-safe
  radius). The original x=0 max-height spec was abandoned with user approval:
  every point on x=0 pins J1 at ±90° against the r=170 keep-out cylinder and
  placement scatter there measured 10–65 mm.
- Capture pose: **(172, 0, 370)** — near-vertical camera view. The older pose
  (175, 0, 340) shadowed reads near (200, ±60); cubes parked there became
  invisible and picks clipped them. Never park cubes near (200, ±60).
- Inspection pose (grip check): **(220, 66, 250)**.

## What the script does

1. **Vision pick** with the standard calibration stack (table homography +
   cube-top homography + residual layer), pick candidates kept >70 mm from the
   site. Grasp verified by moving clear of the pick point and re-detecting.
2. **Grip validation**: every held cube hovers at the inspection pose; its pixel
   offset from a per-color reference translates to mm. Grips off by >3.5 mm are
   set down at an arm-known spot and re-gripped (max 3 tries). References are
   re-based on verified-good levels.
3. **Delivery**: climb → via (r=235) → over stack → lower → contact release
   (+1 mm above the seat — a 3 mm drop onto a misaligned edge bounces cubes
   30–40 mm) → clear → capture pose. All loaded legs run at approach speed
   (2400 µs): fast cruise on loaded high-z moves stalls steppers (r=315
   precedent). Partial-open release (S=200) was tested and rejected (snags,
   30 mm drags).
4. **Feed-forward**: the arm's XY walks linearly with commanded z —
   (−4.4, −2.9) mm/level at (200, 60), ~(−8, −6) at r≈190. Measured by hovering
   a held cube through the level heights. Compensated via `--shift-per-level`;
   the closed-loop `carry` integrator excludes the feed-forward term.
5. **Vision verification** (monocular height-from-parallax):
   - Two calibrated planes (table 0 mm, cube-top 20 mm) define a parallax line
     per point; camera height Hc self-fits from observations (~700 mm).
   - **Anchor-relative**: level 1's observed pixel defines the column, cancelling
     map extrapolation error. Drift is measured pairwise (level N vs N−1's
     observed top), cancelling accumulated bias.
   - **Height/distance ambiguity**: a read "above" expected height is physically
     impossible — it means the cube landed on the table beyond the column.
     Classified as `missed_low`; recovered with a normal table pick at
     `site + ground_offset(found − anchor, h=cube)`.
   - **Evidence-gated corrections**: physical re-pick only when drift >9 mm AND
     |height error| >8 mm (perched). Moderate drift (6–45 mm) without height
     anomaly = measurement bias — never touch the stack, at most nudge the next
     level's command (capped 4 mm) via the carry integrator. This ended the
     false-positive corrections that used to topple good stacks.

## Established physics / hard-won facts

- Kinematic walk: XY drifts linearly with commanded z, everywhere, 4–9 mm per
  20 mm depending on radius. Root cause candidate for a proper fix: kinematic
  constant calibration (link lengths / joint offsets) fit from hover profiles.
- Stepper stalls: fast cruise (travel_speed_us=700) on loaded high-z extended
  moves loses steps. Slow to 2400 µs — homing is NOT the fix and per-move homing
  is banned (user directive, twice).
- Release drag: 2–6 mm semi-random even from a centred grip.
- Pick grips scatter several mm; validation (above) rejects the bad ones.
- Camera cold start: after rapid reopen, first frames may be unconverged even
  after 20 warm-up reads — script retries capture 4× at 2 s intervals until
  detect_cubes returns non-empty.
- Camera moved overnight (sun): uniform marker drift fixed by composing a
  translation shim into all three homographies + stored pixels
  (backup: `backups/vision_calibration_pre_sunshim_20260719_082401.json`).

## Open problems (why the stack stalls at level 2)

1. Level-2 landings sometimes miss ~15–20 mm short in x despite feed-forward —
   the hover-measured walk doesn't fully transfer to the place context.
2. Repeated grasp failures on specific cubes (e.g. green at (231.7, 157.6)) —
   the shadow-zone occlusion explains the last two runs; unverified since the
   capture-pose fix.

## Field state at stop

Arm homed and idle. Level-1 blue cube standing at the site (200, 60); other
cubes at parking slots — blue (280, 0), red (140, 160), red (150, −250), green
near (233, 160).

## Recommended next steps

1. Re-run as-is: the last two aborts were the (now fixed) shadow-zone issue.
2. If specific-cube grasps still fail, investigate per-color grip S values.
3. Long term: calibrate kinematic constants from hover profiles at several radii
   to kill the z-walk at the source (benefits everything, not just stacking).
