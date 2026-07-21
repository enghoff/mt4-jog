# Cube stacking — state of progress (2026-07-19)

Goal: `stack_cubes.py` builds the tallest possible cube stack using vision-based
picking, dead-reckoned level heights (20 mm cubes), and vision-based verification
of each placed level.

**Record so far: 2 verified levels** (when classification behaved). The remaining
gap is mechanical repeatability plus one recently fixed false-positive path
that treated seated level-2s as table misses.

## Where the stack site is and why

- Site: **(200, 60)** robot frame (a calibrated cube-top probe point, torque-safe
  radius). The original x=0 max-height spec was abandoned with user approval:
  every point on x=0 pins J1 at ±90° against the r=170 keep-out cylinder and
  placement scatter there measured 10–65 mm.
- Capture pose: **(172, 0, 340)** — near-vertical camera view (z=370 needs J3
  past the soft max and `mp` rejects it). The older pose (175, 0, 340) shadowed
  reads near (200, ±60); cubes parked there became invisible and picks clipped
  them. Never park cubes near (200, ±60).
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
   - **Along-axis coupling (root cause of false “table miss”)**: residual along
     the parallax unit `u` is *one* observation shared by height and XY. A
     stacked cube short in X projects onto `u` and reads “too tall”. Replay of
     the field case `pair-drift (−16.5, +1.0)` at true h=40 predicts
     `h_est=67.0` — matches the logged `67mm` to 0.1 mm. Positive height error
     is therefore **not** evidence of a table miss.
   - **Failure modes**:
     - **Seated / offset** (default when drift ≤45 mm): never touch the stack;
       nudge the next level’s command (capped 4 mm) via carry. This is what
       `height 67 / drift 16.5` must do.
     - **Perched**: only when drift ≤6 mm (on-column, height trusted) AND
       height reads low by >8 mm. Re-pick at measured stack height.
     - **Abort**: drift >45 mm, or on-column short by more than one cube.
   - The old `missed_low` table-pick path is removed.

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
- Monocular “height” and pairwise XY drift share the along-parallax residual.
  At site (200, 60), `u≈(+0.76,−0.65)` in pixel space maps robot −X offset into
  positive fake height (~1.6 mm height error per mm of −X). Never diagnose a
  table miss from height overshoot.

## Open problems (why the stack stalls at level 2)

1. Level-2 landings sometimes miss ~15–20 mm short in x despite feed-forward —
   the hover-measured walk doesn't fully transfer to the place context.
2. Repeated grasp failures on specific cubes (e.g. green at (231.7, 157.6)) —
   the shadow-zone occlusion explains earlier aborts; unverified since the
   capture-pose / parking fix.
3. After the false `missed_low` path was interrupted mid-recovery, the desk
   layout may not match "clean level-1 only" — check with vision before re-run.

## Field state at stop

Last run interrupted during a false table-miss recovery on level 2 (green on
red). Arm was mid-`deliver` when stopped. Re-check scene before assuming the
earlier parking map (level-1 blue at site; blues/reds/green at park slots).

## Recommended next steps

1. Re-run with the tightened `missed_low` / signed-perch gates — the
   (55 mm / 11 mm drift) case should now seat-and-nudge, not table-pick.
2. If specific-cube grasps still fail, investigate per-color grip S values.
3. Long term: calibrate kinematic constants from hover profiles at several radii
   to kill the z-walk at the source (benefits everything, not just stacking).

## Code review findings (2026-07-19, uncommitted changes) — ADDRESSED

All six fixed in code on 2026-07-19 (same day, later session); #6 retains a
small hardware confirmation item.

1. **Line-ending churn** — fixed: `core.autocrlf=true` was already set (staged
   blobs verified LF); added `.gitattributes` (`* text=auto`, jpg/png binary)
   so normalization no longer depends on client config, and ran
   `git add --renormalize .`.
2. **Site-clearing off-by-one** — fixed: the loop now runs
   `SITE_CLEAR_ATTEMPTS + 1` iterations with the abort moved inside, so the
   "still occupied" stop only fires on a fresh occupancy reading taken *after*
   the last clear. The `for/else` is gone.
3. **`add_observation` contradicting the coupling insight** — fixed: the
   seated branch only feeds `along` into the `hc` fit when
   `drift <= DRIFT_OK_MM` (on-column); off-column reads still update
   `prev_found` for pairwise drift but never train the height model.
4. **`avoid_xy` last-park-only memory** — fixed: `avoid_xys` is now a list;
   every rejected-cube park spot this level is excluded from candidate
   selection, so a cube parked on try 1 can't be re-picked on try 3.
5. **`DRIFT_REPICK_MM`** — deleted.
6. **Held-cube phantom in `choose_park`** — **verified on hardware and
   fixed** (2026-07-19, two pick-hover-return cycles with a red cube).
   Measured: the gripped cube at the capture pose IS detected by
   `detect_cubes` — at (640,307)±1 px, 54–57 px from the parallax-predicted
   pixel (686,339), area **622 / 568 px²**. Two lessons vs the offline
   prediction:
   - **Area does not identify it.** Parallax scaling predicted ~2×
     (700–1070 px²), but the gripper fingers occlude part of the top face and
     the blob lands inside the normal table-cube range — an area gate never
     fires. Identity is **held color + proximity to the predicted pixel**
     (`scene.is_held_cube_blob`, 90 px radius; the caller always knows what
     it grips, and the nearest real same-color cubes sat 101/149 px away).
     `capture_scene(held_cube_px=…, held_color=…)`; `choose_park`/`park_held`
     pass the level color through.
   - **Blast radius is small at this capture pose.** The phantom's
     table-plane projection lands at robot **(−10,−28)** — inside the r=170
     keep-out, ~200 mm from every marker/slot — so it never actually vetoed
     a park spot (and `filter_phantoms` already excluded it from pick
     candidates via reachability). The filter still matters if the capture
     pose ever moves or slots creep inward; it is verified to drop exactly
     the held detection and keep all six real cubes.

Minor (leave unless it grows): `validated_grip`'s mixed return contract
(3-tuple / `("parked", xy)` / `None`), plus an unreachable trailing `return`.
