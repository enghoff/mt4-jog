# Shuffle post-move vision inconsistency — handover

Date: 2026-07-14  
Branch: `vision-pick-place` (local WIP on vision/shuffle)

## Issue to investigate

Operator reported that a **pick+place succeeded** (green left marker 0 and was set down at the commanded open-table slot), but the **next scene still showed green on marker 0** and the planner immediately repeated the same move.

If that physical claim is correct, vision/state after the drop is wrong and must be understood — not papered over with move stigma.

## Concrete log (repro evidence)

From `shuffle_blocks.py --camera 1`:

```
scene: cubes=5 blockers=3 free_markers=0 occupied=3 unknown=1 free_slots=6 phantoms_dropped=4
  red (126,-145) area=606 open
  blue (240,-84) area=536 open
  red (184,-163) area=513 marker 2
  green (42,-262) area=430 marker 0
  green (173,159) area=313 open
action: pick -- to_slot: pick green from marker 0 (42,-262) -> (150,100)
pick-and-place: green (42,-262) -> (150,100)
scene: cubes=5 blockers=3 free_markers=0 occupied=3 unknown=1 free_slots=6 phantoms_dropped=3
  red (127,-144) area=632 open
  blue (240,-84) area=540 open
  red (184,-163) area=506 marker 2
  green (42,-262) area=426 marker 0
  green (173,159) area=314 open
action: pick -- to_slot: pick green from marker 0 (42,-262) -> (150,100)
```

### Why this is inconsistent with a successful relocate

| Expectation if green actually moved 0 → (150,100) | What the post-move frame shows |
|---|---|
| No green on marker 0 | `green (42,-262) marker 0` still present |
| Green near place `(150,100)` | **No** green near `(150,100)` |
| Other greens mostly unchanged | `green (173,159)` still present (~63 mm from place, ~26 mm from marker 3) |

Areas/positions drift slightly between scenes (430→426, etc.), so this is almost certainly a **new frame**, not a raw buffered copy of the pre-move image.

## Capture path (not the primary suspect for “stale buffer”)

After each completed pick+place, shuffle:

1. drains frames during motion (`grab` loop),
2. settles (`--pause`, default 0.5 s),
3. calls `grab_frame` (flushes 5 frames, then `read`),
4. builds a new `Scene` and plans again.

So the loop **does** require a post-drop capture before the next plan. The problem is whether that capture’s **detections** match the physical desk.

## Working hypotheses (ordered)

### A. Grasp failed despite “move success” (log-consistent)

`pick()` always closes the gripper and returns ok; `place()` always runs. There is no force/sense of retention.  
Post-move vision showing green still on marker 0 and **nothing** near `(150,100)` matches **cube never left**.

Software will then correctly replan the same pick. That is annoying, but not a false map of an empty pad.

**Check:** after the first move, physically look at marker 0 vs ~(150,100).

### B. Successful move + false green still on empty marker 0 (vision bug)

If the cube is physically gone from marker 0 and sitting at the place slot:

- Detector still emits a green blob mapping to ~(42,-262).
- Detector fails to emit green at ~(150,100), **or** maps the placed cube far away (e.g. as `(173,159)` ≈ 63 mm error — large vs cubetop residual ~5–11 mm).

A live follow-up crop of **empty** marker 0 showed **0 green-mask pixels** on the ArUco paper, so “empty tag reads as green cube” is **not** the steady-state explanation. A transient false green (arm fragment, lighting, another object) during the post-move frame is still possible and needs annotated pre/post frames from the failing cycle.

### C. Related: park-adjacent / false greens elsewhere

Earlier same session:

```
green (193,-51) area=412 open
→ repeatedly planned as blocker → marker 3
```

~(193,-51) is ~51 mm from camera-park `(200,0)` — barely outside the old 50 mm park exclusion. That class is a **pick ghost** (processing labels non-cube pixels as a cube), separate from but related to “why would we grip ghosts.”

## What we already tried (and what was reverted)

| Change | Intent | Status |
|---|---|---|
| Detection-as-state `Scene` + `plan_shuffle` | Only plan from latest frame; no synthetic placed cubes | Still in use |
| Occupancy from **raw** detections; picks from filtered subset | Don’t free a marker because its occupant was phantom-filtered | Still in use |
| One-cycle `LastAttempt` stigma (don’t re-use last origin/dest) | Stop immediate repeat after misleading post-move frame | **Removed** — papered over ghosts/grip fails instead of fixing cause |
| Raise camera-park pick/place clearance 50→80 mm | Drop park-adjacent ghosts like ~(193,-51) | Still in use |

Principle agreed with operator: **do not grip ghosts**; reject them in detection/filtering. Do not “handle” them by remembering last moves.

## How to reproduce / instrument next

1. Clear desk to a known layout: one green on marker 0, nothing at `(150,100)`.
2. Run shuffle (or a one-shot script) that plans `marker0 green → (150,100)`.
3. On that cycle only, save:
   - pre-move annotated frame (all cubes + marker centers),
   - post-move annotated frame immediately after place + flush,
   - printed list of every raw green blob (robot XY, pixel, area).
4. Operator records physical: green on marker 0? on place slot? neither?
5. Compare:
   - If still on 0 and absent at place → grasp retention (hardware or pick XY error).
   - If on place but vision still reports green on 0 → false-positive green; dump green HSV mask around marker 0 pixel.
   - If on place but vision reports it only as far offset blob → cubetop / mapping issue at that XY.

Helper started for ad-hoc checks: `scripts/diagnose_marker0_green.py` (live dump + marker-0 crop/mask). Extend it to wrap a commanded pick/place with pre/post saves when the arm port is free.

## Related open context

- Earlier handover noted mechanical gripper fault (fingers close but may not retain cubes). That must be ruled in/out before blaming vision for this specific log.
- `green (173,159)` is ~26 mm from marker 3 center: outside occupy radius (22 mm) so labeled **open**, inside place clearance (45 mm) so marker 3 stays non-free/`unknown`. Worth tightening occupy vs place policy separately; not the same as the marker-0 persistence puzzle.
- Marker 4 sits near camera-park; park exclusion also removes it as a place target.

## Bottom line

The repeat plan is possible because the **post-drop scene still reports green on marker 0** and does **not** report green at the place target.  
Whether that is accurate (failed grasp) or inaccurate (vision bug after a real move) is the open investigation. The capture pipeline does fetch a new frame; the mismatch to inspect is **detection content vs physical desk**, with pre/post annotated frames from a controlled successful move.

## Resolution (2026-07-14)

Root-caused live against the physical arm (`scripts/repro_marker_persist.py`, `scripts/repro_hover_check.py`, `scripts/repro_settle_timing.py`):

1. **Gripper/kinematics are not at fault.** A ground-truth touch test (`goto_marker(..., touch=True)` onto marker 4's own calibrated robot XY) landed the fingertip exactly on that marker's tag -- positioning is accurate. Grasp mechanics and coordinate mapping (pixel -> robot round-trip) both check out.
2. **The post-move frame can be transiently stale.** `scripts/repro_settle_timing.py` sampled the scene at increasing delays after a real, successful pick+place: at `t=0.00s` (immediately after `place()` returns) the frame still showed the cube at its *old* spot and nothing at the destination -- by `t≈0.3-0.5s` it was correct. The camera driver buffers frames during the multi-second gap while nothing reads it during arm motion (same class of issue as the documented DSHOW staleness quirk); a single capture right after a fixed pause can land on that stale frame.
3. **The planner had no way to tell the difference** between "stale frame after a real success" and "genuine grasp failure" -- it trusted whatever the first post-move capture said and replanned from it, reproducing exactly this log (marker 0 still green, nothing at the place target, same pick repeated).

**Fix applied:** wired the already-written-but-unused `verify_pick_place()` (`mt4_vision/scene.py`) into `shuffle.py`'s post-move step. After a pick+place, the loop now classifies the outcome and, if it isn't `"placed"`, re-captures up to twice (0.4s apart) before trusting the result -- see `_plan_after_move` in `mt4_vision/shuffle.py`.

**Verified live** (`scripts/verify_fix_live.py`): a real successful move that initially read as ambiguous resolved to `"placed"` after one recheck, confirmed against ground truth. A separate genuine near-miss pick (same red cube nudged ~5mm twice in a row, a known ~10mm pick-tolerance issue, not this bug) still correctly reported `"grasp_failed"` even after rechecks -- the fix doesn't paper over real failures, only the transient stale-frame misread.
