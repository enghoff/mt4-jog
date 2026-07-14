# Shuffle / pick-accuracy investigation — handover

Date: 2026-07-14
Branch: `vision-pick-place`

## Repo state

The 11 commits described below were briefly lost from `vision-pick-place` — the branch got reset back to `4d30bfa` (the commit before this work started) by something outside this session, not a command run during the investigation. They've since been recovered onto a new branch:

**`vision-pick-place-fixes`** (created at `d071af8`) — has the full set: calibration refit + fixes 2, 3, 4, 5, 6, 7, 9, 10 committed, with fix 8 (carry speed) reverted per a later decision.

`vision-pick-place` itself was left untouched at `4d30bfa` and does **not** contain any of this work. To merge the recovered fixes back into it:

```
git checkout vision-pick-place
git merge vision-pick-place-fixes   # fast-forward, since 4d30bfa is an ancestor of d071af8
```

## Problem reported

The shuffle loop (`mt4_vision/shuffle.py`, driven by `shuffle_blocks.py` / `python -m mt4_vision shuffle`):
- Did not consistently identify empty marker locations and sometimes attempted to stack a cube on top of another.
- Picked frequently off-target and missed, with cubes grabbed by a corner rather than centered in the gripper.
- Never rotated the gripper (J4) to align with cube orientation.

## Root cause

**The deployed calibration was missing its cube-top height correction** (`cube_top_homography` was `null` in `vision_calibration.json`). Cube detections were mapped through the raw table-plane homography, which is only accurate for objects lying flat on the table (like the ArUco markers used to fit it). Cubes are ~20mm tall and the camera views the desk at a low oblique angle, so every cube-top detection carried **5–27mm of position error, varying across the desk** (measured directly with `calibrate_height.py`, an 7-point robot-frame probe grid).

That single defect explains most of the reported symptoms:
- Off-target/corner picks: the gripper descended to a position 1–27mm from the cube's true center.
- Missed grasps and shoved cubes: near-miss picks (~10–15mm error) don't grip — the fingers squeeze the cube out and displace it a few mm instead.
- Marker occupancy misreads → stacking: a cube actually on a marker could read 20–35mm away from it, straddling or exceeding the (then) 40mm occupancy radius, so the marker was misclassified as free.

Fixed by refitting: `python calibrate_height.py` → held-out validation error 4.4mm; a subsequent live vision-guided pick→place roundtrip landed 3.3mm from the commanded position. Pre- and post-refit calibration snapshots are saved in `backups/vision_calibration_pre_cubetop_20260714.json` and `backups/vision_calibration_cubetop_20260714.json` (the live `vision_calibration.json` is gitignored, per-setup).

Beyond the calibration gap, the workspace/occupancy model itself (`mt4_vision/workspace.py`) had several independent bugs that made stacking possible even with good position accuracy — see fixes 2, 5, 10 below — plus the capture pipeline sourced its planning frame while the arm was still occluding part of the desk (fix 3).

## Fixes implemented (each its own commit)

| # | Commit | What changed |
|---|--------|--------------|
| 1 | `58f8ede` | Cube-top calibration refit (see Root cause above). Backups committed since the live calibration file is gitignored. |
| 5 | `a9a7775` | `apply_completed_move` no longer deletes every cube within clearance of the pick point (which could erase an innocent neighboring cube and falsely free its marker) — only the single nearest detection is removed. Also dedupes the synthetic placed-cube for `to_slot` destinations the same way it already did for markers. |
| 10 | `df95bb5` | `partition_cubes_on_markers` now does global nearest-pair matching over all (cube, marker) pairs instead of per-cube greedy assignment — a cube that loses a tie for one marker can still claim a different marker it's actually sitting on, instead of being dropped off-marker entirely. |
| 2 | `b23fb1d` | Marker occupancy is now classified from **ArUco tag visibility**, not proximity alone: a decoded tag proves the marker empty; an undecoded tag with a nearby cube is occupied; an undecoded tag with *no* nearby cube is a new **`unknown`** state — never offered as a placement target. (Previously, "no cube detected near this marker" was read as free, which is exactly how the arm occluding a marker caused a stack.) Occupancy radius tightened from 40mm to 22mm now that cube-top detections are accurate; separate clearance constants added for pick (45mm) vs. place (45mm) vs. marker-paper standoff (40mm). |
| 4 | `92f30fb` | `MarkerOccupancyTracker`: carries a marker's occupied state across frames while it reads `unknown` (e.g., arm transiently occluding it), for up to 4 consecutive unconfirmed frames, clearing immediately the moment the tag decodes or a move picks from that marker. |
| 6 | `bab0e35` | Planner only picks cubes with ≥45mm clearance from every other detected cube (`pickable_cubes`) — the opened gripper was clipping close neighbors instead of grasping the intended cube. Also stops the planner from blind-picking a marker occupant that's only known via the tracker's carry (not currently detected). |
| 3 | `b14f67b` | Arm retreats to a camera-clear "park" pose (`retreat_for_camera`, the homed TCP position) before the post-move capture, instead of capturing with the arm still hovering over the just-placed cube. Live A/B comparison: hovering produced an 832px² phantom "red cube" detection (the arm's own forearm) at a reachable position and shrank/shifted the real cube underneath it; parked, neither artifact appeared. |
| 7 | `b517032` | `verify_move_outcome` checks the post-move frame for the picked cube's color at the destination, at the origin (`grasp_failed`), or neither (`lost`) before folding the move into the model — previously every commanded move was assumed successful and folded in regardless, so a failed grasp left a phantom cube at the destination and a forgotten real one at the pick point. |
| 8 | `fa43af3`, reverted by `d071af8` | Added a slower `carry_speed_us` for transits while a cube is held (a marginal grip was observed dropping its cube mid-transit at full travel speed). **Rolled back per a later explicit request** — not currently active. |
| 9 | `1ef7518` | Groundwork only: `CubeDetection.angle` estimates each cube's top-face yaw from its blob's `minAreaRect`, mapped into the robot frame via the calibration's local Jacobian. `pick()` accepts an optional `yaw_deg` to hold a world-frame gripper orientation through the descent. **Not wired into the planner** — see blocker below. Also fixed a latent bug found in passing: `move_to()` formatted floats at full precision, and a computed j4 value could push the `mp` command past the firmware's 64-byte serial line buffer, truncating and rejecting the command; coordinates are now formatted at 0.01 precision. |

All of the above (except the calibration refit) have unit test coverage in `tests/test_workspace.py` (23 tests, no hardware required — run with `python tests/test_workspace.py`).

## Outstanding blocker: gripper hardware fault

While attempting to measure the jaw-axis yaw convention for fix 9 (`scripts/calibrate_jaw_yaw.py`), every pick attempt failed with the cube completely unmoved, across multiple positions, gripper-open widths (including the servo minimum, S=120), and commanded yaws. A photo comparison of the gripper at S=120 (open) vs. S=255 (closed) at the same position confirms the **servo is moving** (fingers visibly narrow between the two states — pixel diff of ~940 changed pixels in the finger region) but **the fingers are not gripping anything** — no cube was ever retained through a lift.

This looks like a mechanical fault (slipped servo horn, bent finger, loose linkage) rather than a software/calibration issue, and needs a hands-on inspection before:
- the jaw-yaw calibration script can be completed, and
- yaw-aligned picking (fix 9) can be enabled in the shuffle planner.

Until repaired, expect every pick to grasp-fail; the loop's fix-7 verification will detect this and replan rather than corrupting its model, but no cubes will actually move.

## Also noted, not yet acted on

- Two cubes are currently stranded inside the arm's 170mm keep-out radius (approx. `(-19, 162)` and `(161, -21)`) from an earlier dropped-grip incident — `mp` can never reach them; they need to be moved by hand.
- A fifth physical ArUco marker is on the desk but absent from `vision_calibration.json`'s calibration set — its paper is currently treated as open table.
- The current cube-top homography fit is a low-DOF "similarity" (4 points used in the actual fit, one held out for validation); residual error runs 5–11mm in some regions. Re-running `calibrate_height.py` with more grid points would allow the affine fit and likely tighten this further.

## Next steps

1. `git reset --hard d071af8` to restore the branch to the state this document describes (see warning at top).
2. Physically inspect/repair the gripper.
3. Re-run `scripts/calibrate_jaw_yaw.py`, wire the resulting offset into `plan_shuffle_move`/`pick()` calls to enable yaw-aligned picking.
4. Manually relocate the two keep-out-stranded cubes.
5. Re-run a supervised shuffle session with the arm properly homed to confirm end-to-end behavior (grasp verification, no stacking) once the gripper works again.
