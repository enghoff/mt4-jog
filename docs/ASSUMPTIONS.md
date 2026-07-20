# Assumption audit — pick / place / stacking accuracy (2026-07-20)

Every row names an assumption the system currently relies on, what we
actually know about its validity (uncertainty), and how strongly system
performance depends on it (sensitivity). Evidence cites the 2026-07-19/20
instrumented runs where applicable.

## A. Arm / kinematics

| # | Assumption | Uncertainty | Sensitivity | Notes / evidence |
|---|---|---|---|---|
| 1 | Kinematic constants are exact: `LINKAGE1=130`, `LINKAGE2=150`, `CENCER_OFFSET=45`, `HEAD_OFFSET=35`, `HEAD_HEIGHT=14.43`, home angles (103°, 4.7°) | ±1–2 mm per link, ±0.5–1° per home angle — hand-measured (tape + phone clinometer), never fit from data | **HIGH** | The measured "z-walk" (XY drifts 4–9 mm per 20 mm of commanded z, radius-dependent) is exactly the signature of link / home-angle error; it forced the whole feed-forward + servo apparatus. A hover-profile fit would attack the root cause. |
| 2 | Steps/degree are integer and identical across J1–J3 (35.0) | A 1% ratio error ≈ 1° over 100° excursion ≈ several mm at the TCP, growing with joint travel | **HIGH**, position-dependent | Assumed from "shared gearbox design"; the EEPROM dump itself had 35.556 for another axis. Indistinguishable from map error at calibration poses, divergent elsewhere — co-candidate for the z-walk. |
| 3 | The arm is rigid (no sag, no elastic deflection) | Unmeasured; arm is demonstrably springy (exploited for press-releases), load = one 20mm cube | **MEDIUM** | "Hover-measured walk doesn't transfer to the place context" (documented) is consistent with load/pose-dependent deflection. A few mm, folded invisibly into every calibration. |
| 4 | Commanded moves position the TCP exactly (no backlash/stiction) | Measured ~6–9mm dead-zone on small reversing moves at r=209 (attempts 1, 5); apparent response "gain ~2" with sign flips in x even after take-up moves (attempt 10) | **HIGH** | Current convergence floor of the landing servo. Confounded with #11 — the "gain 2" could equally be measurement noise, not backlash. |
| 5 | Step counters = physical position (no lost steps in normal operation) | No encoders; `get_tcp` reports *commanded* steps, not reality | **CATASTROPHIC when violated** | r=315 stall precedent shows this fails under load+speed; slow loaded legs mitigate, but nothing *detects* a silent stall — every later arm-frame coordinate is wrong until re-homing. |
| 6 | Motion between waypoints is a Cartesian straight line | Unmeasured; plausibly 5–15mm mid-path sag/arc at these radii (firmware interpolates joints, not Cartesian) | **MEDIUM-HIGH** near the stack | Retreat legs clear the placed cube by only ~12–16mm — same order as plausible sag. |
| 7 | Commanded z is true z (`pick_z=149` everywhere) | Unconstrained; monocular vision cannot observe z error (it couples into XY) | **HIGH** | ±3mm decides contact-set vs. edge-drop. Edge-drops bounce cubes 30–60mm (attempts 5–7); current mitigation (release 3mm below nominal seat) validated once. |

## B. Camera / vision geometry

| # | Assumption | Uncertainty | Sensitivity | Notes / evidence |
|---|---|---|---|---|
| 8 | The camera does not move after calibration | Violated twice in two days (sun/heat); drifts of ~4px ≈ 4mm observed, shim-corrected to <0.6px | **HIGH** for absolute pick/place | Detectable only when markers are visible — parked cubes occluded 3 of 5 markers today. Anchor-relative stack logic is partially immune; picks and level-1 placement inherit the full error. |
| 9 | Pinhole camera, planar table, exact homographies — no lens distortion | Up to several px of position-dependent error; a 720p webcam typically has 1–3% radial distortion | **MEDIUM-HIGH, hidden** | No undistortion exists anywhere in the pipeline. Corrupts *all* derived geometry (residual layer, parallax direction `u`, px→mm Jacobian) a little, rather than one thing a lot. A one-time chessboard intrinsic calibration would remove it. |
| 10 | Camera height Hc ≈ 700mm, "self-fitting" from observations | ±10%? Fit requires ≥2 on-column observations and has effectively stayed at the seed guess in every recent run | **LOW-MEDIUM** | Scales height estimates and px→mm conversions ~proportionally; wrong Hc bends classification thresholds. |
| 11 | The top-face centroid is the cube's true center | ±2–5px per reading | **HIGH** | V-seeded segmentation assumes symmetric occlusion; the gripper fingers demonstrably occlude part of the held face (620px² vs ~2x expected). Likely alternative explanation for the ±5–8mm alternating hover readings currently attributed to backlash "gain 2" (#4) — currently confounded. Discriminating experiment: repeatedly measure a *static* cube through the servo path. |
| 12 | Cubes are the only colored blobs, separable, in fixed HSV bands | Frequent, environment-dependent failures | **MEDIUM-HIGH** | The arm itself is orange (reads red; `MAX_BLOB_AREA` + keep-out/hull the guards), hand/watch enter frame, adjacent cubes merge, lighting shifts move V/S. Area caps are mount-distance-sensitive: after moving the camera closer (2026-07-20) a real cube read ~2790px² and was dropped by the old 900/650 caps while arm flecks survived. Static-lock/misidentification class (attempts 3, 6, 8) patched with motion checks/exclusions/tight radii, not better perception. |
| 13 | A frame taken ≥0.8s after motion shows the settled scene | Unmeasured | **LOW-MEDIUM** | Vibration/pendulum sway of a held cube after a move is assumed damped within the settle sleep. Adds to #11's noise floor. |
| 14 | Partial occlusion does not materially shift reported cube XY | Centroid is the mean of *visible* mask pixels — asymmetric occlusion pulls it toward the unobstructed side (mm-scale on the desk). Gripper on held cube: fingers hide part of the top face; measured 2026-07-19 area 622px² (not the ~2× parallax-predicted size) and centroid ~54px from prediction. No geometric occlusion model anywhere in the pipeline. | **MEDIUM-HIGH** | On-desk cubes: open-loop picks use the last capture; a gripper or neighbour hiding part of the face reports a shifted `(x,y)` or drops below area filters. Held cube in the desk scene: excluded by color + predicted pixel (`is_held_cube_blob`), not repositioned. Held cube for inspect/servo: position *is* used and is biased — mitigated by tight/tracked search, motion checks, and re-measure loops (#24), not by correcting for fingers. Related to #11 (symmetric top-face) but distinct: occlusion is dynamic and pose-dependent. |

## C. Cube and grasp physics

| # | Assumption | Uncertainty | Sensitivity | Notes / evidence |
|---|---|---|---|---|
| 15 | Cubes are ideal rigid 20.000mm cubes | Real wooden cubes vary ±0.3–0.5mm with rounded corners; cumulative ±2–4mm over 8 dead-reckoned levels | **MEDIUM, grows with height** | The −3mm contact-release offset silently becomes a 0…−7mm press (or a drop) at higher levels. Worth switching release z to *measured* stack height once the model has observations. |
| 16 | A gripped cube stays fixed in the jaws | Release drag of 2–6mm proves the cube-jaw interface moves; slip during transport unmeasured | **MEDIUM** | Grip offset is measured once at inspect pose, assumed constant through transport. The hover servo re-measures at the stack and cancels most of it — one of its main contributions. |
| 17 | Opening the gripper is mechanically neutral | Measured ~4–5mm drag on a normal contact release; ~28mm flick opening under a deliberate 2mm press (attempt 10); 30–60mm bounce releasing from a >2–3mm drop onto an edge (attempts 5–7) | **EXTREME** | The least-controlled event in the pipeline, and today's dominant per-level failure mechanism. Mitigation: contact-height release without press; residual ~4–5mm scatter is the floor the verification/nudge loop must absorb. |
| 18 | Placed cubes stay put | Near-miss picks measured to shove neighbours (→ isolation-preferring selection added); a passing gripper 12mm above a stack assumed to clear it (see #6) | **MEDIUM** | |
| 19 | Cube yaw is irrelevant | `j4_face_offset_deg` (jaws vs j4=0) measured on hardware via `calibrate_j4.py` (33.0°) | **LOW** | Detection reports robot-frame edge yaw (`CubeDetection.yaw_deg`); `pick_cube` face-aligns J4 via `pickplace.j4_for_face_align`. `Calibration.face_align_picks` now defaults **on** (both the dataclass default and `vision_calibration.json`) now that the mount offset is validated. If the mount is disturbed or the offset is re-measured differently, flip `face_align_picks` off in `vision_calibration.json` until revalidated (wrong offset → more corner grips than a fixed yaw). |
| 20 | A ≤6mm-offset cube is a stable platform; tilt does not accumulate | Attempt 6 disproved the naive version: an on-column cube reading +11mm high was tilted, and the next level slid off | **HIGH for the max-height goal** | Now classified as perched → re-seat, but inter-face friction, lean accumulation, and the topple threshold of an 8-high column are all unmodeled. Expect a slow-lean failure class past ~5 levels. |

## D. Calibration, software logic, environment

| # | Assumption | Uncertainty | Sensitivity | Notes / evidence |
|---|---|---|---|---|
| 21 | Probe-point accuracy generalizes across the desk | 2–4mm at calibrated probe points; measured 10–65mm scatter at the reach edge; ±10mm pick tolerance assumed uniform | **HIGH outside the calibrated core** | Attempt-8 looped on a cube at r=284; attempt-9 crashed on an IK-unsolvable candidate (now guarded). |
| 22 | One grip force fits all (`grip_close_s=255`) | Per-color/per-cube differences suspected historically, never characterized | **LOW-MEDIUM** | |
| 23 | The inspect-pose reference frame is stable within a session | References motion-verified at creation now, but in-session drift is uncorrected | **MEDIUM** | A 3mm reference error biases every grip decision for that color. |
| 24 | A blob near a predicted pixel IS the object we reasoned about | Was the deepest, most-violated software assumption | **Was CATASTROPHIC, now MEDIUM** | Every past phantom failure (stack read as held cube, desk cube as reference, arm paint as cube, held cube as desk cube) is this assumption failing in a different costume. Current defenses (exclusion zones, tight/tracked search radii, motion checks, response-verified servo identity) are heuristic patches on a perception layer with no real object identity. |
| 25 | Serial acks mean motion finished | Gripper acks on parse, not arrival (patched with a settle sleep); moves are awaited; 64-byte line buffer bounds command size | **LOW now** | Any new command pattern must re-respect these constraints. |
| 26 | The desk is ours alone | User's hand appeared mid-session; cubes were re-arranged between runs | **LOW-MEDIUM, safety-relevant** | Per-level re-scanning makes the system mostly robust to this, but nothing guards against interference *during* a motion. |
| 27 | `cube_height_mm=20` serves two masters (physical z arithmetic AND vision parallax scale) | If real cubes average e.g. 19.6mm, calibration absorbs it on the vision side while the physical side accumulates error | **MEDIUM at height** | The two silently disagree as the stack grows — same mechanism as #15 but subtler (partial, inconsistent absorption). |
| 28 | Failures are software/state bugs, not hardware faults (working doctrine) | Right so far | **Unknown by construction** | A worn gripper servo, loosened finger, or degrading joint would masquerade as familiar "scatter" and be chased in software indefinitely. Periodic physical inspection is the only check. |

## Ranked: where accuracy actually leaks today

| Rank | Leak | Related # | Why |
|---|---|---|---|
| 1 | Release event physics | #17, #7 | Extreme, dominant per-level risk |
| 2 | Held-cube measurement noise vs. backlash / occlusion confound | #11, #4, #14 | Blocks servo convergence below ~3mm; needs the discriminating experiment |
| 3 | Kinematic constants / z-walk root cause | #1, #2, #7 | Taxes everything downstream |
| 4 | Blob identity as object identity | #24, #12 | Patched, still fragile |
| 5 | Silent lost steps | #5 | Rare but unbounded damage, undetected |
| 6 | Camera drift + occluded markers | #8 | Slow, recurring, partially detected |
| 7 | Cube tolerance & tilt accumulation at height | #15, #20, #27 | Will surface as the stack record climbs |
