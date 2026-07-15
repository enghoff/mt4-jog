# Sort-into-rows: occupancy and anti-stacking requirements

Triggered by **S** in the live shuffle loop. This document states the
behavior we want. It does not prescribe an implementation, and it does not
claim a root cause for the current stacking failures.

## Goal

Sort every reachable pick-quality cube into **three fixed parallel rows by
color** (blue / green / red) near the center of the reachable workspace.

When sort finishes successfully:

- Each reachable cube sits in its color’s row.
- Each row slot holds **at most one** cube.
- Cubes are spaced so they do not sit on top of each other or share a pad.

## Definitions

| Term | Meaning |
|------|---------|
| **Slot** | One planned place pose `(x, y)` assigned to a color for this sort run. |
| **Occupied** | A slot that already has its cube — either confirmed by a successful place into that slot this run, or by seeing a same-color cube on that pad that is treated as filling it. |
| **Free** | A slot that is not occupied and has no cube (any color) sitting on that pad in the way of a safe place. |
| **Stack** | Commanding a place onto a pad that already holds a cube (same or other color), so cubes end up on top of each other or forced into the same footprint. |

“Free” means **nothing is on that pad**. It does **not** mean “our matcher
failed to associate a cube with this slot.” If a cube is physically on the
pad, the slot is occupied, even if vision is noisy.

## Occupancy rules (anti-stacking)

1. **Never place onto an occupied slot.**  
   Once a slot is occupied, it must not be chosen again as a place target
   for the rest of that sort run (unless the cube is later deliberately
   moved away as part of declutter / reassignment — see below).

2. **One cube → one slot.**  
   After a place into slot S is accepted as successful, that cube is bound
   to S. No second cube may be sent to S while that binding stands.

3. **Success is about the destination pad, not “any same-color blob nearby.”**  
   A move that was meant to fill S must not be treated as filling S solely
   because an *already present* cube on S was visible. Conversely, if a
   cube was just released onto S and that pad now holds a cube, S is
   occupied and must not be targeted again.

4. **Physical pad occupancy overrides planner association.**  
   If a cube of any color is sitting on the pad for slot S, do not place
   another cube there. Move the blocking cube first (preferably straight
   to *its* free final slot), or park it temporarily, then fill S.

5. **Adjacent free slots are allowed.**  
   If slot B is free (empty pad) and slot A already has its cube, sending a
   cube to B is correct — even if A’s cube is nearby on the desk. Spacing
   between slots must be enough that filling B does not require stacking on
   A; neighbor proximity alone does not make B “occupied.”

6. **Do not “refill” a filled slot because the same coordinates appear again
   in a rebuilt plan.**  
   Slot identity for occupancy must survive re-planning within one sort
   (rebuild of layout/counts must not forget that a pad was already
   filled).

## Sort progress rules

1. **Plan** one slot per reachable cube of blue / green / red (empty colors
   get no slots; row geometry for the three colors stays fixed).

2. **Each step** is one pick+place: either  
   - move a cube into a **free** final slot of its color, or  
   - move a cube that is blocking a needed pad to its own free final slot
     if one exists, else to a temporary park that is not an occupied final
     slot.

3. Prefer **direct-to-final** for blockers over intermediate park when the
   blocker already has a free final slot.

4. **Complete** when every planned slot is occupied and every reachable
   cube is bound to a slot (or no reachable cubes remain to place).

5. **Stuck** (not “complete”) when remaining free slots cannot be filled
   safely (no parking, no free final for a blocker, etc.). Do not invent
   places onto occupied pads to force progress.

## Acceptance checks (live)

During / after a sort run:

- [ ] No place target repeats a slot that already accepted a successful
      place in this run while that cube remains on that pad.
- [ ] Camera / desk inspection shows at most one cube per final slot pad.
- [ ] Cubes that started on a row pad but belong to another color are
      cleared before that pad is filled — not covered.
- [ ] Ending layout is three color rows (as present); every reachable cube
      of those colors is in its row when the run reports complete.
- [ ] Reporting `sort complete` never happens while any planned slot pad
      is empty and a matching unsettled cube still exists and is placeable.

## Non-goals

- Perfect millimeter accuracy of place vs command (small residuals are OK
  as long as they do not cause a second place onto the same pad).
- Sorting colors other than blue / green / red.
- Changing shuffle (non-S) behavior.
