#ifndef MT4_DDA_H
#define MT4_DDA_H

#include <Arduino.h>

#include "config.h"

// Low-level Bresenham/DDA multi-axis stepping engine (Timer1/3 ISRs).
// motion.cpp is the only intended caller; it owns the higher-level
// cartesian/relative-move/status logic built on top of this.

// Set by homing.cpp's do_home(); read by the Timer1 ISR to suppress
// jog/DDA stepping while a blocking home sequence owns the drive pins.
extern volatile bool homing_active;
extern volatile bool jog_active;
// Bounded `m` move bookkeeping: true while a move is in progress; the ISR
// sets move_done_pending once every armed axis has used up its budget.
extern volatile bool move_mode;
extern volatile bool move_done_pending;
extern volatile uint8_t dda_axis_mask;
extern volatile int32_t joint_steps[MT4_NUM_JOINTS];

void dda_init();
void dda_reset();
void dda_set_joint_steps(const long steps[MT4_NUM_JOINTS]);
void dda_clear_axes();
bool dda_add_axis_by_pin(uint8_t pin);
bool dda_remove_axis_by_pin(uint8_t pin);
void dda_set_speed_us(long us);
/* Same clamp/apply as dda_set_speed_us() but without the serial ack line. */
void dda_set_speed_us_quiet(long us);
uint16_t dda_get_speed_us();

/* Cartesian-jog speed smoothing -- separate from dda_set_speed_us()'s
 * instant apply, and fully inert until a host opts in with a nonzero ramp
 * step (`cjramp <us>`), which is also the rollback lever: `cjramp 0` (the
 * power-on default) makes dda_set_cj_target_speed_us() apply immediately,
 * byte-for-byte the old instant-write behavior, no reflash needed to
 * disable this. `speed`/mp/legacy-jog paths are untouched either way. */
void dda_set_cj_target_speed_us(long us);
void dda_set_cj_ramp_step_us(long us);
uint16_t dda_get_cj_ramp_step_us();
/* Step the applied cj speed toward its target by at most the ramp step;
 * call once per CJ_REFRESH_MS tick while a cartesian jog is active. No-op
 * if ramping is disabled (step == 0) or already at target. */
void dda_tick_cj_speed_ramp();
/* Configure an acceleration ramp for an upcoming multi-segment coordinated
 * move (the `mp` command): starts at start_us, ramps toward cruise_us over
 * up to ramp_ticks master ticks, holds cruise, then ramps symmetrically back
 * toward start_us over the last ramp_ticks of total_ticks (the summed
 * per-segment master ticks along the planned `mp` path).
 *
 * Persists across dda_arm()/dda_engage() calls (segment boundaries) --
 * `mp` calls dda_stop() between every segment, but neither dda_stop() nor
 * dda_arm() touch ramp state, so the ramp continues seamlessly across
 * segments. Only a fresh call to this function or dda_ramp_clear() changes
 * it. Falls back to plain constant-speed stepping (no ramp) if cruise_us is
 * already as slow or slower than start_us, or total_ticks is too short for
 * a meaningful ramp. */
void dda_set_ramp(uint16_t start_us, uint16_t cruise_us, int32_t total_ticks,
                   uint16_t ramp_ticks);
/* Disable ramping -- OCR1A stays fixed at whatever dda_set_speed_us() last
 * set. Call before starting any non-`mp` coordinated move (`m`, cj jog,
 * legacy jog) so a still-active `mp` ramp can't bleed into unrelated
 * motion. */
void dda_ramp_clear();
void dda_stop();
void dda_engage();
/* Arms a coordinated move: master ticks + per-joint signed step deltas
 * (0 = axis untouched); sets each axis's DIR pin from the delta's sign.
 * track_move=true marks it as a bounded `m` move (move_done_pending fires
 * once every armed axis finishes); false is an indefinite jog (cartesian/
 * legacy). Returns true if at least one axis was armed. */
bool dda_arm(int32_t master, const int32_t deltas[MT4_NUM_JOINTS], bool track_move);
void dda_refresh_pins();

#endif // MT4_DDA_H
