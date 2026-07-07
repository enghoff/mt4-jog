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
