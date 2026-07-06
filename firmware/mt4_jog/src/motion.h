#ifndef MT4_MOTION_H
#define MT4_MOTION_H

#include <Arduino.h>

#include "config.h"
#include "kinematics.h"

// Cartesian wrist-unwind on/off (`orient on|off` command); plain data, no
// hardware-register concerns like the DDA/speed state in dda.h. When on,
// J4 counters base yaw 1:1 (dq4 = -dq1).
extern bool cart_orient_hold;

void motion_init();
void reset_joint_steps();
/* setpos command: directly overwrite the joint step counters (no motion). */
void motion_set_joint_steps(const long steps[MT4_NUM_JOINTS]);

void stop_jog();
/* "!"/"stop" command: stop_jog() plus canceling any in-progress `m` move
 * bookkeeping (so a stopped move doesn't later report "m done"). */
void motion_cancel_move();
void print_status();
void print_joint_pos();

/* Legacy single/multi-axis jog (x/x+/x-/xc/j commands). */
void clear_jog_axes();
bool add_jog_pin(uint8_t pin);
bool remove_jog_pin(uint8_t pin);
/* "j"/"jog" command: starts the jog ISR if any legacy axis is armed. */
bool motion_start_jog();

/* Cartesian jog (`cj` command): resolved-rate world-frame TCP jog, on-device
 * Jacobian/DLS solve re-run every CJ_REFRESH_MS while active. */
void start_cartesian_jog(Vec3 dir);
void refresh_cartesian_jog_if_due();

/* Bounded relative move (`m` command): all joints move proportionally so
 * they finish together; completion is reported asynchronously from loop(). */
void start_relative_move(const long d[MT4_NUM_JOINTS], long dg);
/* Call every loop() iteration; prints "m done pos ..." and stops the jog
 * exactly once when a relative move completes. */
void motion_poll_move_done();

/* "speed <us>" command: clamps, applies to the timer live, and prints the
 * accepted value. */
void motion_set_speed_us(long us);

#endif // MT4_MOTION_H
