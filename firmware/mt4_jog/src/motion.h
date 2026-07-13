#ifndef MT4_MOTION_H
#define MT4_MOTION_H

#include <Arduino.h>

#include "config.h"
#include "kinematics.h"

// Cartesian wrist-unwind on/off (`orient on|off` command); plain data, no
// hardware-register concerns like the DDA/speed state in dda.h. When on,
// J4 counters base yaw 1:1 (dq4 = -dq1).
extern bool cart_orient_hold;

// True once do_home() has completed successfully this session; never reset
// except by a power cycle/reflash. Gates the `mp` absolute-position command.
extern bool mt4_homed;

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
 * Jacobian/DLS solve re-run every CJ_REFRESH_MS while active. j4_roll
 * (-1/0/+1) adds a wrist roll on top of the solved rates (including on top
 * of the orient-hold counter-rotation), so J4 can rotate while the TCP
 * translates; a zero dir with nonzero roll is a pure J4 roll. */
void start_cartesian_jog(Vec3 dir, int8_t j4_roll);
void refresh_cartesian_jog_if_due();

/* Bounded relative move (`m` command): all joints move proportionally so
 * they finish together; completion is reported asynchronously from loop(). */
void start_relative_move(const long d[MT4_NUM_JOINTS], long dg);
/* Call every loop() iteration; prints "m done pos ..." and stops the jog
 * exactly once when a relative move completes. */
void motion_poll_move_done();

/* "mp" command: move to an absolute TCP position (x, y, z mm, origin at the
 * base under J1's pivot) + world-frame J4 gripper yaw (deg) + absolute gripper
 * S (0 = leave the gripper alone) + optional speed_us (700-4000, same units
 * as `speed`; 0 = leave the current step period unchanged). Rejected with
 * "err not homed" unless mt4_homed. TCP xyz is interpolated along straight
 * world-frame lines in short segments. When the commanded J4 matches the
 * current world-frame yaw, gripper orientation is held fixed in world space
 * (J4 counters J1 1:1); otherwise world-frame J4 is interpolated linearly.
 * Async "mp done pos ..." matches `m`. */
bool start_absolute_move(float x, float y, float z, float j4_deg, long g,
                         long speed_us);

/* "speed <us>" command: clamps, applies to the timer live, and prints the
 * accepted value. */
void motion_set_speed_us(long us);

#endif // MT4_MOTION_H
