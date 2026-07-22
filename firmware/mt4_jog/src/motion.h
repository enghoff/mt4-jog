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

// Soft joint step limits and ground plane. Defaults from config.h; J1
// switch-side min (-center) and J2 min (0 = limit) are refreshed at do_home().
extern int32_t joint_soft_min[MT4_NUM_JOINTS];
extern int32_t joint_soft_max[MT4_NUM_JOINTS];
extern float mt4_ground_z_mm;

void motion_init();
void reset_joint_steps();
/* setpos command: directly overwrite the joint step counters (no motion). */
void motion_set_joint_steps(const long steps[MT4_NUM_JOINTS]);
/* j4zero command: rewrite J4's step counter so the *current* physical wrist
 * pose reports world-frame J4 = 0 (joint_j4 = -j1). No motion. After this,
 * jaws aligned with the arm at j1=0 read as world j4=0; face-align picks can
 * treat offset=0. Survives subsequent `home` (J4 is not re-zeroed there). */
void motion_zero_j4_world();

/* Install defaults, then (after home) set J1 switch-side min from
 * j1_center and J2 min to 0 (limit-referenced). j2/j3_pull are unused for
 * limits (pull only sets post-home park counters). Prints `home limits ...`. */
void motion_apply_home_soft_limits(uint16_t j1_center, uint16_t j2_pull,
                                   uint16_t j3_pull);
/* True when every joint step counter is inside [soft_min, soft_max]. */
bool motion_joints_within_soft_limits(const long steps[MT4_NUM_JOINTS]);
/* True when a single step in `positive` direction on `joint` stays inside. */
bool motion_step_allowed(uint8_t joint, bool positive);

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

/* "cjspeed <us>" / "cjramp <us>": see dda.h for the cj speed-ramp this
 * feeds -- opt-in and fully inert (identical to instant `speed`) until
 * cjramp is set nonzero. */
void motion_set_cj_target_speed_us(long us);
void motion_set_cj_ramp_step_us(long us);

#endif // MT4_MOTION_H
