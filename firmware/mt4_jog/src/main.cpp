/*
 * MT4 jog firmware — 4-axis step/dir jog + J1/J2 limit homing + Cartesian jog.
 * Serial @ 115200 (host: DTR/RTS off).
 *
 * Joint jog (legacy):
 *   all f | e0 | e1
 *   d<pin> f|l|h    direction / float
 *   x<pin> | x+<pin> | x-<pin> | xc   step pin(s) for jog
 *   j | stop          start/stop Timer1 jog ISR (equal step rate on active axes)
 *   speed <us>        set the shared jog step period (joint jog + cj), live-
 *                        adjustable mid-jog; clamped to [700,4000] us
 *
 * Cartesian jog:
 *   cj +x|-x|+y|-y|+z|-z   world-frame TCP jog (multi-axis DDA on device)
 *   cj <dx> <dy> <dz>      direction vector (integer components, normalized)
 *   orient on|off          J4 wrist unwind when J1 moves (default on, 1:1)
 *   pos                      print joint step counters (since last home),
 *                              plus a derived "tcp x=.. y=.. z=.. j4=..
 *                              grip=.." line (mm/deg/S -- same frame and
 *                              units the `mp` command below accepts)
 *   setpos <j1> <j2> <j3> <j4>
 *       Directly overwrite the joint step counters (no motion) -- for
 *       correcting drift after an external reference (e.g. a soft-contact
 *       seek on an unreferenced joint like J3, which has no limit switch).
 *
 * Relative move (bounded, coordinated):
 *   m <dj1> <dj2> <dj3> <dj4> [dg]
 *       Move each joint by the signed number of steps relative to the current
 *       position (multi-axis DDA, proportional rates, all joints finish
 *       together) and sweep the gripper by dg S-units relative to its current
 *       S (clamped to S120-285). Replies "ok m" on accept, then an async
 *       "m done pos ..." line when the joint motion completes. Drivers are
 *       left ENABLED (holding) after the move.
 *
 * Absolute move (bounded, coordinated):
 *   mp <x> <y> <z> <j4> <g>
 *       Move to an absolute TCP position in mm (origin at the base, under
 *       J1's pivot) with an absolute J4 orientation in degrees and an
 *       absolute gripper S (0 = leave the gripper alone). Solved via
 *       closed-form IK, nearest-branch to the current pose. Rejected with
 *       "err not homed" unless `home`/`$H` has completed successfully this
 *       session -- absolute coordinates are meaningless against an
 *       unreferenced step counter. Same "ok mp" / async "mp done pos ..."
 *       reply convention as `m`.
 *
 *   home [j1 j2]    widen J2/J3 off their min-angle extremes, home J1 (seek
 *                     I21, return to center), seek J2 to its raw I20
 *                     trigger, drive J3 into interference with J2 until I20
 *                     releases (J3's indirect end-of-travel reference, since
 *                     it has no limit switch of its own), then pull J2 and
 *                     J3 both off by the same amount (default/arg j2)
 *   g o | g c | g stop   gripper sweep open/close (S120–S285 on device)
 *   g <120-285>           set S clamped to limits (manual)
 *   ? | s           status / limits
 *
 * Source layout: config.h (pin map/timing constants), pins.{h,cpp} (raw I/O,
 * limit switches), gripper.{h,cpp} (PWM gripper), motion.{h,cpp} (DDA/ISR
 * jog engine, cartesian + relative-move logic), homing.{h,cpp} (do_home),
 * commands.{h,cpp} (serial line parser), kinematics.{h,cpp} (FK/IK, on-
 * device Cartesian rate solve).
 */

#include <Arduino.h>

#include "commands.h"
#include "gripper.h"
#include "homing.h"
#include "motion.h"
#include "pins.h"

void setup() {
  pins_init();
  motion_init();
  gripperPwmInit();

  Serial.begin(115200);
  delay(400);
  reset_joint_steps();
  Serial.println(F("MT4 jog firmware ready (joint + cartesian)"));
  print_status();
}

void loop() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line_buf[line_len] = '\0';
      handle_line(line_buf);
      line_len = 0;
      continue;
    }
    if (line_len < sizeof(line_buf) - 1) {
      line_buf[line_len++] = c;
    }
  }
  poll_limits();
  gripperSweepTick();
  refresh_cartesian_jog_if_due();
  motion_poll_move_done();
}
