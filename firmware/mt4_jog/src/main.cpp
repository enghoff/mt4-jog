/*
 * MT4 jog firmware — 4-axis step/dir jog + J1/J2 limit homing + Cartesian jog.
 * Serial @ 115200 (host: DTR/RTS off).
 *
 * Joint jog (legacy):
 *   all f | e0 | e1
 *   d<pin> f|l|h    direction / float
 *   x<pin> | x+<pin> | x-<pin> | xc   step pin(s) for jog
 *   j | stop          start/stop Timer1 jog ISR (equal step rate on active axes)
 *   speed <us>        set the shared jog step period (joint jog + cj + mp),
 *                        live-adjustable mid-jog; clamped to [700,4000] us
 *                        (session state, reset on reboot)
 *
 * Cartesian jog:
 *   cj +x|-x|+y|-y|+z|-z   world-frame TCP jog (multi-axis DDA on device)
 *   cj <dx> <dy> <dz> [j4] direction vector (integer components, normalized)
 *                            + optional J4 roll -1|0|1 layered on top of the
 *                            solved rates (incl. orient hold), so the wrist
 *                            rotates while the TCP moves; all-zero dir with
 *                            nonzero j4 = pure wrist roll. At the keep-out
 *                            cylinder (see below) the inward velocity
 *                            component is clamped so the jog slides along
 *                            the boundary instead of hitting the base.
 *
 * Keep-out: the TCP cannot physically approach the base column closer than
 * ~MT4_KEEPOUT_RADIUS_MM (170mm) from the J1 axis at any Z. `mp` rejects
 * targets inside the cylinder ("err mp keepout") and automatically routes
 * paths that would cross it around the boundary (entry tangent, shortest
 * arc, exit tangent); a start inside the cylinder first escapes radially.
 * Soft joint limits (envelope + home limit-switch pull-offs) and ground
 * plane MT4_GROUND_Z_MM also reject/clamp out-of-range jog and `mp`.
 * Joint-space moves (`m`, homing) are NOT covered -- they command raw steps.
 *   orient on|off          J4 wrist unwind when J1 moves (default on, 1:1)
 *   pos                      print joint step counters (since last home),
 *                              plus a derived "tcp x=.. y=.. z=.. j4=..
 *                              grip=.. speed=.." line (mm/deg/S/us -- x/y/z,
 *                              j4, and speed use the same units `mp` accepts)
 *   setpos <j1> <j2> <j3> <j4>
 *       Directly overwrite the joint step counters (no motion) -- for
 *       correcting drift after an external reference (e.g. a soft-contact
 *       seek on an unreferenced joint like J3, which has no limit switch).
 *   j4zero
 *       Rewrite J4's step counter so the current physical wrist pose reports
 *       world-frame J4 = 0 (no motion). Used after the operator aligns the
 *       jaws with the arm axis; subsequent face-align picks assume offset 0.
 *       Survives `home` (J4 counter is preserved across homing). Lost on
 *       power cycle / reflash until re-run.
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
 * Absolute move (bounded, coordinated, Cartesian-linear):
 *   mp <x> <y> <z> <j4> <g> [speed_us]
 *       Move to an absolute TCP position in mm (origin at the base, under
 *       J1's pivot) with a world-frame J4 gripper yaw in degrees and an
 *       absolute gripper S (0 = leave the gripper alone). Optional speed_us
 *       sets the shared step period for the move (700-4000 us, same units as
 *       `speed`; 0 or omitted = leave the current period unchanged). TCP xyz is
 *       interpolated along straight world-frame lines in short segments;
 *       each segment solves closed-form IK (nearest branch to the current
 *       pose) and runs a coordinated joint DDA move. When the commanded J4
 *       matches the pose at move start, gripper yaw is held fixed in
 *       world space (J4 counters J1 1:1, like `orient on`); otherwise J4
 *       is interpolated linearly to the target. Rejected with
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
 *                     J3 both off by the same amount (default/arg j2), then
 *                     rotate J4 to its calibrated zero (step counter → 0;
 *                     after `j4zero` that is jaws-along-arm / world J4=0 at
 *                     the homed pose). J4's counter is preserved across the
 *                     J1–J3 zeroing so this move is meaningful.
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
