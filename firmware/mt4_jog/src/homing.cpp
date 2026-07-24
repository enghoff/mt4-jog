#include "homing.h"

#include <string.h>

#include "config.h"
#include "dda.h"
#include "motion.h"
#include "pins.h"

char line_buf[64];
uint8_t line_len = 0;

static void do_one_step(uint8_t pin) {
  if (pin == 0) {
    return;
  }
  if (!drivers_enabled) {
    set_enable(true);
    delay(ENABLE_SETTLE_MS);
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(pin, LOW);
  delayMicroseconds(HOME_STEP_PERIOD_US);
}

static void prepare_axis(uint8_t drive, uint8_t dir, bool dir_high) {
  apply_all(PIN_FLOAT);
  set_enable(true);
  delay(ENABLE_SETTLE_MS);
  set_dir(dir, dir_high);
  step_pin = drive;
}

static bool serial_abort() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line_buf[line_len] = '\0';
      const bool abort = !strcmp(line_buf, "!") || !strcmp(line_buf, "stop");
      line_len = 0;
      if (abort) {
        return true;
      }
      continue;
    }
    if (line_len < sizeof(line_buf) - 1) {
      line_buf[line_len++] = c;
    }
  }
  return false;
}

// Drives `drive`/`dir` until pin `lim` reads `target_triggered`, or returns
// false on abort/HOME_SEEK_MAX timeout. seek_limit/seek_release below are
// thin wrappers naming the two directions this is used in.
static bool seek_until(uint8_t drive, uint8_t dir, bool dir_high, uint8_t lim,
                        bool target_triggered) {
  prepare_axis(drive, dir, dir_high);
  if (limit_triggered(lim) == target_triggered) {
    return true;
  }
  for (uint32_t i = 0; i < HOME_SEEK_MAX; ++i) {
    if (serial_abort()) {
      return false;
    }
    do_one_step(drive);
    poll_limits();
    if (limit_triggered(lim) == target_triggered) {
      return true;
    }
  }
  return false;
}

static bool seek_limit(uint8_t drive, uint8_t dir, bool dir_high, uint8_t lim) {
  return seek_until(drive, dir, dir_high, lim, true);
}

// Like seek_limit, but drives `drive`/`dir` while watching a DIFFERENT limit
// pin (`watch_lim`) for it to go from triggered to released -- used for J3,
// which has no limit switch of its own, via its interference with J2.
static bool seek_release(uint8_t drive, uint8_t dir, bool dir_high, uint8_t watch_lim) {
  return seek_until(drive, dir, dir_high, watch_lim, false);
}

static void move_steps(uint8_t drive, uint8_t dir, bool dir_high, uint32_t n) {
  prepare_axis(drive, dir, dir_high);
  for (uint32_t i = 0; i < n; ++i) {
    if (serial_abort()) {
      return;
    }
    do_one_step(drive);
    poll_limits();
  }
}

// Moves a joint away from its home-seek direction (widen off an extreme, or
// pull off after reaching one) -- i.e. move_steps with the direction
// flipped from home_dir_high, so call sites read as "back off" rather than
// having to negate the home direction inline.
static void back_off(uint8_t drive, uint8_t dir, bool home_dir_high, uint32_t n) {
  move_steps(drive, dir, !home_dir_high, n);
}

void do_home(uint16_t j1_center, uint16_t j2_pull) {
  stop_jog();
  // Homing rewrites the joint step counters from scratch; a stale
  // in-flight/queued mp/mq path planned against the pre-home counters must
  // not be allowed to resume afterward.
  motion_cancel_move();
  homing_active = true;
  Serial.println(F("home start"));

  // 1) Back J3 off its minimum-angle extreme, so it isn't already sitting
  // somewhere that interferes with J2's approach to its own limit switch
  // below. J3's interference reference is reached by FOLDING toward J2
  // (the narrow side), so this widens the opposite direction from its
  // seek-toward-interference in step 3.
  back_off(J3_DRIVE, J3_DIR, J3_HOME_DIR_HIGH, J3_PREWIDEN_STEPS);

  // 1b) If J3 pre-widen left J2 sitting on its limit switch, widen J2
  // until the switch releases, then widen a little more before seeking.
  poll_limits();
  if (limit_triggered(J2_LIMIT)) {
    if (!seek_until(J2_DRIVE, J2_DIR, !J2_HOME_DIR_HIGH, J2_LIMIT, false)) {
      Serial.println(F("home fail J2 prewiden"));
      homing_active = false;
      return;
    }
    back_off(J2_DRIVE, J2_DIR, J2_HOME_DIR_HIGH, J2_PREWIDEN_STEPS);
  }

  // 2) Home J2: seek its limit switch and stop right at the raw trigger
  // (no pulloff yet -- J3's seek below needs J2 held there).
  if (!seek_limit(J2_DRIVE, J2_DIR, J2_HOME_DIR_HIGH, J2_LIMIT)) {
    Serial.println(F("home fail J2"));
    homing_active = false;
    return;
  }

  // 3) Home J3 indirectly: drive it into interference with J2 until that
  // displaces J2 enough to release J2's OWN limit switch. J3 has no switch
  // of its own, so this release point is its end-of-travel reference.
  if (!seek_release(J3_DRIVE, J3_DIR, J3_HOME_DIR_HIGH, J2_LIMIT)) {
    Serial.println(F("home fail J3"));
    homing_active = false;
    return;
  }

  // 4) Home J1: seek its limit switch, pause briefly to let it settle at
  // the switch, then return to center. Done while J2/J3 are still held at
  // their raw limit/interference extremes from steps 2-3.
  if (!seek_limit(J1_DRIVE, J1_DIR, J1_HOME_DIR_HIGH, J1_LIMIT)) {
    Serial.println(F("home fail J1"));
    homing_active = false;
    return;
  }
  delay(J1_HOME_LIMIT_PAUSE_MS);
  back_off(J1_DRIVE, J1_DIR, J1_HOME_DIR_HIGH, j1_center);

  // 5) Pull J2 and J3 off their limit/interference extremes now that J1
  // is done (J3 uses a shorter pull-off than J2).
  back_off(J2_DRIVE, J2_DIR, J2_HOME_DIR_HIGH, j2_pull);
  back_off(J3_DRIVE, J3_DIR, J3_HOME_DIR_HIGH, J3_HOME_PULL_DEFAULT);

  stop_jog();
  step_pin = 0;
  // J2/J3 angle constants are at the limit/interference reference
  // (steps=0). After pull-off the counters sit at +pull so FK matches the
  // tape-fit park pose. Preserve J4 across the J1–J3 rewrite: J4 has no
  // limit switch, and `j4zero` would otherwise be wiped. Homing never
  // drives J4 above, so the counter stays physically meaningful.
  const long j4_keep = joint_steps[3];
  reset_joint_steps();
  {
    long steps[MT4_NUM_JOINTS] = {0, static_cast<long>(j2_pull),
                                  static_cast<long>(J3_HOME_PULL_DEFAULT),
                                  j4_keep};
    dda_set_joint_steps(steps);
  }
  motion_apply_home_soft_limits(j1_center, j2_pull, J3_HOME_PULL_DEFAULT);

  // 6) Rotate J4 to the calibrated zero (joint steps → 0). After `j4zero`,
  // that is jaws-along-arm; at the homed pose (j1=0) it is also world J4=0.
  if (j4_keep != 0) {
    // J_DIR_POS_HIGH[3]==false: positive steps ⇒ DIR low; to decrease the
    // counter when j4_keep>0 we need DIR high, and the reverse when <0.
    const bool dir_high =
        (j4_keep > 0) ? !J_DIR_POS_HIGH[3] : J_DIR_POS_HIGH[3];
    const int32_t step_sign = (j4_keep > 0) ? -1 : 1;
    const uint32_t n = (j4_keep > 0) ? static_cast<uint32_t>(j4_keep)
                                     : static_cast<uint32_t>(-j4_keep);
    prepare_axis(J4_DRIVE, J4_DIR, dir_high);
    for (uint32_t i = 0; i < n; ++i) {
      if (serial_abort()) {
        Serial.println(F("home fail J4"));
        homing_active = false;
        return;
      }
      do_one_step(J4_DRIVE);
      joint_steps[3] += step_sign;
    }
  }

  step_pin = 0;
  homing_active = false;
  mt4_homed = true;
  Serial.println(F("home ok"));
}
