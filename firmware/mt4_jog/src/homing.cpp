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

static bool seek_limit(uint8_t drive, uint8_t dir, bool dir_high, uint8_t lim) {
  prepare_axis(drive, dir, dir_high);
  if (limit_triggered(lim)) {
    return true;
  }
  for (uint32_t i = 0; i < HOME_SEEK_MAX; ++i) {
    if (serial_abort()) {
      return false;
    }
    do_one_step(drive);
    poll_limits();
    if (limit_triggered(lim)) {
      return true;
    }
  }
  return false;
}

// Like seek_limit, but drives `drive`/`dir` while watching a DIFFERENT limit
// pin (`watch_lim`) for it to go from triggered to released -- used for J3,
// which has no limit switch of its own, via its interference with J2.
static bool seek_release(uint8_t drive, uint8_t dir, bool dir_high, uint8_t watch_lim) {
  prepare_axis(drive, dir, dir_high);
  if (!limit_triggered(watch_lim)) {
    return true;
  }
  for (uint32_t i = 0; i < HOME_SEEK_MAX; ++i) {
    if (serial_abort()) {
      return false;
    }
    do_one_step(drive);
    poll_limits();
    if (!limit_triggered(watch_lim)) {
      return true;
    }
  }
  return false;
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

void do_home(uint16_t j1_center, uint16_t j2_pull) {
  stop_jog();
  homing_active = true;
  Serial.println(F("home start"));

  // 1) Back J2/J3 off their minimum-angle extremes first, so J1 can't lock
  // up mechanically against either of them while it rotates to find its own
  // limit and center. J3's interference reference is reached by FOLDING
  // toward J2 (the narrow side), so J3 widens the opposite direction from
  // its seek-toward-interference below.
  move_steps(J3_DRIVE, J3_DIR, !J3_HOME_DIR_HIGH, J23_PREWIDEN_STEPS);
  move_steps(J2_DRIVE, J2_DIR, !J2_HOME_DIR_HIGH, J23_PREWIDEN_STEPS);

  // 2) Home J1: seek its limit switch, then return to center.
  if (!seek_limit(J1_DRIVE, J1_DIR, J1_HOME_DIR_HIGH, J1_LIMIT)) {
    Serial.println(F("home fail J1"));
    homing_active = false;
    return;
  }
  Serial.println(F("home J1 limit"));
  move_steps(J1_DRIVE, J1_DIR, !J1_HOME_DIR_HIGH, j1_center);

  // 3) Home J2: seek its limit switch and stop right at the raw trigger
  // (no pulloff yet -- J3's seek below needs J2 held there).
  if (!seek_limit(J2_DRIVE, J2_DIR, J2_HOME_DIR_HIGH, J2_LIMIT)) {
    Serial.println(F("home fail J2"));
    homing_active = false;
    return;
  }
  Serial.println(F("home J2 limit"));

  // 4) Home J3 indirectly: drive it into interference with J2 until that
  // displaces J2 enough to release J2's OWN limit switch. J3 has no switch
  // of its own, so this release point is its end-of-travel reference.
  if (!seek_release(J3_DRIVE, J3_DIR, J3_HOME_DIR_HIGH, J2_LIMIT)) {
    Serial.println(F("home fail J3"));
    homing_active = false;
    return;
  }
  Serial.println(F("home J3 release"));

  // 5) Pull both J2 and J3 off their limit/interference extremes.
  move_steps(J2_DRIVE, J2_DIR, !J2_HOME_DIR_HIGH, j2_pull);
  move_steps(J3_DRIVE, J3_DIR, !J3_HOME_DIR_HIGH, j2_pull);

  stop_jog();
  step_pin = 0;
  homing_active = false;
  reset_joint_steps();
  Serial.println(F("home ok"));
}
