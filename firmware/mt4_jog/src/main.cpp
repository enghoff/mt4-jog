/*
 * MT4 jog firmware — 4-axis step/dir jog + J1/J2 limit homing.
 * Serial @ 115200 (host: DTR/RTS off).
 *
 * Commands used by jog_keyboard.py:
 *   all f | e0 | e1
 *   d<pin> f|l|h    direction / float
 *   x<pin> | x+<pin> | x-<pin> | xc   step pin(s) for jog
 *   j | stop          start/stop Timer1 jog ISR
 * Jog: Timer1 @ JOG_STEP_PERIOD_US fires parallel STEP pulses on all active
 *      drive pins; Timer3 clears pulse after STEP_PULSE_US (Grbl-style).
 *   home [j1 j2]    widen J2/J3 off their min-angle extremes, home J1 (seek
 *                     I21, return to center), seek J2 to its raw I20
 *                     trigger, drive J3 into interference with J2 until I20
 *                     releases (J3's indirect end-of-travel reference, since
 *                     it has no limit switch of its own), then pull J2 and
 *                     J3 both off by the same amount (default/arg j2)
 *   g o | g c | g stop   gripper sweep open/close (S120–S285 on device)
 *   g <120-285>           set S clamped to limits (manual)
 *   ? | s           status / limits
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

struct StepPinIO {
  volatile uint8_t *port;
  uint8_t mask;
};

enum PinModeSetting : uint8_t { PIN_FLOAT = 0, PIN_LOW = 1, PIN_HIGH = 2 };

static const uint16_t TIMER_TICK_US = 1; // Timer1/3 prescaler 8 @ 16 MHz

static const uint8_t LAB_PINS[] = {
    22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37,
    40,
};
static const uint8_t LAB_PIN_COUNT = sizeof(LAB_PINS) / sizeof(LAB_PINS[0]);
static const uint8_t ENABLE_PIN = 40;

static const uint8_t LIMIT_PINS[] = {20, 21};
static const uint8_t LIMIT_PIN_COUNT = 2;

static const uint16_t STEP_PULSE_US = 10;
static const uint16_t JOG_STEP_PERIOD_US = 1067; // 75% of 800 µs full jog rate
static const uint16_t HOME_STEP_PERIOD_US = 800;
static const uint16_t ENABLE_SETTLE_MS = 5;
static const bool ENABLE_ACTIVE_LOW = true;
static const bool LIMIT_ACTIVE_LOW = true;

static const uint8_t J1_DRIVE = 23;
static const uint8_t J1_DIR = 22;
static const uint8_t J1_LIMIT = 21;
static const uint8_t J2_DRIVE = 25;
static const uint8_t J2_DIR = 24;
static const uint8_t J2_LIMIT = 20;
static const uint8_t J3_DRIVE = 27;
static const uint8_t J3_DIR = 26;
static const bool J1_HOME_DIR_HIGH = true;
static const bool J2_HOME_DIR_HIGH = true;
// J3 has no limit switch of its own. This is the direction that drives it
// into mechanical interference with J2 (which, at J2's raw limit trigger,
// displaces J2 enough to release J2's OWN limit switch) -- used as J3's
// end-of-travel reference. The opposite direction pulls both joints off.
static const bool J3_HOME_DIR_HIGH = true;
static const uint16_t J1_HOME_CENTER_DEFAULT = 4580;
static const uint16_t J2_HOME_PULL_DEFAULT = 1000;
// Pre-home backoff for J2/J3 so J1 can't lock up against a joint sitting at
// its minimum-angle limit while J1 rotates to find its own limit/center.
static const uint16_t J23_PREWIDEN_STEPS = 200;
static const uint32_t HOME_SEEK_MAX = 25000;

static const uint16_t GRIPPER_S_PWM_SCALE = 1000;
static const uint16_t GRIPPER_S_OPEN = 120;
static const uint16_t GRIPPER_S_CLOSED = 285;
static const uint16_t GRIPPER_SWEEP_RATE = 120; // S units per second
static const uint16_t GRIPPER_SWEEP_TICK_MS = 10;
static const uint16_t GRIPPER_PWM_TOP = 0x447A;

enum GripperSweepDir : int8_t {
  GRIP_SWEEP_STOP = 0,
  GRIP_SWEEP_OPEN = -1,
  GRIP_SWEEP_CLOSE = 1,
};

static uint8_t step_pin = 0;
static uint16_t gripper_s =
    static_cast<uint16_t>((GRIPPER_S_OPEN + GRIPPER_S_CLOSED) / 2);
static bool gripper_pwm_on = false;
static int8_t gripper_sweep = GRIP_SWEEP_STOP;
static unsigned long gripper_last_ms = 0;
static uint32_t gripper_sweep_carry = 0;
static uint8_t jog_pins[4];
static uint8_t jog_pin_count = 0;
static StepPinIO jog_pin_io[4];
static volatile bool jog_active = false;
static volatile bool homing_active = false;
static volatile bool step_pulse_high = false;
static bool drivers_enabled = false;
static PinModeSetting pin_mode[LAB_PIN_COUNT];
static int8_t last_limit_raw[2];
static bool last_limit_valid = false;

static char line_buf[48];
static uint8_t line_len = 0;

static void gripperEnablePins();
static void gripperPwmOff();
static void gripperPwmOn();

static uint16_t gripperSToOcr(uint16_t s) {
  if (s == 0) {
    return 0;
  }
  if (s > GRIPPER_S_PWM_SCALE) {
    s = GRIPPER_S_PWM_SCALE;
  }
  return static_cast<uint16_t>((uint32_t)s * GRIPPER_PWM_TOP / GRIPPER_S_PWM_SCALE);
}

static bool gripperSValid(long s) {
  return s == 0 || (s >= GRIPPER_S_OPEN && s <= GRIPPER_S_CLOSED);
}

static uint16_t clampGripperS(uint16_t s) {
  if (s == 0) {
    return 0;
  }
  if (s < GRIPPER_S_OPEN) {
    return GRIPPER_S_OPEN;
  }
  if (s > GRIPPER_S_CLOSED) {
    return GRIPPER_S_CLOSED;
  }
  return s;
}

static void applyGripperPwm(uint16_t s) {
  s = clampGripperS(s);
  gripper_s = s;
  gripperEnablePins();
  gripperPwmOn();
  OCR4B = gripperSToOcr(s);
  gripper_pwm_on = true;
}

static void gripperPwmRelease() {
  gripper_pwm_on = false;
  gripperPwmOff();
}

static bool gripperAtOpen() {
  return gripper_s <= GRIPPER_S_OPEN;
}

static bool gripperAtClosed() {
  return gripper_s >= GRIPPER_S_CLOSED;
}

static void gripperSweepStop() {
  gripper_sweep = GRIP_SWEEP_STOP;
  gripper_sweep_carry = 0;
  gripperPwmRelease();
}

static void gripperSweepStart(int8_t dir) {
  if (dir == GRIP_SWEEP_STOP) {
    gripperSweepStop();
    return;
  }

  gripper_sweep_carry = 0;
  gripper_last_ms = millis();

  if (dir == GRIP_SWEEP_OPEN && gripperAtOpen()) {
    gripper_s = GRIPPER_S_OPEN;
    gripper_sweep = GRIP_SWEEP_STOP;
    gripperPwmRelease();
    return;
  }
  if (dir == GRIP_SWEEP_CLOSE && gripperAtClosed()) {
    gripper_s = GRIPPER_S_CLOSED;
    gripper_sweep = GRIP_SWEEP_STOP;
    gripperPwmRelease();
    return;
  }

  if (!gripper_pwm_on) {
    applyGripperPwm(gripper_s);
  }
  gripper_sweep = dir;
}

static void gripperSweepTick() {
  if (gripper_sweep == GRIP_SWEEP_STOP) {
    return;
  }

  const unsigned long now = millis();
  const unsigned long elapsed = now - gripper_last_ms;
  if (elapsed < GRIPPER_SWEEP_TICK_MS) {
    return;
  }

  const uint32_t advance =
      static_cast<uint32_t>(GRIPPER_SWEEP_RATE) * elapsed + gripper_sweep_carry;
  const uint16_t step = static_cast<uint16_t>(advance / 1000U);
  gripper_sweep_carry = advance % 1000U;
  if (step == 0) {
    return;
  }
  gripper_last_ms = now;

  uint16_t next = gripper_s;
  if (gripper_sweep == GRIP_SWEEP_OPEN) {
    if (next <= GRIPPER_S_OPEN + step) {
      next = GRIPPER_S_OPEN;
      gripper_s = next;
      gripper_sweep = GRIP_SWEEP_STOP;
      gripper_sweep_carry = 0;
      gripperPwmRelease();
      return;
    }
    next = static_cast<uint16_t>(next - step);
  } else if (gripper_sweep == GRIP_SWEEP_CLOSE) {
    if (next + step >= GRIPPER_S_CLOSED) {
      next = GRIPPER_S_CLOSED;
      gripper_s = next;
      gripper_sweep = GRIP_SWEEP_STOP;
      gripper_sweep_carry = 0;
      gripperPwmRelease();
      return;
    }
    next = static_cast<uint16_t>(next + step);
  }

  applyGripperPwm(next);
}

static void printGripperStatus() {
  Serial.print(F("grip S="));
  Serial.print(gripper_s);
  Serial.print(F(" lim="));
  Serial.print(GRIPPER_S_OPEN);
  Serial.print('-');
  Serial.print(GRIPPER_S_CLOSED);
  Serial.print(F(" pwm="));
  Serial.print(gripper_pwm_on ? F("on") : F("off"));
  Serial.print(F(" sweep="));
  if (gripper_sweep == GRIP_SWEEP_OPEN) {
    Serial.println(F("open"));
  } else if (gripper_sweep == GRIP_SWEEP_CLOSE) {
    Serial.println(F("close"));
  } else {
    Serial.println(F("stop"));
  }
}

static void gripperEnablePins() {
  DDRH |= (1 << PH4);
  DDRE |= (1 << PE3);
  PORTE |= (1 << PE5);
}

static void gripperPwmInit() {
  gripperEnablePins();
  TCCR4A = 0x23;
  TCCR4B = (TCCR4B & 0xE0) | 0x1A;
  ICR4 = GRIPPER_PWM_TOP;
  OCR4A = 0xFFFF;
  OCR4B = 0;
}

static void gripperPwmOff() {
  OCR4B = 0;
  TCCR4A &= static_cast<uint8_t>(~0x20);
}

static void gripperPwmOn() {
  TCCR4A |= 0x20;
}

static void setGripperS(uint16_t s) {
  gripper_sweep = GRIP_SWEEP_STOP;
  gripper_sweep_carry = 0;
  if (s == 0) {
    gripperPwmRelease();
    return;
  }
  s = clampGripperS(s);
  gripper_s = s;
  if (gripperAtOpen() || gripperAtClosed()) {
    gripperPwmRelease();
    return;
  }
  applyGripperPwm(s);
}

static int8_t lab_index(uint8_t pin) {
  for (uint8_t i = 0; i < LAB_PIN_COUNT; ++i) {
    if (LAB_PINS[i] == pin) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

static void set_enable(bool on) {
  drivers_enabled = on;
  const bool level = ENABLE_ACTIVE_LOW ? !on : on;
  const int8_t idx = lab_index(ENABLE_PIN);
  if (idx >= 0) {
    pin_mode[idx] = on ? PIN_LOW : PIN_HIGH;
  }
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, level ? HIGH : LOW);
}

static void apply_pin(uint8_t index, PinModeSetting mode) {
  const uint8_t pin = LAB_PINS[index];
  pin_mode[index] = mode;
  if (mode == PIN_FLOAT) {
    pinMode(pin, INPUT);
    return;
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, mode == PIN_HIGH ? HIGH : LOW);
}

static void apply_all(PinModeSetting mode) {
  for (uint8_t i = 0; i < LAB_PIN_COUNT; ++i) {
    apply_pin(i, mode);
  }
  if (mode == PIN_LOW) {
    drivers_enabled = ENABLE_ACTIVE_LOW;
  } else if (mode == PIN_HIGH) {
    drivers_enabled = !ENABLE_ACTIVE_LOW;
  }
}

static bool limit_triggered(uint8_t pin) {
  const int raw = digitalRead(pin);
  return LIMIT_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

static void emit_limit(uint8_t pin, int raw) {
  Serial.print(F("lim I"));
  Serial.print(pin);
  Serial.print('=');
  Serial.print(raw);
  Serial.print(' ');
  Serial.println(limit_triggered(pin) ? F("TRIG") : F("open"));
}

static void poll_limits() {
  for (uint8_t i = 0; i < LIMIT_PIN_COUNT; ++i) {
    const uint8_t pin = LIMIT_PINS[i];
    const int raw = digitalRead(pin);
    if (!last_limit_valid || raw != last_limit_raw[i]) {
      last_limit_raw[i] = static_cast<int8_t>(raw);
      emit_limit(pin, raw);
    }
  }
  last_limit_valid = true;
}

static void print_limits() {
  Serial.print(F("I20="));
  Serial.print(digitalRead(20));
  Serial.print(F(" I21="));
  Serial.println(digitalRead(21));
}

static void clear_jog_pins() {
  jog_pin_count = 0;
  step_pin = 0;
}

static void refresh_jog_pin_io() {
  for (uint8_t i = 0; i < jog_pin_count; ++i) {
    const uint8_t pin = jog_pins[i];
    jog_pin_io[i].port = portOutputRegister(digitalPinToPort(pin));
    jog_pin_io[i].mask = digitalPinToBitMask(pin);
    pinMode(pin, OUTPUT);
    *jog_pin_io[i].port &= ~jog_pin_io[i].mask;
  }
}

static void jog_pulse_low_all() {
  for (uint8_t i = 0; i < jog_pin_count; ++i) {
    *jog_pin_io[i].port &= ~jog_pin_io[i].mask;
  }
  step_pulse_high = false;
}

static void jog_timers_init() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = static_cast<uint16_t>(JOG_STEP_PERIOD_US / TIMER_TICK_US - 1);

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = static_cast<uint16_t>(STEP_PULSE_US / TIMER_TICK_US - 1);
}

static void jog_timer_stop() {
  cli();
  TIMSK1 = 0;
  TCCR1B = 0;
  TIMSK3 = 0;
  TCCR3B = 0;
  jog_pulse_low_all();
  sei();
}

static void jog_timer_start() {
  refresh_jog_pin_io();
  cli();
  TIMSK1 = 0;
  TCCR1B = 0;
  TIMSK3 = 0;
  TCCR3B = 0;
  jog_pulse_low_all();
  TCNT1 = 0;
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  TIMSK1 = (1 << OCIE1A);
  sei();
}

ISR(TIMER1_COMPA_vect) {
  if (!jog_active || homing_active || jog_pin_count == 0 || step_pulse_high) {
    return;
  }
  for (uint8_t i = 0; i < jog_pin_count; ++i) {
    *jog_pin_io[i].port |= jog_pin_io[i].mask;
  }
  step_pulse_high = true;
  TCNT3 = 0;
  TCCR3A = 0;
  TCCR3B = (1 << WGM32) | (1 << CS31);
  TIMSK3 = (1 << OCIE3A);
}

ISR(TIMER3_COMPA_vect) {
  TCCR3B = 0;
  TIMSK3 = 0;
  jog_pulse_low_all();
}

static bool add_jog_pin(uint8_t pin) {
  for (uint8_t i = 0; i < jog_pin_count; ++i) {
    if (jog_pins[i] == pin) {
      return true;
    }
  }
  if (jog_pin_count >= sizeof(jog_pins)) {
    return false;
  }
  jog_pins[jog_pin_count++] = pin;
  step_pin = jog_pins[0];
  return true;
}

static bool remove_jog_pin(uint8_t pin) {
  for (uint8_t i = 0; i < jog_pin_count; ++i) {
    if (jog_pins[i] != pin) {
      continue;
    }
    for (uint8_t j = i + 1; j < jog_pin_count; ++j) {
      jog_pins[j - 1] = jog_pins[j];
    }
    --jog_pin_count;
    step_pin = jog_pin_count > 0 ? jog_pins[0] : 0;
    return true;
  }
  return false;
}

static void print_status() {
  Serial.println(F("--- MT4 jog ---"));
  Serial.print(F("STEP="));
  if (jog_pin_count == 0) {
    Serial.println(F("none"));
  } else if (jog_pin_count == 1) {
    Serial.print(F("D"));
    Serial.println(jog_pins[0]);
  } else {
    for (uint8_t i = 0; i < jog_pin_count; ++i) {
      if (i > 0) {
        Serial.print('+');
      }
      Serial.print(F("D"));
      Serial.print(jog_pins[i]);
    }
    Serial.println();
  }
  Serial.print(F("EN="));
  Serial.print(drivers_enabled ? F("on") : F("off"));
  Serial.print(F("  JOG="));
  Serial.print(jog_active ? F("on") : F("off"));
  if (jog_active && jog_pin_count > 0) {
    Serial.print(F(" T1"));
  }
  Serial.print(F("  LIM "));
  print_limits();
  Serial.print(F("  GRIP S="));
  Serial.print(gripper_s);
  Serial.print(F(" pwm="));
  Serial.print(gripper_pwm_on ? F("on") : F("off"));
  Serial.print(F(" sweep="));
  if (gripper_sweep == GRIP_SWEEP_OPEN) {
    Serial.println(F("open"));
  } else if (gripper_sweep == GRIP_SWEEP_CLOSE) {
    Serial.println(F("close"));
  } else {
    Serial.println(F("stop"));
  }
  Serial.println(F("---------------"));
}

static void stop_jog() {
  jog_timer_stop();
  jog_active = false;
}

static void do_one_step(uint8_t pin);

static void set_dir(uint8_t dir_pin, bool high) {
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, high ? HIGH : LOW);
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

static void do_home(uint16_t j1_center, uint16_t j2_pull) {
  stop_jog();
  homing_active = true;
  Serial.println(F("home start"));

  // 1) Back J2/J3 off their minimum-angle extremes first, so J1 can't lock
  // up mechanically against either of them while it rotates to find its own
  // limit and center. J2's switch sits at its WIDE extreme, so widening
  // matches its seek-toward-limit direction. J3's interference reference is
  // reached by FOLDING toward J2 (the narrow side), so J3 widens the
  // opposite direction from its seek-toward-interference below.
  move_steps(J2_DRIVE, J2_DIR, J2_HOME_DIR_HIGH, J23_PREWIDEN_STEPS);
  move_steps(J3_DRIVE, J3_DIR, !J3_HOME_DIR_HIGH, J23_PREWIDEN_STEPS);

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
  Serial.println(F("home ok"));
}

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

static bool parse_pin(const char *token, uint8_t *out) {
  if (!token || !*token) {
    return false;
  }
  if (token[0] == 'd' || token[0] == 'D') {
    ++token;
  }
  const long pin = atol(token);
  if (pin < 2 || pin > 53) {
    return false;
  }
  *out = static_cast<uint8_t>(pin);
  return true;
}

static void handle_line(char *line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "!") && strcmp(line, "stop") && strcmp(line, "j") &&
      strcmp(line, "jog") && strcmp(line, "home") && strcmp(line, "$H") &&
      strncmp(line, "home ", 5)) {
    stop_jog();
  }

  if (!strcmp(line, "?") || !strcmp(line, "d")) {
    print_status();
    return;
  }
  if (!strcmp(line, "s")) {
    print_limits();
    return;
  }
  if (line[0] == 'g' || line[0] == 'G') {
    const char *arg = line + 1;
    while (*arg == ' ') {
      ++arg;
    }
    if (*arg == '\0') {
      printGripperStatus();
      return;
    }
    if (!strcmp(arg, "stop") || !strcmp(arg, "0")) {
      gripperSweepStop();
      Serial.println(F("ok grip stop"));
      return;
    }
    if (!strcmp(arg, "o") || !strcmp(arg, "open")) {
      gripperSweepStart(GRIP_SWEEP_OPEN);
      if (gripperAtOpen() && gripper_sweep == GRIP_SWEEP_STOP) {
        Serial.println(F("ok grip at open"));
      } else {
        Serial.println(F("ok grip open"));
      }
      return;
    }
    if (!strcmp(arg, "c") || !strcmp(arg, "close")) {
      gripperSweepStart(GRIP_SWEEP_CLOSE);
      if (gripperAtClosed() && gripper_sweep == GRIP_SWEEP_STOP) {
        Serial.println(F("ok grip at closed"));
      } else {
        Serial.println(F("ok grip close"));
      }
      return;
    }
    long v = atol(arg);
    if (!gripperSValid(v)) {
      Serial.print(F("err grip stop|o|c|"));
      Serial.print(GRIPPER_S_OPEN);
      Serial.print('-');
      Serial.println(GRIPPER_S_CLOSED);
      return;
    }
    setGripperS(static_cast<uint16_t>(v));
    Serial.println(F("ok grip"));
    return;
  }
  if (!strcmp(line, "home") || !strcmp(line, "$H")) {
    do_home(J1_HOME_CENTER_DEFAULT, J2_HOME_PULL_DEFAULT);
    return;
  }
  if (!strncmp(line, "home ", 5)) {
    long j1 = atol(line + 5);
    long j2 = J2_HOME_PULL_DEFAULT;
    char *rest = strchr(line + 5, ' ');
    if (rest) {
      while (*rest == ' ') {
        ++rest;
      }
      j2 = atol(rest);
    }
    if (j1 <= 0 || j1 > 20000 || j2 <= 0 || j2 > 20000) {
      Serial.println(F("err home <j1> <j2>"));
      return;
    }
    do_home(static_cast<uint16_t>(j1), static_cast<uint16_t>(j2));
    return;
  }
  if (!strcmp(line, "j") || !strcmp(line, "jog")) {
    if (jog_pin_count == 0) {
      Serial.println(F("err no step pin"));
      return;
    }
    jog_active = true;
    jog_timer_start();
    Serial.println(F("ok jog"));
    return;
  }
  if (!strcmp(line, "!") || !strcmp(line, "stop")) {
    stop_jog();
    Serial.println(F("ok stop"));
    return;
  }
  if (!strcmp(line, "all f")) {
    stop_jog();
    apply_all(PIN_FLOAT);
    drivers_enabled = false;
    clear_jog_pins();
    Serial.println(F("ok all float"));
    return;
  }
  if (!strcmp(line, "e0")) {
    set_enable(false);
    Serial.println(F("ok enable off"));
    return;
  }
  if (!strcmp(line, "e1")) {
    set_enable(true);
    Serial.println(F("ok enable on"));
    return;
  }
  if (line[0] == 'x' || line[0] == 'X') {
    const bool add = line[1] == '+';
    const bool remove = line[1] == '-';
    const char *tok = line + 1;
    if (add || remove) {
      ++tok;
    }
    if (!strcmp(tok, "c") || !strcmp(tok, "C")) {
      clear_jog_pins();
      Serial.println(F("ok step clear"));
      return;
    }
    uint8_t pin = 0;
    if (!parse_pin(tok, &pin)) {
      Serial.println(F("err x<pin>|x+<pin>|x-<pin>|xc"));
      return;
    }
    if (remove) {
      if (!remove_jog_pin(pin)) {
        Serial.println(F("err step missing"));
        return;
      }
    } else if (add) {
      if (!add_jog_pin(pin)) {
        Serial.println(F("err step full"));
        return;
      }
    } else {
      clear_jog_pins();
      if (!add_jog_pin(pin)) {
        Serial.println(F("err step"));
        return;
      }
    }
    Serial.println(F("ok step"));
    return;
  }
  if ((line[0] == 'd' || line[0] == 'D') && line[1] >= '0' && line[1] <= '9') {
    char *sp = strchr(line, ' ');
    if (!sp) {
      Serial.println(F("err d<pin> f|l|h"));
      return;
    }
    *sp = '\0';
    uint8_t pin = 0;
    if (!parse_pin(line, &pin)) {
      Serial.println(F("err d<pin> f|l|h"));
      return;
    }
    const char *m = sp + 1;
    while (*m == ' ') {
      ++m;
    }
    PinModeSetting mode;
    if (!strcmp(m, "f")) {
      mode = PIN_FLOAT;
    } else if (!strcmp(m, "l")) {
      mode = PIN_LOW;
    } else if (!strcmp(m, "h")) {
      mode = PIN_HIGH;
    } else {
      Serial.println(F("err mode f|l|h"));
      return;
    }
    const int8_t idx = lab_index(pin);
    if (idx >= 0) {
      apply_pin(static_cast<uint8_t>(idx), mode);
    } else {
      pinMode(pin, mode == PIN_FLOAT ? INPUT : OUTPUT);
      if (mode != PIN_FLOAT) {
        digitalWrite(pin, mode == PIN_HIGH ? HIGH : LOW);
      }
    }
    if (pin == step_pin && mode == PIN_FLOAT) {
      remove_jog_pin(pin);
    }
    Serial.println(F("ok pin"));
    return;
  }

  Serial.println(F("err unknown"));
}

void setup() {
  for (uint8_t i = 0; i < LAB_PIN_COUNT; ++i) {
    pin_mode[i] = PIN_FLOAT;
    pinMode(LAB_PINS[i], INPUT);
  }
  for (uint8_t i = 0; i < LIMIT_PIN_COUNT; ++i) {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
    last_limit_raw[i] = static_cast<int8_t>(digitalRead(LIMIT_PINS[i]));
  }
  last_limit_valid = false;
  drivers_enabled = false;
  step_pin = 0;
  jog_timers_init();
  gripperPwmInit();

  Serial.begin(115200);
  delay(400);
  Serial.println(F("MT4 jog firmware ready"));
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
}
