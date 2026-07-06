/*
 * MT4 jog firmware — 4-axis step/dir jog + J1/J2 limit homing + Cartesian jog.
 * Serial @ 115200 (host: DTR/RTS off).
 *
 * Joint jog (legacy):
 *   all f | e0 | e1
 *   d<pin> f|l|h    direction / float
 *   x<pin> | x+<pin> | x-<pin> | xc   step pin(s) for jog
 *   j | stop          start/stop Timer1 jog ISR (equal step rate on active axes)
 *
 * Cartesian jog:
 *   cj +x|-x|+y|-y|+z|-z   world-frame TCP jog (multi-axis DDA on device)
 *   cj <dx> <dy> <dz>      direction vector (integer components, normalized)
 *   orient on|off|<gain>   J4 wrist unwind when J1 moves (default on, gain 0.82)
 *   pos                      print joint step counters (since last home)
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
#include <math.h>

#include "kinematics.h"

struct StepPinIO {
  volatile uint8_t *port;
  uint8_t mask;
};

enum PinModeSetting : uint8_t { PIN_FLOAT = 0, PIN_LOW = 1, PIN_HIGH = 2 };

// Timer1/3 run at 16 MHz / prescaler 8 = 2 MHz, i.e. 2 ticks per us. The
// previous TIMER_TICK_US = 1 assumption made every jog step at DOUBLE the
// intended rate, costing stepper torque exactly where the arm is weakest.
static const uint16_t TIMER_TICKS_PER_US = 2;

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
static const uint16_t JOG_STEP_PERIOD_US = 1524; // 70% of prior 1067 µs tick rate
static const uint16_t CJ_REFRESH_MS = 40;
/* J4 is 852 steps/deg, so generous; ~2 full sweeps of any joint. */
static const long MOVE_MAX_STEPS = 100000L;
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
static const uint8_t J4_DRIVE = 35;
static const uint8_t J4_DIR = 36;
static const uint8_t J_DRIVE[MT4_NUM_JOINTS] = {J1_DRIVE, J2_DRIVE, J3_DRIVE,
                                                J4_DRIVE};
static const uint8_t J_DIR_PIN[MT4_NUM_JOINTS] = {J1_DIR, J2_DIR, J3_DIR,
                                                  J4_DIR};
static const bool J_DIR_POS_HIGH[MT4_NUM_JOINTS] = {false, false, false,
                                                    false};
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
/* Target-bounded sweep (`m` command's dg): sweep stops at gripper_target
 * instead of the full open/closed endpoint, PWM left on to hold position. */
static uint16_t gripper_target = 0;
static bool gripper_target_active = false;

static volatile int32_t joint_steps[MT4_NUM_JOINTS];
static bool cart_orient_hold = true;
static float cart_orient_gain = MT4_ORIENT_GAIN_DEFAULT;
static bool cart_jog_mode = false;
static Vec3 cart_dir_active = {0.0f, 0.0f, 0.0f};
static bool cart_dir_active_valid = false;
static unsigned long cart_refresh_ms = 0;

static volatile uint8_t dda_axis_mask = 0;
static volatile int32_t dda_master = MT4_CJ_MASTER;
static volatile int32_t dda_delta[MT4_NUM_JOINTS];
static volatile int32_t dda_accum[MT4_NUM_JOINTS];
static StepPinIO dda_pin_io[MT4_NUM_JOINTS];

/* Bounded relative move (`m` command): per-joint steps left; ISR clears an
 * axis from the DDA mask when its count hits zero and raises done_pending
 * for the main loop to finalize + report. */
static volatile bool move_mode = false;
static volatile bool move_done_pending = false;
static volatile int32_t move_remaining[MT4_NUM_JOINTS];

static volatile bool jog_active = false;
static volatile bool homing_active = false;
static volatile bool step_pulse_high = false;
static bool drivers_enabled = false;
static PinModeSetting pin_mode[LAB_PIN_COUNT];
static int8_t last_limit_raw[2];
static bool last_limit_valid = false;

static char line_buf[64];
static uint8_t line_len = 0;

static void gripperEnablePins();
static void gripperPwmOff();
static void gripperPwmOn();
static void set_dir(uint8_t dir_pin, bool high);

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
  gripper_target_active = false;
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

/* Sweep to an absolute S target (clamped) and hold there with PWM on. */
static void gripperSweepToS(long target) {
  if (target < GRIPPER_S_OPEN) {
    target = GRIPPER_S_OPEN;
  } else if (target > GRIPPER_S_CLOSED) {
    target = GRIPPER_S_CLOSED;
  }
  const uint16_t t = static_cast<uint16_t>(target);
  if (t == gripper_s) {
    return;
  }
  gripper_sweep_carry = 0;
  gripper_last_ms = millis();
  gripper_target = t;
  gripper_target_active = true;
  if (!gripper_pwm_on) {
    applyGripperPwm(gripper_s);
  }
  gripper_sweep = (t > gripper_s) ? GRIP_SWEEP_CLOSE : GRIP_SWEEP_OPEN;
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
    const uint16_t bound =
        gripper_target_active ? gripper_target : GRIPPER_S_OPEN;
    if (next <= bound + step) {
      gripper_s = bound;
      gripper_sweep = GRIP_SWEEP_STOP;
      gripper_sweep_carry = 0;
      if (gripper_target_active) {
        /* positioned move: hold at target with PWM on */
        gripper_target_active = false;
        applyGripperPwm(bound);
      } else {
        gripperPwmRelease();
      }
      return;
    }
    next = static_cast<uint16_t>(next - step);
  } else if (gripper_sweep == GRIP_SWEEP_CLOSE) {
    const uint16_t bound =
        gripper_target_active ? gripper_target : GRIPPER_S_CLOSED;
    if (next + step >= bound) {
      gripper_s = bound;
      gripper_sweep = GRIP_SWEEP_STOP;
      gripper_sweep_carry = 0;
      if (gripper_target_active) {
        gripper_target_active = false;
        applyGripperPwm(bound);
      } else {
        gripperPwmRelease();
      }
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

static int8_t drive_to_joint(uint8_t drive) {
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (J_DRIVE[i] == drive) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

static JointAnglesDeg angles_from_steps() {
  JointAnglesDeg q;
  q.j1 = MT4_HOME_J1_DEG + MT4_J1_STEP_SIGN *
                               static_cast<float>(joint_steps[0]) /
                               MT4_STEPS_PER_DEG[0];
  q.j2 = MT4_HOME_J2_DEG + MT4_J2_STEP_SIGN *
                               static_cast<float>(joint_steps[1]) /
                               MT4_STEPS_PER_DEG[1];
  q.j3 = MT4_HOME_J3_DEG + MT4_J3_STEP_SIGN *
                               static_cast<float>(joint_steps[2]) /
                               MT4_STEPS_PER_DEG[2];
  q.j4 = MT4_HOME_J4_DEG + MT4_J4_STEP_SIGN *
                               static_cast<float>(joint_steps[3]) /
                               MT4_STEPS_PER_DEG[3];
  return q;
}

static void reset_joint_steps() {
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    joint_steps[i] = 0;
    dda_accum[i] = 0;
    dda_delta[i] = 0;
  }
  dda_axis_mask = 0;
  cart_jog_mode = false;
  cart_dir_active_valid = false;
}

static void set_joint_dir(uint8_t joint, bool positive) {
  const bool high = positive ? J_DIR_POS_HIGH[joint] : !J_DIR_POS_HIGH[joint];
  set_dir(J_DIR_PIN[joint], high);
}

static void clear_jog_axes() {
  dda_axis_mask = 0;
  cart_jog_mode = false;
  move_mode = false;
  move_done_pending = false;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    dda_delta[i] = 0;
    dda_accum[i] = 0;
    move_remaining[i] = 0;
  }
}

static void refresh_dda_pin_io() {
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (!(dda_axis_mask & (1 << i))) {
      continue;
    }
    const uint8_t pin = J_DRIVE[i];
    dda_pin_io[i].port = portOutputRegister(digitalPinToPort(pin));
    dda_pin_io[i].mask = digitalPinToBitMask(pin);
    pinMode(pin, OUTPUT);
    *dda_pin_io[i].port &= ~dda_pin_io[i].mask;
  }
}

static void jog_pulse_low_all() {
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (dda_axis_mask & (1 << i)) {
      *dda_pin_io[i].port &= ~dda_pin_io[i].mask;
    }
  }
  step_pulse_high = false;
}

static bool add_jog_axis(uint8_t joint) {
  if (joint >= MT4_NUM_JOINTS) {
    return false;
  }
  dda_axis_mask |= static_cast<uint8_t>(1 << joint);
  dda_delta[joint] = dda_master;
  dda_accum[joint] = 0;
  step_pin = J_DRIVE[joint];
  return true;
}

static bool remove_jog_axis(uint8_t joint) {
  if (joint >= MT4_NUM_JOINTS) {
    return false;
  }
  if (!(dda_axis_mask & (1 << joint))) {
    return false;
  }
  dda_axis_mask &= static_cast<uint8_t>(~(1 << joint));
  dda_delta[joint] = 0;
  dda_accum[joint] = 0;
  step_pin = 0;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (dda_axis_mask & (1 << i)) {
      step_pin = J_DRIVE[i];
      break;
    }
  }
  return true;
}

static bool setup_cartesian_jog(const Vec3 *dir) {
  const JointAnglesDeg q = angles_from_steps();
  CartesianRates rates;
  if (!mt4_cartesian_rates(&q, dir, cart_orient_hold, cart_orient_gain,
                           &rates)) {
    return false;
  }

  clear_jog_axes();
  cart_jog_mode = true;
  dda_master = rates.master;

  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    int32_t delta = 0;
    switch (i) {
    case 0:
      delta = rates.j1;
      break;
    case 1:
      delta = rates.j2;
      break;
    case 2:
      delta = rates.j3;
      break;
    case 3:
      delta = rates.j4;
      break;
    }
    if (delta == 0) {
      continue;
    }
    set_joint_dir(i, delta > 0);
    dda_delta[i] = delta > 0 ? delta : -delta;
    dda_accum[i] = 0;
    dda_axis_mask |= static_cast<uint8_t>(1 << i);
  }
  return dda_axis_mask != 0;
}

static void print_joint_pos() {
  Serial.print(F("pos J1="));
  Serial.print(joint_steps[0]);
  Serial.print(F(" J2="));
  Serial.print(joint_steps[1]);
  Serial.print(F(" J3="));
  Serial.print(joint_steps[2]);
  Serial.print(F(" J4="));
  Serial.println(joint_steps[3]);
}

static void jog_timers_init() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = static_cast<uint16_t>(JOG_STEP_PERIOD_US * TIMER_TICKS_PER_US - 1);

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = static_cast<uint16_t>(STEP_PULSE_US * TIMER_TICKS_PER_US - 1);
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
  refresh_dda_pin_io();
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
  if (!jog_active || homing_active || dda_axis_mask == 0 || step_pulse_high) {
    return;
  }

  uint8_t step_mask = 0;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (!(dda_axis_mask & (1 << i))) {
      continue;
    }
    const int32_t mag = dda_delta[i];
    if (mag <= 0) {
      continue;
    }
    dda_accum[i] += mag;
    if (dda_accum[i] >= dda_master) {
      dda_accum[i] -= dda_master;
      step_mask |= static_cast<uint8_t>(1 << i);
    }
  }
  if (step_mask == 0) {
    return;
  }

  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (!(step_mask & (1 << i))) {
      continue;
    }
    *dda_pin_io[i].port |= dda_pin_io[i].mask;
    const bool high = digitalRead(J_DIR_PIN[i]) == HIGH;
    const bool positive = high == J_DIR_POS_HIGH[i];
    joint_steps[i] += positive ? 1 : -1;
    if (move_mode && move_remaining[i] > 0 && --move_remaining[i] == 0) {
      dda_axis_mask &= static_cast<uint8_t>(~(1 << i));
      dda_delta[i] = 0;
    }
  }
  if (move_mode && dda_axis_mask == 0) {
    move_done_pending = true;
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
  const int8_t joint = drive_to_joint(pin);
  if (joint < 0) {
    return false;
  }
  if (dda_axis_mask == 0) {
    cart_jog_mode = false;
    dda_master = MT4_CJ_MASTER;
  }
  return add_jog_axis(static_cast<uint8_t>(joint));
}

static bool remove_jog_pin(uint8_t pin) {
  const int8_t joint = drive_to_joint(pin);
  if (joint < 0) {
    return false;
  }
  return remove_jog_axis(static_cast<uint8_t>(joint));
}

static void print_status() {
  Serial.println(F("--- MT4 jog ---"));
  Serial.print(F("MODE="));
  Serial.print(cart_jog_mode ? F("cart") : F("joint"));
  Serial.print(F("  ORIENT="));
  Serial.print(cart_orient_hold ? F("hold") : F("free"));
  Serial.print(F(" gain="));
  Serial.println(cart_orient_gain, 3);
  print_joint_pos();
  Serial.print(F("STEP="));
  if (dda_axis_mask == 0) {
    Serial.println(F("none"));
  } else {
    bool first = true;
    for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
      if (!(dda_axis_mask & (1 << i))) {
        continue;
      }
      if (!first) {
        Serial.print('+');
      }
      Serial.print(F("D"));
      Serial.print(J_DRIVE[i]);
      first = false;
    }
    Serial.println();
  }
  Serial.print(F("EN="));
  Serial.print(drivers_enabled ? F("on") : F("off"));
  Serial.print(F("  JOG="));
  Serial.print(jog_active ? F("on") : F("off"));
  if (jog_active && dda_axis_mask != 0) {
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
  // limit and center. J3's interference reference is reached by FOLDING
  // toward J2 (the narrow side), so J3 widens the opposite direction from
  // its seek-toward-interference below.
  move_steps(J2_DRIVE, J2_DIR, !J2_HOME_DIR_HIGH, J23_PREWIDEN_STEPS);
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
  reset_joint_steps();
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

static void normalize_vec3(Vec3 *v) {
  const float n =
      sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
  if (n < 1e-6f) {
    return;
  }
  v->x /= n;
  v->y /= n;
  v->z /= n;
}

static bool parse_cj_vec(const char *arg, Vec3 *out) {
  while (*arg == ' ') {
    ++arg;
  }
  if (*arg == '\0') {
    return false;
  }

  // Single-axis shorthand: an optional sign followed by x/y/z (e.g. "-x").
  // Peek at a separate cursor so a leading '-' on the general signed-triple
  // form below (e.g. "-1 0 0") is NOT consumed here.
  int sign = 1;
  const char *p = arg;
  if (*p == '+' || *p == '-') {
    sign = (*p == '-') ? -1 : 1;
    ++p;
  }

  if ((p[0] == 'x' || p[0] == 'X') && p[1] == '\0') {
    out->x = static_cast<float>(sign);
    out->y = 0.0f;
    out->z = 0.0f;
    return true;
  }
  if ((p[0] == 'y' || p[0] == 'Y') && p[1] == '\0') {
    out->x = 0.0f;
    out->y = static_cast<float>(sign);
    out->z = 0.0f;
    return true;
  }
  if ((p[0] == 'z' || p[0] == 'Z') && p[1] == '\0') {
    out->x = 0.0f;
    out->y = 0.0f;
    out->z = static_cast<float>(sign);
    return true;
  }

  // General signed triple, parsed from the ORIGINAL (unstripped) arg so
  // each component keeps its own sign.
  long dx = 0;
  long dy = 0;
  long dz = 0;
  if (sscanf(arg, "%ld %ld %ld", &dx, &dy, &dz) == 3) {
    out->x = static_cast<float>(dx);
    out->y = static_cast<float>(dy);
    out->z = static_cast<float>(dz);
    return fabsf(out->x) + fabsf(out->y) + fabsf(out->z) > 1e-6f;
  }
  return false;
}

static void start_cartesian_jog(Vec3 dir) {
  normalize_vec3(&dir);
  cart_dir_active = dir;
  cart_dir_active_valid = true;
  cart_refresh_ms = millis();

  const bool was_active = jog_active;
  if (!was_active) {
    stop_jog();
    apply_all(PIN_FLOAT);
    set_enable(true);
    delay(ENABLE_SETTLE_MS);
  }

  if (!setup_cartesian_jog(&dir)) {
    Serial.println(F("err cj"));
    return;
  }

  if (!was_active) {
    jog_active = true;
    jog_timer_start();
  } else {
    refresh_dda_pin_io();
  }
  Serial.println(F("ok cj"));
}

/* `m` command: bounded relative move of all joints (+ optional gripper
 * delta). Joint deltas are signed steps; the DDA runs each axis at a rate
 * proportional to its distance so all joints finish together. Replies
 * "ok m" immediately; "m done pos ..." is emitted from loop() when the
 * joint motion completes. Drivers stay enabled (holding) afterwards. */
static void start_relative_move(const long d[MT4_NUM_JOINTS], long dg) {
  clear_jog_axes();

  int32_t master = 0;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    const int32_t mag = d[i] < 0 ? static_cast<int32_t>(-d[i])
                                 : static_cast<int32_t>(d[i]);
    if (mag > master) {
      master = mag;
    }
  }

  if (dg != 0) {
    gripperSweepToS(static_cast<long>(gripper_s) + dg);
  }

  if (master == 0) {
    /* gripper-only (or no-op) request: nothing to step */
    Serial.println(F("ok m"));
    Serial.print(F("m done "));
    print_joint_pos();
    return;
  }

  apply_all(PIN_FLOAT);
  set_enable(true);
  delay(ENABLE_SETTLE_MS);

  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (d[i] == 0) {
      continue;
    }
    const int32_t mag = d[i] < 0 ? static_cast<int32_t>(-d[i])
                                 : static_cast<int32_t>(d[i]);
    set_joint_dir(i, d[i] > 0);
    dda_delta[i] = mag;
    move_remaining[i] = mag;
    dda_accum[i] = 0;
    dda_axis_mask |= static_cast<uint8_t>(1 << i);
  }
  dda_master = master;
  move_mode = true;
  jog_active = true;
  jog_timer_start();
  Serial.println(F("ok m"));
}

static void refresh_cartesian_jog_if_due() {
  if (!cart_jog_mode || !cart_dir_active_valid || !jog_active) {
    return;
  }
  const unsigned long now = millis();
  if (now - cart_refresh_ms < CJ_REFRESH_MS) {
    return;
  }
  cart_refresh_ms = now;
  if (!setup_cartesian_jog(&cart_dir_active)) {
    stop_jog();
    Serial.println(F("err cj refresh"));
  } else {
    refresh_dda_pin_io();
  }
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
      strncmp(line, "home ", 5) && strncmp(line, "cj ", 3)) {
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
  if (!strcmp(line, "pos")) {
    print_joint_pos();
    return;
  }
  if (!strncmp(line, "setpos ", 7)) {
    long v[MT4_NUM_JOINTS] = {0, 0, 0, 0};
    if (sscanf(line + 7, "%ld %ld %ld %ld", &v[0], &v[1], &v[2], &v[3]) != 4) {
      Serial.println(F("err setpos <j1> <j2> <j3> <j4>"));
      return;
    }
    stop_jog();
    for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
      joint_steps[i] = v[i];
    }
    Serial.print(F("ok "));
    print_joint_pos();
    return;
  }
  if (!strncmp(line, "orient ", 7)) {
    const char *arg = line + 7;
    while (*arg == ' ') {
      ++arg;
    }
    if (!strcmp(arg, "on") || !strcmp(arg, "hold")) {
      cart_orient_hold = true;
    } else if (!strcmp(arg, "off") || !strcmp(arg, "free")) {
      cart_orient_hold = false;
    } else {
      char *end = nullptr;
      const float gain = strtod(arg, &end);
      if (end == arg || *end != '\0') {
        Serial.println(F("err orient on|off|<gain>"));
        return;
      }
      cart_orient_gain = gain;
      cart_orient_hold = gain != 0.0f;
    }
    Serial.print(F("ok orient "));
    Serial.print(cart_orient_hold ? F("hold gain=") : F("free gain="));
    Serial.println(cart_orient_gain, 3);
    return;
  }
  if (!strncmp(line, "cj ", 3)) {
    Vec3 dir = {0.0f, 0.0f, 0.0f};
    if (!parse_cj_vec(line + 3, &dir)) {
      Serial.println(F("err cj +x|-x|+y|-y|+z|-z|dx dy dz"));
      return;
    }
    start_cartesian_jog(dir);
    return;
  }
  if ((line[0] == 'm' || line[0] == 'M') && line[1] == ' ') {
    long d[MT4_NUM_JOINTS] = {0, 0, 0, 0};
    long dg = 0;
    const int n = sscanf(line + 2, "%ld %ld %ld %ld %ld",
                         &d[0], &d[1], &d[2], &d[3], &dg);
    if (n < 4) {
      Serial.println(F("err m <dj1> <dj2> <dj3> <dj4> [dg]"));
      return;
    }
    if (n == 4) {
      /* avr-libc's sscanf does not reliably leave a trailing unmatched %ld
       * argument untouched -- force the optional gripper delta to 0 rather
       * than trust whatever it wrote. */
      dg = 0;
    }
    for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
      if (d[i] > MOVE_MAX_STEPS || d[i] < -MOVE_MAX_STEPS) {
        Serial.println(F("err m step delta too large"));
        return;
      }
    }
    /* GRIPPER_S_* are uint16_t: their difference promotes to unsigned int,
     * and negating an unsigned value wraps instead of going negative (165
     * became 65371, not -165) -- cast to a signed type first. */
    const long gripper_span =
        static_cast<long>(GRIPPER_S_CLOSED) - static_cast<long>(GRIPPER_S_OPEN);
    if (dg > gripper_span || dg < -gripper_span) {
      Serial.println(F("err m gripper delta too large"));
      return;
    }
    start_relative_move(d, dg);
    return;
  }
  if (!strcmp(line, "j") || !strcmp(line, "jog")) {
    if (dda_axis_mask == 0) {
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
    move_mode = false;
    move_done_pending = false;
    Serial.println(F("ok stop"));
    return;
  }
  if (!strcmp(line, "all f")) {
    stop_jog();
    apply_all(PIN_FLOAT);
    drivers_enabled = false;
    clear_jog_axes();
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
    clear_jog_axes();
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
      clear_jog_axes();
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

  if (move_done_pending) {
    move_done_pending = false;
    stop_jog();
    move_mode = false;
    Serial.print(F("m done "));
    print_joint_pos();
  }
}
