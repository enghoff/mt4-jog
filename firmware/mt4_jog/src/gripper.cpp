#include "gripper.h"

uint16_t gripper_s =
    static_cast<uint16_t>((GRIPPER_S_OPEN + GRIPPER_S_CLOSED) / 2);
bool gripper_pwm_on = false;
int8_t gripper_sweep = GRIP_SWEEP_STOP;

static unsigned long gripper_last_ms = 0;
static uint32_t gripper_sweep_carry = 0;
/* Target-bounded sweep (`m` command's dg): sweep stops at gripper_target
 * instead of the full open/closed endpoint, PWM left on to hold position. */
static uint16_t gripper_target = 0;
static bool gripper_target_active = false;

static void gripperEnablePins() {
  DDRH |= (1 << PH4);
  DDRE |= (1 << PE3);
  PORTE |= (1 << PE5);
}

static void gripperPwmOff() {
  OCR4B = 0;
  TCCR4A &= static_cast<uint8_t>(~0x20);
}

static void gripperPwmOn() {
  TCCR4A |= 0x20;
}

static uint16_t gripperSToOcr(uint16_t s) {
  if (s == 0) {
    return 0;
  }
  if (s > GRIPPER_S_PWM_SCALE) {
    s = GRIPPER_S_PWM_SCALE;
  }
  return static_cast<uint16_t>((uint32_t)s * GRIPPER_PWM_TOP / GRIPPER_S_PWM_SCALE);
}

bool gripperSValid(long s) {
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

void gripperPwmInit() {
  gripperEnablePins();
  TCCR4A = 0x23;
  TCCR4B = (TCCR4B & 0xE0) | 0x1A;
  ICR4 = GRIPPER_PWM_TOP;
  OCR4A = 0xFFFF;
  OCR4B = 0;
}

void setGripperS(uint16_t s) {
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

void gripperSweepStop() {
  gripper_sweep = GRIP_SWEEP_STOP;
  gripper_sweep_carry = 0;
  gripperPwmRelease();
}

void gripperSweepStart(int8_t dir) {
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

void gripperSweepToS(long target) {
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

void gripperSweepTick() {
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

void printGripperStatus() {
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
