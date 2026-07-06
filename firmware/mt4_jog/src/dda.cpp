#include "dda.h"

#include "pins.h"

volatile bool homing_active = false;
volatile bool jog_active = false;
volatile bool move_mode = false;
volatile bool move_done_pending = false;
volatile uint8_t dda_axis_mask = 0;
volatile int32_t joint_steps[MT4_NUM_JOINTS];

// Runtime-adjustable via `speed <us>` (bounded); default/initial value here.
// 70% of prior 1067 us tick rate.
static uint16_t jog_step_period_us = 1524;

static volatile int32_t dda_master = MT4_CJ_MASTER;
static volatile int32_t dda_delta[MT4_NUM_JOINTS];
static volatile int32_t dda_accum[MT4_NUM_JOINTS];
static StepPinIO dda_pin_io[MT4_NUM_JOINTS];
static volatile bool step_pulse_high = false;
static volatile int32_t move_remaining[MT4_NUM_JOINTS];

static void set_joint_dir(uint8_t joint, bool positive) {
  const bool high = positive ? J_DIR_POS_HIGH[joint] : !J_DIR_POS_HIGH[joint];
  set_dir(J_DIR_PIN[joint], high);
}

void dda_clear_axes() {
  dda_axis_mask = 0;
  move_mode = false;
  move_done_pending = false;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    dda_delta[i] = 0;
    dda_accum[i] = 0;
    move_remaining[i] = 0;
  }
}

void dda_reset() {
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    joint_steps[i] = 0;
    dda_accum[i] = 0;
    dda_delta[i] = 0;
  }
  dda_axis_mask = 0;
}

void dda_set_joint_steps(const long steps[MT4_NUM_JOINTS]) {
  dda_stop();
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    joint_steps[i] = steps[i];
  }
}

void dda_refresh_pins() {
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

bool dda_add_axis_by_pin(uint8_t pin) {
  const int8_t joint = drive_to_joint(pin);
  if (joint < 0) {
    return false;
  }
  if (dda_axis_mask == 0) {
    dda_master = MT4_CJ_MASTER;
  }
  return add_jog_axis(static_cast<uint8_t>(joint));
}

bool dda_remove_axis_by_pin(uint8_t pin) {
  const int8_t joint = drive_to_joint(pin);
  if (joint < 0) {
    return false;
  }
  return remove_jog_axis(static_cast<uint8_t>(joint));
}

bool dda_arm(int32_t master, const int32_t deltas[MT4_NUM_JOINTS], bool track_move) {
  dda_clear_axes();
  dda_master = master;
  move_mode = track_move;
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (deltas[i] == 0) {
      continue;
    }
    set_joint_dir(i, deltas[i] > 0);
    const int32_t mag = deltas[i] > 0 ? deltas[i] : -deltas[i];
    dda_delta[i] = mag;
    dda_accum[i] = 0;
    if (track_move) {
      move_remaining[i] = mag;
    }
    dda_axis_mask |= static_cast<uint8_t>(1 << i);
  }
  return dda_axis_mask != 0;
}

static void apply_jog_speed() {
  // May be called live while a jog is active (speed nudged from the
  // keyboard client); guard the 16-bit register write against a
  // concurrent Timer1 ISR tick.
  cli();
  OCR1A = static_cast<uint16_t>(jog_step_period_us * TIMER_TICKS_PER_US - 1);
  sei();
}

void dda_set_speed_us(long us) {
  if (us < JOG_STEP_PERIOD_MIN_US) {
    us = JOG_STEP_PERIOD_MIN_US;
  } else if (us > JOG_STEP_PERIOD_MAX_US) {
    us = JOG_STEP_PERIOD_MAX_US;
  }
  jog_step_period_us = static_cast<uint16_t>(us);
  apply_jog_speed();
  Serial.print(F("ok speed "));
  Serial.println(jog_step_period_us);
}

void dda_init() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  apply_jog_speed();

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = static_cast<uint16_t>(STEP_PULSE_US * TIMER_TICKS_PER_US - 1);
}

void dda_stop() {
  cli();
  TIMSK1 = 0;
  TCCR1B = 0;
  TIMSK3 = 0;
  TCCR3B = 0;
  jog_pulse_low_all();
  sei();
  jog_active = false;
}

void dda_engage() {
  dda_refresh_pins();
  jog_active = true;
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
