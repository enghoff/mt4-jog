#include "dda.h"

#include "motion.h"
#include "pins.h"

volatile bool homing_active = false;
volatile bool jog_active = false;
volatile bool move_mode = false;
volatile bool move_done_pending = false;
volatile uint8_t dda_axis_mask = 0;
volatile int32_t joint_steps[MT4_NUM_JOINTS];

// Runtime-adjustable via `speed <us>` (bounded); session state, reset on reboot.
// 70% of prior 1067 us tick rate.
static uint16_t jog_step_period_us = 1524;

static volatile int32_t dda_master = MT4_CJ_MASTER;
static volatile int32_t dda_delta[MT4_NUM_JOINTS];
static volatile int32_t dda_accum[MT4_NUM_JOINTS];
static StepPinIO dda_pin_io[MT4_NUM_JOINTS];
static volatile bool step_pulse_high = false;
static volatile int32_t move_remaining[MT4_NUM_JOINTS];

// Acceleration ramp for `mp` moves -- see dda_set_ramp() in dda.h. Lives
// independently of the per-segment dda_arm()/dda_stop() state above so it
// survives the segment-boundary dda_stop() calls motion.cpp's mp_continue_path()
// makes between segments of the same `mp` command.
enum RampPhase : uint8_t { RAMP_NONE, RAMP_ACCEL, RAMP_CRUISE, RAMP_DECEL };
static volatile RampPhase ramp_phase = RAMP_NONE;
static volatile uint16_t ramp_current_ticks;  // live OCR1A target, timer ticks
static volatile uint16_t ramp_start_ticks;
static volatile uint16_t ramp_cruise_ticks;
static volatile uint16_t ramp_step_ticks;     // per-master-tick change during accel/decel
static volatile int32_t ramp_decel_at;        // begin decel once ramp_remaining <= this
static volatile int32_t ramp_remaining;       // master ticks left in the whole planned move

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

static void set_speed_us_impl(long us, bool echo) {
  if (us < JOG_STEP_PERIOD_MIN_US) {
    us = JOG_STEP_PERIOD_MIN_US;
  } else if (us > JOG_STEP_PERIOD_MAX_US) {
    us = JOG_STEP_PERIOD_MAX_US;
  }
  jog_step_period_us = static_cast<uint16_t>(us);
  apply_jog_speed();
  if (echo) {
    Serial.print(F("ok speed "));
    Serial.println(jog_step_period_us);
  }
}

void dda_set_speed_us(long us) { set_speed_us_impl(us, true); }

void dda_set_speed_us_quiet(long us) { set_speed_us_impl(us, false); }

uint16_t dda_get_speed_us() { return jog_step_period_us; }

// cj speed-ramp state. cj_ramp_step_us == 0 (the power-on default) keeps
// dda_set_cj_target_speed_us() applying instantly -- see dda.h for why that's
// the rollback lever.
static uint16_t cj_target_period_us = 1524;
static uint16_t cj_ramp_step_us = 0;

void dda_set_cj_target_speed_us(long us) {
  if (us < JOG_STEP_PERIOD_MIN_US) {
    us = JOG_STEP_PERIOD_MIN_US;
  } else if (us > JOG_STEP_PERIOD_MAX_US) {
    us = JOG_STEP_PERIOD_MAX_US;
  }
  cj_target_period_us = static_cast<uint16_t>(us);
  if (cj_ramp_step_us == 0) {
    jog_step_period_us = cj_target_period_us;
    apply_jog_speed();
  }
  Serial.print(F("ok cjspeed "));
  Serial.println(cj_target_period_us);
}

void dda_set_cj_ramp_step_us(long us) {
  if (us < 0) {
    us = 0;
  } else if (us > JOG_STEP_PERIOD_MAX_US) {
    us = JOG_STEP_PERIOD_MAX_US;
  }
  cj_ramp_step_us = static_cast<uint16_t>(us);
  Serial.print(F("ok cjramp "));
  Serial.println(cj_ramp_step_us);
}

uint16_t dda_get_cj_ramp_step_us() { return cj_ramp_step_us; }

void dda_tick_cj_speed_ramp() {
  if (cj_ramp_step_us == 0 || jog_step_period_us == cj_target_period_us) {
    return;
  }
  int32_t diff = static_cast<int32_t>(cj_target_period_us) -
                 static_cast<int32_t>(jog_step_period_us);
  if (diff > static_cast<int32_t>(cj_ramp_step_us)) {
    diff = cj_ramp_step_us;
  } else if (diff < -static_cast<int32_t>(cj_ramp_step_us)) {
    diff = -static_cast<int32_t>(cj_ramp_step_us);
  }
  jog_step_period_us =
      static_cast<uint16_t>(static_cast<int32_t>(jog_step_period_us) + diff);
  apply_jog_speed();
}

void dda_ramp_clear() { ramp_phase = RAMP_NONE; }

void dda_set_ramp(uint16_t start_us, uint16_t cruise_us, int32_t total_ticks,
                   uint16_t ramp_ticks) {
  const uint16_t start_ticks =
      static_cast<uint16_t>(start_us * TIMER_TICKS_PER_US);
  const uint16_t cruise_ticks =
      static_cast<uint16_t>(cruise_us * TIMER_TICKS_PER_US);
  // Nothing to ramp: already at/below the safe-start speed, the move is too
  // short to estimate a ramp for, or too short to fit one at all.
  if (cruise_ticks >= start_ticks || total_ticks < 4 || ramp_ticks == 0) {
    ramp_phase = RAMP_NONE;
    return;
  }
  uint16_t len = ramp_ticks;
  if (static_cast<int32_t>(len) * 2 > total_ticks) {
    len = static_cast<uint16_t>(total_ticks / 2);
  }
  if (len == 0) {
    ramp_phase = RAMP_NONE;
    return;
  }
  uint16_t step = static_cast<uint16_t>((start_ticks - cruise_ticks) / len);
  if (step == 0) {
    step = 1;
  }

  cli();
  ramp_start_ticks = start_ticks;
  ramp_cruise_ticks = cruise_ticks;
  ramp_step_ticks = step;
  ramp_decel_at = len;
  ramp_remaining = total_ticks;
  ramp_current_ticks = start_ticks;
  OCR1A = static_cast<uint16_t>(ramp_current_ticks - 1);
  ramp_phase = RAMP_ACCEL;
  sei();
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

  // Advance the `mp` acceleration ramp, if one is active, before this
  // tick's stepping -- integer add/sub/compare only (no float, no divide;
  // that was all precomputed once in dda_set_ramp(), outside the ISR).
  if (ramp_phase != RAMP_NONE) {
    if (ramp_phase == RAMP_ACCEL) {
      if (ramp_current_ticks > ramp_cruise_ticks + ramp_step_ticks) {
        ramp_current_ticks -= ramp_step_ticks;
      } else {
        ramp_current_ticks = ramp_cruise_ticks;
        ramp_phase = RAMP_CRUISE;
      }
    }
    if (ramp_phase != RAMP_DECEL && ramp_remaining <= ramp_decel_at) {
      ramp_phase = RAMP_DECEL;
    }
    if (ramp_phase == RAMP_DECEL) {
      if (ramp_current_ticks + ramp_step_ticks < ramp_start_ticks) {
        ramp_current_ticks += ramp_step_ticks;
      } else {
        ramp_current_ticks = ramp_start_ticks;
      }
    }
    OCR1A = static_cast<uint16_t>(ramp_current_ticks - 1);
    if (ramp_remaining > 0) {
      --ramp_remaining;
    }
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
    const bool high = digitalRead(J_DIR_PIN[i]) == HIGH;
    const bool positive = high == J_DIR_POS_HIGH[i];
    if (!motion_step_allowed(i, positive)) {
      // Soft joint limit: abort the entire jog/move, not just this axis.
      for (uint8_t j = 0; j < MT4_NUM_JOINTS; ++j) {
        dda_delta[j] = 0;
      }
      dda_axis_mask = 0;
      step_mask = 0;
      if (move_mode) {
        move_done_pending = true;
      }
      return;
    }
    *dda_pin_io[i].port |= dda_pin_io[i].mask;
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
