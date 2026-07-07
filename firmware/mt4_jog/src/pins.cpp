#include "pins.h"

#include "dda.h"

bool drivers_enabled = false;
uint8_t step_pin = 0;

static PinModeSetting pin_mode[LAB_PIN_COUNT];
static int8_t last_limit_raw[LIMIT_PIN_COUNT];
static bool last_limit_valid = false;

void pins_init() {
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
}

int8_t lab_index(uint8_t pin) {
  for (uint8_t i = 0; i < LAB_PIN_COUNT; ++i) {
    if (LAB_PINS[i] == pin) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

void set_enable(bool on) {
  drivers_enabled = on;
  const bool level = ENABLE_ACTIVE_LOW ? !on : on;
  const int8_t idx = lab_index(ENABLE_PIN);
  if (idx >= 0) {
    pin_mode[idx] = on ? PIN_LOW : PIN_HIGH;
  }
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, level ? HIGH : LOW);
}

void set_dir(uint8_t dir_pin, bool high) {
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, high ? HIGH : LOW);
}

void apply_pin(uint8_t index, PinModeSetting mode) {
  const uint8_t pin = LAB_PINS[index];
  pin_mode[index] = mode;
  if (mode == PIN_FLOAT) {
    pinMode(pin, INPUT);
    return;
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, mode == PIN_HIGH ? HIGH : LOW);
}

void apply_all(PinModeSetting mode) {
  for (uint8_t i = 0; i < LAB_PIN_COUNT; ++i) {
    apply_pin(i, mode);
  }
  if (mode == PIN_LOW) {
    drivers_enabled = ENABLE_ACTIVE_LOW;
  } else if (mode == PIN_HIGH) {
    drivers_enabled = !ENABLE_ACTIVE_LOW;
  }
}

bool limit_triggered(uint8_t pin) {
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

void poll_limits() {
  for (uint8_t i = 0; i < LIMIT_PIN_COUNT; ++i) {
    const uint8_t pin = LIMIT_PINS[i];
    const int raw = digitalRead(pin);
    if (!last_limit_valid || raw != last_limit_raw[i]) {
      last_limit_raw[i] = static_cast<int8_t>(raw);
      // Homing already reports its own start/complete/fail lines; suppress
      // the per-transition debug prints so its output stays terse.
      if (!homing_active) {
        emit_limit(pin, raw);
      }
    }
  }
  last_limit_valid = true;
}

void print_limits() {
  Serial.print(F("I20="));
  Serial.print(digitalRead(20));
  Serial.print(F(" I21="));
  Serial.println(digitalRead(21));
}

int8_t drive_to_joint(uint8_t drive) {
  for (uint8_t i = 0; i < MT4_NUM_JOINTS; ++i) {
    if (J_DRIVE[i] == drive) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}
