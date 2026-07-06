#ifndef MT4_CONFIG_H
#define MT4_CONFIG_H

#include <Arduino.h>

#include "kinematics.h"

// Timer1/3 run at 16 MHz / prescaler 8 = 2 MHz, i.e. 2 ticks per us.
static const uint16_t TIMER_TICKS_PER_US = 2;

struct StepPinIO {
  volatile uint8_t *port;
  uint8_t mask;
};

enum PinModeSetting : uint8_t { PIN_FLOAT = 0, PIN_LOW = 1, PIN_HIGH = 2 };

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
static const uint16_t JOG_STEP_PERIOD_MIN_US = 700;
static const uint16_t JOG_STEP_PERIOD_MAX_US = 4000;
static const uint16_t CJ_REFRESH_MS = 40;
/* Generous headroom; ~2 full sweeps of any joint at current steps/deg. */
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
static const uint16_t J23_PREWIDEN_STEPS = 1000;
static const uint32_t HOME_SEEK_MAX = 25000;

#endif // MT4_CONFIG_H
