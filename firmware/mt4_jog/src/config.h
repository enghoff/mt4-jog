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
/* `mp` absolute moves: linear TCP interpolation step size (mm) and cap on
 * the number of segments per move (longer moves use a larger effective step). */
static const float MP_CART_SEGMENT_MM = 2.0f;
static const uint16_t MP_MAX_SEGMENTS = 250;
/* Keep-out cylinder around the J1 axis (any Z): the TCP physically cannot
 * get closer to the base column than roughly this. `mp` targets inside it
 * are rejected; `mp` paths that would cross it are routed around it
 * (tangent-arc-tangent, shortest side); Cartesian jog clamps the inward
 * velocity component at the boundary so jogging slides along the cylinder
 * instead of driving into the base. */
static const float MT4_KEEPOUT_RADIUS_MM = 170.0f;
/* Desk / ground plane: TCP Z below this is rejected (`mp`) and Cartesian jog
 * clamps downward velocity. From envelope_samples.json in-range min Z
 * 135.7mm (2026-07-19); rounded up slightly for clearance. */
static const float MT4_GROUND_Z_MM = 136.0f;
/* Soft joint step limits (counters after home = 0). Switch-side ends for
 * J1/J2 are overwritten at the end of do_home() from the pull-off distance
 * (limit switch = -pull_steps). Opposite ends and J3/J4 both ends come from
 * the measured envelope (in-range samples, 2026-07-19). */
static const int32_t MT4_JOINT_SOFT_MIN_DEFAULT[MT4_NUM_JOINTS] = {
    -4800L, -1000L, -2050L, -8100L};
static const int32_t MT4_JOINT_SOFT_MAX_DEFAULT[MT4_NUM_JOINTS] = {
    4580L, 2950L, 1150L, 8100L};
/* Coupled J2+J3 extension limit (step counters). Because J2 and J3 have
 * opposite step signs, j2_deg - j3_deg = const - (j2_steps + j3_steps)/spd,
 * so a *minimum* link-angle difference at full stretch is a *maximum* on
 * j2_steps + j3_steps. In-sample max sum was 2910 (r≈352mm, j2−j3≈15.2°);
 * over-extension outs sit at 2961–3108. Soft min on the sum is loose —
 * the folded extreme is gated by J3 min + ground Z. */
static const int32_t MT4_J2_J3_SUM_MAX = 2910L;
static const int32_t MT4_J2_J3_SUM_MIN = -1700L;
/* Max path pieces for a routed `mp` move: radial escape (when starting
 * inside the cylinder) + entry tangent + arc + exit tangent. */
static const uint8_t MP_MAX_PIECES = 4;
/* `mp` acceleration ramp (dda.cpp): a move starts at this slower, safe-to-
 * start-from-rest step period and ramps toward the move's cruise speed over
 * up to this many master ticks, then symmetrically ramps back up to
 * MP_ACCEL_START_US before the move ends. No-ops (falls back to the old
 * constant-speed stepping) when the requested cruise speed is already this
 * slow or slower, or the move is too short for a full ramp -- see
 * dda_set_ramp(). Untuned against real stall/skip behavior yet; the values
 * below are a conservative starting point for reaching the 700us max. */
static const uint16_t MP_ACCEL_START_US = 1800;
static const uint16_t MP_ACCEL_RAMP_TICKS = 60;
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
static const uint16_t J3_HOME_PULL_DEFAULT = 500;
// Pre-home backoff for J3 so it isn't already sitting in a position that
// interferes with J2's approach to its own limit switch.
static const uint16_t J3_PREWIDEN_STEPS = 1000;
// J2 widen steps after releasing its limit switch (if already
// triggered after J3 pre-widen) before seeking back toward the switch.
static const uint16_t J2_PREWIDEN_STEPS = 500;
static const uint32_t HOME_SEEK_MAX = 25000;
// Dwell after J1 hits its limit switch, before reversing back toward
// center. Lets J1 fully settle at the switch (no ramp/creep on approach)
// instead of reversing direction immediately.
static const uint16_t J1_HOME_LIMIT_PAUSE_MS = 300;

#endif // MT4_CONFIG_H
