#ifndef MT4_GRIPPER_H
#define MT4_GRIPPER_H

#include <Arduino.h>

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

// Read by motion.cpp's print_status.
extern uint16_t gripper_s;
extern bool gripper_pwm_on;
extern int8_t gripper_sweep;

void gripperPwmInit();
bool gripperSValid(long s);
void setGripperS(uint16_t s);
void gripperSweepStop();
void gripperSweepStart(int8_t dir);
/* Sweep to an absolute S target (clamped) and hold there with PWM on --
 * used by the `m` command's gripper delta (dg). */
void gripperSweepToS(long target);
void gripperSweepTick();
void printGripperStatus();

#endif // MT4_GRIPPER_H
