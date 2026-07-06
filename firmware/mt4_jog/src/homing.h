#ifndef MT4_HOMING_H
#define MT4_HOMING_H

#include <Arduino.h>

// Shared serial line buffer: loop() (main.cpp) fills it for normal command
// dispatch; serial_abort() (homing.cpp) also drains Serial into it while a
// blocking home sequence has control, to catch "!"/"stop" without needing
// the main loop to be running.
extern char line_buf[64];
extern uint8_t line_len;

void do_home(uint16_t j1_center, uint16_t j2_pull);

#endif // MT4_HOMING_H
