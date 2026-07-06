#ifndef MT4_PINS_H
#define MT4_PINS_H

#include <Arduino.h>

#include "config.h"

// Drivers enabled (shared enable line) and the currently-selected legacy
// single/multi-axis jog drive pin (x/x+/x-/xc commands).
extern bool drivers_enabled;
extern uint8_t step_pin;

void pins_init();
int8_t lab_index(uint8_t pin);
void set_enable(bool on);
void set_dir(uint8_t dir_pin, bool high);
void apply_pin(uint8_t index, PinModeSetting mode);
void apply_all(PinModeSetting mode);
bool limit_triggered(uint8_t pin);
void poll_limits();
void print_limits();
int8_t drive_to_joint(uint8_t drive);

#endif // MT4_PINS_H
