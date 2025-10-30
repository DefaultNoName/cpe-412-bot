// arm_control.h
#ifndef ARM_CLAW_H
#define ARM_CLAW_H

#include <Arduino.h>
#include <stdint.h>

// Arm pins
static const int ARM_PIN_UPDOWN = 4;    // GPIO4 (Up/Down)
static const int ARM_PIN_BACKFORTH = 5; // GPIO5 (Back/Forth)
static const int ARM_PIN_BASE = 16;     // GPIO16 (Base rotation)
static const int ARM_PIN_CLAW = 17;     // GPIO17 (Claw / Grabber)

// public API
extern volatile bool arm_Enabled; // defined in main.cpp

// start/stop the arm controller task (call start from setup)
void start_Arm_Controller();
void stop_Arm_Controller();

// Set target positions (0..180) for base, updown, back/forth, claw (0..180).
// These values are written by the websocket handler (or UI) and the task will
// smoothly move the servos toward targets.
void arm_set_targets(uint8_t base_deg, uint8_t up_deg, uint8_t back_deg, uint8_t claw_deg);

// Optional: set speeds (deg per step and delay) could be added later.

#endif // ARM_CONTROL_H
