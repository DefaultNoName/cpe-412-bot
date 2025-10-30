// motor_control.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <stdint.h>

// === PIN DEFINITIONS (change if your wiring differs) ===
#ifndef STBY
#define STBY 33
#endif

#ifndef AIN1
#define AIN1 26
#endif
#ifndef AIN2
#define AIN2 27
#endif
#ifndef PWMA
#define PWMA 25
#endif

#ifndef BIN1
#define BIN1 12
#endif
#ifndef BIN2
#define BIN2 13
#endif
#ifndef PWMB
#define PWMB 14

#endif

// PWM / LEDC configuration - changed to 11-bit / 25 kHz
static const uint8_t PWM_CH_A = 0;                        // LEDC channel for motor A enable (PWMA)
static const uint8_t PWM_CH_B = 1;                        // LEDC channel for motor B enable (PWMB)
static const uint32_t PWM_FREQ = 32000;                   // 32 kHz PWM frequency
static const uint8_t PWM_RESO = 11;                       // 11-bit resolution (0..2047)
static const int16_t PWM_MAX_INPUT = (1 << PWM_RESO) - 1; // 2047

// public API
void init_Motors();                                  // call in setup()
void movement_Control(uint8_t motor, int16_t speed); // motor: 0=A, 1=B ; speed: -2047..+2047

#endif // MOTOR_CONTROL_H
