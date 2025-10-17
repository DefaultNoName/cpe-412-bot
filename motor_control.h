#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <stdint.h>

// PWM / motor constants used by movement_Control
extern const uint8_t PWM_CH_A;
extern const uint8_t PWM_CH_B;
extern const uint32_t PWM_FREQ;
extern const uint8_t PWM_RESO;
extern const int32_t PWM_MAX_INPUT;

// Motor pins (as in your project)
#define AIN1 GPIO_NUM_26
#define AIN2 GPIO_NUM_27
#define PWMA GPIO_NUM_25
#define BIN1 GPIO_NUM_12
#define BIN2 GPIO_NUM_13
#define PWMB GPIO_NUM_14
#define STBY GPIO_NUM_33

// initialize motors + PWM (call once)
void init_Motors();

// control (motor: 0=left, 1=right). speed in -PWM_MAX_INPUT..+PWM_MAX_INPUT
void movement_Control(uint8_t motor, int16_t speed);

#endif // MOTOR_CONTROL_H
