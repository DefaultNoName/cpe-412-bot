#include "motor_control.h"
#include <Arduino.h>

// keep constants in one place
const uint8_t PWM_CH_A = 0x00;
const uint8_t PWM_CH_B = 0x01;
const uint32_t PWM_FREQ = 16000;    // 16 kHz
const uint8_t PWM_RESO = 12;        // 12-bit
const int32_t PWM_MAX_INPUT = 4095; // logical range 0..4095

void init_Motors()
{
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, LOW);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    // LEDC PWM setup
    ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RESO);
    ledcAttachPin(PWMA, PWM_CH_A);
    ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RESO);
    ledcAttachPin(PWMB, PWM_CH_B);

    digitalWrite(STBY, HIGH); // enable driver
    Serial.println("Motors - Initialized");

    vTaskDelay(pdMS_TO_TICKS(250));
}

// motor control implementation
void movement_Control(uint8_t motorIndex, int16_t motorSpeed)
{
    if (motorIndex > 1)
    {
        Serial.printf("movement_Control: invalid motor %u\n", motorIndex);
        return;
    }

    // clamp to -MAX..+MAX
    if (motorSpeed > (int16_t)PWM_MAX_INPUT)
        motorSpeed = (int16_t)PWM_MAX_INPUT;
    if (motorSpeed < -(int16_t)PWM_MAX_INPUT)
        motorSpeed = -(int16_t)PWM_MAX_INPUT;

    // magnitude and percent scaling
    int32_t absSpeed = (motorSpeed < 0) ? -(int32_t)motorSpeed : (int32_t)motorSpeed;
    const uint32_t maxDuty = (1UL << PWM_RESO) - 1;
    uint32_t dutyCycle = (uint32_t)absSpeed * maxDuty / (uint32_t)PWM_MAX_INPUT;
    if (dutyCycle > maxDuty)
        dutyCycle = maxDuty;

    // zero => coast
    if (absSpeed == 0)
    {
        if (motorIndex == 0)
        {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW); // coast
            ledcWrite(PWM_CH_A, 0);
        }
        else
        {
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, LOW); // coast
            ledcWrite(PWM_CH_B, 0);
        }
        return;
    }

    // set direction & write duty
    if (motorIndex == 0)
    {
        // Left motor: adjust HIGH/LOW depending on wiring; this matches the corrected logic
        digitalWrite(AIN1, motorSpeed > 0 ? LOW : HIGH);
        digitalWrite(AIN2, motorSpeed > 0 ? HIGH : LOW);
        ledcWrite(PWM_CH_A, dutyCycle);
    }
    else
    {
        digitalWrite(BIN1, motorSpeed > 0 ? HIGH : LOW);
        digitalWrite(BIN2, motorSpeed > 0 ? LOW : HIGH);
        ledcWrite(PWM_CH_B, dutyCycle);
    }
}
