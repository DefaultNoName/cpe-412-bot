// motor_control.cpp
#include "motor_control.h"

// initialize motor pins and LEDC PWM channels (11-bit, 25kHz)
void init_Motors()
{
    // STBY (kept for compatibility) - set HIGH
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    // direction pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    // pwm pins as outputs (ledcAttachPin will configure them)
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);

    // default to stopped
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    // LEDC setup - 11-bit resolution -> duty 0..2047, 25kHz frequency
    ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RESO);
    ledcAttachPin(PWMA, PWM_CH_A);

    ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RESO);
    ledcAttachPin(PWMB, PWM_CH_B);

    ledcWrite(PWM_CH_A, 0);
    ledcWrite(PWM_CH_B, 0);

    delay(20);
    Serial.println("Motors initialized (L298N, 11-bit PWM @ 25kHz).");
}

// movement_Control: motor 0 => A, 1 => B
// speed: signed in -PWM_MAX_INPUT .. +PWM_MAX_INPUT (i.e. -2047..+2047)
void movement_Control(uint8_t motor, int16_t speed)
{
    // clamp to allowed range
    if (speed > (int16_t)PWM_MAX_INPUT)
        speed = (int16_t)PWM_MAX_INPUT;
    if (speed < -(int16_t)PWM_MAX_INPUT)
        speed = -(int16_t)PWM_MAX_INPUT;

    // magnitude and duty mapping (0..2047)
    int32_t magnitude = (speed < 0) ? -(int32_t)speed : (int32_t)speed;
    uint32_t duty = (uint32_t)magnitude; // already in 0..2047 since input is that range

    // if using any generic mapping from other input ranges, map accordingly here.
    // Now apply to selected motor
    if (motor == 0)
    {
        // Motor A: AIN1/AIN2 + PWMA
        if (magnitude == 0)
        {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);
            ledcWrite(PWM_CH_A, 0);
        }
        else
        {
            bool forward = (speed > 0);
            digitalWrite(AIN1, forward ? HIGH : LOW);
            digitalWrite(AIN2, forward ? LOW : HIGH);
            ledcWrite(PWM_CH_A, duty); 
        }
    }
    else
    {
        // Motor B: BIN1/BIN2 + PWMB
        if (magnitude == 0)
        {
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, LOW);
            ledcWrite(PWM_CH_B, 0);
        }
        else
        {
            bool forward = (speed > 0);
            digitalWrite(BIN1, forward ? HIGH : LOW);
            digitalWrite(BIN2, forward ? LOW : HIGH);
            ledcWrite(PWM_CH_B, duty); // 0..2047
        }
    }
}
