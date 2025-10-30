// arm_control.cpp
#include "arm_claw.h"
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ---------------- CONFIG --------------------
static const uint16_t SERVO_MIN_US = 500;
static const uint16_t SERVO_MAX_US = 2500;
static const uint8_t DEFAULT_STEP_DEG = 2;              // degrees per step for smoothing
static const TickType_t STEP_DELAY = pdMS_TO_TICKS(20); // 20 ms per step (50 Hz-ish)

// ----------------- STATE ---------------------
static Servo s_base, s_up, s_back, s_claw;

// current servo angles (float for smooth stepping)
static float cur_base = 90.0f;
static float cur_up = 90.0f;
static float cur_back = 90.0f;
static float cur_claw = 90.0f;

// target angles written by main/ws handler (0..180)
static volatile uint8_t tgt_base = 90;
static volatile uint8_t tgt_up = 90;
static volatile uint8_t tgt_back = 90;
static volatile uint8_t tgt_claw = 90;

static TaskHandle_t s_task = nullptr;

// public flag defined in main.cpp
//volatile bool arm_Enabled = false; // ensure there's one definition in main.cpp too (we'll set extern there)

// ----------------- HELPERS -------------------
static inline float fclamp(float v, float a, float b)
{
    return (v < a) ? a : (v > b) ? b
                                 : v;
}

static void attach_servo_safe(Servo &sv, int pin)
{
    sv.setPeriodHertz(50);
    sv.attach(pin, SERVO_MIN_US, SERVO_MAX_US);
}

// write servo with integer angle (0..180)
static void write_servo(Servo &sv, float angle)
{
    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > 180.0f)
        angle = 180.0f;
    sv.write((int)round(angle));
}

// step one current value toward target by stepDeg
static float step_toward(float cur, float target, float stepDeg)
{
    if (fabs(target - cur) < 0.01f)
        return target;
    if (target > cur)
    {
        cur += stepDeg;
        if (cur > target)
            cur = target;
    }
    else
    {
        cur -= stepDeg;
        if (cur < target)
            cur = target;
    }
    return cur;
}

// ----------------- TASK --------------------
static void arm_task(void *pv)
{
    // attach servos
    attach_servo_safe(s_base, ARM_PIN_BASE);
    attach_servo_safe(s_up, ARM_PIN_UPDOWN);
    attach_servo_safe(s_back, ARM_PIN_BACKFORTH);
    attach_servo_safe(s_claw, ARM_PIN_CLAW);

    // initialize positions (write current to servos)
    write_servo(s_base, cur_base);
    write_servo(s_up, cur_up);
    write_servo(s_back, cur_back);
    write_servo(s_claw, cur_claw);
    vTaskDelay(pdMS_TO_TICKS(200));

    const float step = (float)DEFAULT_STEP_DEG;

    while (true)
    {
        // if disabled, sleep but keep servo positions (so manual UI still allowed)
        if (!arm_Enabled)
        {
            // small sleep to avoid busy loop
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // read volatile targets into local (atomic-ish)
        uint8_t tb = tgt_base;
        uint8_t tu = tgt_up;
        uint8_t tbk = tgt_back;
        uint8_t tc = tgt_claw;

        // step current values toward targets
        cur_base = step_toward(cur_base, (float)tb, step);
        cur_up = step_toward(cur_up, (float)tu, step);
        cur_back = step_toward(cur_back, (float)tbk, step);
        cur_claw = step_toward(cur_claw, (float)tc, step);

        // write out to servos
        write_servo(s_base, cur_base);
        write_servo(s_up, cur_up);
        write_servo(s_back, cur_back);
        write_servo(s_claw, cur_claw);

        // If all reached, we can sleep a bit longer
        if (fabs(cur_base - tb) < 0.5f &&
            fabs(cur_up - tu) < 0.5f &&
            fabs(cur_back - tbk) < 0.5f &&
            fabs(cur_claw - tc) < 0.5f)
        {
            // reached - but keep modest poll so UI still feels responsive
            vTaskDelay(pdMS_TO_TICKS(60));
        }
        else
        {
            vTaskDelay(STEP_DELAY);
        }
    }
}

// ----------------- PUBLIC API ----------------
void start_Arm_Controller()
{
    if (s_task != nullptr)
        return;
    BaseType_t r = xTaskCreatePinnedToCore(
        arm_task,
        "ArmTask",
        4096,
        nullptr,
        2,
        &s_task,
        1 // run on core 1 (optional)
    );
    if (r != pdPASS)
    {
        Serial.println("Arm controller task create failed");
        s_task = nullptr;
    }
    else
    {
        Serial.println("Arm controller started");
    }
}

void stop_Arm_Controller()
{
    if (s_task == nullptr)
        return;
    vTaskDelete(s_task);
    s_task = nullptr;
    Serial.println("Arm controller stopped");
}

void arm_set_targets(uint8_t base_deg, uint8_t up_deg, uint8_t back_deg, uint8_t claw_deg)
{
    // clamp 0..180 and set volatile targets
    tgt_base = (uint8_t)fclamp((float)base_deg, 0.0f, 180.0f);
    tgt_up = (uint8_t)fclamp((float)up_deg, 0.0f, 180.0f);
    tgt_back = (uint8_t)fclamp((float)back_deg, 0.0f, 180.0f);
    tgt_claw = (uint8_t)fclamp((float)claw_deg, 0.0f, 180.0f);
}
