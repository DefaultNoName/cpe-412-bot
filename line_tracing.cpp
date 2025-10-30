// line_tracing.cpp
#include "line_tracing.h"
#include "motor_control.h"
#include <Arduino.h>

// Task handle so we can stop the task if needed
static TaskHandle_t s_lineTracerTaskHandle = nullptr;

// Simple debounce read: sample 3 times and majority vote.
// LM339 outputs are typically active-LOW (sink to GND), so LOW == line detected.
static bool read_IR_Debounced(int pin)
{
    int count = 0;
    for (int i = 0; i < 3; ++i)
    {
        int v = digitalRead(pin);
        if (v == LOW)
            ++count;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return (count >= 2);
}

// helper: map 0..255 -> 0..PWM_MAX_INPUT (integer)
static inline int32_t speed255_To_MaxInput(uint8_t v255)
{
    return (int32_t)((uint32_t)v255 * (uint32_t)PWM_MAX_INPUT / 255U);
}

static void line_Tracer_Task(void *pv)
{
    // configure pins (use internal pull-ups where available)
    pinMode(IR_PIN_LEFT, INPUT_PULLUP);
    pinMode(IR_PIN_CENTER, INPUT_PULLUP);
    pinMode(IR_PIN_RIGHT, INPUT_PULLUP);

    // last known side: 1 => last on right, -1 => last on left
    int last_error_sign = 1;

    while (true)
    {
        // If line tracing is disabled, sleep longer (but keep task alive)
        if (!line_Tracing_Enabled)
        {
            // ensure motors are not driven by tracer while disabled
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // When enabled: run the control loop
        bool L = read_IR_Debounced(IR_PIN_LEFT);
        bool C = read_IR_Debounced(IR_PIN_CENTER);
        bool R = read_IR_Debounced(IR_PIN_RIGHT);

        int sum = (int)L + (int)C + (int)R;

        // Intersection detection: all 3 active
        if (L && C && R)
        {
            // stop briefly, then a small forward "clear" move
            movement_Control(0, 0);
            movement_Control(1, 0);
            vTaskDelay(pdMS_TO_TICKS(INTERSECTION_STOP_MS));

            int32_t forward = speed255_To_MaxInput(IR_SPEED) / 2;
            movement_Control(0, (int16_t)forward);
            movement_Control(1, (int16_t)forward);
            vTaskDelay(pdMS_TO_TICKS(INTERSECTION_FORWARD_MS));

            // reset last sign preference (prefer right by default)
            last_error_sign = 1;

            vTaskDelay(TASK_SLEEP_MS);
            continue;
        }

        // Normal tracking if at least one sensor sees the line
        if (sum > 0)
        {
            // centroid-like error for 3 sensors: weights L=-1, C=0, R=+1
            float vL = L ? 1.0f : 0.0f;
            float vC = C ? 1.0f : 0.0f;
            float vR = R ? 1.0f : 0.0f;
            float error = (vR - vL) / (float)sum; // -1..+1

            if (error > 0.01f)
                last_error_sign = 1;
            else if (error < -0.01f)
                last_error_sign = -1;

            // compute wheel outputs
            int32_t speedMax = speed255_To_MaxInput(IR_SPEED);
            int32_t base = speedMax / 2;  // nominal forward baseline
            int32_t scale = speedMax / 2; // correction scale

            float corr = error * (float)scale;
            int32_t left = (int32_t)roundf((float)base + corr);
            int32_t right = (int32_t)roundf((float)base - corr);

            // clamp
            if (left < 0)
                left = 0;
            if (right < 0)
                right = 0;
            if (left > PWM_MAX_INPUT)
                left = PWM_MAX_INPUT;
            if (right > PWM_MAX_INPUT)
                right = PWM_MAX_INPUT;

            movement_Control(0, (int16_t)left);
            movement_Control(1, (int16_t)right);

            vTaskDelay(TASK_SLEEP_MS);
            continue;
        }

        // Line lost -> search by rotating in place toward last known side
        int32_t speedMax = speed255_To_MaxInput(IR_SPEED);
        int32_t spin = (int32_t)roundf((float)speedMax * SEARCH_SPEED_FRAC);

        if (last_error_sign >= 0)
        {
            // rotate right: left forward, right reverse
            movement_Control(0, (int16_t)spin);
            movement_Control(1, (int16_t)-spin);
        }
        else
        {
            // rotate left
            movement_Control(0, (int16_t)-spin);
            movement_Control(1, (int16_t)spin);
        }

        vTaskDelay(TASK_SLEEP_MS);
    } // while
}

// start the task (call from setup)
void start_Line_Tracer()
{
    if (s_lineTracerTaskHandle != nullptr)
        return; // already running
    BaseType_t r = xTaskCreate(
        line_Tracer_Task,
        "LineTracer",
        4096, // stack (bytes vary by port; 4096 is usually safe)
        nullptr,
        2, // priority
        &s_lineTracerTaskHandle);
    if (r != pdPASS)
    {
        Serial.println("Line Tracer - Cannot Initialize");
        s_lineTracerTaskHandle = nullptr;
    }
    else
    {
        Serial.println("Line Tracer - Initialized");
    }
    vTaskDelay(pdMS_TO_TICKS(250));
}

void stop_Line_Tracer()
{
    if (s_lineTracerTaskHandle == nullptr)
        return;
    vTaskDelete(s_lineTracerTaskHandle);
    s_lineTracerTaskHandle = nullptr;
    // ensure motors are stopped
    movement_Control(0, 0);
    movement_Control(1, 0);
}
