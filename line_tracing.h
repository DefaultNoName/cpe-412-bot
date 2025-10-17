#ifndef LINE_TRACING_H
#define LINE_TRACING_H

// line_tracer.h
#include <Arduino.h>
#include <stdint.h>

// IR sensor pins
static const int IR_PIN_LEFT = 18;   // P18
static const int IR_PIN_CENTER = 19; // P19
static const int IR_PIN_RIGHT = 21;  // P21

// IR behavior constants
static const uint8_t IR_SPEED = 255;         // 0..255 (user requested)
static const float SEARCH_SPEED_FRAC = 0.50; // fraction of IR_SPEED when searching
static const uint32_t INTERSECTION_STOP_MS = 150;
static const uint32_t INTERSECTION_FORWARD_MS = 120;
static const TickType_t TASK_SLEEP_MS = pdMS_TO_TICKS(20); // main loop ~50 Hz

// lineTracingEnabled is defined in main.ino; declare it here for linkage
extern volatile bool lineTracingEnabled;

// start the FreeRTOS task that performs line tracing
void startLineTracer();

// stop the task (optional) - not strictly required, provided for completeness
void stopLineTracer();

#endif // LINE_TRACING_H