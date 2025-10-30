// line_tracing.h
#ifndef LINE_TRACING_H
#define LINE_TRACING_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

// IR sensor pins 
#define IR_PIN_LEFT GPIO_NUM_18
#define IR_PIN_CENTER GPIO_NUM_19
#define IR_PIN_RIGHT GPIO_NUM_21

// IR behavior constants
static const uint8_t IR_SPEED = 255;         // 0..255
static const float SEARCH_SPEED_FRAC = 0.80; // fraction of IR_SPEED when searching
static const uint32_t INTERSECTION_STOP_MS = 150;
static const uint32_t INTERSECTION_FORWARD_MS = 120;
static const TickType_t TASK_SLEEP_MS = pdMS_TO_TICKS(30); // main loop ~33 Hz

// line_Tracing_Enabled is defined in main.cpp; declare it here for linkage
extern volatile bool line_Tracing_Enabled;

// start the FreeRTOS task that performs line tracing
void start_Line_Tracer();

// stop the task (optional)
void stop_Line_Tracer();

#endif // LINE_TRACING_H
