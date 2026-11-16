#pragma once

#include <Arduino.h>
#include <arduino_freertos.h>
#include <queue.h>
#include <semphr.h>

#include "app_types.h"

extern QueueHandle_t g_flightQueue;
extern QueueHandle_t g_logQueue;
extern QueueHandle_t g_debugQueue;

extern SemaphoreHandle_t g_stateMutex;
extern SemaphoreHandle_t g_spi0Mutex;
extern SemaphoreHandle_t g_spi1Mutex;
extern SemaphoreHandle_t g_serialMutex;

extern flight_state_t g_flightState;
extern flight_sample_t g_latestSample;   // most recent sensor sample, guarded by g_stateMutex
extern system_status_t g_systemStatus;
extern bool g_debugForcePhaseEnabled;
extern uint8_t g_debugForcedPhase;

void serialPrintfSafe(const char* fmt, ...);
void serialPrintlnSafe(const char* s);
