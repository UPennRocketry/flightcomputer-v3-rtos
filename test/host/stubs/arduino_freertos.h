#pragma once
#include <stdint.h>
typedef void* SemaphoreHandle_t;
typedef void* QueueHandle_t;
typedef uint32_t TickType_t;
typedef long BaseType_t;
#define pdPASS 1
#define pdTRUE 1
#define portMAX_DELAY 0xFFFFFFFFUL
#define pdMS_TO_TICKS(ms) (ms)
#define pdTICKS_TO_MS(t) (t)
#define taskSCHEDULER_RUNNING 2
inline int xTaskGetSchedulerState() { return taskSCHEDULER_RUNNING; }
extern uint32_t g_host_millis;
inline TickType_t xTaskGetTickCount() { return g_host_millis; }
inline void vTaskDelay(TickType_t t) { g_host_millis += t; }
inline void vTaskDelayUntil(TickType_t* last, TickType_t period) { *last += period; g_host_millis = *last; }
inline int xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdPASS; }
inline int xSemaphoreGive(SemaphoreHandle_t) { return pdPASS; }
