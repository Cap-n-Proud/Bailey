// Simulator stand-in for the ESP32 task watchdog: counts what the firmware does.
#pragma once
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
typedef struct { uint32_t timeout_ms; uint32_t idle_core_mask; bool trigger_panic; } esp_task_wdt_config_t;
struct SimWatchdog { uint32_t timeoutMs = 0; int added = 0; long feeds = 0; };
extern SimWatchdog simWatchdog;
inline esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t *c) { simWatchdog.timeoutMs = c->timeout_ms; return ESP_OK; }
inline esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t *c) { simWatchdog.timeoutMs = c->timeout_ms; return ESP_OK; }
inline esp_err_t esp_task_wdt_add(void *) { simWatchdog.added++; return ESP_OK; }
inline esp_err_t esp_task_wdt_reset() { simWatchdog.feeds++; return ESP_OK; }
