#include "supervisor.h"

#include <Arduino.h>
#include <arduino_freertos.h>
#include <semphr.h>

#include "app_config.h"
#include "flight_logic.h"
#include "persist.h"
#include "sensors.h"
#include "rtos_context.h"

// i.MX RT1062 WDOG1 register bits. imxrt.h in the Teensy core defines these;
// the fallbacks keep the file self-contained if a core version lacks one.
#ifndef WDOG_WCR_WT
#define WDOG_WCR_WT(n) ((uint16_t)(((n) & 0xFF) << 8))
#endif
#ifndef WDOG_WCR_WDE
#define WDOG_WCR_WDE ((uint16_t)(1 << 2))
#endif
#ifndef WDOG_WCR_SRS
#define WDOG_WCR_SRS ((uint16_t)(1 << 4))
#endif
#ifndef WDOG_WCR_WDA
#define WDOG_WCR_WDA ((uint16_t)(1 << 5))
#endif
#ifndef WDOG_WCR_WDZST
#define WDOG_WCR_WDZST ((uint16_t)(1 << 0))
#endif

namespace
{

  volatile uint32_t g_sensorBeatMs = 0;
  volatile uint32_t g_flightBeatMs = 0;
  volatile uint32_t g_sensorBeatCount = 0;
  volatile uint32_t g_flightBeatCount = 0;
  volatile uint32_t g_flightUpdateMaxUs = 0;
  volatile uint32_t g_flightUpdateLastUs = 0;

  supervisor_status_t g_status;
  uint32_t g_bootMs = 0;

  void watchdogArm()
  {
    if (!app_config::kEnableWatchdog)
    {
      return;
    }
    // Timeout = (WT + 1) * 0.5 s. Once WDE is set it cannot be cleared except by reset.
    uint32_t wt = (app_config::kWatchdogTimeoutS * 2U);
    wt = (wt == 0U) ? 0U : (wt - 1U);
    if (wt > 0xFFU)
    {
      wt = 0xFFU;
    }
    WDOG1_WCR = WDOG_WCR_WT(wt) | WDOG_WCR_WDE | WDOG_WCR_SRS | WDOG_WCR_WDA | WDOG_WCR_WDZST;
    g_status.watchdog_armed = true;
  }

  void watchdogFeed()
  {
    if (!g_status.watchdog_armed)
    {
      return;
    }
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
    ++g_status.watchdog_feeds;
  }

  bool heartbeatsFresh(uint32_t now_ms)
  {
    if ((now_ms - g_bootMs) < app_config::kHeartbeatGraceAfterBootMs)
    {
      return true; // tasks are still starting up
    }
    const uint32_t sensor_age = now_ms - g_sensorBeatMs;
    const uint32_t flight_age = now_ms - g_flightBeatMs;
    return (sensor_age <= app_config::kHeartbeatSensorMaxMs) &&
           (flight_age <= app_config::kHeartbeatFlightMaxMs);
  }

  void updateRates(uint32_t now_ms)
  {
    static uint32_t last_rate_ms = 0;
    static uint32_t last_sensor_count = 0;
    static uint32_t last_flight_count = 0;

    if ((now_ms - last_rate_ms) >= 1000U)
    {
      const uint32_t sc = g_sensorBeatCount;
      const uint32_t fc = g_flightBeatCount;
      g_status.sensor_rate_hz = sc - last_sensor_count;
      g_status.flight_rate_hz = fc - last_flight_count;
      last_sensor_count = sc;
      last_flight_count = fc;
      g_status.flight_update_max_us = g_flightUpdateMaxUs;
      g_flightUpdateMaxUs = 0;
      last_rate_ms = now_ms;
    }
    g_status.flight_update_last_us = g_flightUpdateLastUs;
    g_status.sensor_last_beat_ms = g_sensorBeatMs;
    g_status.flight_last_beat_ms = g_flightBeatMs;
  }

  // Backup deployment and pyro shut-off. Takes the state mutex briefly; the
  // flight task holds it only for the duration of one update, so this is a
  // short wait at worst.
  void checkPyroTimers(uint32_t now_ms)
  {
    if (g_stateMutex == nullptr)
    {
      return;
    }
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);

    const bool launched = (g_flightState.phase == flight_phase_t::BOOST) ||
                          (g_flightState.phase == flight_phase_t::COAST) ||
                          (g_flightState.phase == flight_phase_t::DESCENT);

    // 1. Backup timer.
    g_status.backup_timer_armed = false;
    g_status.backup_timer_remaining_ms = 0;
    if ((app_config::kBackupDeployAfterLaunchMs > 0U) && launched && !g_flightState.parachute_deployed)
    {
      const uint32_t flown_ms =
          (now_ms - g_flightState.launch_time_ms) + g_flightState.launch_elapsed_before_reset_ms;
      g_status.backup_timer_armed = true;
      if (flown_ms >= app_config::kBackupDeployAfterLaunchMs)
      {
        flight_logic::forceDeploy(g_flightState, now_ms, deploy_reason_t::BACKUP_TIMER);
      }
      else
      {
        g_status.backup_timer_remaining_ms = app_config::kBackupDeployAfterLaunchMs - flown_ms;
      }
    }

    // 2. Keep the persisted record's elapsed-time current so a restore after a
    //    reset knows how long we have really been flying (backup timer).
    static uint32_t last_persist_ms = 0;
    if (launched && ((now_ms - last_persist_ms) >= app_config::kPersistRefreshMs))
    {
      persist::save(g_flightState, sensors::baseline(), now_ms, g_systemStatus.restore_count);
      last_persist_ms = now_ms;
    }

    // 3. Pyro shut-off, independent of the sample stream.
    if (g_flightState.pyro_active &&
        ((now_ms - g_flightState.pyro_fire_time_ms) >= app_config::kPyroOnMs))
    {
      flight_logic::pyroOff(g_flightState);
    }

    xSemaphoreGive(g_stateMutex);
  }

} // namespace

namespace supervisor
{

  void init()
  {
    g_bootMs = millis();
    g_sensorBeatMs = g_bootMs;
    g_flightBeatMs = g_bootMs;
    g_status = {};
    watchdogArm();
    watchdogFeed();
  }

  void feed()
  {
    watchdogFeed();
  }

  void heartbeatSensor()
  {
    g_sensorBeatMs = millis();
    g_sensorBeatCount = g_sensorBeatCount + 1U;
  }

  void heartbeatFlight(uint32_t update_duration_us)
  {
    g_flightBeatMs = millis();
    g_flightBeatCount = g_flightBeatCount + 1U;
    g_flightUpdateLastUs = update_duration_us;
    if (update_duration_us > g_flightUpdateMaxUs)
    {
      g_flightUpdateMaxUs = update_duration_us;
    }
  }

  void step(uint32_t now_ms)
  {
    const bool fresh = heartbeatsFresh(now_ms);
    g_status.heartbeats_ok = fresh;
    if (fresh)
    {
      watchdogFeed();
    }
    else
    {
      ++g_status.heartbeat_faults;
      // Deliberately do not feed: if this persists the watchdog resets the board
      // and the flight resumes from the persisted record.
    }

    checkPyroTimers(now_ms);
    updateRates(now_ms);
  }

  void task(void *)
  {
    TickType_t last_wake = xTaskGetTickCount();

    while (true)
    {
      step(millis());
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(app_config::kSupervisorPeriodMs));
    }
  }

  void testStep()
  {
    step(millis());
  }

  supervisor_status_t status()
  {
    return g_status;
  }

} // namespace supervisor
