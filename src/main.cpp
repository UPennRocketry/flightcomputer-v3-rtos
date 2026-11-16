#include <Arduino.h>
#include <arduino_freertos.h>
#include <queue.h>
#include <semphr.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_types.h"
#include "debug_stream.h"
#include "flight_logic.h"
#include "gps.h"
#include "logger.h"
#include "persist.h"
#include "rfm95.h"
#include "rtos_context.h"
#include "sensors.h"
#include "supervisor.h"
#include "telemetry.h"

// ---------------------------------------------------------------------------
// Task layout (priority, highest first)
//   5 supervisor  50 ms   feeds watchdog on heartbeats, backup deploy timer, pyro off
//   4 sensor      10 ms   reads IMU/baro/mag (non-blocking baro), pushes sample
//   3 flight      event   estimator + state machine + pyro decision, per sample
//   2 radio        5 ms   polls LoRa; state packet @5 Hz, health packet @1 Hz
//   2 logger      event   CSV to SD, with pre-launch ring buffer
//   2 gps         10 ms   u-blox PVT poll into shared state
//   1 debug       event   USB JSON stream (debug build only)
// ---------------------------------------------------------------------------

QueueHandle_t g_flightQueue = nullptr;
QueueHandle_t g_logQueue = nullptr;
QueueHandle_t g_debugQueue = nullptr;

SemaphoreHandle_t g_stateMutex = nullptr;
SemaphoreHandle_t g_spi0Mutex = nullptr;
SemaphoreHandle_t g_spi1Mutex = nullptr;
SemaphoreHandle_t g_serialMutex = nullptr;

flight_state_t g_flightState = {};
flight_sample_t g_latestSample = {};
system_status_t g_systemStatus = {};
bool g_debugForcePhaseEnabled = false;
uint8_t g_debugForcedPhase = static_cast<uint8_t>(flight_phase_t::STARTUP);

namespace
{

  // Push to a queue; if it is full, drop the oldest item rather than block the
  // producer. The flight-critical producer (sensor task) must never wait on a
  // slow consumer.
  void queueSendLatest(QueueHandle_t queue, const void *item)
  {
    if (xQueueSendToBack(queue, item, 0) != pdPASS)
    {
      uint8_t drop_buffer[(sizeof(debug_record_t) > sizeof(log_record_t)) ? sizeof(debug_record_t)
                                                                          : sizeof(log_record_t)];
      xQueueReceive(queue, drop_buffer, 0);
      xQueueSendToBack(queue, item, 0);
    }
  }

  void serialBootBanner()
  {
    serialPrintlnSafe("Booting Teensy 4.1 flight computer (Arduino + FreeRTOS)...");
    serialPrintfSafe("Sensor rate %lu Hz, telemetry %lu ms / health %lu ms\n",
                     static_cast<unsigned long>(1000U / app_config::kSensorPeriodMs),
                     static_cast<unsigned long>(app_config::kTelemetryPeriodMs),
                     static_cast<unsigned long>(app_config::kHealthPeriodMs));
    if (app_config::kFlightDebugMode)
    {
      serialPrintlnSafe("Debug mode enabled: USB debug stream active and pyros disabled.");
    }
    if (!app_config::kEnableWatchdog)
    {
      serialPrintlnSafe("WARNING: hardware watchdog disabled by build flag.");
    }
    if (app_config::kBackupDeployAfterLaunchMs == 0U)
    {
      serialPrintlnSafe("WARNING: backup deployment timer disabled (kBackupDeployAfterLaunchMs = 0).");
    }
  }

  bool taskStarted(BaseType_t result) { return result == pdPASS; }

  void startupBeep()
  {
    pinMode(app_config::kBuzzerPin, arduino::OUTPUT);

    const uint32_t half_period_us =
        1000000UL / (static_cast<uint32_t>(app_config::kStartupBuzzerHz) * 2UL);
    const uint32_t duration_us = app_config::kStartupBuzzerMs * 1000UL;
    const uint32_t start_us = micros();

    while ((micros() - start_us) < duration_us)
    {
      digitalWrite(app_config::kBuzzerPin, arduino::HIGH);
      delayMicroseconds(half_period_us);
      digitalWrite(app_config::kBuzzerPin, arduino::LOW);
      delayMicroseconds(half_period_us);
    }

    digitalWrite(app_config::kBuzzerPin, arduino::LOW);
  }

  // ---------------- tasks ----------------

  void sensorTask(void *)
  {
    TickType_t last_wake = xTaskGetTickCount();

    while (true)
    {
      flight_sample_t sample;
      if (sensors::readSample(sample))
      {
        queueSendLatest(g_flightQueue, &sample);
      }
      supervisor::heartbeatSensor();
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(app_config::kSensorPeriodMs));
    }
  }

  void flightTask(void *)
  {
    flight_sample_t sample;
    uint32_t last_debug_stream_ms = 0U;
    bool first_debug_record = true;

    while (true)
    {
      if (xQueueReceive(g_flightQueue, &sample, portMAX_DELAY) != pdPASS)
      {
        continue;
      }

      debug_record_t debug_record;
      log_record_t log_record;

      const uint32_t t0_us = micros();
      xSemaphoreTake(g_stateMutex, portMAX_DELAY);
      flight_logic::update(sample, g_flightState);
      g_latestSample = sample;
      log_record.sample = sample;
      log_record.state = g_flightState;
      if (app_config::kFlightDebugMode)
      {
        debug_record.sample = sample;
        debug_record.state = g_flightState;
      }
      xSemaphoreGive(g_stateMutex);
      supervisor::heartbeatFlight(micros() - t0_us);

      queueSendLatest(g_logQueue, &log_record);

      if (app_config::kFlightDebugMode && (g_debugQueue != nullptr) &&
          (first_debug_record ||
           ((sample.timestamp_ms - last_debug_stream_ms) >= app_config::kDebugStreamPeriodMs)))
      {
        queueSendLatest(g_debugQueue, &debug_record);
        last_debug_stream_ms = sample.timestamp_ms;
        first_debug_record = false;
      }
    }
  }

  void loggerTask(void *)
  {
    const bool logger_init_ok = logger::init();
    g_systemStatus.sd_initialized = logger_init_ok;
    g_systemStatus.sd_logging_enabled = logger::isEnabled();
    if (!logger_init_ok)
    {
      serialPrintlnSafe("SD logging unavailable; continuing without SD backend.");
    }
    else if (logger::isEnabled())
    {
      serialPrintlnSafe("SD logging enabled!");
    }
    else
    {
      serialPrintlnSafe("SD logging disabled in config.");
    }

    log_record_t record;
    while (true)
    {
      if (xQueueReceive(g_logQueue, &record, portMAX_DELAY) == pdPASS)
      {
        logger::writeSample(record.sample, record.state);
      }
    }
  }

  void collectTelemetryInputs(telemetry::inputs_t &in)
  {
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    in.state = g_flightState;
    in.sample = g_latestSample;
    xSemaphoreGive(g_stateMutex);
    in.system = g_systemStatus;
    in.supervisor = supervisor::status();
    const rfm95::metrics_t m = rfm95::metrics();
    in.radio_tx_count = m.tx_done_count;
    in.radio_tx_fail = m.tx_fail_count;
  }

  void radioTask(void *)
  {
    uint32_t last_state_ms = 0U;
    uint32_t last_health_ms = 0U;
    bool first = true;

    while (true)
    {
      rfm95::service();

      const uint32_t now_ms = millis();
      if (!rfm95::isBusy())
      {
        const bool health_due = first || ((now_ms - last_health_ms) >= app_config::kHealthPeriodMs);
        const bool state_due = first || ((now_ms - last_state_ms) >= app_config::kTelemetryPeriodMs);

        if (health_due || state_due)
        {
          telemetry::inputs_t in;
          collectTelemetryInputs(in);
          radio_packet_t packet;
          // State packets have priority; health goes out when a state slot is free.
          if (state_due)
          {
            telemetry::buildStateRadioPacket(now_ms, in, packet);
            if (rfm95::startTx(packet.data, packet.len))
            {
              last_state_ms = now_ms;
              first = false;
            }
          }
          else
          {
            telemetry::buildHealthRadioPacket(now_ms, in, packet);
            if (rfm95::startTx(packet.data, packet.len))
            {
              last_health_ms = now_ms;
            }
          }
        }
      }

      vTaskDelay(pdMS_TO_TICKS(app_config::kRadioPollPeriodMs));
    }
  }

  void fatalBlink()
  {
    pinMode(app_config::kLedPin, arduino::OUTPUT);
    while (true)
    {
      supervisor::feed(); // stay here visibly instead of watchdog boot-looping
      digitalWrite(app_config::kLedPin, arduino::HIGH);
      delay(150);
      digitalWrite(app_config::kLedPin, arduino::LOW);
      delay(150);
    }
  }

  // Decide whether this boot should resume a flight. Only a watchdog reset with
  // a valid in-flight record qualifies; anything else clears a stale record.
  bool decideRestore(persist::record_t &rec)
  {
    g_systemStatus.reset_cause = persist::readAndClearResetCause();
    g_systemStatus.persistence_enabled = app_config::kEnablePersistence;

    const bool have_record = persist::load(rec);
    if (!have_record)
    {
      return false;
    }
    const bool in_flight = persist::recordIsInFlight(rec);
    const bool wdog = persist::resetCauseWasWatchdog(g_systemStatus.reset_cause);

    if (in_flight && wdog)
    {
      return true;
    }
    if (in_flight && !wdog)
    {
      serialPrintfSafe("Stale in-flight record found (phase %u) after non-watchdog reset (cause 0x%02X); clearing.\n",
                       static_cast<unsigned int>(rec.phase),
                       static_cast<unsigned int>(g_systemStatus.reset_cause));
    }
    persist::clear();
    return false;
  }

} // namespace

void serialPrintfSafe(const char *fmt, ...)
{
  if (g_serialMutex != nullptr)
  {
    xSemaphoreTake(g_serialMutex, portMAX_DELAY);
  }

  char buf[256] = {0};
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);

  if (g_serialMutex != nullptr)
  {
    xSemaphoreGive(g_serialMutex);
  }
}

void serialPrintlnSafe(const char *s)
{
  if (g_serialMutex != nullptr)
  {
    xSemaphoreTake(g_serialMutex, portMAX_DELAY);
  }
  Serial.println(s);
  if (g_serialMutex != nullptr)
  {
    xSemaphoreGive(g_serialMutex);
  }
}

void setup()
{
  // Pyro outputs low before anything else, every boot.
  pinMode(app_config::kPyro1Pin, arduino::OUTPUT);
  pinMode(app_config::kPyro2Pin, arduino::OUTPUT);
  digitalWrite(app_config::kPyro1Pin, arduino::LOW);
  digitalWrite(app_config::kPyro2Pin, arduino::LOW);
  pinMode(app_config::kLedPin, arduino::OUTPUT);
  digitalWrite(app_config::kLedPin, arduino::HIGH);

  // Arm the watchdog as early as possible so a hang during init also recovers.
  supervisor::init();
  g_systemStatus.watchdog_enabled = app_config::kEnableWatchdog;

  // Work out before the beep whether we are resuming a flight: if so, skip the
  // 1 s beep and get the estimator running.
  persist::record_t restore_rec;
  const bool restoring = decideRestore(restore_rec);
  if (!restoring)
  {
    startupBeep();
  }
  supervisor::feed();

  Serial.begin(115200);
  delay(250);

  g_systemStatus.debug_mode = app_config::kFlightDebugMode;
  g_systemStatus.pyros_disabled = app_config::kDisablePyros;
  g_systemStatus.gps_config_enabled = app_config::kEnableGps;
  g_systemStatus.sd_config_enabled = app_config::kEnableSdLogging;
  g_systemStatus.magnetometer_config_enabled = app_config::kEnableMagnetometer;
  g_systemStatus.radio_runtime_enabled = !app_config::kFlightDebugMode;
  g_systemStatus.radio_skipped_debug = app_config::kFlightDebugMode;

  g_stateMutex = xSemaphoreCreateMutex();
  g_spi0Mutex = xSemaphoreCreateMutex();
  g_spi1Mutex = xSemaphoreCreateMutex();
  g_serialMutex = xSemaphoreCreateMutex();
  g_flightQueue = xQueueCreate(app_config::kSampleQueueDepth, sizeof(flight_sample_t));
  g_logQueue = xQueueCreate(app_config::kLogQueueDepth, sizeof(log_record_t));
  if (app_config::kFlightDebugMode)
  {
    g_debugQueue = xQueueCreate(app_config::kDebugQueueDepth, sizeof(debug_record_t));
  }

  g_systemStatus.mutexes_initialized = (g_stateMutex != nullptr) && (g_spi0Mutex != nullptr) &&
                                       (g_spi1Mutex != nullptr) && (g_serialMutex != nullptr);
  g_systemStatus.queues_initialized = (g_flightQueue != nullptr) && (g_logQueue != nullptr);
  g_systemStatus.debug_queue_initialized = !app_config::kFlightDebugMode || (g_debugQueue != nullptr);

  if (!g_systemStatus.mutexes_initialized || !g_systemStatus.queues_initialized ||
      !g_systemStatus.debug_queue_initialized)
  {
    fatalBlink();
  }

  serialBootBanner();
  if (restoring)
  {
    serialPrintfSafe("WATCHDOG RESET IN FLIGHT: resuming phase %u, %lu ms flown, restore #%u\n",
                     static_cast<unsigned int>(restore_rec.phase),
                     static_cast<unsigned long>(restore_rec.launch_elapsed_ms),
                     static_cast<unsigned int>(restore_rec.restore_count + 1U));
  }

  const bool sensors_ok = sensors::init(g_spi0Mutex, restoring ? &restore_rec.baseline : nullptr);
  if (!sensors_ok && !restoring)
  {
    serialPrintlnSafe("FATAL - Sensor init failed.");
    fatalBlink();
  }
  else if (!sensors_ok)
  {
    // In flight with a broken sensor we cannot navigate; the backup timer and
    // restore-limit path below are the only remaining ways to get the chute out.
    serialPrintlnSafe("Sensor init failed while resuming flight; continuing on timers only.");
  }
  else
  {
    serialPrintlnSafe("Sensors initialized!");
    g_systemStatus.sensors_initialized = true;
  }

  supervisor::feed();
  flight_logic::init();
  g_systemStatus.flight_logic_initialized = true;

  if (restoring)
  {
    g_systemStatus.restore_count = static_cast<uint8_t>(restore_rec.restore_count + 1U);
    g_systemStatus.restored_from_persist = true;
    flight_logic::restore(restore_rec, g_flightState, millis());
    // Record the new restore count immediately so a boot loop is counted.
    persist::save(g_flightState, sensors::baseline(), millis(), g_systemStatus.restore_count);

    if (g_systemStatus.restore_count >= app_config::kMaxInFlightRestores)
    {
      serialPrintlnSafe("Restore limit reached while in flight: deploying parachute now.");
      flight_logic::forceDeploy(g_flightState, millis(), deploy_reason_t::RESTORE_LIMIT);
    }
  }

  supervisor::feed();
  const bool gps_enabled = gps::init();
  supervisor::feed();
  g_systemStatus.gps_initialized = gps_enabled;
  serialPrintlnSafe(gps_enabled ? "GPS initialized!" : "GPS unavailable; continuing without GPS.");

  bool radio_enabled = false;
  if (app_config::kFlightDebugMode)
  {
    g_systemStatus.radio_runtime_enabled = false;
    g_systemStatus.radio_skipped_debug = true;
    serialPrintlnSafe("Debug mode: skipping RFM95 radio init.");
  }
  else if (!rfm95::init(g_spi1Mutex))
  {
    g_systemStatus.radio_runtime_enabled = false;
    serialPrintlnSafe("RFM95 unavailable; continuing without radio telemetry.");
  }
  else
  {
    serialPrintlnSafe("RFM95 initialized!");
    g_systemStatus.radio_initialized = true;
    radio_enabled = true;
  }

  supervisor::feed();
  g_systemStatus.supervisor_task_started = taskStarted(xTaskCreate(
      supervisor::task, "supervisor", app_config::kSupervisorTaskStackWords, nullptr,
      app_config::kSupervisorTaskPriority, nullptr));
  g_systemStatus.sensor_task_started = taskStarted(xTaskCreate(
      sensorTask, "sensor", app_config::kSensorTaskStackWords, nullptr,
      app_config::kSensorTaskPriority, nullptr));
  g_systemStatus.flight_task_started = taskStarted(xTaskCreate(
      flightTask, "flight", app_config::kFlightTaskStackWords, nullptr,
      app_config::kFlightTaskPriority, nullptr));
  g_systemStatus.logger_task_started = taskStarted(xTaskCreate(
      loggerTask, "logger", app_config::kLoggerTaskStackWords, nullptr,
      app_config::kLoggerTaskPriority, nullptr));
  if (radio_enabled)
  {
    g_systemStatus.radio_task_started = taskStarted(xTaskCreate(
        radioTask, "radio", app_config::kRadioTaskStackWords, nullptr,
        app_config::kRadioTaskPriority, nullptr));
  }
  if (gps_enabled)
  {
    g_systemStatus.gps_task_started = taskStarted(xTaskCreate(
        gps::task, "gps", app_config::kGpsTaskStackWords, nullptr,
        app_config::kGpsTaskPriority, nullptr));
  }
  if (app_config::kFlightDebugMode)
  {
    g_systemStatus.debug_task_started = taskStarted(xTaskCreate(
        debug_stream::task, "debug", app_config::kDebugTaskStackWords, nullptr,
        app_config::kDebugTaskPriority, nullptr));
  }

  if (!g_systemStatus.supervisor_task_started || !g_systemStatus.sensor_task_started ||
      !g_systemStatus.flight_task_started)
  {
    serialPrintlnSafe("FATAL - a critical task failed to start.");
    fatalBlink();
  }

  serialPrintlnSafe("Setup complete. Starting FreeRTOS scheduler...");
  digitalWrite(app_config::kLedPin, arduino::LOW);
  vTaskStartScheduler();

  serialPrintlnSafe("Scheduler failed to start.");
  fatalBlink();
}

void loop() {}
