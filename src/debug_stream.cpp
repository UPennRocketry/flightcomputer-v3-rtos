#include "debug_stream.h"

#include <Arduino.h>
#include <arduino_freertos.h>
#include <queue.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "app_config.h"
#include "rfm95.h"
#include "rtos_context.h"

namespace
{

#if FLIGHT_DEBUG_MODE
  const char *phaseName(flight_phase_t phase)
  {
    switch (phase)
    {
    case flight_phase_t::STARTUP:
      return "STARTUP";
    case flight_phase_t::BOOST:
      return "BOOST";
    case flight_phase_t::COAST:
      return "COAST";
    case flight_phase_t::DESCENT:
      return "DESCENT";
    case flight_phase_t::TOUCHDOWN:
      return "TOUCHDOWN";
    case flight_phase_t::COMPLETE:
      return "COMPLETE";
    default:
      return "UNKNOWN";
    }
  }

  void serialPrintLine(const char *line)
  {
    if (g_serialMutex != nullptr)
    {
      xSemaphoreTake(g_serialMutex, portMAX_DELAY);
    }

    Serial.println(line);

    if (g_serialMutex != nullptr)
    {
      xSemaphoreGive(g_serialMutex);
    }
  }

  bool validPhaseValue(long phase)
  {
    return (phase >= 0L) && (phase <= static_cast<long>(flight_phase_t::COMPLETE));
  }

  void setDebugPhaseOverride(bool enabled, uint8_t phase)
  {
    if (g_stateMutex != nullptr)
    {
      xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    }

    g_debugForcePhaseEnabled = enabled;
    g_debugForcedPhase = phase;

    if (g_stateMutex != nullptr)
    {
      xSemaphoreGive(g_stateMutex);
    }
  }

  void writeCommandEvent(const char *status, const char *detail, bool enabled, uint8_t phase)
  {
    char line[192] = {0};
    snprintf(line,
             sizeof(line),
             "DBG,{\"event\":\"debug_phase_command\",\"status\":\"%s\",\"detail\":\"%s\","
             "\"override_enabled\":%u,\"forced_phase\":%u,\"forced_phase_name\":\"%s\"}",
             status,
             detail,
             enabled ? 1U : 0U,
             static_cast<unsigned int>(phase),
             enabled ? phaseName(static_cast<flight_phase_t>(phase)) : "AUTO");
    serialPrintLine(line);
  }

  void handleCommandLine(const char *line)
  {
    constexpr const char *kPhasePrefix = "CMD,PHASE,";
    constexpr size_t kPhasePrefixLen = 10U;

    if (strncmp(line, kPhasePrefix, kPhasePrefixLen) != 0)
    {
      writeCommandEvent("error", "unknown_command", g_debugForcePhaseEnabled, g_debugForcedPhase);
      return;
    }

    const char *value = line + kPhasePrefixLen;
    if (strcmp(value, "AUTO") == 0)
    {
      setDebugPhaseOverride(false, g_debugForcedPhase);
      writeCommandEvent("ok", "auto", false, g_debugForcedPhase);
      return;
    }

    char *end = nullptr;
    const long phase = strtol(value, &end, 10);
    if ((end == value) || (*end != '\0') || !validPhaseValue(phase))
    {
      writeCommandEvent("error", "invalid_phase", g_debugForcePhaseEnabled, g_debugForcedPhase);
      return;
    }

    const uint8_t forced_phase = static_cast<uint8_t>(phase);
    setDebugPhaseOverride(true, forced_phase);
    writeCommandEvent("ok", "forced_phase", true, forced_phase);
  }

  void processSerialCommands()
  {
    static char command_buffer[64] = {0};
    static size_t command_len = 0U;

    while (Serial.available() > 0)
    {
      const int value = Serial.read();
      if (value < 0)
      {
        return;
      }

      const char c = static_cast<char>(value);
      if (c == '\r')
      {
        continue;
      }
      if (c == '\n')
      {
        command_buffer[command_len] = '\0';
        if (command_len > 0U)
        {
          handleCommandLine(command_buffer);
        }
        command_len = 0U;
        continue;
      }

      if ((command_len + 1U) < sizeof(command_buffer))
      {
        command_buffer[command_len++] = c;
      }
      else
      {
        command_len = 0U;
        writeCommandEvent("error", "command_too_long", g_debugForcePhaseEnabled, g_debugForcedPhase);
      }
    }
  }
#endif

} // namespace

namespace debug_stream
{

  void writeRecord(const debug_record_t &record)
  {
#if FLIGHT_DEBUG_MODE
    char line[4608] = {0};
    const flight_sample_t &sample = record.sample;
    const flight_state_t &state = record.state;
    const rfm95::metrics_t radio = rfm95::metrics();
    const system_status_t system = g_systemStatus;

    const int written = snprintf(
        line,
        sizeof(line),
        "DBG,{\"t\":%lu,\"phase\":%u,\"phase_name\":\"%s\","
        "\"temp_c\":%.3f,\"pressure_pa\":%.3f,\"baro_alt_m\":%.3f,"
        "\"accel\":[%.5f,%.5f,%.5f],\"gyro\":[%.6f,%.6f,%.6f],\"mag\":[%.1f,%.1f,%.1f],"
        "\"q\":[%.7f,%.7f,%.7f,%.7f],"
        "\"pos\":[%.5f,%.5f,%.5f],\"vel\":[%.5f,%.5f,%.5f],"
        "\"state_alt_m\":%.5f,\"state_vel_mps\":%.5f,"
        "\"deployment_alt_m\":%.5f,\"deployment_vel_mps\":%.5f,"
        "\"descent\":%u,\"pyro_armed\":%u,\"deployed\":%u,\"pyro_active\":%u,"
        "\"gps\":{\"valid\":%u,\"position_valid\":%u,\"altitude_valid\":%u,"
        "\"velocity_valid\":%u,\"vertical_velocity_valid\":%u,"
        "\"gnss_fix_ok\":%u,\"invalid_llh\":%u,\"time_valid\":%u,"
        "\"fix_type\":%u,\"satellites\":%u,\"timestamp_ms\":%lu,\"i_tow_ms\":%lu,"
        "\"latitude_deg\":%.7f,\"longitude_deg\":%.7f,"
        "\"altitude_m\":%.3f,\"altitude_msl_m\":%.3f,\"altitude_ellipsoid_m\":%.3f,"
        "\"vertical_velocity_mps\":%.3f,"
        "\"position\":[%.3f,%.3f,%.3f],\"velocity\":[%.3f,%.3f,%.3f],"
        "\"horizontal_accuracy_m\":%.3f,\"vertical_accuracy_m\":%.3f,"
        "\"speed_accuracy_mps\":%.3f,\"ground_speed_mps\":%.3f,"
        "\"heading_motion_deg\":%.3f,\"heading_accuracy_deg\":%.3f,\"p_dop\":%.2f},"
        "\"estimator\":{\"ahrs_initialized\":%u,\"ahrs_gyro_only\":%u,\"ahrs_using_mag\":%u,"
        "\"ahrs_accel_rejected\":%u,\"ahrs_mag_rejected\":%u,"
        "\"imu_calibration_active\":%u,\"imu_calibration_sample_accepted\":%u,"
        "\"imu_calibration_complete\":%u,\"imu_bias_frozen\":%u,\"imu_bias_applied\":%u,"
        "\"imu_calibration_samples\":%lu,"
        "\"body_up\":[%.6f,%.6f,%.6f],"
        "\"accel_bias_mss\":[%.6f,%.6f,%.6f],\"gyro_bias_rads\":[%.8f,%.8f,%.8f],"
        "\"accel_bias_drift_mss\":[%.6f,%.6f,%.6f],\"gyro_bias_drift_rads\":[%.8f,%.8f,%.8f],"
        "\"baro_update_accepted\":%u,\"gps_position_update_accepted\":%u,"
        "\"gps_velocity_update_accepted\":%u,\"gps_outage\":%u,"
        "\"last_gps_position_update_ms\":%lu,\"last_gps_velocity_update_ms\":%lu,"
        "\"cov_pos\":[%.5f,%.5f,%.5f],\"cov_vel\":[%.5f,%.5f,%.5f],"
        "\"normalized_baro_innovation\":%.3f,"
        "\"normalized_gps_position_innovation\":%.3f,"
        "\"normalized_gps_velocity_innovation\":%.3f},"
        "\"vertical_filter\":{\"initialized\":%u,\"baro_update_accepted\":%u,"
        "\"altitude_m\":%.5f,\"velocity_mps\":%.5f,\"vertical_accel_mss\":%.5f,"
        "\"accel_bias_mss\":%.5f,\"cov_altitude_m2\":%.5f,"
        "\"cov_velocity_m2ps2\":%.5f,\"cov_accel_bias_m2ps4\":%.5f,"
        "\"normalized_baro_innovation\":%.3f},"
        "\"radio\":{\"enabled\":%u,\"initialized\":%u,\"busy\":%u,"
        "\"tx_start_count\":%lu,\"tx_done_count\":%lu,\"tx_fail_count\":%lu,"
        "\"last_payload_len\":%u,\"last_tx_start_ms\":%lu,\"last_tx_done_ms\":%lu,"
        "\"last_tx_duration_ms\":%lu},"
        "\"system\":{\"debug_mode\":%u,\"pyros_disabled\":%u,"
        "\"mutexes_initialized\":%u,\"queues_initialized\":%u,\"debug_queue_initialized\":%u,"
        "\"sensors_initialized\":%u,\"flight_logic_initialized\":%u,"
        "\"gps_config_enabled\":%u,\"gps_initialized\":%u,\"gps_task_started\":%u,"
        "\"radio_runtime_enabled\":%u,\"radio_initialized\":%u,\"radio_task_started\":%u,"
        "\"radio_skipped_debug\":%u,\"sd_config_enabled\":%u,\"sd_initialized\":%u,"
        "\"sd_logging_enabled\":%u,\"magnetometer_config_enabled\":%u,"
        "\"sensor_task_started\":%u,\"flight_task_started\":%u,\"logger_task_started\":%u,"
        "\"debug_task_started\":%u,\"debug_phase_override\":%u,\"debug_forced_phase\":%u}}",
        static_cast<unsigned long>(sample.timestamp_ms),
        static_cast<unsigned int>(state.phase),
        phaseName(state.phase),
        static_cast<double>(sample.temperature_c),
        static_cast<double>(sample.pressure_pa),
        static_cast<double>(sample.altitude_m),
        static_cast<double>(sample.accel_mss[0]),
        static_cast<double>(sample.accel_mss[1]),
        static_cast<double>(sample.accel_mss[2]),
        static_cast<double>(sample.gyro_rads[0]),
        static_cast<double>(sample.gyro_rads[1]),
        static_cast<double>(sample.gyro_rads[2]),
        static_cast<double>(sample.mag_raw[0]),
        static_cast<double>(sample.mag_raw[1]),
        static_cast<double>(sample.mag_raw[2]),
        state.orientation_q.w,
        state.orientation_q.x,
        state.orientation_q.y,
        state.orientation_q.z,
        state.position_m.x,
        state.position_m.y,
        state.position_m.z,
        state.velocity_mps_3d.x,
        state.velocity_mps_3d.y,
        state.velocity_mps_3d.z,
        state.altitude_m,
        state.velocity_mps,
        state.deployment_altitude_m,
        state.deployment_velocity_mps,
        state.descent ? 1U : 0U,
        state.pyro_armed ? 1U : 0U,
        state.parachute_deployed ? 1U : 0U,
        state.pyro_active ? 1U : 0U,
        state.gps.valid ? 1U : 0U,
        state.gps.position_valid ? 1U : 0U,
        state.gps.altitude_valid ? 1U : 0U,
        state.gps.velocity_valid ? 1U : 0U,
        state.gps.vertical_velocity_valid ? 1U : 0U,
        state.gps.gnss_fix_ok ? 1U : 0U,
        state.gps.invalid_llh ? 1U : 0U,
        state.gps.time_valid ? 1U : 0U,
        static_cast<unsigned int>(state.gps.fix_type),
        static_cast<unsigned int>(state.gps.satellites),
        static_cast<unsigned long>(state.gps.timestamp_ms),
        static_cast<unsigned long>(state.gps.i_tow_ms),
        state.gps.latitude_deg,
        state.gps.longitude_deg,
        state.gps.altitude_m,
        state.gps.altitude_msl_m,
        state.gps.altitude_ellipsoid_m,
        state.gps.vertical_velocity_mps,
        state.gps.position_m.x,
        state.gps.position_m.y,
        state.gps.position_m.z,
        state.gps.velocity_mps.x,
        state.gps.velocity_mps.y,
        state.gps.velocity_mps.z,
        state.gps.horizontal_accuracy_m,
        state.gps.vertical_accuracy_m,
        state.gps.speed_accuracy_mps,
        state.gps.ground_speed_mps,
        state.gps.heading_motion_deg,
        state.gps.heading_accuracy_deg,
        state.gps.p_dop,
        state.estimator.ahrs_initialized ? 1U : 0U,
        state.estimator.ahrs_gyro_only ? 1U : 0U,
        state.estimator.ahrs_using_mag ? 1U : 0U,
        state.estimator.ahrs_accel_rejected ? 1U : 0U,
        state.estimator.ahrs_mag_rejected ? 1U : 0U,
        state.estimator.imu_calibration_active ? 1U : 0U,
        state.estimator.imu_calibration_sample_accepted ? 1U : 0U,
        state.estimator.imu_calibration_complete ? 1U : 0U,
        state.estimator.imu_bias_frozen ? 1U : 0U,
        state.estimator.imu_bias_applied ? 1U : 0U,
        static_cast<unsigned long>(state.estimator.imu_calibration_samples),
        state.estimator.body_up[0],
        state.estimator.body_up[1],
        state.estimator.body_up[2],
        state.estimator.accel_bias_mss[0],
        state.estimator.accel_bias_mss[1],
        state.estimator.accel_bias_mss[2],
        state.estimator.gyro_bias_rads[0],
        state.estimator.gyro_bias_rads[1],
        state.estimator.gyro_bias_rads[2],
        state.estimator.accel_bias_drift_mss[0],
        state.estimator.accel_bias_drift_mss[1],
        state.estimator.accel_bias_drift_mss[2],
        state.estimator.gyro_bias_drift_rads[0],
        state.estimator.gyro_bias_drift_rads[1],
        state.estimator.gyro_bias_drift_rads[2],
        state.estimator.baro_update_accepted ? 1U : 0U,
        state.estimator.gps_position_update_accepted ? 1U : 0U,
        state.estimator.gps_velocity_update_accepted ? 1U : 0U,
        state.estimator.gps_outage ? 1U : 0U,
        static_cast<unsigned long>(state.estimator.last_gps_position_update_ms),
        static_cast<unsigned long>(state.estimator.last_gps_velocity_update_ms),
        state.estimator.covariance_position_m2[0],
        state.estimator.covariance_position_m2[1],
        state.estimator.covariance_position_m2[2],
        state.estimator.covariance_velocity_m2ps2[0],
        state.estimator.covariance_velocity_m2ps2[1],
        state.estimator.covariance_velocity_m2ps2[2],
        state.estimator.normalized_baro_innovation,
        state.estimator.normalized_gps_position_innovation,
        state.estimator.normalized_gps_velocity_innovation,
        state.vertical_filter.initialized ? 1U : 0U,
        state.vertical_filter.baro_update_accepted ? 1U : 0U,
        state.vertical_filter.altitude_m,
        state.vertical_filter.velocity_mps,
        state.vertical_filter.vertical_accel_mss,
        state.vertical_filter.accel_bias_mss,
        state.vertical_filter.covariance_altitude_m2,
        state.vertical_filter.covariance_velocity_m2ps2,
        state.vertical_filter.covariance_accel_bias_m2ps4,
        state.vertical_filter.normalized_baro_innovation,
        system.radio_runtime_enabled ? 1U : 0U,
        radio.initialized ? 1U : 0U,
        radio.busy ? 1U : 0U,
        static_cast<unsigned long>(radio.tx_start_count),
        static_cast<unsigned long>(radio.tx_done_count),
        static_cast<unsigned long>(radio.tx_fail_count),
        static_cast<unsigned int>(radio.last_payload_len),
        static_cast<unsigned long>(radio.last_tx_start_ms),
        static_cast<unsigned long>(radio.last_tx_done_ms),
        static_cast<unsigned long>(radio.last_tx_duration_ms),
        system.debug_mode ? 1U : 0U,
        system.pyros_disabled ? 1U : 0U,
        system.mutexes_initialized ? 1U : 0U,
        system.queues_initialized ? 1U : 0U,
        system.debug_queue_initialized ? 1U : 0U,
        system.sensors_initialized ? 1U : 0U,
        system.flight_logic_initialized ? 1U : 0U,
        system.gps_config_enabled ? 1U : 0U,
        system.gps_initialized ? 1U : 0U,
        system.gps_task_started ? 1U : 0U,
        system.radio_runtime_enabled ? 1U : 0U,
        system.radio_initialized ? 1U : 0U,
        system.radio_task_started ? 1U : 0U,
        system.radio_skipped_debug ? 1U : 0U,
        system.sd_config_enabled ? 1U : 0U,
        system.sd_initialized ? 1U : 0U,
        system.sd_logging_enabled ? 1U : 0U,
        system.magnetometer_config_enabled ? 1U : 0U,
        system.sensor_task_started ? 1U : 0U,
        system.flight_task_started ? 1U : 0U,
        system.logger_task_started ? 1U : 0U,
        system.debug_task_started ? 1U : 0U,
        g_debugForcePhaseEnabled ? 1U : 0U,
        static_cast<unsigned int>(g_debugForcedPhase));

    if ((written > 0) && (static_cast<size_t>(written) < sizeof(line)))
    {
      serialPrintLine(line);
    }
#else
    (void)record;
#endif
  }

  void task(void *)
  {
#if FLIGHT_DEBUG_MODE
    serialPrintLine("DBG,{\"event\":\"debug_stream_start\",\"pyros_disabled\":1}");
    debug_record_t record;

    while (true)
    {
      processSerialCommands();
      if ((g_debugQueue != nullptr) &&
          (xQueueReceive(g_debugQueue, &record, pdMS_TO_TICKS(10)) == pdPASS))
      {
        writeRecord(record);
      }
    }
#else
    vTaskDelete(nullptr);
#endif
  }

} // namespace debug_stream
