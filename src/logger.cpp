#include "logger.h"

#include <Arduino.h>
#include <SD.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"

namespace
{

  File g_logFile;
  bool g_enabled = false;
  bool g_loggingStopped = false;
  bool g_startupBufferFlushed = false;
  uint32_t g_lineCountSinceFlush = 0;
  char g_logName[24] = {0};
  log_record_t g_startupLogWindow[app_config::kStartupLogWindowSamples] = {};
  size_t g_startupLogCount = 0;
  size_t g_startupLogHead = 0;

  bool openNumberedLog(const char *base_name, File &out_file, char *out_name, size_t out_name_len)
  {
    for (uint16_t i = 0; i < 1000; ++i)
    {
      char candidate[24] = {0};
      if (i == 0)
      {
        snprintf(candidate, sizeof(candidate), "%s.csv", base_name);
      }
      else
      {
        snprintf(candidate, sizeof(candidate), "%s%u.csv", base_name, i);
      }

      if (!SD.exists(candidate))
      {
        File f = SD.open(candidate, FILE_WRITE);
        if (!f)
        {
          return false;
        }
        out_file = f;
        snprintf(out_name, out_name_len, "%s", candidate);
        return true;
      }
    }
    return false;
  }

  void resetSessionState()
  {
    g_loggingStopped = false;
    g_startupBufferFlushed = false;
    g_lineCountSinceFlush = 0;
    g_startupLogCount = 0;
    g_startupLogHead = 0;
  }

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

  void bufferStartupRecord(const flight_sample_t &sample, const flight_state_t &state)
  {
    g_startupLogWindow[g_startupLogHead].sample = sample;
    g_startupLogWindow[g_startupLogHead].state = state;
    g_startupLogHead = (g_startupLogHead + 1U) % app_config::kStartupLogWindowSamples;
    if (g_startupLogCount < app_config::kStartupLogWindowSamples)
    {
      ++g_startupLogCount;
    }
  }

  bool appendRecordToFile(const flight_sample_t &sample, const flight_state_t &state)
  {
    char line[768] = {0};
    const int written =
        snprintf(line,
                 sizeof(line),
                 "%lu,%u,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.0f,%.0f,%.0f,%u,%s,%u,%u,%u,%u,%u,%u,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f",
                 static_cast<unsigned long>(sample.timestamp_ms),
                 sample.baro_fresh ? 1U : 0U,
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
                 static_cast<unsigned int>(state.phase),
                 phaseName(state.phase),
                 state.descent ? 1U : 0U,
                 state.pyro_armed ? 1U : 0U,
                 state.parachute_deployed ? 1U : 0U,
                 state.pyro_active ? 1U : 0U,
                 static_cast<unsigned int>(state.deploy_reason),
                 state.restored_from_persist ? 1U : 0U,
                 state.altitude_m,
                 state.velocity_mps,
                 state.deployment_altitude_m,
                 state.deployment_velocity_mps,
                 state.vertical_filter.vertical_accel_mss,
                 state.vertical_filter.accel_bias_mss,
                 state.vertical_filter.baro_update_accepted ? 1U : 0U,
                 state.vertical_filter.normalized_baro_innovation,
                 state.position_m.x,
                 state.position_m.y,
                 state.position_m.z,
                 state.velocity_mps_3d.x,
                 state.velocity_mps_3d.y,
                 state.velocity_mps_3d.z,
                 state.orientation_q.w,
                 state.orientation_q.x,
                 state.orientation_q.y,
                 state.orientation_q.z);

    if ((written < 0) || (static_cast<size_t>(written) >= sizeof(line)))
    {
      return false;
    }

    char gps_altitude_buf[24] = {0};
    char gps_vertical_velocity_buf[24] = {0};
    if (state.gps.altitude_valid)
    {
      snprintf(gps_altitude_buf, sizeof(gps_altitude_buf), "%.2f", state.gps.altitude_m);
    }
    if (state.gps.vertical_velocity_valid)
    {
      snprintf(gps_vertical_velocity_buf, sizeof(gps_vertical_velocity_buf), "%.2f", state.gps.vertical_velocity_mps);
    }

    const size_t offset = static_cast<size_t>(written);
    if (state.gps.valid)
    {
      snprintf(line + offset,
               sizeof(line) - offset,
               ",1,%u,%u,%.7f,%.7f,%s,%s",
               static_cast<unsigned int>(state.gps.fix_type),
               static_cast<unsigned int>(state.gps.satellites),
               state.gps.latitude_deg,
               state.gps.longitude_deg,
               gps_altitude_buf,
               gps_vertical_velocity_buf);
    }
    else
    {
      snprintf(line + offset, sizeof(line) - offset, ",0,0,0,,,,");
    }

    g_logFile.println(line);
    ++g_lineCountSinceFlush;
    if (g_lineCountSinceFlush >= app_config::kLoggerFlushEveryNLines)
    {
      g_logFile.flush();
      g_lineCountSinceFlush = 0;
    }
    return true;
  }

  void flushStartupWindow()
  {
    if (g_startupBufferFlushed || (g_startupLogCount == 0U))
    {
      g_startupBufferFlushed = true;
      return;
    }

    const size_t start_index = (g_startupLogCount == app_config::kStartupLogWindowSamples)
                                   ? g_startupLogHead
                                   : 0U;
    for (size_t i = 0; i < g_startupLogCount; ++i)
    {
      const size_t idx = (start_index + i) % app_config::kStartupLogWindowSamples;
      appendRecordToFile(g_startupLogWindow[idx].sample, g_startupLogWindow[idx].state);
    }

    g_startupLogCount = 0U;
    g_startupLogHead = 0U;
    g_startupBufferFlushed = true;
    g_logFile.flush();
    g_lineCountSinceFlush = 0;
  }

} // namespace

namespace logger
{

  bool init()
  {
    if (!app_config::kEnableSdLogging)
    {
      g_enabled = false;
      return true;
    }

    resetSessionState();

    const bool sd_ok = SD.begin(BUILTIN_SDCARD);
    Serial.printf("SD.begin(BUILTIN_SDCARD) = %s\n", sd_ok ? "true" : "false");

    if (!sd_ok)
    {
      g_enabled = false;
      return false;
    }

    if (!openNumberedLog("flight", g_logFile, g_logName, sizeof(g_logName)))
    {
      g_enabled = false;
      return false;
    }

    g_logFile.println(
        "Time_ms,BaroFresh,T_C,P_Pa,Alt_m,AccX_mss,AccY_mss,AccZ_mss,GyroX_rads,GyroY_rads,GyroZ_rads,"
        "MagX_raw,MagY_raw,MagZ_raw,Phase,PhaseName,Descent,PyroArmed,Deployed,Pyro,DeployReason,Restored,StateAlt_m,StateVel_mps,"
        "DeploymentAlt_m,DeploymentVel_mps,VFAccel_mss,VFBias_mss,VFBaroAccepted,VFNIS,"
        "PosX_m,PosY_m,PosZ_m,VelX_mps,VelY_mps,VelZ_mps,Qw,Qx,Qy,Qz,GpsValid,GpsFix,GpsSats,GpsLat_deg,GpsLon_deg,"
        "GpsAlt_m,GpsVVel_mps");
    g_logFile.flush();
    g_enabled = true;
    return true;
  }

  bool isEnabled() { return g_enabled; }

  void writeSample(const flight_sample_t &sample, const flight_state_t &state)
  {
    if (!g_enabled)
    {
      return;
    }

    if (g_loggingStopped)
    {
      return;
    }

    if (state.phase == flight_phase_t::STARTUP)
    {
      if (app_config::kLogStartupSamplesToSd)
      {
        g_startupBufferFlushed = true;
        appendRecordToFile(sample, state);
        return;
      }

      bufferStartupRecord(sample, state);
      return;
    }

    if (!g_startupBufferFlushed)
    {
      flushStartupWindow();
    }

    if (state.phase == flight_phase_t::COMPLETE)
    {
      g_logFile.flush();
      g_logFile.close();
      g_loggingStopped = true;
      return;
    }

    appendRecordToFile(sample, state);
  }

} // namespace logger
