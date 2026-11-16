#include "gps.h"

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include <arduino_freertos.h>
#include <math.h>

#include "app_config.h"
#include "rtos_context.h"

namespace
{

  SFE_UBLOX_GNSS g_gnss;
  bool g_initialized = false;
  bool g_have_altitude_reference = false;
  bool g_have_home_position = false;
  double g_altitude_reference_msl_m = 0.0;
  double g_home_latitude_deg = 0.0;
  double g_home_longitude_deg = 0.0;

  // Lat/lon are considered valid for 2D, 3D, and GNSS+DR fixes.
  // Note that GPS altitude correction is only used at fix type >= 3, so altitude is not considered valid at fix type 2.
  bool hasPositionFix(uint8_t fix_type)
  {
    return (fix_type == 2U) || (fix_type == 3U) || (fix_type == 4U);
  }

  // GPS altitude and vertical velocity correction require a 3D-capable fix.
  bool hasAltitudeFix(uint8_t fix_type)
  {
    return (fix_type == 3U) || (fix_type == 4U);
  }

  double degreesToRadians(double degrees)
  {
    constexpr double kPi = 3.14159265358979323846;
    return degrees * (kPi / 180.0);
  }

  vec3d_t localEnuFromLatLon(double latitude_deg, double longitude_deg, double altitude_m)
  {
    constexpr double kEarthRadiusM = 6378137.0;
    const double home_lat_rad = degreesToRadians(g_home_latitude_deg);
    const double lat_rad = degreesToRadians(latitude_deg);
    const double d_lat_rad = lat_rad - home_lat_rad;
    const double d_lon_rad = degreesToRadians(longitude_deg - g_home_longitude_deg);
    const double mean_lat_rad = 0.5 * (lat_rad + home_lat_rad);

    vec3d_t enu;
    enu.x = d_lon_rad * kEarthRadiusM * cos(mean_lat_rad);
    enu.y = d_lat_rad * kEarthRadiusM;
    enu.z = altitude_m;
    return enu;
  }

  void clearGpsState()
  {
    if (g_stateMutex != nullptr)
    {
      xSemaphoreTake(g_stateMutex, portMAX_DELAY);
      g_flightState.gps = {};
      xSemaphoreGive(g_stateMutex);
    }
  }

  void updateGpsState()
  {
    const uint32_t now_ms = millis();
    const uint8_t fix_type = g_gnss.getFixType();
    const uint8_t satellites = g_gnss.getSIV();
    const bool gnss_fix_ok = g_gnss.getGnssFixOk();
    const bool invalid_llh = g_gnss.getInvalidLlh();
    const bool time_valid = g_gnss.getTimeValid();
    const uint32_t i_tow_ms = g_gnss.getTimeOfWeek();

    const double latitude_deg = static_cast<double>(g_gnss.getLatitude()) / 10000000.0;
    const double longitude_deg = static_cast<double>(g_gnss.getLongitude()) / 10000000.0;
    const double altitude_ellipsoid_m = static_cast<double>(g_gnss.getAltitude()) / 1000.0;
    const double altitude_msl_m = static_cast<double>(g_gnss.getAltitudeMSL()) / 1000.0;
    const double horizontal_accuracy_m = static_cast<double>(g_gnss.getHorizontalAccEst()) / 1000.0;
    const double vertical_accuracy_m = static_cast<double>(g_gnss.getVerticalAccEst()) / 1000.0;
    const double vel_n_mps = static_cast<double>(g_gnss.getNedNorthVel()) / 1000.0;
    const double vel_e_mps = static_cast<double>(g_gnss.getNedEastVel()) / 1000.0;
    const double vel_d_mps = static_cast<double>(g_gnss.getNedDownVel()) / 1000.0;
    const double speed_accuracy_mps = static_cast<double>(g_gnss.getSpeedAccEst()) / 1000.0;
    const double ground_speed_mps = static_cast<double>(g_gnss.getGroundSpeed()) / 1000.0;
    const double heading_motion_deg = static_cast<double>(g_gnss.getHeading()) * 1.0e-5;
    const double heading_accuracy_deg = static_cast<double>(g_gnss.getHeadingAccEst()) * 1.0e-5;
    const double p_dop = static_cast<double>(g_gnss.getPDOP()) * 0.01;
    const bool fix_position_valid = hasPositionFix(fix_type) && gnss_fix_ok && !invalid_llh;
    const bool fix_altitude_valid = fix_position_valid && hasAltitudeFix(fix_type);

    xSemaphoreTake(g_stateMutex, portMAX_DELAY);

    const bool startup_phase = (g_flightState.phase == flight_phase_t::STARTUP);

    g_flightState.gps = {};
    g_flightState.gps.valid = fix_position_valid;
    g_flightState.gps.position_valid = false;
    g_flightState.gps.altitude_valid = false;
    g_flightState.gps.velocity_valid = false;
    g_flightState.gps.vertical_velocity_valid = false;
    g_flightState.gps.gnss_fix_ok = gnss_fix_ok;
    g_flightState.gps.invalid_llh = invalid_llh;
    g_flightState.gps.time_valid = time_valid;
    g_flightState.gps.fix_type = fix_type;
    g_flightState.gps.satellites = satellites;
    g_flightState.gps.timestamp_ms = now_ms;
    g_flightState.gps.i_tow_ms = i_tow_ms;
    g_flightState.gps.horizontal_accuracy_m = horizontal_accuracy_m;
    g_flightState.gps.vertical_accuracy_m = vertical_accuracy_m;
    g_flightState.gps.speed_accuracy_mps = speed_accuracy_mps;
    g_flightState.gps.ground_speed_mps = ground_speed_mps;
    g_flightState.gps.heading_motion_deg = heading_motion_deg;
    g_flightState.gps.heading_accuracy_deg = heading_accuracy_deg;
    g_flightState.gps.p_dop = p_dop;

    if (!fix_position_valid)
    {
      xSemaphoreGive(g_stateMutex);
      return;
    }

    if (!g_have_home_position && startup_phase)
    {
      g_home_latitude_deg = latitude_deg;
      g_home_longitude_deg = longitude_deg;
      g_have_home_position = true;
    }

    g_flightState.gps.latitude_deg = latitude_deg;
    g_flightState.gps.longitude_deg = longitude_deg;
    g_flightState.gps.altitude_msl_m = altitude_msl_m;
    g_flightState.gps.altitude_ellipsoid_m = altitude_ellipsoid_m;

    if (g_have_home_position)
    {
      g_flightState.gps.position_valid = true;
      g_flightState.gps.position_m = localEnuFromLatLon(latitude_deg, longitude_deg, 0.0);
    }

    if (fix_altitude_valid)
    {
      if (!g_have_altitude_reference && startup_phase)
      {
        g_altitude_reference_msl_m = altitude_msl_m - g_flightState.altitude_m;
        g_have_altitude_reference = true;
      }

      if (g_have_altitude_reference)
      {
        const double relative_altitude_m = altitude_msl_m - g_altitude_reference_msl_m;
        g_flightState.gps.altitude_valid = true;
        g_flightState.gps.altitude_m = relative_altitude_m;
        g_flightState.gps.position_m.z = relative_altitude_m;
      }
    }

    if (fix_position_valid)
    {
      g_flightState.gps.velocity_valid = true;
      g_flightState.gps.velocity_mps.x = vel_e_mps;
      g_flightState.gps.velocity_mps.y = vel_n_mps;
      if (fix_altitude_valid)
      {
        g_flightState.gps.vertical_velocity_valid = true;
        g_flightState.gps.velocity_mps.z = -vel_d_mps;
        g_flightState.gps.vertical_velocity_mps = g_flightState.gps.velocity_mps.z;
      }
    }

    xSemaphoreGive(g_stateMutex);
  }

} // namespace

namespace gps
{

  bool init()
  {
    if (!app_config::kEnableGps)
    {
      return false;
    }

    Wire.begin();
    Wire.setClock(app_config::kGpsI2cHz);

    if (!g_gnss.begin(Wire, app_config::kGpsI2cAddress, app_config::kGpsInitMaxWaitMs))
    {
      clearGpsState();
      g_initialized = false;
      return false;
    }

    if (!g_gnss.setI2COutput(COM_TYPE_UBX))
    {
      clearGpsState();
      g_initialized = false;
      return false;
    }

    if (!g_gnss.setNavigationFrequency(app_config::kGpsNavFrequencyHz))
    {
      clearGpsState();
      g_initialized = false;
      return false;
    }

    if (!g_gnss.setAutoPVT(true))
    {
      clearGpsState();
      g_initialized = false;
      return false;
    }

    clearGpsState();
    g_have_altitude_reference = false;
    g_have_home_position = false;
    g_initialized = true;
    return true;
  }

  void task(void *)
  {
    TickType_t last_wake = xTaskGetTickCount();

    while (true)
    {
      if (g_initialized && g_gnss.getPVT())
      {
        updateGpsState();
      }

      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(app_config::kGpsPollPeriodMs));
    }
  }

} // namespace gps
