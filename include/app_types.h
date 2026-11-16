#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"

struct flight_sample_t {
  uint32_t timestamp_ms = 0;
  bool baro_fresh = false;   // false on cycles where the barometer value is a repeat (temperature cycle / first sample)
  bool mag_fresh = false;
  float temperature_c = 0.0f;
  float pressure_pa = 0.0f;
  float altitude_m = 0.0f;
  float accel_mss[3] = {0.0f, 0.0f, 0.0f};
  float gyro_rads[3] = {0.0f, 0.0f, 0.0f};
  float mag_raw[3] = {0.0f, 0.0f, 0.0f};
};

struct vec3d_t {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct quaternion_t {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct gps_fix_t {
  bool valid = false;
  bool position_valid = false;
  bool altitude_valid = false;
  bool velocity_valid = false;
  bool vertical_velocity_valid = false;
  bool gnss_fix_ok = false;
  bool invalid_llh = false;
  bool time_valid = false;
  uint8_t fix_type = 0;
  uint8_t satellites = 0;
  uint32_t timestamp_ms = 0;
  uint32_t i_tow_ms = 0;
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_msl_m = 0.0;
  double altitude_ellipsoid_m = 0.0;
  double altitude_m = 0.0;
  double vertical_velocity_mps = 0.0;
  double horizontal_accuracy_m = 0.0;
  double vertical_accuracy_m = 0.0;
  double speed_accuracy_mps = 0.0;
  double ground_speed_mps = 0.0;
  double heading_motion_deg = 0.0;
  double heading_accuracy_deg = 0.0;
  double p_dop = 0.0;
  vec3d_t position_m = {};
  vec3d_t velocity_mps = {};
};

struct estimator_status_t {
  bool ahrs_initialized = false;
  bool ahrs_gyro_only = false;
  bool ahrs_using_mag = false;
  bool ahrs_accel_rejected = false;
  bool ahrs_mag_rejected = false;
  bool imu_calibration_active = false;
  bool imu_calibration_sample_accepted = false;
  bool imu_calibration_complete = false;
  bool imu_bias_frozen = false;
  bool imu_bias_applied = false;
  bool baro_update_accepted = false;
  bool gps_position_update_accepted = false;
  bool gps_velocity_update_accepted = false;
  bool gps_outage = false;
  uint32_t imu_calibration_samples = 0;
  uint32_t last_gps_position_update_ms = 0;
  uint32_t last_gps_velocity_update_ms = 0;
  double body_up[3] = {0.0, 0.0, 1.0};
  double accel_bias_mss[3] = {0.0, 0.0, 0.0};
  double gyro_bias_rads[3] = {0.0, 0.0, 0.0};
  double accel_bias_drift_mss[3] = {0.0, 0.0, 0.0};
  double gyro_bias_drift_rads[3] = {0.0, 0.0, 0.0};
  double covariance_position_m2[3] = {0.0, 0.0, 0.0};
  double covariance_velocity_m2ps2[3] = {0.0, 0.0, 0.0};
  double normalized_baro_innovation = 0.0;
  double normalized_gps_position_innovation = 0.0;
  double normalized_gps_velocity_innovation = 0.0;
};

struct vertical_filter_status_t {
  bool initialized = false;
  bool baro_update_accepted = false;
  double altitude_m = 0.0;
  double velocity_mps = 0.0;
  double vertical_accel_mss = 0.0;
  double accel_bias_mss = 0.0;
  double covariance_altitude_m2 = 0.0;
  double covariance_velocity_m2ps2 = 0.0;
  double covariance_accel_bias_m2ps4 = 0.0;
  double normalized_baro_innovation = 0.0;
};

enum class flight_phase_t : uint8_t {
  STARTUP = 0,
  BOOST = 1,
  COAST = 2,
  DESCENT = 3,
  TOUCHDOWN = 4,
  COMPLETE = 5,
};

// Why the parachute was fired. Logged and sent in telemetry so a backup-timer
// deployment is distinguishable from a normal one after the flight.
enum class deploy_reason_t : uint8_t {
  NONE = 0,
  ALTITUDE = 1,        // normal: DESCENT and below kDeployAltM
  BACKUP_TIMER = 2,    // supervisor: kBackupDeployAfterLaunchMs elapsed since launch
  COAST_TIMEOUT = 3,   // flight logic: COAST exceeded kMaxCoastMs, then altitude rule fired
  RESTORE_LIMIT = 4,   // too many in-flight watchdog resets; fired immediately on boot
};

struct baro_baseline_t {
  float pressure_pa = 0.0f;
  float temperature_c = 0.0f;
};

struct flight_state_t {
  flight_phase_t phase = flight_phase_t::STARTUP;
  bool descent = false;
  bool pyro_armed = false;
  bool parachute_deployed = false;
  bool pyro_active = false;
  bool restored_from_persist = false;   // this boot resumed an in-flight record after a watchdog reset
  deploy_reason_t deploy_reason = deploy_reason_t::NONE;
  uint32_t phase_entry_time_ms = 0;
  uint32_t launch_time_ms = 0;
  uint32_t launch_elapsed_before_reset_ms = 0; // time already flown before a watchdog reset (0 normally)
  uint32_t pyro_fire_time_ms = 0;
  uint32_t touchdown_time_ms = 0;
  double altitude_m = 0.0;
  double velocity_mps = 0.0;
  double deployment_altitude_m = 0.0;
  double deployment_velocity_mps = 0.0;
  double home_altitude_m = 0.0;
  vec3d_t position_m = {};
  vec3d_t velocity_mps_3d = {};
  quaternion_t orientation_q = {};
  gps_fix_t gps = {};
  estimator_status_t estimator = {};
  vertical_filter_status_t vertical_filter = {};
};

struct radio_packet_t {
  size_t len = 0;
  uint8_t data[app_config::kMaxPacketLen] = {0};
};

struct system_status_t {
  bool debug_mode = false;
  bool pyros_disabled = false;
  bool mutexes_initialized = false;
  bool queues_initialized = false;
  bool debug_queue_initialized = false;
  bool sensors_initialized = false;
  bool flight_logic_initialized = false;
  bool gps_config_enabled = false;
  bool gps_initialized = false;
  bool gps_task_started = false;
  bool radio_runtime_enabled = false;
  bool radio_initialized = false;
  bool radio_task_started = false;
  bool radio_skipped_debug = false;
  bool sd_config_enabled = false;
  bool sd_initialized = false;
  bool sd_logging_enabled = false;
  bool magnetometer_config_enabled = false;
  bool sensor_task_started = false;
  bool flight_task_started = false;
  bool logger_task_started = false;
  bool debug_task_started = false;
  bool supervisor_task_started = false;
  bool watchdog_enabled = false;
  bool persistence_enabled = false;
  bool restored_from_persist = false;
  uint8_t reset_cause = 0;          // raw SRC_SRSR bits at boot (bit 4 = watchdog)
  uint8_t restore_count = 0;        // in-flight restores so far this flight
};

// Filled by the supervisor task; read by telemetry and the debug stream.
struct supervisor_status_t {
  bool watchdog_armed = false;
  bool heartbeats_ok = false;
  bool backup_timer_armed = false;
  uint32_t watchdog_feeds = 0;
  uint32_t heartbeat_faults = 0;        // supervisor cycles where a heartbeat was stale
  uint32_t sensor_rate_hz = 0;          // measured over the last second
  uint32_t flight_rate_hz = 0;
  uint32_t flight_update_max_us = 0;    // longest flight_logic::update seen (resets each second)
  uint32_t flight_update_last_us = 0;
  uint32_t backup_timer_remaining_ms = 0;
  uint32_t sensor_last_beat_ms = 0;
  uint32_t flight_last_beat_ms = 0;
};

struct log_record_t {
  flight_sample_t sample;
  flight_state_t state;
};

struct debug_record_t {
  flight_sample_t sample;
  flight_state_t state;
};
