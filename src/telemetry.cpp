#include "telemetry.h"

#include <math.h>
#include <string.h>

namespace telemetry
{

  namespace
  {
    uint8_t g_stateSeq = 0;
    uint8_t g_healthSeq = 0;

    double accelMagnitude(const flight_sample_t &s)
    {
      const double ax = s.accel_mss[0];
      const double ay = s.accel_mss[1];
      const double az = s.accel_mss[2];
      return sqrt(ax * ax + ay * ay + az * az);
    }
  } // namespace

  void buildStatePacket(uint32_t timestamp_ms, const inputs_t &in, uint8_t seq, tlm::state_packet_t &out)
  {
    const flight_state_t &st = in.state;
    memset(&out, 0, sizeof(out));
    out.type = tlm::kPacketTypeState;
    out.seq = seq;
    out.t_ms = timestamp_ms;
    out.phase = static_cast<uint8_t>(st.phase);

    uint8_t flags = 0;
    if (st.pyro_armed) flags |= tlm::kFlagPyroArmed;
    if (st.parachute_deployed) flags |= tlm::kFlagDeployed;
    if (st.pyro_active) flags |= tlm::kFlagPyroActive;
    if (st.gps.valid) flags |= tlm::kFlagGpsValid;
    if (st.vertical_filter.baro_update_accepted) flags |= tlm::kFlagBaroAccepted;
    if (st.estimator.imu_calibration_complete) flags |= tlm::kFlagImuCalComplete;
    if (st.restored_from_persist) flags |= tlm::kFlagRestored;
    if (in.supervisor.backup_timer_armed) flags |= tlm::kFlagBackupArmed;
    out.flags = flags;

    out.alt_cm = tlm::sat32(st.altitude_m * 100.0);
    out.vel_cms = tlm::sat32(st.velocity_mps * 100.0);
    out.baro_alt_cm = tlm::sat32(static_cast<double>(in.sample.altitude_m) * 100.0);
    out.q[0] = tlm::sat16(st.orientation_q.w * 10000.0);
    out.q[1] = tlm::sat16(st.orientation_q.x * 10000.0);
    out.q[2] = tlm::sat16(st.orientation_q.y * 10000.0);
    out.q[3] = tlm::sat16(st.orientation_q.z * 10000.0);
    out.lat_1e7 = st.gps.valid ? tlm::sat32(st.gps.latitude_deg * 1e7) : 0;
    out.lon_1e7 = st.gps.valid ? tlm::sat32(st.gps.longitude_deg * 1e7) : 0;
    out.accel_mag_cmss = tlm::sat16(accelMagnitude(in.sample) * 100.0);
    out.deploy_reason = static_cast<uint8_t>(st.deploy_reason);
    tlm::seal(out);
  }

  void buildHealthPacket(uint32_t timestamp_ms, const inputs_t &in, uint8_t seq, tlm::health_packet_t &out)
  {
    const flight_state_t &st = in.state;
    const flight_sample_t &sm = in.sample;
    memset(&out, 0, sizeof(out));
    out.type = tlm::kPacketTypeHealth;
    out.seq = seq;
    out.t_ms = timestamp_ms;
    for (int i = 0; i < 3; ++i)
    {
      out.accel_cmss[i] = tlm::sat16(static_cast<double>(sm.accel_mss[i]) * 100.0);
      out.gyro_crads[i] = tlm::sat16(static_cast<double>(sm.gyro_rads[i]) * 100.0);
      out.mag_raw[i] = tlm::sat16(static_cast<double>(sm.mag_raw[i]));
      out.accel_bias_mmss[i] = tlm::sat16(st.estimator.accel_bias_mss[i] * 1000.0);
      out.gyro_bias_x1e4[i] = tlm::sat16(st.estimator.gyro_bias_rads[i] * 10000.0);
    }
    out.pressure_pa = tlm::sat32(static_cast<double>(sm.pressure_pa));
    out.temp_cc = tlm::sat16(static_cast<double>(sm.temperature_c) * 100.0);
    out.pos_cm[0] = tlm::sat32(st.position_m.x * 100.0);
    out.pos_cm[1] = tlm::sat32(st.position_m.y * 100.0);
    out.pos_cm[2] = tlm::sat32(st.position_m.z * 100.0);
    out.vf_accel_bias_mmss = tlm::sat16(st.vertical_filter.accel_bias_mss * 1000.0);
    out.baro_nis_x100 = tlm::sat16(st.estimator.normalized_baro_innovation * 100.0);
    out.vf_baro_nis_x100 = tlm::sat16(st.vertical_filter.normalized_baro_innovation * 100.0);
    out.gps_alt_cm = st.gps.altitude_valid ? tlm::sat32(st.gps.altitude_m * 100.0) : 0;
    out.gps_fix_type = st.gps.fix_type;
    out.gps_sats = st.gps.satellites;
    out.gps_pdop_x100 = tlm::satu16(st.gps.p_dop * 100.0);
    out.gps_hacc_cm = tlm::satu16(st.gps.horizontal_accuracy_m * 100.0);

    uint16_t status = 0;
    if (in.system.sd_logging_enabled) status |= tlm::kStatusSdLogging;
    if (in.system.radio_initialized) status |= tlm::kStatusRadioInit;
    if (in.system.gps_initialized) status |= tlm::kStatusGpsInit;
    if (in.system.magnetometer_config_enabled) status |= tlm::kStatusMagAvailable;
    if (st.estimator.imu_bias_frozen) status |= tlm::kStatusImuBiasFrozen;
    if (st.estimator.gps_outage) status |= tlm::kStatusGpsOutage;
    if (in.supervisor.watchdog_armed) status |= tlm::kStatusWatchdogArmed;
    if (in.supervisor.heartbeats_ok) status |= tlm::kStatusHeartbeatsOk;
    if (in.system.persistence_enabled) status |= tlm::kStatusPersistence;
    if (in.system.debug_mode) status |= tlm::kStatusDebugMode;
    if (in.system.pyros_disabled) status |= tlm::kStatusPyrosDisabled;
    if (st.estimator.ahrs_initialized) status |= tlm::kStatusAhrsInit;
    if (st.estimator.ahrs_gyro_only) status |= tlm::kStatusAhrsGyroOnly;
    if (st.estimator.baro_update_accepted) status |= tlm::kStatusBaro3dAccepted;
    if (st.estimator.gps_position_update_accepted) status |= tlm::kStatusGpsPosAccepted;
    if (st.estimator.gps_velocity_update_accepted) status |= tlm::kStatusGpsVelAccepted;
    out.status = status;

    out.sensor_hz = tlm::satu8(in.supervisor.sensor_rate_hz);
    out.flight_hz = tlm::satu8(in.supervisor.flight_rate_hz);
    out.flight_update_max_us = tlm::satu16(in.supervisor.flight_update_max_us);
    out.reset_cause = in.system.reset_cause;
    out.restore_count = in.system.restore_count;
    out.backup_remaining_ms = in.supervisor.backup_timer_remaining_ms;
    out.radio_tx_count = in.radio_tx_count;
    out.radio_tx_fail = tlm::satu16(in.radio_tx_fail);
    tlm::seal(out);
  }

  void buildStateRadioPacket(uint32_t timestamp_ms, const inputs_t &in, radio_packet_t &packet)
  {
    tlm::state_packet_t p;
    buildStatePacket(timestamp_ms, in, g_stateSeq++, p);
    memcpy(packet.data, &p, sizeof(p));
    packet.len = sizeof(p);
  }

  void buildHealthRadioPacket(uint32_t timestamp_ms, const inputs_t &in, radio_packet_t &packet)
  {
    tlm::health_packet_t p;
    buildHealthPacket(timestamp_ms, in, g_healthSeq++, p);
    memcpy(packet.data, &p, sizeof(p));
    packet.len = sizeof(p);
  }

} // namespace telemetry
