#pragma once

// Binary telemetry protocol shared by the flight computer and the ground station.
//
// This header deliberately depends on nothing but <stdint.h>/<stddef.h> so it can
// be compiled on the host for tests. ground/telemetry_protocol.py is the Python
// mirror; the two MUST be kept in sync (test/host/test_protocol.cpp checks that).
//
// Framing: each packet is sent as one LoRa frame behind the 4-byte RadioHead
// header (to/from/id/flags) the existing ground receivers already expect.
// Every packet starts with a type byte and a sequence number, and ends with a
// CRC-16/CCITT-FALSE over everything before the CRC. All fields are
// little-endian, packed, no padding.
//
//   type 0x01  STATE   fast: altitude, velocity, attitude, GPS, flags      (5 Hz)
//   type 0x02  HEALTH  slow: raw sensors, biases, innovations, task rates  (1 Hz)

#include <stddef.h>
#include <stdint.h>

namespace tlm
{

  constexpr uint8_t kPacketTypeState = 0x01;
  constexpr uint8_t kPacketTypeHealth = 0x02;
  constexpr uint8_t kProtocolVersion = 1;

  // state_packet_t.flags
  constexpr uint8_t kFlagPyroArmed = 1U << 0;
  constexpr uint8_t kFlagDeployed = 1U << 1;
  constexpr uint8_t kFlagPyroActive = 1U << 2;
  constexpr uint8_t kFlagGpsValid = 1U << 3;
  constexpr uint8_t kFlagBaroAccepted = 1U << 4;   // 1D vertical filter accepted the last baro update
  constexpr uint8_t kFlagImuCalComplete = 1U << 5;
  constexpr uint8_t kFlagRestored = 1U << 6;       // this boot resumed a flight after a watchdog reset
  constexpr uint8_t kFlagBackupArmed = 1U << 7;    // backup deployment timer is counting

  // health_packet_t.status
  constexpr uint16_t kStatusSdLogging = 1U << 0;
  constexpr uint16_t kStatusRadioInit = 1U << 1;
  constexpr uint16_t kStatusGpsInit = 1U << 2;
  constexpr uint16_t kStatusMagAvailable = 1U << 3;
  constexpr uint16_t kStatusImuBiasFrozen = 1U << 4;
  constexpr uint16_t kStatusGpsOutage = 1U << 5;
  constexpr uint16_t kStatusWatchdogArmed = 1U << 6;
  constexpr uint16_t kStatusHeartbeatsOk = 1U << 7;
  constexpr uint16_t kStatusPersistence = 1U << 8;
  constexpr uint16_t kStatusDebugMode = 1U << 9;
  constexpr uint16_t kStatusPyrosDisabled = 1U << 10;
  constexpr uint16_t kStatusAhrsInit = 1U << 11;
  constexpr uint16_t kStatusAhrsGyroOnly = 1U << 12;
  constexpr uint16_t kStatusBaro3dAccepted = 1U << 13;
  constexpr uint16_t kStatusGpsPosAccepted = 1U << 14;
  constexpr uint16_t kStatusGpsVelAccepted = 1U << 15;

#pragma pack(push, 1)

  struct state_packet_t
  {
    uint8_t type;           // 0x01
    uint8_t seq;            // increments per packet of this type, wraps
    uint32_t t_ms;          // flight computer uptime
    uint8_t phase;          // flight_phase_t
    uint8_t flags;          // kFlag*
    int32_t alt_cm;         // deployment (1D filter) altitude above pad, cm
    int32_t vel_cms;        // deployment vertical velocity, cm/s (+ up)
    int32_t baro_alt_cm;    // raw barometric altitude above pad, cm
    int16_t q[4];           // orientation quaternion w,x,y,z * 10000 (body -> ENU)
    int32_t lat_1e7;        // GPS latitude  * 1e7 deg, 0 if no fix
    int32_t lon_1e7;        // GPS longitude * 1e7 deg, 0 if no fix
    int16_t accel_mag_cmss; // |accel| * 100 m/s^2
    uint8_t deploy_reason;  // deploy_reason_t
    uint16_t crc;
  };

  struct health_packet_t
  {
    uint8_t type;              // 0x02
    uint8_t seq;
    uint32_t t_ms;
    int16_t accel_cmss[3];     // body accel * 100 m/s^2
    int16_t gyro_crads[3];     // body gyro  * 100 rad/s
    int16_t mag_raw[3];        // magnetometer raw counts
    int32_t pressure_pa;
    int16_t temp_cc;           // temperature * 100 C
    int32_t pos_cm[3];         // 3D estimator ENU position, cm
    int16_t accel_bias_mmss[3];   // estimator accel bias * 1000 m/s^2
    int16_t gyro_bias_x1e4[3];    // estimator gyro bias * 10000 rad/s
    int16_t vf_accel_bias_mmss;   // 1D filter accel bias * 1000
    int16_t baro_nis_x100;        // 3D baro normalised innovation * 100
    int16_t vf_baro_nis_x100;     // 1D baro normalised innovation * 100
    int32_t gps_alt_cm;
    uint8_t gps_fix_type;
    uint8_t gps_sats;
    uint16_t gps_pdop_x100;
    uint16_t gps_hacc_cm;      // horizontal accuracy, cm (saturates)
    uint16_t status;           // kStatus*
    uint8_t sensor_hz;         // measured sensor task rate
    uint8_t flight_hz;         // measured flight task rate
    uint16_t flight_update_max_us; // longest flight update in the last second
    uint8_t reset_cause;       // raw SRC_SRSR low byte at boot
    uint8_t restore_count;
    uint32_t backup_remaining_ms;  // 0 when not armed
    uint32_t radio_tx_count;
    uint16_t radio_tx_fail;
    uint16_t crc;
  };

#pragma pack(pop)

  static_assert(sizeof(state_packet_t) == 41, "state_packet_t layout changed; update ground/telemetry_protocol.py");
  static_assert(sizeof(health_packet_t) == 90, "health_packet_t layout changed; update ground/telemetry_protocol.py");

  // CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
  inline uint16_t crc16(const uint8_t *data, size_t len)
  {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; ++i)
    {
      crc ^= static_cast<uint16_t>(data[i]) << 8U;
      for (uint8_t bit = 0; bit < 8U; ++bit)
      {
        crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                              : static_cast<uint16_t>(crc << 1U);
      }
    }
    return crc;
  }

  template <typename T>
  inline void seal(T &packet)
  {
    packet.crc = crc16(reinterpret_cast<const uint8_t *>(&packet), sizeof(T) - sizeof(uint16_t));
  }

  template <typename T>
  inline bool verify(const T &packet)
  {
    return packet.crc == crc16(reinterpret_cast<const uint8_t *>(&packet), sizeof(T) - sizeof(uint16_t));
  }

  // Saturating conversions so a wild value can never wrap into a plausible one.
  inline int16_t sat16(double v)
  {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(v >= 0.0 ? v + 0.5 : v - 0.5);
  }
  inline uint16_t satu16(double v)
  {
    if (v > 65535.0) return 65535;
    if (v < 0.0) return 0;
    return static_cast<uint16_t>(v + 0.5);
  }
  inline int32_t sat32(double v)
  {
    if (v > 2147483647.0) return 2147483647;
    if (v < -2147483648.0) return static_cast<int32_t>(-2147483647 - 1);
    return static_cast<int32_t>(v >= 0.0 ? v + 0.5 : v - 0.5);
  }
  inline uint8_t satu8(double v)
  {
    if (v > 255.0) return 255;
    if (v < 0.0) return 0;
    return static_cast<uint8_t>(v + 0.5);
  }

} // namespace tlm
