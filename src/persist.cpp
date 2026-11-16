#include "persist.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <stddef.h>
#include <string.h>

#include "app_config.h"

namespace
{

  constexpr uint32_t kMagic = 0x55504652UL; // "UPFR"
  constexpr int kEepromAddress = 0;

  // SRC_SRSR (System Reset Controller, reset status register) on the i.MX RT1062.
  // Bit 4 is "wdog_rst_b": the reset was caused by WDOG1 timing out.
  constexpr uint32_t kSrsrWatchdogBit = (1UL << 4);

  struct stored_t
  {
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    persist::record_t record;
    uint16_t crc;
    uint16_t pad; // keeps sizeof a multiple of 4 with no implicit tail padding
  };

  uint16_t crc16(const uint8_t *data, size_t len)
  {
    // CRC-16/CCITT-FALSE, same polynomial used by the telemetry packets.
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

  uint16_t storedCrc(const stored_t &s)
  {
    // Everything up to (not including) the crc field. Using offsetof rather than
    // sizeof - 2 keeps trailing struct padding out of the CRC.
    return crc16(reinterpret_cast<const uint8_t *>(&s), offsetof(stored_t, crc));
  }

} // namespace

namespace persist
{

  bool load(record_t &out)
  {
    if (!app_config::kEnablePersistence)
    {
      return false;
    }

    stored_t s;
    EEPROM.get(kEepromAddress, s);
    if ((s.magic != kMagic) || (s.version != app_config::kPersistSchemaVersion))
    {
      return false;
    }
    if (storedCrc(s) != s.crc)
    {
      return false;
    }
    out = s.record;
    return true;
  }

  bool recordIsInFlight(const record_t &rec)
  {
    const flight_phase_t phase = static_cast<flight_phase_t>(rec.phase);
    return (phase == flight_phase_t::BOOST) ||
           (phase == flight_phase_t::COAST) ||
           (phase == flight_phase_t::DESCENT);
  }

  void save(const flight_state_t &state,
            const baro_baseline_t &baseline,
            uint32_t now_ms,
            uint8_t restore_count)
  {
    if (!app_config::kEnablePersistence)
    {
      return;
    }

    stored_t s = {};
    s.magic = kMagic;
    s.version = app_config::kPersistSchemaVersion;
    s.record.phase = static_cast<uint8_t>(state.phase);
    s.record.parachute_deployed = state.parachute_deployed ? 1U : 0U;
    s.record.deploy_reason = static_cast<uint8_t>(state.deploy_reason);
    s.record.restore_count = restore_count;
    s.record.launch_elapsed_ms =
        (state.launch_time_ms != 0U || state.phase != flight_phase_t::STARTUP)
            ? (now_ms - state.launch_time_ms) + state.launch_elapsed_before_reset_ms
            : 0U;
    s.record.home_altitude_m = static_cast<float>(state.home_altitude_m);
    s.record.baseline = baseline;
    for (int i = 0; i < 3; ++i)
    {
      s.record.body_up[i] = static_cast<float>(state.estimator.body_up[i]);
      s.record.accel_bias_mss[i] = static_cast<float>(state.estimator.accel_bias_mss[i]);
      s.record.gyro_bias_rads[i] = static_cast<float>(state.estimator.gyro_bias_rads[i]);
    }
    s.record.vf_accel_bias_mss = static_cast<float>(state.vertical_filter.accel_bias_mss);
    s.crc = storedCrc(s);
    EEPROM.put(kEepromAddress, s); // EEPROM.put only rewrites bytes that changed
  }

  void clear()
  {
    if (!app_config::kEnablePersistence)
    {
      return;
    }
    stored_t s = {};
    EEPROM.put(kEepromAddress, s);
  }

  uint8_t readAndClearResetCause()
  {
    const uint32_t srsr = SRC_SRSR;
    SRC_SRSR = srsr; // bits are write-1-to-clear, so the next boot sees only its own cause
    return static_cast<uint8_t>(srsr & 0xFFU);
  }

  bool resetCauseWasWatchdog(uint8_t cause)
  {
    return (static_cast<uint32_t>(cause) & kSrsrWatchdogBit) != 0U;
  }

} // namespace persist
