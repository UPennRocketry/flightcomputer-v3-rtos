#pragma once

#include <Arduino.h>

#include "app_types.h"

// Flight-state persistence across a watchdog reset.
//
// A small record is written to the Teensy's EEPROM emulation on every phase
// transition (and when the parachute fires). Writes are rare, so flash wear is
// not a concern. On boot, main.cpp asks whether a record describes an in-flight
// vehicle and, if the reset was caused by the watchdog, restores it.
//
// Restoring is deliberately limited to watchdog resets: a stale in-flight record
// after a power cycle must never arm the pyros on the bench.
namespace persist
{

  struct record_t
  {
    uint8_t phase = 0;                     // flight_phase_t
    uint8_t parachute_deployed = 0;
    uint8_t deploy_reason = 0;             // deploy_reason_t
    uint8_t restore_count = 0;             // in-flight restores so far this flight
    uint32_t launch_elapsed_ms = 0;        // ms flown before the record was written
    float home_altitude_m = 0.0f;
    baro_baseline_t baseline = {};         // pressure/temperature reference for altitude
    float body_up[3] = {0.0f, 0.0f, 1.0f}; // startup gravity axis (IMU calibration)
    float accel_bias_mss[3] = {0.0f, 0.0f, 0.0f};
    float gyro_bias_rads[3] = {0.0f, 0.0f, 0.0f};
    float vf_accel_bias_mss = 0.0f;        // 1D deployment filter accel bias
  };

  // Returns true if a valid record is stored. Does not interpret it.
  bool load(record_t &out);

  // True when the record describes a vehicle between launch and touchdown.
  bool recordIsInFlight(const record_t &rec);

  // Write a record built from the current state. Safe to call from the flight
  // task (takes a few ms on first write to a page; subsequent writes are quick).
  void save(const flight_state_t &state,
            const baro_baseline_t &baseline,
            uint32_t now_ms,
            uint8_t restore_count);

  // Invalidate the stored record (COMPLETE, or stale record found at boot).
  void clear();

  // Reset cause bits as read from SRC_SRSR at boot (bit 4 = WDOG1 timeout).
  uint8_t readAndClearResetCause();
  bool resetCauseWasWatchdog(uint8_t cause);

} // namespace persist
