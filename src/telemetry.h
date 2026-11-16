#pragma once

#include "app_types.h"
#include "telemetry_protocol.h"

namespace telemetry {

// Inputs the packet builders need beyond the flight state. Collected by the
// radio task under the appropriate locks and passed in by value.
struct inputs_t {
  flight_state_t state;
  flight_sample_t sample;
  system_status_t system;
  supervisor_status_t supervisor;
  uint32_t radio_tx_count = 0;
  uint32_t radio_tx_fail = 0;
};

// Pure functions: no locking, no hardware. Testable on the host.
void buildStatePacket(uint32_t timestamp_ms, const inputs_t& in, uint8_t seq, tlm::state_packet_t& out);
void buildHealthPacket(uint32_t timestamp_ms, const inputs_t& in, uint8_t seq, tlm::health_packet_t& out);

// Convenience wrappers that fill a radio_packet_t and keep per-type sequence counters.
void buildStateRadioPacket(uint32_t timestamp_ms, const inputs_t& in, radio_packet_t& packet);
void buildHealthRadioPacket(uint32_t timestamp_ms, const inputs_t& in, radio_packet_t& packet);

}  // namespace telemetry
