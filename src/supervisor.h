#pragma once

#include <Arduino.h>

#include "app_types.h"

// Supervisor: the one task that is allowed to be paranoid.
//
// Runs at the highest priority on a fixed period and does four things that must
// not depend on the sensor queue or the flight task being alive:
//   1. Feeds the hardware watchdog, but only while the sensor and flight tasks
//      are both producing heartbeats. A stalled task => watchdog reset =>
//      persisted flight state is restored on boot.
//   2. Backup deployment timer: fires the parachute a fixed time after launch
//      if nothing else has.
//   3. Pyro shut-off: turns the igniter outputs off kPyroOnMs after firing,
//      even if no new sensor sample ever arrives.
//   4. Measures the real sensor/flight rates and the flight update execution
//      time, for the health telemetry packet.
namespace supervisor
{

  // Call once from setup() before the scheduler starts. Arms the watchdog if
  // enabled in app_config.
  void init();

  // Unconditional watchdog feed for use during setup() (before the heartbeat
  // tasks exist) and in the fatal-error blink loop, so a board that failed
  // self-test stays visibly failed instead of boot-looping.
  void feed();

  // Heartbeats. Cheap; call once per loop iteration from the owning task.
  void heartbeatSensor();
  void heartbeatFlight(uint32_t update_duration_us);

  // The RTOS task body.
  void task(void *);

  // One iteration of the task body, for host tests.
  void testStep();

  // Snapshot of supervisor state for telemetry/logging (no locking required:
  // fields are written only by the supervisor task and read as plain words).
  supervisor_status_t status();

} // namespace supervisor
