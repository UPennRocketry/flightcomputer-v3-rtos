#pragma once

#include "app_types.h"
#include "persist.h"

namespace flight_logic {

void init();

// Runs the estimator and the flight state machine for one sensor sample.
// Caller must hold g_stateMutex.
void update(const flight_sample_t& sample, flight_state_t& state);

// Fire the parachute now, whatever the state machine thinks. Used by the
// supervisor's backup timer and by the restore-limit path. Caller must hold
// g_stateMutex. Idempotent once the parachute is deployed.
void forceDeploy(flight_state_t& state, uint32_t now_ms, deploy_reason_t reason);

// Turn the igniter outputs off and clear pyro_active. Caller must hold g_stateMutex.
void pyroOff(flight_state_t& state);

// Resume an in-flight state from a persisted record after a watchdog reset.
// Call before the scheduler starts, after sensors and the estimator are initialised.
void restore(const persist::record_t& rec, flight_state_t& state, uint32_t now_ms);

}  // namespace flight_logic
