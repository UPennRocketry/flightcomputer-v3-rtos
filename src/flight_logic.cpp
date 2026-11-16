#include "flight_logic.h"

#include <Arduino.h>
#include <math.h>

#include "app_config.h"
#include "persist.h"
#include "rtos_context.h"
#include "sensors.h"
#include "state_estimator.h"
#include "vertical_filter.h"

namespace
{

  // Time-based confirmation windows. A transition condition must hold
  // continuously for the configured number of milliseconds. This replaces the
  // old sample counters, which silently changed meaning with the sample rate.
  struct confirm_t
  {
    bool active = false;
    uint32_t start_ms = 0;

    bool update(bool condition, uint32_t now_ms, uint32_t required_ms)
    {
      if (!condition)
      {
        active = false;
        return false;
      }
      if (!active)
      {
        active = true;
        start_ms = now_ms;
      }
      return (now_ms - start_ms) >= required_ms;
    }

    void reset() { active = false; }
  };

  confirm_t g_launchConfirm;
  confirm_t g_boostEndConfirm;
  confirm_t g_apogeeConfirm;
  confirm_t g_touchdownConfirm;

  void resetConfirmations()
  {
    g_launchConfirm.reset();
    g_boostEndConfirm.reset();
    g_apogeeConfirm.reset();
    g_touchdownConfirm.reset();
  }

  void setPyros(bool active)
  {
    const bool output_active = active && !app_config::kDisablePyros;
    digitalWrite(app_config::kPyro1Pin, output_active ? arduino::HIGH : arduino::LOW);
    digitalWrite(app_config::kPyro2Pin, output_active ? arduino::HIGH : arduino::LOW);
  }

  void persistState(const flight_state_t &state, uint32_t now_ms)
  {
    persist::save(state, sensors::baseline(), now_ms, g_systemStatus.restore_count);
  }

  double accelMagnitudeMss(const flight_sample_t &sample)
  {
    const double ax = static_cast<double>(sample.accel_mss[0]);
    const double ay = static_cast<double>(sample.accel_mss[1]);
    const double az = static_cast<double>(sample.accel_mss[2]);
    return sqrt((ax * ax) + (ay * ay) + (az * az));
  }

  bool launchCondition(const flight_sample_t &sample)
  {
    return accelMagnitudeMss(sample) >= app_config::kLaunchAccelThresholdMss;
  }

  double deploymentAltitudeM(const flight_state_t &state)
  {
    return state.deployment_altitude_m;
  }

  double deploymentVelocityMps(const flight_state_t &state)
  {
    return state.deployment_velocity_mps;
  }

  void enterPhase(flight_state_t &state, flight_phase_t phase, uint32_t timestamp_ms)
  {
    state.phase = phase;
    state.phase_entry_time_ms = timestamp_ms;

    if (phase == flight_phase_t::BOOST)
    {
      state.launch_time_ms = timestamp_ms;
    }
    else if (phase == flight_phase_t::TOUCHDOWN)
    {
      state.touchdown_time_ms = timestamp_ms;
    }

    resetConfirmations();

    if (phase == flight_phase_t::COMPLETE)
    {
      persist::clear();
    }
    else
    {
      persistState(state, timestamp_ms);
    }
  }

  uint32_t timeInPhaseMs(const flight_state_t &state, uint32_t now_ms)
  {
    return now_ms - state.phase_entry_time_ms;
  }

  void fire(flight_state_t &state, uint32_t now_ms, deploy_reason_t reason)
  {
    state.parachute_deployed = true;
    state.pyro_active = true;
    state.pyro_armed = false;
    state.pyro_fire_time_ms = now_ms;
    state.deploy_reason = reason;
    setPyros(true);
    persistState(state, now_ms);
  }

#if FLIGHT_DEBUG_MODE
  bool validDebugPhase(uint8_t phase)
  {
    return phase <= static_cast<uint8_t>(flight_phase_t::COMPLETE);
  }

  void applyDebugPhaseOverride(const flight_sample_t &sample, flight_state_t &state)
  {
    if (!g_debugForcePhaseEnabled || !validDebugPhase(g_debugForcedPhase))
    {
      return;
    }

    const flight_phase_t forced_phase = static_cast<flight_phase_t>(g_debugForcedPhase);
    if (state.phase != forced_phase)
    {
      state.phase = forced_phase;
      state.phase_entry_time_ms = sample.timestamp_ms;
      resetConfirmations();
    }

    state.pyro_armed = false;
    state.pyro_active = false;
    setPyros(false);
  }
#endif

  void updateLaunchPhase(const flight_sample_t &sample, flight_state_t &state)
  {
    if (g_launchConfirm.update(launchCondition(sample), sample.timestamp_ms, app_config::kLaunchConfirmMs))
    {
      enterPhase(state, flight_phase_t::BOOST, sample.timestamp_ms);
    }
  }

  void updateBoostPhase(const flight_sample_t &sample, flight_state_t &state)
  {
    const bool boost_ended = accelMagnitudeMss(sample) < app_config::kLaunchAccelThresholdMss;
    const bool timed_out = (app_config::kMaxBoostMs > 0U) &&
                           (timeInPhaseMs(state, sample.timestamp_ms) >= app_config::kMaxBoostMs);
    if (g_boostEndConfirm.update(boost_ended, sample.timestamp_ms, app_config::kBoostEndConfirmMs) || timed_out)
    {
      enterPhase(state, flight_phase_t::COAST, sample.timestamp_ms);
    }
  }

  void updateApogeePhase(const flight_sample_t &sample, flight_state_t &state)
  {
    const bool settled = !state.restored_from_persist ||
                         (timeInPhaseMs(state, sample.timestamp_ms) >= app_config::kRestoreSettleMs);
    const bool apogee_condition = settled &&
                                  (deploymentAltitudeM(state) >= app_config::kApogeeArmAltM) &&
                                  (deploymentVelocityMps(state) <= app_config::kApogeeDescentVelMps);
    const bool timed_out = (app_config::kMaxCoastMs > 0U) &&
                           (timeInPhaseMs(state, sample.timestamp_ms) >= app_config::kMaxCoastMs);
    if (g_apogeeConfirm.update(apogee_condition, sample.timestamp_ms, app_config::kApogeeConfirmMs) || timed_out)
    {
      enterPhase(state, flight_phase_t::DESCENT, sample.timestamp_ms);
      state.pyro_armed = true;
      if (timed_out)
      {
        // Remember that we got here by timeout so the log explains the deployment.
        state.deploy_reason = deploy_reason_t::COAST_TIMEOUT;
      }
    }
  }

  void updateTouchdownPhase(const flight_sample_t &sample, flight_state_t &state)
  {
    const bool touchdown_condition =
        (deploymentAltitudeM(state) <= (state.home_altitude_m + app_config::kTouchdownAltMarginM)) &&
        (fabs(deploymentVelocityMps(state)) <= app_config::kTouchdownVelocityLimitMps);
    if (g_touchdownConfirm.update(touchdown_condition, sample.timestamp_ms, app_config::kTouchdownConfirmMs))
    {
      enterPhase(state, flight_phase_t::TOUCHDOWN, sample.timestamp_ms);
      state.pyro_armed = false;
    }
  }

  void updatePyroOutputs(const flight_sample_t &sample, flight_state_t &state)
  {
    if ((state.phase == flight_phase_t::DESCENT) && state.pyro_armed && !state.parachute_deployed &&
        (deploymentAltitudeM(state) <= app_config::kDeployAltM))
    {
      const deploy_reason_t reason = (state.deploy_reason == deploy_reason_t::COAST_TIMEOUT)
                                         ? deploy_reason_t::COAST_TIMEOUT
                                         : deploy_reason_t::ALTITUDE;
      fire(state, sample.timestamp_ms, reason);
    }

    // The supervisor also enforces this shut-off on its own clock; keeping it
    // here too costs nothing and makes the flight task self-consistent.
    if (state.pyro_active &&
        ((sample.timestamp_ms - state.pyro_fire_time_ms) >= app_config::kPyroOnMs))
    {
      state.pyro_active = false;
      setPyros(false);
    }
  }

} // namespace

namespace flight_logic
{

  void init()
  {
    pinMode(app_config::kPyro1Pin, arduino::OUTPUT);
    pinMode(app_config::kPyro2Pin, arduino::OUTPUT);
    setPyros(false);
    resetConfirmations();
    state_estimator::reset();
  }

  void update(const flight_sample_t &sample, flight_state_t &state)
  {
#if FLIGHT_DEBUG_MODE
    const bool debug_forced_phase = g_debugForcePhaseEnabled && validDebugPhase(g_debugForcedPhase);
    if (debug_forced_phase)
    {
      applyDebugPhaseOverride(sample, state);
    }
#else
    constexpr bool debug_forced_phase = false;
#endif

    state_estimator::update(sample, state);

    if (!debug_forced_phase && (state.phase == flight_phase_t::STARTUP))
    {
      if (!launchCondition(sample))
      {
        state.home_altitude_m = deploymentAltitudeM(state);
      }
      updateLaunchPhase(sample, state);
    }
    else if (!debug_forced_phase && (state.phase == flight_phase_t::BOOST))
    {
      updateBoostPhase(sample, state);
    }
    else if (!debug_forced_phase && (state.phase == flight_phase_t::COAST))
    {
      updateApogeePhase(sample, state);
    }
    else if (!debug_forced_phase && (state.phase == flight_phase_t::DESCENT))
    {
      updateTouchdownPhase(sample, state);
    }
    else if (!debug_forced_phase && (state.phase == flight_phase_t::TOUCHDOWN))
    {
      if ((sample.timestamp_ms - state.touchdown_time_ms) >= app_config::kPostTouchdownLogMs)
      {
        enterPhase(state, flight_phase_t::COMPLETE, sample.timestamp_ms);
        state.pyro_armed = false;
        state.pyro_active = false;
        setPyros(false);
      }
    }

    if (!debug_forced_phase)
    {
      updatePyroOutputs(sample, state);
    }
#if FLIGHT_DEBUG_MODE
    else
    {
      state.pyro_armed = false;
      state.pyro_active = false;
      setPyros(false);
    }
#endif

    state.descent = (state.phase == flight_phase_t::DESCENT) ||
                    (state.phase == flight_phase_t::TOUCHDOWN) ||
                    (state.phase == flight_phase_t::COMPLETE);
  }

  void forceDeploy(flight_state_t &state, uint32_t now_ms, deploy_reason_t reason)
  {
    if (state.parachute_deployed)
    {
      return;
    }
    if (state.phase == flight_phase_t::BOOST || state.phase == flight_phase_t::COAST)
    {
      // A forced deployment means we are past apogee by definition of the timer.
      state.phase = flight_phase_t::DESCENT;
      state.phase_entry_time_ms = now_ms;
      resetConfirmations();
    }
    fire(state, now_ms, reason);
    state.descent = true;
  }

  void pyroOff(flight_state_t &state)
  {
    state.pyro_active = false;
    setPyros(false);
  }

  void restore(const persist::record_t &rec, flight_state_t &state, uint32_t now_ms)
  {
    state.phase = static_cast<flight_phase_t>(rec.phase);
    state.phase_entry_time_ms = now_ms;
    state.launch_time_ms = now_ms;
    state.launch_elapsed_before_reset_ms = rec.launch_elapsed_ms;
    state.home_altitude_m = rec.home_altitude_m;
    state.parachute_deployed = (rec.parachute_deployed != 0U);
    state.deploy_reason = static_cast<deploy_reason_t>(rec.deploy_reason);
    state.pyro_armed = (state.phase == flight_phase_t::DESCENT) && !state.parachute_deployed;
    state.pyro_active = false;
    state.restored_from_persist = true;
    state.descent = (state.phase == flight_phase_t::DESCENT);
    setPyros(false);
    resetConfirmations();

    // Hand the IMU calibration back to the estimator so the vertical-axis
    // projection and bias removal are correct immediately.
    state_estimator::restoreCalibration(rec.body_up, rec.accel_bias_mss, rec.gyro_bias_rads);
    vertical_filter::restoreBias(rec.vf_accel_bias_mss);
  }

} // namespace flight_logic
