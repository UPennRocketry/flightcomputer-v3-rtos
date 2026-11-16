#include "vertical_filter.h"

#include <math.h>
#include <stddef.h>

#include "app_config.h"

namespace
{

  constexpr size_t kStateCount = 3U;
  constexpr size_t kAltitudeIndex = 0U;
  constexpr size_t kVelocityIndex = 1U;
  constexpr size_t kAccelBiasIndex = 2U;

  struct filter_state_t
  {
    bool initialized = false;
    bool baro_update_accepted = false;
    double x[kStateCount] = {0.0, 0.0, 0.0};
    double p[kStateCount][kStateCount] = {};
    double vertical_accel_mss = 0.0;
    double normalized_baro_innovation = 0.0;
  };

  filter_state_t g_filter;

  double square(double value) { return value * value; }

  double finiteOrZero(double value)
  {
    return isfinite(value) ? value : 0.0;
  }

  double finiteOrFallback(double value, double fallback)
  {
    return isfinite(value) ? value : fallback;
  }

  double clampDouble(double value, double min_value, double max_value)
  {
    if (value < min_value)
    {
      return min_value;
    }
    if (value > max_value)
    {
      return max_value;
    }
    return value;
  }

  void resetCovariance(double altitude_sigma_m,
                       double velocity_sigma_mps,
                       double accel_bias_sigma_mss)
  {
    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        g_filter.p[row][col] = 0.0;
      }
    }

    g_filter.p[kAltitudeIndex][kAltitudeIndex] = square(altitude_sigma_m);
    g_filter.p[kVelocityIndex][kVelocityIndex] = square(velocity_sigma_mps);
    g_filter.p[kAccelBiasIndex][kAccelBiasIndex] = square(accel_bias_sigma_mss);
  }

  bool g_pendingBiasValid = false;
  double g_pendingBiasMss = 0.0;
  uint32_t g_consecutiveBaroRejects = 0;

  void initialize(double baro_altitude_m, double vertical_accel_mss)
  {
    g_filter = filter_state_t{};
    g_filter.initialized = true;
    g_filter.x[kAltitudeIndex] = finiteOrZero(baro_altitude_m);
    g_filter.x[kVelocityIndex] = 0.0;
    g_filter.x[kAccelBiasIndex] = g_pendingBiasValid ? g_pendingBiasMss : finiteOrZero(vertical_accel_mss);
    if (g_pendingBiasValid)
    {
      // Mid-flight start: we know neither altitude nor velocity well, so let the
      // barometer pull the state in quickly instead of trusting zero velocity.
      resetCovariance(app_config::kVerticalFilterRestoreAltitudeSigmaM,
                      app_config::kVerticalFilterRestoreVelocitySigmaMps,
                      app_config::kVerticalFilterInitialAccelBiasSigmaMss);
    }
    else
    {
      resetCovariance(app_config::kVerticalFilterInitialAltitudeSigmaM,
                      app_config::kVerticalFilterInitialVelocitySigmaMps,
                      app_config::kVerticalFilterInitialAccelBiasSigmaMss);
    }
    g_pendingBiasValid = false;
    g_consecutiveBaroRejects = 0;
  }

  void updatePadBias(double vertical_accel_mss, double dt_s)
  {
    const double alpha = clampDouble(dt_s / (app_config::kVerticalFilterAccelBiasTauS + dt_s), 0.0, 1.0);
    g_filter.x[kAccelBiasIndex] += alpha * (vertical_accel_mss - g_filter.x[kAccelBiasIndex]);
  }

  void holdPad(double baro_altitude_m, double vertical_accel_mss, double dt_s)
  {
    updatePadBias(vertical_accel_mss, dt_s);
    g_filter.x[kAltitudeIndex] = finiteOrFallback(baro_altitude_m, g_filter.x[kAltitudeIndex]);
    g_filter.x[kVelocityIndex] = 0.0;
    g_filter.vertical_accel_mss = vertical_accel_mss - g_filter.x[kAccelBiasIndex];
    g_filter.baro_update_accepted = true;
    g_filter.normalized_baro_innovation = 0.0;
    resetCovariance(app_config::kVerticalFilterBaroSigmaM,
                    app_config::kVerticalFilterInitialVelocitySigmaMps,
                    app_config::kVerticalFilterInitialAccelBiasSigmaMss);
  }

  flight_phase_t effectivePhase(const flight_state_t &state, bool hold_on_pad)
  {
    if ((state.phase == flight_phase_t::STARTUP) && !hold_on_pad)
    {
      return flight_phase_t::BOOST;
    }
    return state.phase;
  }

  bool useAccelForPhase(flight_phase_t phase)
  {
    return (phase == flight_phase_t::STARTUP) ||
           (phase == flight_phase_t::BOOST) ||
           (phase == flight_phase_t::COAST);
  }

  double accelSigmaForPhase(flight_phase_t phase)
  {
    switch (phase)
    {
    case flight_phase_t::STARTUP:
      return app_config::kVerticalFilterIdleAccelSigmaMss;
    case flight_phase_t::BOOST:
      return app_config::kVerticalFilterBoostAccelSigmaMss;
    case flight_phase_t::COAST:
      return app_config::kVerticalFilterCoastAccelSigmaMss;
    case flight_phase_t::DESCENT:
    case flight_phase_t::TOUCHDOWN:
    case flight_phase_t::COMPLETE:
    default:
      return app_config::kVerticalFilterDescentAccelSigmaMss;
    }
  }

  void predict(double vertical_accel_mss,
               double dt_s,
               flight_phase_t phase)
  {
    const bool use_accel = useAccelForPhase(phase);
    const double dt2 = dt_s * dt_s;
    const double corrected_accel_mss = use_accel ? (vertical_accel_mss - g_filter.x[kAccelBiasIndex]) : 0.0;

    g_filter.x[kAltitudeIndex] +=
        (g_filter.x[kVelocityIndex] * dt_s) + (0.5 * corrected_accel_mss * dt2);
    g_filter.x[kVelocityIndex] += corrected_accel_mss * dt_s;
    g_filter.vertical_accel_mss = corrected_accel_mss;

    double f[kStateCount][kStateCount] = {};
    for (size_t i = 0; i < kStateCount; ++i)
    {
      f[i][i] = 1.0;
    }
    f[kAltitudeIndex][kVelocityIndex] = dt_s;
    if (use_accel)
    {
      f[kAltitudeIndex][kAccelBiasIndex] = -0.5 * dt2;
      f[kVelocityIndex][kAccelBiasIndex] = -dt_s;
    }

    double fp[kStateCount][kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        for (size_t k = 0; k < kStateCount; ++k)
        {
          fp[row][col] += f[row][k] * g_filter.p[k][col];
        }
      }
    }

    double next_p[kStateCount][kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        for (size_t k = 0; k < kStateCount; ++k)
        {
          next_p[row][col] += fp[row][k] * f[col][k];
        }
      }
    }

    const double accel_sigma_mss = accelSigmaForPhase(phase);
    const double qa = square(accel_sigma_mss);
    const double dt3 = dt2 * dt_s;
    const double dt4 = dt2 * dt2;
    next_p[kAltitudeIndex][kAltitudeIndex] += 0.25 * dt4 * qa;
    next_p[kAltitudeIndex][kVelocityIndex] += 0.5 * dt3 * qa;
    next_p[kVelocityIndex][kAltitudeIndex] += 0.5 * dt3 * qa;
    next_p[kVelocityIndex][kVelocityIndex] += dt2 * qa;
    if (use_accel)
    {
      next_p[kAccelBiasIndex][kAccelBiasIndex] +=
          square(app_config::kVerticalFilterAccelBiasRandomWalkMss) * dt_s;
    }

    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        g_filter.p[row][col] = 0.5 * (next_p[row][col] + next_p[col][row]);
      }
    }
  }

  bool updateAltitudeMeasurement(double altitude_m,
                                 double variance,
                                 double gate_sigma,
                                 double &normalized_innovation)
  {
    if (!isfinite(altitude_m))
    {
      normalized_innovation = 0.0;
      return false;
    }

    variance = fmax(variance, 1.0e-6);
    const double residual = altitude_m - g_filter.x[kAltitudeIndex];
    const double innovation_variance = g_filter.p[kAltitudeIndex][kAltitudeIndex] + variance;
    if (innovation_variance <= 1.0e-9)
    {
      normalized_innovation = 0.0;
      return false;
    }

    normalized_innovation = fabs(residual) / sqrt(innovation_variance);
    if (normalized_innovation > gate_sigma)
    {
      return false;
    }

    double gain[kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      gain[row] = g_filter.p[row][kAltitudeIndex] / innovation_variance;
    }
    gain[kAccelBiasIndex] = 0.0;

    for (size_t row = 0; row < kStateCount; ++row)
    {
      g_filter.x[row] += gain[row] * residual;
    }

    double a[kStateCount][kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      a[row][row] = 1.0;
      a[row][kAltitudeIndex] -= gain[row];
    }

    double ap[kStateCount][kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        for (size_t k = 0; k < kStateCount; ++k)
        {
          ap[row][col] += a[row][k] * g_filter.p[k][col];
        }
      }
    }

    double next_p[kStateCount][kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        for (size_t k = 0; k < kStateCount; ++k)
        {
          next_p[row][col] += ap[row][k] * a[col][k];
        }
        next_p[row][col] += gain[row] * variance * gain[col];
      }
    }

    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        g_filter.p[row][col] = 0.5 * (next_p[row][col] + next_p[col][row]);
      }
    }

    return true;
  }

  void updateWithBarometer(double altitude_m)
  {
    double normalized = 0.0;
    bool accepted = updateAltitudeMeasurement(altitude_m,
                                              square(app_config::kVerticalFilterBaroSigmaM),
                                              app_config::kVerticalFilterBaroGateSigma,
                                              normalized);
    if (!accepted)
    {
      ++g_consecutiveBaroRejects;
      if (g_consecutiveBaroRejects >= app_config::kVerticalFilterMaxConsecutiveBaroRejects)
      {
        // Gate escape: the model has clearly lost the vehicle (e.g. parachute
        // opening). Admit that in the covariance and take the measurement.
        g_filter.p[kAltitudeIndex][kAltitudeIndex] += square(app_config::kVerticalFilterRecoveryAltitudeSigmaM);
        g_filter.p[kVelocityIndex][kVelocityIndex] += square(app_config::kVerticalFilterRecoveryVelocitySigmaMps);
        g_filter.p[kAltitudeIndex][kVelocityIndex] = 0.0;
        g_filter.p[kVelocityIndex][kAltitudeIndex] = 0.0;
        accepted = updateAltitudeMeasurement(altitude_m,
                                             square(app_config::kVerticalFilterBaroSigmaM),
                                             1.0e9, // no gate on the recovery update
                                             normalized);
        g_consecutiveBaroRejects = 0;
      }
    }
    else
    {
      g_consecutiveBaroRejects = 0;
    }
    g_filter.baro_update_accepted = accepted;
    g_filter.normalized_baro_innovation = normalized;
  }

  void publish(flight_state_t &state)
  {
    state.vertical_filter.initialized = g_filter.initialized;
    state.vertical_filter.baro_update_accepted = g_filter.baro_update_accepted;
    state.vertical_filter.altitude_m = finiteOrZero(g_filter.x[kAltitudeIndex]);
    state.vertical_filter.velocity_mps = finiteOrZero(g_filter.x[kVelocityIndex]);
    state.vertical_filter.vertical_accel_mss = finiteOrZero(g_filter.vertical_accel_mss);
    state.vertical_filter.accel_bias_mss = finiteOrZero(g_filter.x[kAccelBiasIndex]);
    state.vertical_filter.covariance_altitude_m2 =
        finiteOrZero(g_filter.p[kAltitudeIndex][kAltitudeIndex]);
    state.vertical_filter.covariance_velocity_m2ps2 =
        finiteOrZero(g_filter.p[kVelocityIndex][kVelocityIndex]);
    state.vertical_filter.covariance_accel_bias_m2ps4 =
        finiteOrZero(g_filter.p[kAccelBiasIndex][kAccelBiasIndex]);
    state.vertical_filter.normalized_baro_innovation =
        finiteOrZero(g_filter.normalized_baro_innovation);

    state.deployment_altitude_m = state.vertical_filter.altitude_m;
    state.deployment_velocity_mps = state.vertical_filter.velocity_mps;
    state.altitude_m = state.vertical_filter.altitude_m;
    state.velocity_mps = state.vertical_filter.velocity_mps;
  }

} // namespace

namespace vertical_filter
{

  void reset()
  {
    g_filter = filter_state_t{};
    g_pendingBiasValid = false;
    g_consecutiveBaroRejects = 0;
  }

  void restoreBias(double accel_bias_mss)
  {
    g_pendingBiasMss = finiteOrZero(accel_bias_mss);
    g_pendingBiasValid = true;
  }

  void update(const flight_sample_t &sample,
              flight_state_t &state,
              double vertical_accel_mss,
              double dt_s,
              bool hold_on_pad)
  {
    const double bounded_dt_s = clampDouble(dt_s, 0.001, app_config::kEstimatorMaxDtS);
    const double baro_altitude_m = static_cast<double>(sample.altitude_m);
    const double measured_vertical_accel_mss = finiteOrZero(vertical_accel_mss);

    if (!g_filter.initialized)
    {
      initialize(baro_altitude_m, measured_vertical_accel_mss);
    }

    if (hold_on_pad)
    {
      holdPad(baro_altitude_m, measured_vertical_accel_mss, bounded_dt_s);
    }
    else
    {
      const flight_phase_t phase = effectivePhase(state, hold_on_pad);
      predict(measured_vertical_accel_mss, bounded_dt_s, phase);
      if (sample.baro_fresh)
      {
        updateWithBarometer(baro_altitude_m);
      }
    }

    publish(state);
  }

} // namespace vertical_filter
