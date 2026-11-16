#include "state_estimator.h"

#include <math.h>
#include <stddef.h>

#include "app_config.h"
#include "vertical_filter.h"

#if __has_include(<Fusion.h>)
#include <Fusion.h>
#elif __has_include(<Fusion/Fusion.h>)
#include <Fusion/Fusion.h>
#else
#error "Fusion AHRS library not found. Add https://github.com/xioTechnologies/Fusion.git to lib_deps."
#endif

namespace
{

  constexpr size_t kStateCount = 6U;
  constexpr size_t kAxisCount = 3U;
  constexpr size_t kPositionIndex = 0U;
  constexpr size_t kVelocityIndex = 3U;

  struct vec3f_t
  {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
  };

  enum class ahrs_mode_t : uint8_t
  {
    UNINITIALIZED = 0,
    STATIC = 1,
    GYRO_ONLY = 2,
  };

  struct estimator_state_t
  {
    bool initialized = false;
    bool have_baro = false;
    uint32_t last_timestamp_ms = 0U;
    uint32_t last_gps_position_measurement_id = 0U;
    uint32_t last_gps_velocity_measurement_id = 0U;
    FusionAhrs ahrs = {};
    ahrs_mode_t ahrs_mode = ahrs_mode_t::UNINITIALIZED;
    bool imu_bias_frozen = false;
    bool imu_calibration_reference_set = false;
    uint32_t imu_calibration_samples = 0U;
    vec3f_t accel_bias_mss = {};
    vec3f_t gyro_bias_rads = {};
    vec3f_t accel_bias_reference_mss = {};
    vec3f_t gyro_bias_reference_rads = {};
    vec3f_t accel_bias_drift_mss = {};
    vec3f_t gyro_bias_drift_rads = {};
    vec3f_t body_up = {0.0f, 0.0f, 1.0f};
    float baro_altitude_m = 0.0f;
    double x[kStateCount] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double p[kStateCount][kStateCount] = {};
    estimator_status_t status = {};
  };

  estimator_state_t g_estimator;

  float clampFloat(float value, float min_value, float max_value)
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

  double square(double value) { return value * value; }

  vec3f_t makeVec3(float x, float y, float z)
  {
    vec3f_t v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
  }

  vec3f_t sampleVec3(const float values[3])
  {
    return makeVec3(values[0], values[1], values[2]);
  }

  float dot(const vec3f_t &a, const vec3f_t &b)
  {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
  }

  vec3f_t cross(const vec3f_t &a, const vec3f_t &b)
  {
    return makeVec3((a.y * b.z) - (a.z * b.y),
                    (a.z * b.x) - (a.x * b.z),
                    (a.x * b.y) - (a.y * b.x));
  }

  float norm(const vec3f_t &v)
  {
    return sqrtf(dot(v, v));
  }

  vec3f_t scale(const vec3f_t &v, float scalar)
  {
    return makeVec3(v.x * scalar, v.y * scalar, v.z * scalar);
  }

  vec3f_t subtract(const vec3f_t &a, const vec3f_t &b)
  {
    return makeVec3(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  vec3f_t normalize(const vec3f_t &v, const vec3f_t &fallback)
  {
    const float magnitude = norm(v);
    if (magnitude <= 1.0e-6f)
    {
      return fallback;
    }
    return scale(v, 1.0f / magnitude);
  }

  FusionQuaternion normalizeQuaternion(const FusionQuaternion &q)
  {
    const float magnitude = sqrtf((q.element.w * q.element.w) +
                                  (q.element.x * q.element.x) +
                                  (q.element.y * q.element.y) +
                                  (q.element.z * q.element.z));
    if (magnitude <= 1.0e-6f)
    {
      return FUSION_QUATERNION_IDENTITY;
    }

    FusionQuaternion out = FUSION_QUATERNION_IDENTITY;
    out.element.w = q.element.w / magnitude;
    out.element.x = q.element.x / magnitude;
    out.element.y = q.element.y / magnitude;
    out.element.z = q.element.z / magnitude;
    return out;
  }

  FusionQuaternion quaternionFromTwoVectors(const vec3f_t &from, const vec3f_t &to)
  {
    const vec3f_t f = normalize(from, makeVec3(0.0f, 0.0f, 1.0f));
    const vec3f_t t = normalize(to, makeVec3(0.0f, 0.0f, 1.0f));
    const float d = clampFloat(dot(f, t), -1.0f, 1.0f);

    if (d > 0.9999f)
    {
      return FUSION_QUATERNION_IDENTITY;
    }

    if (d < -0.9999f)
    {
      const vec3f_t axis = (fabsf(f.x) < 0.5f) ? makeVec3(1.0f, 0.0f, 0.0f)
                                                : makeVec3(0.0f, 1.0f, 0.0f);
      const vec3f_t ortho = normalize(cross(f, axis), makeVec3(0.0f, 1.0f, 0.0f));
      FusionQuaternion q = FUSION_QUATERNION_IDENTITY;
      q.element.w = 0.0f;
      q.element.x = ortho.x;
      q.element.y = ortho.y;
      q.element.z = ortho.z;
      return normalizeQuaternion(q);
    }

    const vec3f_t c = cross(f, t);
    FusionQuaternion q = FUSION_QUATERNION_IDENTITY;
    q.element.w = 1.0f + d;
    q.element.x = c.x;
    q.element.y = c.y;
    q.element.z = c.z;
    return normalizeQuaternion(q);
  }

  bool accelLooksLikeGravity(float accel_mag_mss)
  {
    return fabsf(accel_mag_mss - app_config::kEstimatorGravityMss) <=
           app_config::kEstimatorStartupAccelToleranceMss;
  }

  bool gyroLooksStationary(float gyro_mag_rads)
  {
    return gyro_mag_rads <= app_config::kEstimatorStartupGyroStationaryRads;
  }

  bool startupIdle(const flight_state_t &state, float accel_mag_mss, float gyro_mag_rads)
  {
    return (state.phase == flight_phase_t::STARTUP) &&
           accelLooksLikeGravity(accel_mag_mss) &&
           gyroLooksStationary(gyro_mag_rads);
  }

  bool magLooksUsable(const flight_sample_t &sample)
  {
    if (!app_config::kEstimatorUseMagnetometer)
    {
      return false;
    }

    const vec3f_t mag = sampleVec3(sample.mag_raw);
    return norm(mag) > 1.0f;
  }

  float elapsedSeconds(uint32_t timestamp_ms)
  {
    if (!g_estimator.initialized || (timestamp_ms == g_estimator.last_timestamp_ms))
    {
      return static_cast<float>(app_config::kSensorPeriodMs) / 1000.0f;
    }

    const uint32_t dt_ms = timestamp_ms - g_estimator.last_timestamp_ms;
    return clampFloat(static_cast<float>(dt_ms) / 1000.0f, 0.001f, app_config::kEstimatorMaxDtS);
  }

  FusionVector makeFusionVector(float x, float y, float z)
  {
    FusionVector v = FUSION_VECTOR_ZERO;
    v.axis.x = x;
    v.axis.y = y;
    v.axis.z = z;
    return v;
  }

  FusionVector accelToFusionG(const vec3f_t &accel_mss)
  {
    const float inv_g = 1.0f / app_config::kEstimatorGravityMss;
    return makeFusionVector(accel_mss.x * inv_g, accel_mss.y * inv_g, accel_mss.z * inv_g);
  }

  FusionVector gyroToFusionDps(const vec3f_t &gyro_rads)
  {
    constexpr float kRadToDeg = 57.29577951308232f;
    return makeFusionVector((gyro_rads.x - g_estimator.gyro_bias_rads.x) * kRadToDeg,
                            (gyro_rads.y - g_estimator.gyro_bias_rads.y) * kRadToDeg,
                            (gyro_rads.z - g_estimator.gyro_bias_rads.z) * kRadToDeg);
  }

  FusionVector magToFusion(const flight_sample_t &sample)
  {
    return makeFusionVector(sample.mag_raw[0], sample.mag_raw[1], sample.mag_raw[2]);
  }

  FusionAhrsSettings ahrsSettings(bool gyro_only)
  {
    FusionAhrsSettings settings = fusionAhrsDefaultSettings;
    settings.convention = FusionConventionEnu;
    settings.gain = gyro_only ? app_config::kEstimatorAhrsFlightGain
                              : app_config::kEstimatorAhrsStaticGain;
    settings.gyroscopeRange = app_config::kEstimatorAhrsGyroRangeDps;
    settings.accelerationRejection = app_config::kEstimatorAhrsAccelerationRejectionDeg;
    settings.magneticRejection = app_config::kEstimatorAhrsMagneticRejectionDeg;
    settings.recoveryTriggerPeriod = static_cast<unsigned int>(app_config::kEstimatorAhrsRecoveryPeriodSamples);
    return settings;
  }

  void configureAhrs(ahrs_mode_t mode)
  {
    if (g_estimator.ahrs_mode == ahrs_mode_t::UNINITIALIZED)
    {
      FusionAhrsInitialise(&g_estimator.ahrs);
    }

    if (g_estimator.ahrs_mode != mode)
    {
      const bool gyro_only = (mode == ahrs_mode_t::GYRO_ONLY);
      const FusionAhrsSettings settings = ahrsSettings(gyro_only);
      FusionAhrsSetSettings(&g_estimator.ahrs, &settings);
      g_estimator.ahrs_mode = mode;
    }
  }

  void initialiseAhrsFromGravity(const vec3f_t &accel_mss)
  {
    const vec3f_t up_body = normalize(accel_mss, makeVec3(0.0f, 0.0f, 1.0f));
    const vec3f_t up_world = makeVec3(0.0f, 0.0f, 1.0f);
    const FusionQuaternion q = quaternionFromTwoVectors(up_body, up_world);
    g_estimator.body_up = up_body;
    FusionAhrsSetQuaternion(&g_estimator.ahrs, q);
  }

  vec3f_t estimateStaticAccelBias(const vec3f_t &accel_mss)
  {
    const vec3f_t measured_gravity_body = normalize(accel_mss, makeVec3(0.0f, 0.0f, 1.0f));
    const vec3f_t expected_accel_mss = scale(measured_gravity_body, app_config::kEstimatorGravityMss);
    return subtract(accel_mss, expected_accel_mss);
  }

  void updateImuBiasDrift()
  {
    if (!g_estimator.imu_calibration_reference_set)
    {
      g_estimator.accel_bias_drift_mss = {};
      g_estimator.gyro_bias_drift_rads = {};
      return;
    }

    g_estimator.accel_bias_drift_mss =
        subtract(g_estimator.accel_bias_mss, g_estimator.accel_bias_reference_mss);
    g_estimator.gyro_bias_drift_rads =
        subtract(g_estimator.gyro_bias_rads, g_estimator.gyro_bias_reference_rads);
  }

  void updateImuCalibration(const vec3f_t &accel_mss,
                            const vec3f_t &gyro_rads,
                            float dt_s,
                            bool accept_sample)
  {
    if (g_estimator.imu_bias_frozen || !accept_sample)
    {
      return;
    }

    const vec3f_t accel_bias_sample = estimateStaticAccelBias(accel_mss);
    if (g_estimator.imu_calibration_samples == 0U)
    {
      g_estimator.accel_bias_mss = accel_bias_sample;
      g_estimator.gyro_bias_rads = gyro_rads;
      g_estimator.accel_bias_reference_mss = accel_bias_sample;
      g_estimator.gyro_bias_reference_rads = gyro_rads;
      g_estimator.imu_calibration_reference_set = true;
      g_estimator.imu_calibration_samples = 1U;
      updateImuBiasDrift();
      return;
    }

    const float alpha = clampFloat(dt_s / (app_config::kEstimatorGyroBiasTauS + dt_s), 0.0f, 1.0f);
    g_estimator.accel_bias_mss.x += alpha * (accel_bias_sample.x - g_estimator.accel_bias_mss.x);
    g_estimator.accel_bias_mss.y += alpha * (accel_bias_sample.y - g_estimator.accel_bias_mss.y);
    g_estimator.accel_bias_mss.z += alpha * (accel_bias_sample.z - g_estimator.accel_bias_mss.z);
    g_estimator.gyro_bias_rads.x += alpha * (gyro_rads.x - g_estimator.gyro_bias_rads.x);
    g_estimator.gyro_bias_rads.y += alpha * (gyro_rads.y - g_estimator.gyro_bias_rads.y);
    g_estimator.gyro_bias_rads.z += alpha * (gyro_rads.z - g_estimator.gyro_bias_rads.z);

    if (g_estimator.imu_calibration_samples < 0xFFFFFFFFUL)
    {
      ++g_estimator.imu_calibration_samples;
    }
    updateImuBiasDrift();
  }

  bool imuCalibrationComplete()
  {
    return g_estimator.imu_calibration_samples >= app_config::kEstimatorImuCalibrationMinSamples;
  }

  bool imuBiasAvailable()
  {
    return g_estimator.imu_calibration_samples > 0U;
  }

  bool launchAccelerationSeen(const flight_state_t &state, float accel_mag_mss)
  {
    return (state.phase == flight_phase_t::STARTUP) &&
           (accel_mag_mss >= app_config::kLaunchAccelThresholdMss);
  }

  void freezeImuBiases()
  {
    if (!g_estimator.imu_bias_frozen)
    {
      g_estimator.imu_bias_frozen = true;
      updateImuBiasDrift();
    }
  }

  vec3f_t applyAccelBiasIfReady(const vec3f_t &accel_mss, bool apply_bias)
  {
    if (!apply_bias)
    {
      return accel_mss;
    }
    return subtract(accel_mss, g_estimator.accel_bias_mss);
  }

  double verticalAccelerationFromStartupAxis(const vec3f_t &accel_mss)
  {
    return static_cast<double>(dot(accel_mss, g_estimator.body_up)) -
           static_cast<double>(app_config::kEstimatorGravityMss);
  }

  void updateImuCalibrationStatus(bool calibration_active,
                                  bool calibration_sample_accepted,
                                  bool bias_applied)
  {
    g_estimator.status.imu_calibration_active = calibration_active;
    g_estimator.status.imu_calibration_sample_accepted = calibration_sample_accepted;
    g_estimator.status.imu_calibration_complete = imuCalibrationComplete();
    g_estimator.status.imu_bias_frozen = g_estimator.imu_bias_frozen;
    g_estimator.status.imu_bias_applied = bias_applied;
    g_estimator.status.imu_calibration_samples = g_estimator.imu_calibration_samples;
    g_estimator.status.body_up[0] = g_estimator.body_up.x;
    g_estimator.status.body_up[1] = g_estimator.body_up.y;
    g_estimator.status.body_up[2] = g_estimator.body_up.z;

    g_estimator.status.accel_bias_mss[0] = g_estimator.accel_bias_mss.x;
    g_estimator.status.accel_bias_mss[1] = g_estimator.accel_bias_mss.y;
    g_estimator.status.accel_bias_mss[2] = g_estimator.accel_bias_mss.z;
    g_estimator.status.gyro_bias_rads[0] = g_estimator.gyro_bias_rads.x;
    g_estimator.status.gyro_bias_rads[1] = g_estimator.gyro_bias_rads.y;
    g_estimator.status.gyro_bias_rads[2] = g_estimator.gyro_bias_rads.z;
    g_estimator.status.accel_bias_drift_mss[0] = g_estimator.accel_bias_drift_mss.x;
    g_estimator.status.accel_bias_drift_mss[1] = g_estimator.accel_bias_drift_mss.y;
    g_estimator.status.accel_bias_drift_mss[2] = g_estimator.accel_bias_drift_mss.z;
    g_estimator.status.gyro_bias_drift_rads[0] = g_estimator.gyro_bias_drift_rads.x;
    g_estimator.status.gyro_bias_drift_rads[1] = g_estimator.gyro_bias_drift_rads.y;
    g_estimator.status.gyro_bias_drift_rads[2] = g_estimator.gyro_bias_drift_rads.z;
  }

  vec3f_t updateAhrs(const flight_sample_t &sample,
                     const vec3f_t &accel_mss,
                     const vec3f_t &gyro_rads,
                     float dt_s,
                     bool idle)
  {
    const ahrs_mode_t mode = idle ? ahrs_mode_t::STATIC : ahrs_mode_t::GYRO_ONLY;
    const bool use_mag = idle && magLooksUsable(sample);
    configureAhrs(mode);
    if (idle)
    {
      initialiseAhrsFromGravity(accel_mss);
    }

    const FusionVector gyro_dps = gyroToFusionDps(gyro_rads);
    const FusionVector accel_g = accelToFusionG(accel_mss);

    if (use_mag)
    {
      FusionAhrsUpdate(&g_estimator.ahrs, gyro_dps, accel_g, magToFusion(sample), dt_s);
    }
    else
    {
      FusionAhrsUpdateNoMagnetometer(&g_estimator.ahrs, gyro_dps, accel_g, dt_s);
    }

    const FusionVector earth_accel_g = FusionAhrsGetEarthAcceleration(&g_estimator.ahrs);
    g_estimator.status.ahrs_initialized = true;
    g_estimator.status.ahrs_gyro_only = (mode == ahrs_mode_t::GYRO_ONLY);
    g_estimator.status.ahrs_using_mag = use_mag;

    const FusionAhrsInternalStates internal = FusionAhrsGetInternalStates(&g_estimator.ahrs);
    g_estimator.status.ahrs_accel_rejected = internal.accelerometerIgnored;
    g_estimator.status.ahrs_mag_rejected = internal.magnetometerIgnored;

    return makeVec3(earth_accel_g.axis.x * app_config::kEstimatorGravityMss,
                    earth_accel_g.axis.y * app_config::kEstimatorGravityMss,
                    earth_accel_g.axis.z * app_config::kEstimatorGravityMss);
  }

  void updateBarometerReference(float raw_altitude_m, float dt_s)
  {
    if (!g_estimator.have_baro)
    {
      g_estimator.baro_altitude_m = raw_altitude_m;
      g_estimator.have_baro = true;
      return;
    }

    const float alpha = clampFloat(dt_s / (app_config::kEstimatorBaroTauS + dt_s), 0.0f, 1.0f);
    g_estimator.baro_altitude_m += alpha * (raw_altitude_m - g_estimator.baro_altitude_m);
  }

  void initialiseNavigation(float altitude_m)
  {
    for (size_t row = 0; row < kStateCount; ++row)
    {
      g_estimator.x[row] = 0.0;
      for (size_t col = 0; col < kStateCount; ++col)
      {
        g_estimator.p[row][col] = 0.0;
      }
    }

    g_estimator.x[2] = altitude_m;
    g_estimator.p[0][0] = square(app_config::kEstimatorInitialHorizontalPositionSigmaM);
    g_estimator.p[1][1] = square(app_config::kEstimatorInitialHorizontalPositionSigmaM);
    g_estimator.p[2][2] = square(app_config::kEstimatorInitialVerticalPositionSigmaM);
    g_estimator.p[3][3] = square(app_config::kEstimatorInitialVelocitySigmaMps);
    g_estimator.p[4][4] = square(app_config::kEstimatorInitialVelocitySigmaMps);
    g_estimator.p[5][5] = square(app_config::kEstimatorInitialVelocitySigmaMps);
  }

  void holdIdleNavigation(float altitude_m)
  {
    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        g_estimator.p[row][col] = 0.0;
      }
    }

    g_estimator.x[0] = 0.0;
    g_estimator.x[1] = 0.0;
    g_estimator.x[2] = altitude_m;
    g_estimator.x[3] = 0.0;
    g_estimator.x[4] = 0.0;
    g_estimator.x[5] = 0.0;

    g_estimator.p[0][0] = square(app_config::kEstimatorInitialHorizontalPositionSigmaM);
    g_estimator.p[1][1] = square(app_config::kEstimatorInitialHorizontalPositionSigmaM);
    g_estimator.p[2][2] = square(app_config::kEstimatorBaroSigmaM);
    g_estimator.p[3][3] = square(app_config::kEstimatorInitialVelocitySigmaMps);
    g_estimator.p[4][4] = square(app_config::kEstimatorInitialVelocitySigmaMps);
    g_estimator.p[5][5] = square(app_config::kEstimatorInitialVelocitySigmaMps);
  }

  void processAccelSigma(const flight_state_t &state,
                         bool idle,
                         double &horizontal_sigma_mss,
                         double &vertical_sigma_mss)
  {
    if (idle)
    {
      horizontal_sigma_mss = app_config::kEstimatorIdleAccelSigmaMss;
      vertical_sigma_mss = app_config::kEstimatorIdleAccelSigmaMss;
      return;
    }

    if (state.phase == flight_phase_t::DESCENT ||
        state.phase == flight_phase_t::TOUCHDOWN ||
        state.phase == flight_phase_t::COMPLETE)
    {
      horizontal_sigma_mss = app_config::kEstimatorDescentHorizontalAccelSigmaMss;
      vertical_sigma_mss = app_config::kEstimatorDescentVerticalAccelSigmaMss;
      return;
    }

    horizontal_sigma_mss = app_config::kEstimatorFlightHorizontalAccelSigmaMss;
    vertical_sigma_mss = app_config::kEstimatorFlightVerticalAccelSigmaMss;
  }

  void predictNavigation(const vec3f_t &accel_enu_mss,
                         const flight_state_t &state,
                         bool idle,
                         double dt_s)
  {
    const double dt2 = dt_s * dt_s;

    const double accel[kAxisCount] = {
        static_cast<double>(accel_enu_mss.x),
        static_cast<double>(accel_enu_mss.y),
        static_cast<double>(accel_enu_mss.z),
    };

    for (size_t axis = 0; axis < kAxisCount; ++axis)
    {
      g_estimator.x[kPositionIndex + axis] +=
          (g_estimator.x[kVelocityIndex + axis] * dt_s) + (0.5 * accel[axis] * dt2);
      g_estimator.x[kVelocityIndex + axis] += accel[axis] * dt_s;
    }

    double f[kStateCount][kStateCount] = {};
    for (size_t i = 0; i < kStateCount; ++i)
    {
      f[i][i] = 1.0;
    }
    for (size_t axis = 0; axis < kAxisCount; ++axis)
    {
      f[kPositionIndex + axis][kVelocityIndex + axis] = dt_s;
    }

    double fp[kStateCount][kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        for (size_t k = 0; k < kStateCount; ++k)
        {
          fp[row][col] += f[row][k] * g_estimator.p[k][col];
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

    double horizontal_sigma_mss = 0.0;
    double vertical_sigma_mss = 0.0;
    processAccelSigma(state, idle, horizontal_sigma_mss, vertical_sigma_mss);

    const double dt3 = dt2 * dt_s;
    const double dt4 = dt2 * dt2;
    const double sigma_mss[kAxisCount] = {
        horizontal_sigma_mss,
        horizontal_sigma_mss,
        vertical_sigma_mss,
    };

    for (size_t axis = 0; axis < kAxisCount; ++axis)
    {
      const double qa = square(sigma_mss[axis]);
      const size_t p_index = kPositionIndex + axis;
      const size_t v_index = kVelocityIndex + axis;

      next_p[p_index][p_index] += 0.25 * dt4 * qa;
      next_p[p_index][v_index] += 0.5 * dt3 * qa;
      next_p[v_index][p_index] += 0.5 * dt3 * qa;
      next_p[v_index][v_index] += dt2 * qa;
    }

    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        g_estimator.p[row][col] = 0.5 * (next_p[row][col] + next_p[col][row]);
      }
    }
  }

  bool updateScalarMeasurement(size_t state_index,
                               double measurement,
                               double variance,
                               double gate_sigma,
                               double &normalized_innovation)
  {
    variance = fmax(variance, 1.0e-6);
    const double residual = measurement - g_estimator.x[state_index];
    const double innovation_variance = g_estimator.p[state_index][state_index] + variance;
    if (innovation_variance <= 1.0e-9)
    {
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
      gain[row] = g_estimator.p[row][state_index] / innovation_variance;
      g_estimator.x[row] += gain[row] * residual;
    }

    double a[kStateCount][kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      a[row][row] = 1.0;
      a[row][state_index] -= gain[row];
    }

    double ap[kStateCount][kStateCount] = {};
    for (size_t row = 0; row < kStateCount; ++row)
    {
      for (size_t col = 0; col < kStateCount; ++col)
      {
        for (size_t k = 0; k < kStateCount; ++k)
        {
          ap[row][col] += a[row][k] * g_estimator.p[k][col];
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
        g_estimator.p[row][col] = 0.5 * (next_p[row][col] + next_p[col][row]);
      }
    }

    return true;
  }

  void updateWithBarometer(float altitude_m)
  {
    double normalized = 0.0;
    g_estimator.status.baro_update_accepted =
        updateScalarMeasurement(2U,
                                static_cast<double>(altitude_m),
                                square(app_config::kEstimatorBaroSigmaM),
                                app_config::kEstimatorBaroGateSigma,
                                normalized);
    g_estimator.status.normalized_baro_innovation = normalized;
  }

  bool gpsFresh(const gps_fix_t &gps, uint32_t timestamp_ms)
  {
    return (gps.timestamp_ms > 0U) &&
           (timestamp_ms >= gps.timestamp_ms) &&
           ((timestamp_ms - gps.timestamp_ms) <= app_config::kEstimatorGpsMaxAgeMs);
  }

  uint32_t gpsMeasurementId(const gps_fix_t &gps)
  {
    return (gps.i_tow_ms != 0U) ? gps.i_tow_ms : gps.timestamp_ms;
  }

  double gpsPositionVariance(double sigma_m)
  {
    const double bounded_sigma_m = fmax(sigma_m, app_config::kEstimatorMinGpsHorizontalSigmaM);
    return square(bounded_sigma_m);
  }

  double gpsVerticalVariance(double sigma_m)
  {
    const double bounded_sigma_m = fmax(sigma_m, app_config::kEstimatorMinGpsVerticalSigmaM);
    return square(bounded_sigma_m);
  }

  double gpsVelocityVariance(double sigma_mps)
  {
    const double bounded_sigma_mps = fmax(sigma_mps, app_config::kEstimatorMinGpsVelocitySigmaMps);
    return square(bounded_sigma_mps);
  }

  void updateWithGpsPosition(const gps_fix_t &gps, uint32_t timestamp_ms)
  {
    g_estimator.status.gps_position_update_accepted = false;
    g_estimator.status.normalized_gps_position_innovation = 0.0;

    const uint32_t measurement_id = gpsMeasurementId(gps);
    if (!gps.position_valid || !gpsFresh(gps, timestamp_ms) ||
        (measurement_id == g_estimator.last_gps_position_measurement_id))
    {
      return;
    }

    double largest_normalized = 0.0;
    bool accepted_any = false;
    double normalized = 0.0;

    if (updateScalarMeasurement(0U,
                                gps.position_m.x,
                                gpsPositionVariance(gps.horizontal_accuracy_m),
                                app_config::kEstimatorGpsPositionGateSigma,
                                normalized))
    {
      accepted_any = true;
      largest_normalized = fmax(largest_normalized, normalized);
    }

    if (updateScalarMeasurement(1U,
                                gps.position_m.y,
                                gpsPositionVariance(gps.horizontal_accuracy_m),
                                app_config::kEstimatorGpsPositionGateSigma,
                                normalized))
    {
      accepted_any = true;
      largest_normalized = fmax(largest_normalized, normalized);
    }

    if (gps.altitude_valid &&
        updateScalarMeasurement(2U,
                                gps.position_m.z,
                                gpsVerticalVariance(gps.vertical_accuracy_m),
                                app_config::kEstimatorGpsPositionGateSigma,
                                normalized))
    {
      accepted_any = true;
      largest_normalized = fmax(largest_normalized, normalized);
    }

    if (accepted_any)
    {
      g_estimator.last_gps_position_measurement_id = measurement_id;
      g_estimator.status.last_gps_position_update_ms = gps.timestamp_ms;
    }

    g_estimator.status.gps_position_update_accepted = accepted_any;
    g_estimator.status.normalized_gps_position_innovation = largest_normalized;
  }

  void updateWithGpsVelocity(const gps_fix_t &gps, uint32_t timestamp_ms)
  {
    g_estimator.status.gps_velocity_update_accepted = false;
    g_estimator.status.normalized_gps_velocity_innovation = 0.0;

    const uint32_t measurement_id = gpsMeasurementId(gps);
    if ((!gps.velocity_valid && !gps.vertical_velocity_valid) || !gpsFresh(gps, timestamp_ms) ||
        (measurement_id == g_estimator.last_gps_velocity_measurement_id))
    {
      return;
    }

    const double variance = gpsVelocityVariance(gps.speed_accuracy_mps);
    double largest_normalized = 0.0;
    bool accepted_any = false;
    double normalized = 0.0;

    if (gps.velocity_valid &&
        updateScalarMeasurement(3U,
                                gps.velocity_mps.x,
                                variance,
                                app_config::kEstimatorGpsVelocityGateSigma,
                                normalized))
    {
      accepted_any = true;
      largest_normalized = fmax(largest_normalized, normalized);
    }

    if (gps.velocity_valid &&
        updateScalarMeasurement(4U,
                                gps.velocity_mps.y,
                                variance,
                                app_config::kEstimatorGpsVelocityGateSigma,
                                normalized))
    {
      accepted_any = true;
      largest_normalized = fmax(largest_normalized, normalized);
    }

    if (gps.vertical_velocity_valid &&
        updateScalarMeasurement(5U,
                                gps.velocity_mps.z,
                                variance,
                                app_config::kEstimatorGpsVelocityGateSigma,
                                normalized))
    {
      accepted_any = true;
      largest_normalized = fmax(largest_normalized, normalized);
    }

    if (accepted_any)
    {
      g_estimator.last_gps_velocity_measurement_id = measurement_id;
      g_estimator.status.last_gps_velocity_update_ms = gps.timestamp_ms;
    }

    g_estimator.status.gps_velocity_update_accepted = accepted_any;
    g_estimator.status.normalized_gps_velocity_innovation = largest_normalized;
  }

  void updateGpsOutageStatus(const gps_fix_t &gps, uint32_t timestamp_ms)
  {
    g_estimator.status.gps_outage =
        !gps.valid || (gps.timestamp_ms == 0U) ||
        (timestamp_ms < gps.timestamp_ms) ||
        ((timestamp_ms - gps.timestamp_ms) > app_config::kEstimatorGpsOutageMs);
  }

  double finiteOrZero(double value)
  {
    return isfinite(value) ? value : 0.0;
  }

  void publishState(flight_state_t &state)
  {
    const FusionQuaternion q = FusionAhrsGetQuaternion(&g_estimator.ahrs);

    state.position_m.x = finiteOrZero(g_estimator.x[0]);
    state.position_m.y = finiteOrZero(g_estimator.x[1]);
    state.position_m.z = finiteOrZero(g_estimator.x[2]);
    state.velocity_mps_3d.x = finiteOrZero(g_estimator.x[3]);
    state.velocity_mps_3d.y = finiteOrZero(g_estimator.x[4]);
    state.velocity_mps_3d.z = finiteOrZero(g_estimator.x[5]);
    state.altitude_m = state.position_m.z;
    state.velocity_mps = state.velocity_mps_3d.z;
    state.orientation_q.w = static_cast<double>(q.element.w);
    state.orientation_q.x = static_cast<double>(q.element.x);
    state.orientation_q.y = static_cast<double>(q.element.y);
    state.orientation_q.z = static_cast<double>(q.element.z);

    for (size_t axis = 0; axis < kAxisCount; ++axis)
    {
      g_estimator.status.covariance_position_m2[axis] =
          finiteOrZero(g_estimator.p[kPositionIndex + axis][kPositionIndex + axis]);
      g_estimator.status.covariance_velocity_m2ps2[axis] =
          finiteOrZero(g_estimator.p[kVelocityIndex + axis][kVelocityIndex + axis]);
    }
    state.estimator = g_estimator.status;
  }

  void resetTransientStatus()
  {
    const uint32_t last_gps_position_update_ms = g_estimator.status.last_gps_position_update_ms;
    const uint32_t last_gps_velocity_update_ms = g_estimator.status.last_gps_velocity_update_ms;
    g_estimator.status = estimator_status_t{};
    g_estimator.status.last_gps_position_update_ms = last_gps_position_update_ms;
    g_estimator.status.last_gps_velocity_update_ms = last_gps_velocity_update_ms;
  }

} // namespace

namespace state_estimator
{

  void reset()
  {
    g_estimator = estimator_state_t{};
    vertical_filter::reset();
  }

  void restoreCalibration(const float body_up[3],
                          const float accel_bias_mss[3],
                          const float gyro_bias_rads[3])
  {
    g_estimator.body_up = {body_up[0], body_up[1], body_up[2]};
    const float n = norm(g_estimator.body_up);
    if (n > 0.5f)
    {
      g_estimator.body_up = {g_estimator.body_up.x / n, g_estimator.body_up.y / n, g_estimator.body_up.z / n};
    }
    else
    {
      g_estimator.body_up = {0.0f, 0.0f, 1.0f};
    }
    g_estimator.accel_bias_mss = {accel_bias_mss[0], accel_bias_mss[1], accel_bias_mss[2]};
    g_estimator.gyro_bias_rads = {gyro_bias_rads[0], gyro_bias_rads[1], gyro_bias_rads[2]};
    g_estimator.accel_bias_reference_mss = g_estimator.accel_bias_mss;
    g_estimator.gyro_bias_reference_rads = g_estimator.gyro_bias_rads;
    g_estimator.imu_calibration_reference_set = true;
    g_estimator.imu_calibration_samples = app_config::kEstimatorImuCalibrationMinSamples;
    g_estimator.imu_bias_frozen = true;
  }

  void update(const flight_sample_t &sample, flight_state_t &state)
  {
    resetTransientStatus();

    const vec3f_t accel_mss = sampleVec3(sample.accel_mss);
    const vec3f_t gyro_rads = sampleVec3(sample.gyro_rads);
    const float accel_mag_mss = norm(accel_mss);
    const float gyro_mag_rads = norm(gyro_rads);
    const float dt_s = elapsedSeconds(sample.timestamp_ms);
    const bool idle = startupIdle(state, accel_mag_mss, gyro_mag_rads);
    const bool calibration_active = (state.phase == flight_phase_t::STARTUP) && !g_estimator.imu_bias_frozen;
    const bool calibration_sample_accepted = calibration_active && idle;

    updateImuCalibration(accel_mss, gyro_rads, dt_s, calibration_sample_accepted);
    if ((state.phase != flight_phase_t::STARTUP) || launchAccelerationSeen(state, accel_mag_mss))
    {
      freezeImuBiases();
    }

    const bool imu_bias_applied = g_estimator.imu_bias_frozen && imuBiasAvailable();
    const vec3f_t calibrated_accel_mss = applyAccelBiasIfReady(accel_mss, imu_bias_applied);

    updateBarometerReference(sample.altitude_m, dt_s);

    const vec3f_t earth_accel_mss = updateAhrs(sample, calibrated_accel_mss, gyro_rads, dt_s, idle);
    const double vertical_accel_mss = verticalAccelerationFromStartupAxis(accel_mss);

    if (!g_estimator.initialized)
    {
      initialiseNavigation(g_estimator.baro_altitude_m);
    }

    if (state.phase == flight_phase_t::STARTUP)
    {
      holdIdleNavigation(g_estimator.baro_altitude_m);
    }
    else
    {
      predictNavigation(earth_accel_mss, state, idle, static_cast<double>(dt_s));
      if (sample.baro_fresh)
      {
        updateWithBarometer(sample.altitude_m);
      }
      updateWithGpsPosition(state.gps, sample.timestamp_ms);
      updateWithGpsVelocity(state.gps, sample.timestamp_ms);
    }

    updateGpsOutageStatus(state.gps, sample.timestamp_ms);
    updateImuCalibrationStatus((state.phase == flight_phase_t::STARTUP) && !g_estimator.imu_bias_frozen,
                               calibration_sample_accepted,
                               imu_bias_applied);

    g_estimator.initialized = true;
    g_estimator.last_timestamp_ms = sample.timestamp_ms;
    publishState(state);
    vertical_filter::update(sample, state, vertical_accel_mss, static_cast<double>(dt_s), idle);
  }

} // namespace state_estimator
