#pragma once

#include <Arduino.h>

#ifndef FLIGHT_DEBUG_MODE
#define FLIGHT_DEBUG_MODE 0
#endif

#ifndef DISABLE_PYROS
#define DISABLE_PYROS 0
#endif

#ifndef FLIGHT_DEBUG_STREAM_HZ
#define FLIGHT_DEBUG_STREAM_HZ 20
#endif

#ifndef ENABLE_WATCHDOG
#define ENABLE_WATCHDOG 1
#endif

namespace app_config
{

    constexpr bool kFlightDebugMode = (FLIGHT_DEBUG_MODE != 0);
    constexpr bool kDisablePyros = (DISABLE_PYROS != 0) || kFlightDebugMode;

    /* ====== RTOS TASK FREQUENCIES ======
       The sensor task is the master clock of the flight computer. Everything that was
       previously counted in "samples" is now expressed in milliseconds and converted
       below, so changing kSensorPeriodMs does not silently change flight behaviour. */
    constexpr uint32_t kDebugStreamHz =
        (FLIGHT_DEBUG_STREAM_HZ > 0) ? static_cast<uint32_t>(FLIGHT_DEBUG_STREAM_HZ) : 1U;
    constexpr uint32_t kDebugStreamPeriodMs = 1000U / kDebugStreamHz;

    constexpr uint32_t kSensorPeriodMs = 10;      // 100 Hz IMU + flight logic
    constexpr uint32_t kSupervisorPeriodMs = 50;  // watchdog / backup-deploy / pyro timer check
    constexpr uint32_t kTelemetryPeriodMs = 200;  // fast state packet, 5 Hz (see radio notes below)
    constexpr uint32_t kHealthPeriodMs = 1000;    // health/diagnostic packet, 1 Hz
    constexpr uint32_t kRadioPollPeriodMs = 5;
    constexpr uint32_t kGpsPollPeriodMs = 10;
    constexpr uint32_t kLoggerFlushEveryNLines = 50;
    constexpr uint32_t kBaselineSamples = 16;
    constexpr bool kLogStartupSamplesToSd = kFlightDebugMode;

    constexpr uint32_t msToSamples(uint32_t ms)
    {
        return (ms + kSensorPeriodMs - 1U) / kSensorPeriodMs;
    }

    /* ====== BAROMETER (MS56xx) CONVERSION ======
       The barometer is driven as a non-blocking state machine: a conversion is started
       on one sensor cycle and read on the next, so the sensor task never sleeps inside
       the barometer driver. The conversion time must be shorter than kSensorPeriodMs.
         OSR 4096: cmd 0x48/0x58, 9.04 ms max  (too close to a 10 ms period)
         OSR 2048: cmd 0x46/0x56, 4.54 ms max  (default)
         OSR 1024: cmd 0x44/0x54, 2.28 ms max */
    constexpr uint8_t kBaroConvertD1Cmd = 0x46; // pressure, OSR 2048
    constexpr uint8_t kBaroConvertD2Cmd = 0x56; // temperature, OSR 2048
    constexpr uint32_t kBaroConversionMs = 5;   // must be < kSensorPeriodMs
    constexpr uint32_t kBaroTemperatureEveryN = 10; // temperature conversion every N pressure reads
    static_assert(kBaroConversionMs < kSensorPeriodMs, "barometer conversion must fit inside one sensor period");

    /* ====== STATE TRANSITION THRESHOLDS ====== */
    constexpr double kLaunchAccelThresholdMss = 40.0;  // STARTUP to BOOST detect
    constexpr uint32_t kLaunchConfirmMs = 150;         // BOOST transition must persist this long (was 5 samples @ 33 Hz)
    constexpr uint32_t kBoostEndConfirmMs = 150;       // COAST transition must persist this long
    constexpr double kApogeeArmAltM = 100.0;           // DESCENT only beyond 100 m AGL
    constexpr double kApogeeDescentVelMps = -3.0;      // DESCENT min descent rate
    constexpr uint32_t kApogeeConfirmMs = 450;         // DESCENT transition must persist this long (was 15 samples)
    constexpr double kDeployAltM = 123.0;              // DEPLOY ALTITUDE 400FT
    constexpr double kTouchdownAltMarginM = 20.0;      // TOUCHDOWN <= 20m
    constexpr double kTouchdownVelocityLimitMps = 5.0; // TOUCHDOWN vertical speed <= 5m/s
    constexpr uint32_t kTouchdownConfirmMs = 750;      // TOUCHDOWN transition must persist this long (was 25 samples)
    constexpr uint32_t kPostTouchdownLogMs = 10000;    // Log 10 more seconds
    constexpr uint32_t kPyroOnMs = 2000;               // Igniter on-time. Was 5000 ms; check your igniter spec.
    constexpr uint32_t kStartupBuzzerMs = 1000;
    constexpr uint16_t kStartupBuzzerHz = 2730;

    /* ====== PHASE TIMEOUTS (safety net) ======
       Any phase that overruns its timeout is forced forward so a bad estimate cannot
       stall the state machine. 0 disables a timeout. */
    constexpr uint32_t kMaxBoostMs = 15000;   // BOOST -> COAST after this even if accel never drops
    constexpr uint32_t kMaxCoastMs = 60000;   // COAST -> DESCENT (arms pyro) after this even if apogee never detected

    /* ====== BACKUP DEPLOYMENT TIMER (safety net) ======
       Fires the parachute kBackupDeployAfterLaunchMs after launch detection regardless of
       the estimator, the sample queue or the flight task. SET THIS FROM YOUR SIMULATION:
       expected time to apogee + margin. 0 disables the timer.
       Deploying during boost can destroy the rocket, so do not guess this value. */
    constexpr uint32_t kBackupDeployAfterLaunchMs = 0;

    /* ====== WATCHDOG / SUPERVISOR (safety net) ======
       The supervisor task feeds the hardware watchdog only while the sensor and flight
       tasks are both producing heartbeats. If either stalls, the Teensy resets, the
       persisted flight state is restored and the flight continues. */
    constexpr bool kEnableWatchdog = (ENABLE_WATCHDOG != 0);
    constexpr uint32_t kWatchdogTimeoutS = 2;         // WDOG1 granularity is 0.5 s, min 0.5 s, max 128 s
    constexpr uint32_t kHeartbeatSensorMaxMs = 200;   // sensor task must beat at least this often
    constexpr uint32_t kHeartbeatFlightMaxMs = 300;   // flight task must beat at least this often
    constexpr uint32_t kHeartbeatGraceAfterBootMs = 3000; // allow slow startup before enforcing

    /* ====== FLIGHT STATE PERSISTENCE (safety net) ======
       Phase, home altitude, barometer baseline and IMU calibration are written to EEPROM
       on every phase transition. After a WATCHDOG reset (and only a watchdog reset, so a
       stale record can never arm pyros on the bench) the flight resumes from the record. */
    constexpr bool kEnablePersistence = true;
    constexpr uint8_t kMaxInFlightRestores = 3;       // after this many in-flight resets, deploy immediately
    constexpr uint32_t kPersistRefreshMs = 1000;      // in flight, refresh the record (elapsed time) this often
    constexpr uint32_t kRestoreSettleMs = 1500;       // after a restore, let the filters converge before trusting apogee detection
    constexpr uint8_t kPersistSchemaVersion = 1;

    /* ====== QUEUE AND BUFFER SIZES ====== */
    constexpr size_t kSampleQueueDepth = 16;
    constexpr size_t kLogQueueDepth = 64;             // ~640 ms of samples at 100 Hz; SD write latency spikes are long
    constexpr size_t kDebugQueueDepth = 4;
    constexpr size_t kMaxPacketLen = 255;
    constexpr size_t kStartupLogWindowSamples = 300; // ~3 seconds at 100 Hz.

    /* ====== RADIO CONFIGURATION ======
       Airtime at SF7 / CR 4/5 / 8-symbol preamble, including the 4-byte RadioHead header:
                          125 kHz   250 kHz   500 kHz
         state packet     ~118 ms   ~59 ms    ~30 ms
         health packet    ~160 ms   ~80 ms    ~40 ms
       At 125 kHz the default 5 Hz state + 1 Hz health keeps the radio ~75% busy, which is
       the ceiling. To go to 10 Hz state, set kRadioModemConfig1 to 0x82 (250 kHz) AND
       change the ground receiver to match; each doubling of bandwidth costs ~3 dB of range. */
    constexpr float kRadioFreqMHz = 427.760f;
    constexpr int8_t kRadioTxPowerDbm = 20;
    constexpr uint8_t kRadioModemConfig1 = 0x72;      // 0x72 = 125 kHz, 0x82 = 250 kHz, 0x92 = 500 kHz (all CR 4/5, explicit header)
    constexpr uint8_t kRadioModemConfig2 = 0x74;      // SF7, CRC on
    constexpr uint8_t kRadioModemConfig3 = 0x04;      // AGC auto

    /* ====== HARDWARE ENABLE FLAGS ====== */
    constexpr bool kEnableGps = true;
    constexpr bool kEnableSdLogging = true;
    constexpr bool kEnableMagnetometer = true;

    /* ====== IMU CONFIGURATION (BMI088) ======
       Set explicitly rather than trusting library defaults. The anti-alias bandwidth
       should be below half the sensor rate. */
    constexpr bool kImuConfigureRates = true;

    /* ====== GPS CONFIGURATION ====== */
    constexpr uint8_t kGpsNavFrequencyHz = 10;
    constexpr uint8_t kGpsI2cAddress = 0x42;
    constexpr uint16_t kGpsInitMaxWaitMs = 250;
    constexpr uint32_t kGpsI2cHz = 400000UL;
    constexpr double kGpsAltitudeBlendWeight = 0.20;
    constexpr double kGpsVelocityBlendWeight = 0.10;

    /* ====== STATE ESTIMATOR CONFIGURATION ====== */
    constexpr float kEstimatorGravityMss = 9.80665f;
    constexpr float kEstimatorMaxDtS = 0.10f;
    constexpr float kEstimatorStartupAccelToleranceMss = 2.0f;
    constexpr float kEstimatorStartupGyroStationaryRads = 0.20f;
    constexpr float kEstimatorStartupAttitudeKp = 2.0f;
    constexpr float kEstimatorGyroBiasTauS = 2.0f;
    constexpr uint32_t kEstimatorImuCalibrationMinMs = 750U;
    constexpr uint32_t kEstimatorImuCalibrationMinSamples = msToSamples(kEstimatorImuCalibrationMinMs);
    constexpr float kEstimatorBaroTauS = 0.20f;
    constexpr float kEstimatorBaroPositionGain = 0.12f;
    constexpr float kEstimatorBaroVelocityGain = 0.025f;
    constexpr float kEstimatorMaxBaroCorrectionM = 30.0f;

    /* ====== AHRS + KALMAN FILTER TUNING ======
       World frame is ENU: x=east, y=north, z=up. P is stored as 6x6.
       Q is applied as per-axis acceleration-noise blocks, and R is applied per scalar measurement. */
    constexpr bool kEstimatorUseMagnetometer = false; // Enable only after hard/soft-iron calibration.
    constexpr float kEstimatorAhrsStaticGain = 0.5f;
    constexpr float kEstimatorAhrsFlightGain = 0.0f; // 0 = gyro only propagation after launch
    constexpr float kEstimatorAhrsGyroRangeDps = 2000.0f;
    constexpr float kEstimatorAhrsAccelerationRejectionDeg = 10.0f;
    constexpr float kEstimatorAhrsMagneticRejectionDeg = 20.0f;
    constexpr uint32_t kEstimatorAhrsRecoveryPeriodMs = 5000U;
    constexpr uint32_t kEstimatorAhrsRecoveryPeriodSamples = msToSamples(kEstimatorAhrsRecoveryPeriodMs);

    constexpr double kEstimatorInitialHorizontalPositionSigmaM = 5.0;
    constexpr double kEstimatorInitialVerticalPositionSigmaM = 1.0;
    constexpr double kEstimatorInitialVelocitySigmaMps = 0.5;
    constexpr double kEstimatorIdleAccelSigmaMss = 0.35;
    constexpr double kEstimatorFlightHorizontalAccelSigmaMss = 10.0;
    constexpr double kEstimatorFlightVerticalAccelSigmaMss = 5.0;
    constexpr double kEstimatorDescentHorizontalAccelSigmaMss = 4.0;
    constexpr double kEstimatorDescentVerticalAccelSigmaMss = 4.0;
    constexpr double kEstimatorBaroSigmaM = 1.5;
    constexpr double kEstimatorMinGpsHorizontalSigmaM = 2.0;
    constexpr double kEstimatorMinGpsVerticalSigmaM = 3.0;
    constexpr double kEstimatorMinGpsVelocitySigmaMps = 0.5;
    constexpr double kEstimatorBaroGateSigma = 5.0;
    constexpr double kEstimatorGpsPositionGateSigma = 5.0;
    constexpr double kEstimatorGpsVelocityGateSigma = 5.0;
    constexpr uint32_t kEstimatorGpsMaxAgeMs = 1000U;
    constexpr uint32_t kEstimatorGpsOutageMs = 3000U;

    /* ====== 1D DEPLOYMENT VERTICAL FILTER TUNING ======
       Runs in parallel with the 3D estimator. Deployment-critical state transitions use
       this barometer-corrected altitude and vertical velocity estimate. */
    constexpr double kVerticalFilterInitialAltitudeSigmaM = 1.0;
    constexpr double kVerticalFilterInitialVelocitySigmaMps = 0.5;
    constexpr double kVerticalFilterInitialAccelBiasSigmaMss = 1.0;
    constexpr double kVerticalFilterIdleAccelSigmaMss = 0.35;
    constexpr double kVerticalFilterBoostAccelSigmaMss = 12.0;
    constexpr double kVerticalFilterCoastAccelSigmaMss = 4.0;
    constexpr double kVerticalFilterDescentAccelSigmaMss = 8.0;
    constexpr double kVerticalFilterAccelBiasRandomWalkMss = 0.08;
    constexpr double kVerticalFilterAccelBiasTauS = 2.0;
    constexpr double kVerticalFilterBaroSigmaM = 1.5;
    constexpr double kVerticalFilterBaroGateSigma = 5.0;
    // Gate escape. The innovation gate protects against a single bad barometer
    // reading, but after a large unmodelled velocity change (parachute opening,
    // resuming after a reset) every reading fails the gate and the filter can
    // never recover. After this many consecutive rejections the covariance is
    // inflated to the recovery sigmas and the measurement is accepted.
    constexpr uint32_t kVerticalFilterMaxConsecutiveBaroRejects = 30; // 0.3 s at 100 Hz
    constexpr double kVerticalFilterRecoveryAltitudeSigmaM = 20.0;
    constexpr double kVerticalFilterRecoveryVelocitySigmaMps = 50.0;
    // Initial sigmas when the filter starts mid-flight after a watchdog reset.
    constexpr double kVerticalFilterRestoreAltitudeSigmaM = 30.0;
    constexpr double kVerticalFilterRestoreVelocitySigmaMps = 80.0;

    /* ====== PIN CONFIGURATIONS ====== */
    constexpr uint8_t kLedPin = 2;
    constexpr uint8_t kBuzzerPin = 32;
    constexpr uint8_t kPyro1Pin = 6;
    constexpr uint8_t kPyro2Pin = 5;

    constexpr uint8_t kBaroCsPin = 10;
    constexpr uint8_t kAccelCsPin = 37;
    constexpr uint8_t kGyroCsPin = 36;
    constexpr uint8_t kMagCsPin = 38;

    constexpr uint8_t kRfm95MisoPin = 1;
    constexpr uint8_t kRfm95MosiPin = 26;
    constexpr uint8_t kRfm95SckPin = 27;
    constexpr uint8_t kRfm95CsPin = 0;
    constexpr uint8_t kRfm95RstPin = 9;
    // Note: DIO0 pin is not used for interrupt, poll status instead

    /* ====== SPI CONFIGURATION ====== */
    constexpr uint32_t kBaroSpiHz = 20000000UL;
    constexpr uint32_t kImuSpiHz = 10000000UL;
    constexpr uint32_t kMagSpiHz = 10000000UL;
    constexpr uint32_t kRadioSpiHz = 8000000UL;

    /* ====== TASK STACK SIZES (words) ====== */
    constexpr uint16_t kMainTaskStackWords = 1024;
    constexpr uint16_t kSupervisorTaskStackWords = 1024;
    constexpr uint16_t kSensorTaskStackWords = 2048;
    constexpr uint16_t kFlightTaskStackWords = 2048;
    constexpr uint16_t kLoggerTaskStackWords = 2048;
    constexpr uint16_t kRadioTaskStackWords = 2048;
    constexpr uint16_t kGpsTaskStackWords = 3072;
    constexpr uint16_t kDebugTaskStackWords = 2048;

    /* ====== TASK PRIORITIES (higher number = higher priority) ======
       supervisor > sensor > flight > {radio, logger, gps} > debug.
       The radio was previously equal to flight; telemetry must never delay deployment. */
    constexpr uint8_t kSupervisorTaskPriority = 5;
    constexpr uint8_t kSensorTaskPriority = 4;
    constexpr uint8_t kFlightTaskPriority = 3;
    constexpr uint8_t kRadioTaskPriority = 2;
    constexpr uint8_t kLoggerTaskPriority = 2;
    constexpr uint8_t kGpsTaskPriority = 2;
    constexpr uint8_t kDebugTaskPriority = 1;

} // namespace app_config
