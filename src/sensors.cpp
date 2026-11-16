#include "sensors.h"

#include <BMI088.h>
#include <SPI.h>
#include <math.h>

#include "app_config.h"

void serialPrintfSafe(const char *fmt, ...);
void serialPrintlnSafe(const char *s);

namespace
{

  constexpr uint8_t MAG_WHO_AM_I = 0x4F;
  constexpr uint8_t MAG_CFG_A = 0x60;
  constexpr uint8_t MAG_CFG_B = 0x61;
  constexpr uint8_t MAG_CFG_C = 0x62;
  constexpr uint8_t MAG_STATUS = 0x67;
  constexpr uint8_t MAG_OUTX_L = 0x68;

  constexpr uint8_t BARO_RESET_CMD = 0x1E;
  constexpr uint8_t BARO_ADC_READ_CMD = 0x00;
  constexpr uint8_t BARO_PROM_READ_BASE = 0xA0;

  constexpr double LAPSE_RATE = 0.0065;
  constexpr double GRAVITY = 9.80665;
  constexpr double MOLAR_MASS = 0.0289644;
  constexpr double UNIVERSAL_GAS_CONST = 8.3144598;

  SPISettings g_baroSpi(app_config::kBaroSpiHz, arduino::MSBFIRST, SPI_MODE0);
  SPISettings g_imuSpi(app_config::kImuSpiHz, arduino::MSBFIRST, SPI_MODE0);
  SPISettings g_magSpi(app_config::kMagSpiHz, arduino::MSBFIRST, SPI_MODE0);

  uint16_t g_cal[8] = {0};
  float g_baseTempC = 0.0f;
  float g_basePressPa = 0.0f;
  SemaphoreHandle_t g_spi0Mutex = nullptr;
  bool g_magAvailable = false;

  Bmi088Accel g_accel(SPI, app_config::kAccelCsPin);
  Bmi088Gyro g_gyro(SPI, app_config::kGyroCsPin);

  // ---- Non-blocking barometer state ----
  enum class baro_pending_t : uint8_t
  {
    NONE = 0,
    PRESSURE = 1,
    TEMPERATURE = 2,
  };
  baro_pending_t g_baroPending = baro_pending_t::NONE;
  uint32_t g_baroStartUs = 0;
  uint32_t g_baroCycle = 0;
  uint32_t g_baroRawD1 = 0; // pressure ADC
  uint32_t g_baroRawD2 = 0; // temperature ADC
  bool g_baroHaveD1 = false;
  bool g_baroHaveD2 = false;
  float g_lastTempC = 0.0f;
  float g_lastPressPa = 0.0f;
  float g_lastMag[3] = {0.0f, 0.0f, 0.0f};

  void lockSpi0()
  {
    if (g_spi0Mutex != nullptr)
    {
      xSemaphoreTake(g_spi0Mutex, portMAX_DELAY);
    }
  }

  void unlockSpi0()
  {
    if (g_spi0Mutex != nullptr)
    {
      xSemaphoreGive(g_spi0Mutex);
    }
  }

  // ---- Barometer primitives (caller holds SPI0) ----

  void baroCommand(uint8_t cmd)
  {
    SPI.beginTransaction(g_baroSpi);
    digitalWrite(app_config::kBaroCsPin, arduino::LOW);
    SPI.transfer(cmd);
    digitalWrite(app_config::kBaroCsPin, arduino::HIGH);
    SPI.endTransaction();
  }

  uint32_t baroReadAdc()
  {
    SPI.beginTransaction(g_baroSpi);
    digitalWrite(app_config::kBaroCsPin, arduino::LOW);
    SPI.transfer(BARO_ADC_READ_CMD);
    uint32_t value = (static_cast<uint32_t>(SPI.transfer(0x00)) << 16U);
    value |= (static_cast<uint32_t>(SPI.transfer(0x00)) << 8U);
    value |= static_cast<uint32_t>(SPI.transfer(0x00));
    digitalWrite(app_config::kBaroCsPin, arduino::HIGH);
    SPI.endTransaction();
    return value;
  }

  void readBaroCalibration()
  {
    for (uint8_t i = 0; i < 8; ++i)
    {
      SPI.beginTransaction(g_baroSpi);
      digitalWrite(app_config::kBaroCsPin, arduino::LOW);
      SPI.transfer(BARO_PROM_READ_BASE | (i << 1));
      uint8_t hi = SPI.transfer(0x00);
      uint8_t lo = SPI.transfer(0x00);
      digitalWrite(app_config::kBaroCsPin, arduino::HIGH);
      SPI.endTransaction();
      g_cal[i] = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8U) | lo);
    }
  }

  bool baroCalibrationLooksValid()
  {
    for (uint8_t i = 1; i <= 6; ++i)
    {
      if ((g_cal[i] == 0U) || (g_cal[i] == 0xFFFFU))
      {
        return false;
      }
    }
    return true;
  }

  // Second-order-free MS56xx compensation from the raw ADC values.
  void baroCompensate(uint32_t d1, uint32_t d2, float &temperature_c, float &pressure_pa)
  {
    const double d_t = static_cast<double>(d2) - (static_cast<double>(g_cal[5]) * 256.0);
    const double temp = 2000.0 + d_t * (static_cast<double>(g_cal[6]) / 8388608.0);
    const double off = static_cast<double>(g_cal[2]) * 131072.0 + (static_cast<double>(g_cal[4]) * d_t) / 64.0;
    const double sens = static_cast<double>(g_cal[1]) * 65536.0 + (static_cast<double>(g_cal[3]) * d_t) / 128.0;
    const double press = (((static_cast<double>(d1) * sens) / 2097152.0) - off) / 32768.0;
    temperature_c = static_cast<float>(temp / 100.0);
    pressure_pa = static_cast<float>(press);
  }

  // Blocking read used only during init (scheduler not running yet).
  void readBarometerBlocking(float &temperature_c, float &pressure_pa)
  {
    lockSpi0();
    baroCommand(app_config::kBaroConvertD2Cmd);
    unlockSpi0();
    delay(app_config::kBaroConversionMs + 1U);
    lockSpi0();
    const uint32_t d2 = baroReadAdc();
    baroCommand(app_config::kBaroConvertD1Cmd);
    unlockSpi0();
    delay(app_config::kBaroConversionMs + 1U);
    lockSpi0();
    const uint32_t d1 = baroReadAdc();
    unlockSpi0();
    baroCompensate(d1, d2, temperature_c, pressure_pa);
    g_baroRawD1 = d1;
    g_baroRawD2 = d2;
    g_baroHaveD1 = g_baroHaveD2 = true;
  }

  // One step of the non-blocking state machine. Returns true when a fresh
  // pressure value was produced this call.
  bool baroStep()
  {
    bool fresh_pressure = false;
    lockSpi0();

    if (g_baroPending != baro_pending_t::NONE)
    {
      const uint32_t elapsed_us = micros() - g_baroStartUs;
      if (elapsed_us >= (app_config::kBaroConversionMs * 1000UL))
      {
        const uint32_t raw = baroReadAdc();
        if (raw != 0U) // 0 means the ADC was read too early; keep the old value
        {
          if (g_baroPending == baro_pending_t::PRESSURE)
          {
            g_baroRawD1 = raw;
            g_baroHaveD1 = true;
            fresh_pressure = true;
          }
          else
          {
            g_baroRawD2 = raw;
            g_baroHaveD2 = true;
          }
        }
        g_baroPending = baro_pending_t::NONE;
      }
    }

    if (g_baroPending == baro_pending_t::NONE)
    {
      const bool want_temperature =
          !g_baroHaveD2 || ((g_baroCycle % app_config::kBaroTemperatureEveryN) == 0U);
      baroCommand(want_temperature ? app_config::kBaroConvertD2Cmd : app_config::kBaroConvertD1Cmd);
      g_baroPending = want_temperature ? baro_pending_t::TEMPERATURE : baro_pending_t::PRESSURE;
      g_baroStartUs = micros();
      ++g_baroCycle;
    }

    unlockSpi0();

    if (fresh_pressure && g_baroHaveD2)
    {
      baroCompensate(g_baroRawD1, g_baroRawD2, g_lastTempC, g_lastPressPa);
      return true;
    }
    return false;
  }

  // ---- Magnetometer ----

  void magWriteReg(uint8_t reg, uint8_t value)
  {
    SPI.beginTransaction(g_magSpi);
    digitalWrite(app_config::kMagCsPin, arduino::LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(value);
    digitalWrite(app_config::kMagCsPin, arduino::HIGH);
    SPI.endTransaction();
  }

  uint8_t magReadReg(uint8_t reg)
  {
    SPI.beginTransaction(g_magSpi);
    digitalWrite(app_config::kMagCsPin, arduino::LOW);
    SPI.transfer(reg | 0x80);
    const uint8_t value = SPI.transfer(0x00);
    digitalWrite(app_config::kMagCsPin, arduino::HIGH);
    SPI.endTransaction();
    return value;
  }

  // Single status check, no waiting. If the magnetometer has nothing new we
  // keep the previous reading; the magnetometer runs at 100 Hz so this is rare.
  bool readMagnetometer(float out[3])
  {
    if (!app_config::kEnableMagnetometer || !g_magAvailable)
    {
      out[0] = out[1] = out[2] = 0.0f;
      return false;
    }

    lockSpi0();
    const bool ready = (magReadReg(MAG_STATUS) & 0x08U) != 0U;
    if (!ready)
    {
      unlockSpi0();
      out[0] = g_lastMag[0];
      out[1] = g_lastMag[1];
      out[2] = g_lastMag[2];
      return false;
    }

    // Register-by-register read (address auto-increment is not enabled in CFG_C).
    uint8_t raw[6];
    for (uint8_t i = 0; i < 6; ++i)
    {
      raw[i] = magReadReg(MAG_OUTX_L + i);
    }
    unlockSpi0();

    const int16_t mx = static_cast<int16_t>((static_cast<uint16_t>(raw[1]) << 8U) | raw[0]);
    const int16_t my = static_cast<int16_t>((static_cast<uint16_t>(raw[3]) << 8U) | raw[2]);
    const int16_t mz = static_cast<int16_t>((static_cast<uint16_t>(raw[5]) << 8U) | raw[4]);

    out[0] = g_lastMag[0] = static_cast<float>(mx);
    out[1] = g_lastMag[1] = static_cast<float>(my);
    out[2] = g_lastMag[2] = static_cast<float>(mz);
    return true;
  }

  double calculateAltitudeMeters(double current_pressure_pa,
                                 double reference_pressure_pa,
                                 double reference_temperature_k)
  {
    const double exponent = (UNIVERSAL_GAS_CONST * LAPSE_RATE) / (GRAVITY * MOLAR_MASS);
    double ratio = current_pressure_pa / reference_pressure_pa;
    if (ratio <= 0.0)
    {
      ratio = 1e-6;
    }
    return (reference_temperature_k / LAPSE_RATE) * (1.0 - pow(ratio, exponent));
  }

} // namespace

namespace sensors
{

  bool init(SemaphoreHandle_t spi0_mutex, const baro_baseline_t *restore_baseline)
  {
    g_spi0Mutex = spi0_mutex;
    bool init_ok = true;
    g_magAvailable = false;

    serialPrintlnSafe("Sensor init starting...");

    pinMode(app_config::kBaroCsPin, arduino::OUTPUT);
    pinMode(app_config::kAccelCsPin, arduino::OUTPUT);
    pinMode(app_config::kGyroCsPin, arduino::OUTPUT);
    pinMode(app_config::kMagCsPin, arduino::OUTPUT);
    digitalWrite(app_config::kBaroCsPin, arduino::HIGH);
    digitalWrite(app_config::kAccelCsPin, arduino::HIGH);
    digitalWrite(app_config::kGyroCsPin, arduino::HIGH);
    digitalWrite(app_config::kMagCsPin, arduino::HIGH);

    SPI.begin();
    delay(50);

    lockSpi0();
    baroCommand(BARO_RESET_CMD);
    unlockSpi0();
    delay(10);

    lockSpi0();
    readBaroCalibration();
    unlockSpi0();
    delay(10);

    if (baroCalibrationLooksValid())
    {
      serialPrintlnSafe("Sensor init: barometer OK");
    }
    else
    {
      serialPrintlnSafe("Sensor init: barometer FAILED");
      init_ok = false;
    }

    lockSpi0();
    const int accel_status = g_accel.begin();
    unlockSpi0();
    if (accel_status < 0)
    {
      serialPrintfSafe("Sensor init: BMI088 accelerometer FAILED (%d)\n", accel_status);
      init_ok = false;
    }
    else
    {
      serialPrintlnSafe("Sensor init: BMI088 accelerometer OK");
    }

    lockSpi0();
    const int gyro_status = g_gyro.begin();
    unlockSpi0();
    if (gyro_status < 0)
    {
      serialPrintfSafe("Sensor init: BMI088 gyro FAILED (%d)\n", gyro_status);
      init_ok = false;
    }
    else
    {
      serialPrintlnSafe("Sensor init: BMI088 gyro OK");
    }

    if (init_ok && app_config::kImuConfigureRates)
    {
      // 400 Hz ODR with ~40-47 Hz anti-alias bandwidth suits a 100 Hz sample rate
      // (Nyquist 50 Hz). Full range so boost acceleration never clips.
      lockSpi0();
      const bool a_range = g_accel.setRange(Bmi088Accel::RANGE_24G);
      const bool a_odr = g_accel.setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);
      const bool g_range = g_gyro.setRange(Bmi088Gyro::RANGE_2000DPS);
      const bool g_odr = g_gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
      unlockSpi0();
      if (!(a_range && a_odr && g_range && g_odr))
      {
        serialPrintlnSafe("Sensor init: WARNING BMI088 rate/range configuration failed; using library defaults.");
      }
    }

    if (app_config::kEnableMagnetometer)
    {
      lockSpi0();
      magWriteReg(MAG_CFG_C, 0x34);
      magWriteReg(MAG_CFG_B, 0x0C);
      magWriteReg(MAG_CFG_A, 0x8C);
      const uint8_t whoami = magReadReg(MAG_WHO_AM_I);
      unlockSpi0();
      if (whoami != 0x40U)
      {
        serialPrintfSafe("Sensor init: magnetometer FAILED (WHOAMI=0x%02X)\n", whoami);
        if (app_config::kEstimatorUseMagnetometer)
        {
          init_ok = false;
        }
        else
        {
          serialPrintlnSafe("Sensor init: magnetometer unavailable; continuing without it.");
        }
      }
      else
      {
        serialPrintlnSafe("Sensor init: magnetometer OK");
        g_magAvailable = true;
      }
    }
    else
    {
      serialPrintlnSafe("Sensor init: magnetometer DISABLED");
    }

    if (!init_ok)
    {
      serialPrintlnSafe("Sensor init: one or more required sensors failed.");
      return false;
    }

    if (restore_baseline != nullptr && restore_baseline->pressure_pa > 10000.0f)
    {
      // Resuming a flight: altitude must stay relative to the launch pad, not to
      // wherever the rocket happens to be right now.
      g_baseTempC = restore_baseline->temperature_c;
      g_basePressPa = restore_baseline->pressure_pa;
      readBarometerBlocking(g_lastTempC, g_lastPressPa);
      serialPrintfSafe("Sensor init: barometer baseline RESTORED (%.1f Pa, %.1f C)\n",
                       static_cast<double>(g_basePressPa), static_cast<double>(g_baseTempC));
    }
    else
    {
      float t_sum = 0.0f;
      float p_sum = 0.0f;
      for (uint32_t i = 0; i < app_config::kBaselineSamples; ++i)
      {
        float t = 0.0f;
        float p = 0.0f;
        readBarometerBlocking(t, p);
        t_sum += t;
        p_sum += p;
      }
      g_baseTempC = t_sum / static_cast<float>(app_config::kBaselineSamples);
      g_basePressPa = p_sum / static_cast<float>(app_config::kBaselineSamples);
      g_lastTempC = g_baseTempC;
      g_lastPressPa = g_basePressPa;
    }

    g_baroPending = baro_pending_t::NONE;
    g_baroCycle = 0;
    return true;
  }

  bool readSample(flight_sample_t &sample)
  {
    const bool baro_fresh = baroStep();

    lockSpi0();
    g_accel.readSensor();
    const float ax = g_accel.getAccelX_mss();
    const float ay = g_accel.getAccelY_mss();
    const float az = g_accel.getAccelZ_mss();
    g_gyro.readSensor();
    const float gx = g_gyro.getGyroX_rads();
    const float gy = g_gyro.getGyroY_rads();
    const float gz = g_gyro.getGyroZ_rads();
    unlockSpi0();

    float mag_raw[3] = {0.0f, 0.0f, 0.0f};
    const bool mag_fresh = readMagnetometer(mag_raw);

    sample.timestamp_ms = millis();
    sample.baro_fresh = baro_fresh;
    sample.mag_fresh = mag_fresh;
    sample.temperature_c = g_lastTempC;
    sample.pressure_pa = g_lastPressPa;
    sample.altitude_m = static_cast<float>(
        calculateAltitudeMeters(g_lastPressPa, g_basePressPa, static_cast<double>(g_baseTempC) + 273.15));
    sample.accel_mss[0] = ax;
    sample.accel_mss[1] = ay;
    sample.accel_mss[2] = az;
    sample.gyro_rads[0] = gx;
    sample.gyro_rads[1] = gy;
    sample.gyro_rads[2] = gz;
    sample.mag_raw[0] = mag_raw[0];
    sample.mag_raw[1] = mag_raw[1];
    sample.mag_raw[2] = mag_raw[2];
    return true;
  }

  baro_baseline_t baseline()
  {
    baro_baseline_t b;
    b.pressure_pa = g_basePressPa;
    b.temperature_c = g_baseTempC;
    return b;
  }

  float referenceTemperatureC() { return g_baseTempC; }
  float referencePressurePa() { return g_basePressPa; }

} // namespace sensors
