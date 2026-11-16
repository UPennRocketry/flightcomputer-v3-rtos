#include "rfm95.h"

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#include "app_config.h"

namespace {

constexpr uint8_t REG_FIFO = 0x00;
constexpr uint8_t REG_OP_MODE = 0x01;
constexpr uint8_t REG_FRF_MSB = 0x06;
constexpr uint8_t REG_FRF_MID = 0x07;
constexpr uint8_t REG_FRF_LSB = 0x08;
constexpr uint8_t REG_PA_CONFIG = 0x09;
constexpr uint8_t REG_OCP = 0x0B;
constexpr uint8_t REG_LNA = 0x0C;
constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
constexpr uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
constexpr uint8_t REG_IRQ_FLAGS_MASK = 0x11;
constexpr uint8_t REG_IRQ_FLAGS = 0x12;
constexpr uint8_t REG_PAYLOAD_LENGTH = 0x22;
constexpr uint8_t REG_MODEM_CONFIG1 = 0x1D;
constexpr uint8_t REG_MODEM_CONFIG2 = 0x1E;
constexpr uint8_t REG_SYMB_TIMEOUT_LSB = 0x1F;
constexpr uint8_t REG_PREAMBLE_MSB = 0x20;
constexpr uint8_t REG_PREAMBLE_LSB = 0x21;
constexpr uint8_t REG_MODEM_CONFIG3 = 0x26;
constexpr uint8_t REG_SYNC_WORD = 0x39;
constexpr uint8_t REG_VERSION = 0x42;
constexpr uint8_t REG_PA_DAC = 0x4D;

constexpr uint8_t MODE_LONG_RANGE = 0x80;
constexpr uint8_t MODE_SLEEP = 0x00;
constexpr uint8_t MODE_STDBY = 0x01;
constexpr uint8_t MODE_TX = 0x03;

constexpr uint8_t IRQ_TX_DONE = 0x08;
constexpr uint8_t SX1276_VERSION = 0x12;
constexpr uint8_t RADIOHEAD_SYNC_WORD = 0x12;
constexpr uint8_t RADIOHEAD_BROADCAST_ADDRESS = 0xFF;
constexpr uint8_t RADIOHEAD_HEADER_LEN = 4;
constexpr uint32_t TX_DONE_TIMEOUT_MS = 1000;

SPISettings g_radioSpi(app_config::kRadioSpiHz, arduino::MSBFIRST, SPI_MODE0);
SemaphoreHandle_t g_spi1Mutex = nullptr;
volatile bool g_busy = false;
rfm95::metrics_t g_metrics = {};

void lockSpi1() {
  if (g_spi1Mutex != nullptr) {
    xSemaphoreTake(g_spi1Mutex, portMAX_DELAY);
  }
}

void unlockSpi1() {
  if (g_spi1Mutex != nullptr) {
    xSemaphoreGive(g_spi1Mutex);
  }
}

void csSelect() { digitalWrite(app_config::kRfm95CsPin, arduino::LOW); }
void csDeselect() { digitalWrite(app_config::kRfm95CsPin, arduino::HIGH); }

uint8_t readReg(uint8_t reg) {
  SPI1.beginTransaction(g_radioSpi);
  csSelect();
  SPI1.transfer(reg & 0x7F);
  const uint8_t val = SPI1.transfer(0x00);
  csDeselect();
  SPI1.endTransaction();
  return val;
}

void writeReg(uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(g_radioSpi);
  csSelect();
  SPI1.transfer(reg | 0x80);
  SPI1.transfer(val);
  csDeselect();
  SPI1.endTransaction();
}

void writeBurst(uint8_t reg, const uint8_t* data, size_t len) {
  SPI1.beginTransaction(g_radioSpi);
  csSelect();
  SPI1.transfer(reg | 0x80);
  for (size_t i = 0; i < len; ++i) {
    SPI1.transfer(data[i]);
  }
  csDeselect();
  SPI1.endTransaction();
}

void setMode(uint8_t mode) {
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | mode);
  delayMicroseconds(100);
}

void setFrequencyHz(uint32_t freq_hz) {
  const uint64_t frf = (static_cast<uint64_t>(freq_hz) << 19U) / 32000000ULL;
  writeReg(REG_FRF_MSB, static_cast<uint8_t>(frf >> 16U));
  writeReg(REG_FRF_MID, static_cast<uint8_t>(frf >> 8U));
  writeReg(REG_FRF_LSB, static_cast<uint8_t>(frf));
}

void setTxPowerDbm(int8_t dbm) {
  if (dbm < 2) {
    dbm = 2;
  }
  if (dbm > 20) {
    dbm = 20;
  }

  if (dbm > 17) {
    writeReg(REG_PA_DAC, 0x87);
    writeReg(REG_OCP, 0x3B);
    dbm -= 3;
  } else {
    writeReg(REG_PA_DAC, 0x84);
    writeReg(REG_OCP, 0x2B);
  }

  writeReg(REG_PA_CONFIG, 0x80 | static_cast<uint8_t>(dbm - 2));
}

void resetRadio() {
  digitalWrite(app_config::kRfm95RstPin, arduino::LOW);
  delay(10);
  digitalWrite(app_config::kRfm95RstPin, arduino::HIGH);
  delay(10);
}

}  // namespace

namespace rfm95 {

bool init(SemaphoreHandle_t spi1_mutex) {
  g_spi1Mutex = spi1_mutex;

  pinMode(app_config::kRfm95CsPin, arduino::OUTPUT);
  pinMode(app_config::kRfm95RstPin, arduino::OUTPUT);
  digitalWrite(app_config::kRfm95CsPin, arduino::HIGH);
  digitalWrite(app_config::kRfm95RstPin, arduino::HIGH);

  SPI1.setMISO(app_config::kRfm95MisoPin);
  SPI1.setMOSI(app_config::kRfm95MosiPin);
  SPI1.setSCK(app_config::kRfm95SckPin);
  SPI1.begin();

  lockSpi1();
  resetRadio();

  const uint8_t version = readReg(REG_VERSION);
  if (version != SX1276_VERSION) {
    unlockSpi1();
    g_metrics.initialized = false;
    return false;
  }

  setMode(MODE_SLEEP);
  setMode(MODE_STDBY);
  writeReg(REG_FIFO_TX_BASE_ADDR, 0x00);
  writeReg(REG_FIFO_RX_BASE_ADDR, 0x00);
  writeReg(REG_LNA, 0x23);
  writeReg(REG_MODEM_CONFIG1, app_config::kRadioModemConfig1); // bandwidth / coding rate (see app_config)
  writeReg(REG_MODEM_CONFIG2, app_config::kRadioModemConfig2); // spreading factor, CRC
  writeReg(REG_SYMB_TIMEOUT_LSB, 0x08);
  writeReg(REG_MODEM_CONFIG3, app_config::kRadioModemConfig3); // AGC auto
  writeReg(REG_PREAMBLE_MSB, 0x00);
  writeReg(REG_PREAMBLE_LSB, 0x08);
  writeReg(REG_SYNC_WORD, RADIOHEAD_SYNC_WORD);
  writeReg(REG_IRQ_FLAGS_MASK, 0x00);
  writeReg(REG_IRQ_FLAGS, 0xFF);
  setFrequencyHz(static_cast<uint32_t>(app_config::kRadioFreqMHz * 1000000.0f));
  setTxPowerDbm(app_config::kRadioTxPowerDbm);
  unlockSpi1();

  g_busy = false;
  g_metrics = {};
  g_metrics.initialized = true;
  return true;
}

bool startTx(const uint8_t* data, size_t len) {
  if ((data == nullptr) || (len == 0U) || ((len + RADIOHEAD_HEADER_LEN) > 255U) || g_busy) {
    ++g_metrics.tx_fail_count;
    return false;
  }

  uint8_t frame[255] = {
      RADIOHEAD_BROADCAST_ADDRESS,
      RADIOHEAD_BROADCAST_ADDRESS,
      0x00,
      0x00,
  };
  memcpy(frame + RADIOHEAD_HEADER_LEN, data, len);
  const size_t frame_len = len + RADIOHEAD_HEADER_LEN;

  lockSpi1();
  writeReg(REG_IRQ_FLAGS, 0xFF);
  setMode(MODE_STDBY);
  writeReg(REG_FIFO_ADDR_PTR, 0x00);
  writeBurst(REG_FIFO, frame, frame_len);
  writeReg(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(frame_len));
  setMode(MODE_TX);
  unlockSpi1();

  g_busy = true;
  g_metrics.busy = true;
  ++g_metrics.tx_start_count;
  g_metrics.last_tx_start_ms = millis();
  g_metrics.last_payload_len = len;
  return true;
}

bool isBusy() { return g_busy; }

void service() {
  if (!g_busy) {
    return;
  }

  lockSpi1();
  const uint8_t irq_flags = readReg(REG_IRQ_FLAGS);
  if ((irq_flags & IRQ_TX_DONE) == 0U) {
    const uint32_t now_ms = millis();
    if ((now_ms - g_metrics.last_tx_start_ms) >= TX_DONE_TIMEOUT_MS) {
      writeReg(REG_IRQ_FLAGS, 0xFF);
      setMode(MODE_STDBY);
      unlockSpi1();

      g_busy = false;
      g_metrics.busy = false;
      ++g_metrics.tx_fail_count;
      return;
    }

    unlockSpi1();
    return;
  }

  writeReg(REG_IRQ_FLAGS, IRQ_TX_DONE);
  setMode(MODE_STDBY);
  unlockSpi1();

  g_busy = false;
  g_metrics.busy = false;
  ++g_metrics.tx_done_count;
  g_metrics.last_tx_done_ms = millis();
  g_metrics.last_tx_duration_ms = g_metrics.last_tx_done_ms - g_metrics.last_tx_start_ms;
}

metrics_t metrics() {
  g_metrics.busy = g_busy;
  return g_metrics;
}

}  // namespace rfm95
