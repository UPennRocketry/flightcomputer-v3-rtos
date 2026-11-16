#pragma once

#include <Arduino.h>
#include <arduino_freertos.h>
#include <semphr.h>

namespace rfm95 {

struct metrics_t {
  bool initialized = false;
  bool busy = false;
  uint32_t tx_start_count = 0;
  uint32_t tx_done_count = 0;
  uint32_t tx_fail_count = 0;
  uint32_t last_tx_start_ms = 0;
  uint32_t last_tx_done_ms = 0;
  uint32_t last_tx_duration_ms = 0;
  size_t last_payload_len = 0;
};

bool init(SemaphoreHandle_t spi1_mutex);
bool startTx(const uint8_t* data, size_t len);
bool isBusy();
void service();
metrics_t metrics();

}  // namespace rfm95
