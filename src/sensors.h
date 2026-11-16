#pragma once

#include <Arduino.h>
#include <arduino_freertos.h>
#include <semphr.h>

#include "app_types.h"

namespace sensors {

// Initialise all sensors. If restore_baseline is non-null the barometer
// reference is taken from it (resuming a flight after a watchdog reset)
// instead of being measured on the pad.
bool init(SemaphoreHandle_t spi0_mutex, const baro_baseline_t* restore_baseline = nullptr);

// Non-blocking: reads whatever is ready and starts the next barometer
// conversion. Never sleeps. Intended to be called on a fixed period that is
// longer than app_config::kBaroConversionMs.
bool readSample(flight_sample_t& sample);

baro_baseline_t baseline();
float referenceTemperatureC();
float referencePressurePa();

}  // namespace sensors
