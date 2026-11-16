#pragma once

#include "app_types.h"

namespace logger {

bool init();
bool isEnabled();
void writeSample(const flight_sample_t& sample, const flight_state_t& state);

}  // namespace logger
