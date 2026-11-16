#pragma once

#include "app_types.h"

namespace vertical_filter
{

void reset();

// After a watchdog reset in flight the filter cannot learn its accelerometer
// bias on the pad. Supply the bias saved before the reset; it is used instead
// of the "measured vertical accel == bias" assumption at the next initialise.
void restoreBias(double accel_bias_mss);

void update(const flight_sample_t &sample,
            flight_state_t &state,
            double vertical_accel_mss,
            double dt_s,
            bool hold_on_pad);

} // namespace vertical_filter
