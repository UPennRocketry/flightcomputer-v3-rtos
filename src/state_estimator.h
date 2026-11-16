#pragma once

#include "app_types.h"

namespace state_estimator
{

void reset();
void update(const flight_sample_t &sample, flight_state_t &state);

// After a watchdog reset mid-flight there is no stationary period to calibrate
// in. Install the startup gravity axis and IMU biases saved before the reset and
// mark calibration complete/frozen so flight-mode processing starts immediately.
void restoreCalibration(const float body_up[3],
                        const float accel_bias_mss[3],
                        const float gyro_bias_rads[3]);

} // namespace state_estimator
