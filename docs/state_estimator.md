# State Estimator

This document describes the flight computer state estimator implemented in
`src/state_estimator.cpp`. The estimator is split into three layers:

1. Fusion AHRS estimates vehicle orientation from gyro, accelerometer, and
   optionally magnetometer data.
2. A linear Kalman filter estimates 3D position and velocity from earth-frame
   acceleration, barometer altitude, and GPS position/velocity.
3. A separate 1D vertical filter estimates deployment altitude and vertical
   velocity from the startup vertical axis, accelerometer, and barometer.

The Kalman filter does not estimate attitude. It treats the AHRS quaternion and
the earth-frame acceleration from Fusion as known inputs. That keeps the
navigation filter linear and small, but it also means attitude uncertainty,
accelerometer bias, and gyro drift are represented only indirectly through the
process-noise tuning.

## Coordinate Frame

The world frame is ENU:

- `x`: east
- `y`: north
- `z`: up

The navigation state is:

```text
x = [p_e, p_n, p_u, v_e, v_n, v_u]^T
```

where:

- `p_e`, `p_n`, `p_u` are east, north, and up position in meters
- `v_e`, `v_n`, `v_u` are east, north, and up velocity in meters per second

The local GPS position is generated from the first valid GPS fix and then
reported in this same ENU frame. `flight_state_t::position_m.z` and
`flight_state_t::velocity_mps_3d.z` remain the 3D estimator's ENU up states.
The public `flight_state_t::altitude_m`, `flight_state_t::velocity_mps`, and
deployment fields are published from the 1D vertical filter.

## Overall Control Flow

Each sensor sample calls `state_estimator::update(sample, state)`.

1. Compute `dt` from sample timestamps, bounded by `kEstimatorMaxDtS`.
2. Decide whether the vehicle is idle on the pad:
   - flight phase is `STARTUP`
   - accelerometer magnitude is close to gravity
   - gyro magnitude is below the stationary threshold
3. Update the barometer reference filter.
4. Update Fusion AHRS:
   - startup/idle uses accelerometer correction
   - startup initializes attitude from the gravity vector
   - magnetometer correction is disabled unless `kEstimatorUseMagnetometer` is true
   - flight uses `kEstimatorAhrsFlightGain`, currently `0`, so it is gyro-only
5. Read earth-frame linear acceleration from Fusion. Fusion has already removed
   gravity from this value.
6. If the estimator is not initialized, initialize the navigation state and
   covariance.
7. If idle, hold navigation state at:

   ```text
   [0, 0, filtered_baro_altitude, 0, 0, 0]
   ```

   and reset covariance to the startup values.
8. If not idle, run the Kalman filter:
   - predict position and velocity using earth-frame acceleration
   - apply barometer altitude measurement
   - apply GPS ENU position measurements when fresh and valid
   - apply GPS ENU velocity measurements when fresh and valid
9. Run the 1D vertical filter in parallel:
   - hold altitude and velocity on the pad while the startup sample is idle
   - project raw acceleration onto the startup gravity-derived vertical axis
   - subtract gravity and the learned 1D acceleration bias
   - apply barometer altitude correction
10. Publish:
   - `state.position_m`
   - `state.velocity_mps_3d`
   - `state.orientation_q`
   - `state.estimator`
   - `state.vertical_filter`
   - `state.altitude_m` and `state.velocity_mps` from the 1D filter

## Prediction Model

The filter assumes constant acceleration over one time step. For each axis:

```text
p_k+1 = p_k + v_k * dt + 0.5 * a_k * dt^2
v_k+1 = v_k + a_k * dt
```

In matrix form:

```text
x_k+1 = F * x_k + B * a_k
```

where:

```text
F =
[ 1 0 0 dt 0  0  ]
[ 0 1 0 0  dt 0  ]
[ 0 0 1 0  0  dt ]
[ 0 0 0 1  0  0  ]
[ 0 0 0 0  1  0  ]
[ 0 0 0 0  0  1  ]
```

and:

```text
B =
[ 0.5 * dt^2  0            0           ]
[ 0            0.5 * dt^2  0           ]
[ 0            0            0.5 * dt^2 ]
[ dt           0            0           ]
[ 0            dt           0           ]
[ 0            0            dt          ]
```

The input vector is:

```text
a = [a_e, a_n, a_u]^T
```

`a` comes from `FusionAhrsGetEarthAcceleration()` and is converted from `g` to
`m/s^2`. Because Fusion returns linear earth acceleration, gravity is already
removed. The estimator does not subtract `9.80665` again in the Kalman filter.

## Covariance Prediction

The covariance prediction is:

```text
P_k+1 = F * P_k * F^T + Q
```

`P` is stored as a persistent 6x6 matrix:

```text
P[i][j] = covariance between x[i] and x[j]
```

The diagonal entries are variances:

```text
P[0][0] = variance of east position, units m^2
P[1][1] = variance of north position, units m^2
P[2][2] = variance of up position, units m^2
P[3][3] = variance of east velocity, units (m/s)^2
P[4][4] = variance of north velocity, units (m/s)^2
P[5][5] = variance of up velocity, units (m/s)^2
```

The off-diagonal terms represent coupling between states. For example,
`P[0][3]` is east-position/east-velocity covariance. These off-diagonal terms
matter because a position measurement can also reduce velocity uncertainty if
position and velocity have become correlated through the prediction step.

## Initial P Matrix

On first initialization, `P` is diagonal:

```text
P0 =
[ sigma_ph^2 0          0          0          0          0          ]
[ 0          sigma_ph^2 0          0          0          0          ]
[ 0          0          sigma_pu^2 0          0          0          ]
[ 0          0          0          sigma_v^2  0          0          ]
[ 0          0          0          0          sigma_v^2  0          ]
[ 0          0          0          0          0          sigma_v^2  ]
```

with:

```text
sigma_ph = kEstimatorInitialHorizontalPositionSigmaM
sigma_pu = kEstimatorInitialVerticalPositionSigmaM
sigma_v  = kEstimatorInitialVelocitySigmaMps
```

During idle/startup, navigation is held fixed and `P` is reset each update to:

```text
P_idle =
[ sigma_ph^2 0              0          0          0          0         ]
[ 0          sigma_ph^2     0          0          0          0         ]
[ 0          0              sigma_b^2  0          0          0         ]
[ 0          0              0          sigma_v^2  0          0         ]
[ 0          0              0          0          sigma_v^2  0         ]
[ 0          0              0          0          0          sigma_v^2 ]
```

with:

```text
sigma_b = kEstimatorBaroSigmaM
```

This prevents prelaunch IMU noise from integrating into fake pad position or
velocity.

## Q Matrix

`Q` is the process covariance. It represents uncertainty added during the
prediction step. In this estimator, `Q` mostly means:

"How much do we distrust acceleration integration over this time step?"

The implementation does not store a dense 6x6 `Q` matrix. It applies the exact
nonzero entries directly to the predicted covariance. This is equivalent to
using a block-diagonal `Q` generated from acceleration noise.

For one axis, if acceleration uncertainty is `sigma_a`, then:

```text
q_a = sigma_a^2

Q_axis =
[ 0.25 * dt^4 * q_a   0.5 * dt^3 * q_a ]
[ 0.5  * dt^3 * q_a   dt^2 * q_a       ]
```

For the full ENU state:

```text
Q =
[ Q_e.pp 0      0      Q_e.pv 0      0      ]
[ 0      Q_n.pp 0      0      Q_n.pv 0      ]
[ 0      0      Q_u.pp 0      0      Q_u.pv ]
[ Q_e.vp 0      0      Q_e.vv 0      0      ]
[ 0      Q_n.vp 0      0      Q_n.vv 0      ]
[ 0      0      Q_u.vp 0      0      Q_u.vv ]
```

where each axis block uses the same formula above.

The acceleration sigma values are selected by flight phase:

```text
idle:
  horizontal = kEstimatorIdleAccelSigmaMss
  vertical   = kEstimatorIdleAccelSigmaMss

flight/boost/coast:
  horizontal = kEstimatorFlightHorizontalAccelSigmaMss
  vertical   = kEstimatorFlightVerticalAccelSigmaMss

descent/touchdown/complete:
  horizontal = kEstimatorDescentHorizontalAccelSigmaMss
  vertical   = kEstimatorDescentVerticalAccelSigmaMss
```

Current control flow bypasses prediction while idle and holds the navigation
state fixed, so the idle acceleration sigma only matters if prediction is later
enabled during idle.

Higher `Q` makes the filter less confident in integrated IMU motion and more
willing to move toward barometer/GPS measurements. Lower `Q` makes it trust the
acceleration integration more strongly.

## R Matrix

`R` is measurement covariance. It represents measurement uncertainty.

The implementation applies measurements one scalar at a time, so it does not
store one large `R` matrix. Each scalar update passes a single variance. This is
equivalent to using diagonal `R` entries and independent measurement axes.

Measurement models:

```text
barometer altitude:
  z = p_u + noise
  H = [0 0 1 0 0 0]
  R = kEstimatorBaroSigmaM^2

GPS east position:
  z = p_e + noise
  H = [1 0 0 0 0 0]
  R = max(gps.hAcc, kEstimatorMinGpsHorizontalSigmaM)^2

GPS north position:
  z = p_n + noise
  H = [0 1 0 0 0 0]
  R = max(gps.hAcc, kEstimatorMinGpsHorizontalSigmaM)^2

GPS up position:
  z = p_u + noise
  H = [0 0 1 0 0 0]
  R = max(gps.vAcc, kEstimatorMinGpsVerticalSigmaM)^2

GPS east velocity:
  z = v_e + noise
  H = [0 0 0 1 0 0]
  R = max(gps.sAcc, kEstimatorMinGpsVelocitySigmaMps)^2

GPS north velocity:
  z = v_n + noise
  H = [0 0 0 0 1 0]
  R = max(gps.sAcc, kEstimatorMinGpsVelocitySigmaMps)^2

GPS up velocity:
  z = v_u + noise
  H = [0 0 0 0 0 1]
  R = max(gps.sAcc, kEstimatorMinGpsVelocitySigmaMps)^2
```

Higher `R` means "trust this measurement less." Lower `R` means "trust this
measurement more." The GPS minimum sigmas prevent u-blox reported accuracy from
making the filter unrealistically confident.

## Scalar Measurement Update

For a scalar measurement, the standard Kalman update is:

```text
y = z - H * x
S = H * P * H^T + R
K = P * H^T * inv(S)
x = x + K * y
P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
```

where:

- `z` is the measurement
- `H` selects the measured state component
- `y` is the residual/innovation
- `S` is innovation variance
- `K` is Kalman gain
- the final covariance equation is the Joseph form, which is numerically safer
  than the simpler `(I - K * H) * P`

Because each measurement observes exactly one state element, `H` is a row vector
with one `1` and the rest `0`. For example, barometer altitude uses:

```text
H = [0 0 1 0 0 0]
```

That lets the implementation avoid explicitly building `H`. It uses the measured
state index instead.

Before applying an update, the estimator computes normalized innovation:

```text
normalized_innovation = abs(y) / sqrt(S)
```

If this exceeds the configured gate, the measurement is rejected:

```text
barometer gate = kEstimatorBaroGateSigma
GPS position gate = kEstimatorGpsPositionGateSigma
GPS velocity gate = kEstimatorGpsVelocityGateSigma
```

This rejects measurements that are too far away from the predicted state given
the combined state and measurement uncertainty.

## GPS Handling

GPS position updates are applied only when:

- `gps.position_valid` is true
- the GPS packet is fresh according to `kEstimatorGpsMaxAgeMs`
- the GPS measurement ID has not already been fused

The measurement ID uses `gps.i_tow_ms` when available, otherwise it falls back to
the packet timestamp. This prevents fusing the same GPS solution multiple times
while the sensor task runs faster than the GPS output rate.

GPS velocity updates use the same freshness and duplicate checks, but require
`gps.velocity_valid`.

The GPS outage status flag is separate from fusion acceptance. It is set when no
valid/recent GPS has been observed for `kEstimatorGpsOutageMs`.

## AHRS Details Relevant to the Filter

The Kalman filter's acceleration input depends on the AHRS:

1. Raw accelerometer is in body frame.
2. Fusion AHRS estimates body-to-world orientation.
3. Fusion rotates acceleration into the ENU earth frame.
4. Fusion removes gravity.
5. The filter integrates the resulting linear acceleration.

At idle/startup, the AHRS uses the accelerometer as a gravity reference and also
initializes the quaternion from the measured gravity vector. This aligns pitch
and roll on the pad. With the magnetometer disabled, yaw is not observable from
gravity and remains arbitrary/fixed from startup. That is acceptable for
vertical altitude/velocity, but horizontal ENU position and velocity can be
rotated by an unknown yaw angle until magnetometer calibration or another yaw
reference is added.

During flight, `kEstimatorAhrsFlightGain` is currently `0`, so the AHRS performs
gyro-only propagation. This avoids treating high-thrust acceleration as gravity.

## Tuning Summary

Tune `P` when startup confidence is wrong:

- Increase initial position sigma if the initial local position is uncertain.
- Increase initial velocity sigma if liftoff velocity might not be near zero.
- Decrease sigma only when you are sure the state starts tightly known.

Tune `Q` when prediction behavior is wrong:

- Increase acceleration sigma if IMU integration drifts too confidently or GPS
  and baro corrections are too slow to pull the state back.
- Decrease acceleration sigma if GPS/baro noise causes too much movement in the
  state estimate.
- Horizontal `Q` should usually be larger than vertical `Q` during flight if yaw
  is not well known, because yaw error mostly corrupts horizontal acceleration.

Tune `R` when measurement trust is wrong:

- Increase barometer sigma if baro noise is shaking altitude/vertical velocity.
- Decrease barometer sigma if altitude lags too much and baro data is clean.
- Increase GPS minimum sigmas if GPS jumps pull the estimate around.
- Decrease GPS minimum sigmas only after validating GPS quality in real logs.

Tune gates when rejection behavior is wrong:

- Increase a gate if good measurements are being rejected during expected flight
  dynamics.
- Decrease a gate if bad outliers are being accepted.

Watch the debug stream fields:

```text
estimator.cov_pos
estimator.cov_vel
estimator.normalized_baro_innovation
estimator.normalized_gps_position_innovation
estimator.normalized_gps_velocity_innovation
estimator.baro_update_accepted
estimator.gps_position_update_accepted
estimator.gps_velocity_update_accepted
estimator.gps_outage
estimator.ahrs_gyro_only
estimator.ahrs_accel_rejected
```

Normalized innovation near the gate means a measurement is barely accepted.
Repeated rejected updates mean either the measurement is bad, the filter is too
confident, or the prediction is biased.

## Current Limitations

The current filter is intentionally small and linear. It does not estimate:

- accelerometer bias
- gyro bias during flight
- attitude covariance
- GPS/barometer bias
- correlated GPS measurement covariance
- yaw without a calibrated magnetometer or another heading reference

For deployment and airbrake control, the vertical states are the most important:

```text
altitude_m = p_u
velocity_mps = v_u
```

Horizontal position is most useful for recovery and logging. It is corrected by
GPS, but dead-reckoned horizontal motion between GPS updates depends strongly on
orientation quality, especially yaw.
