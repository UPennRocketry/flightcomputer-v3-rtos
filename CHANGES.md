# Changes: safety net, 100 Hz sampling, binary telemetry, ground-station API

Everything hardware-independent was compiled and run on a PC (`make -C test/host run`):
a simulated 100 Hz flight pad → boost → coast → apogee → deploy at 121 m → touchdown,
the same flight interrupted by a watchdog reset during coast and resumed, the backup
timer / pyro shut-off paths, and a byte-for-byte check that the Python decoder reads
what the firmware encoder writes. **The firmware has not been compiled for the Teensy
here** (no PlatformIO toolchain in this environment), so expect to fix a handful of
include/API nits on first build; the "verify on hardware" list is at the bottom.

## 1. Safety net (new: `supervisor.*`, `persist.*`)

| What | Where | Behaviour |
|---|---|---|
| Hardware watchdog | `supervisor.cpp` | WDOG1, 2 s. Fed by the supervisor task **only** while the sensor task (≤200 ms) and flight task (≤300 ms) both produce heartbeats. Fed unconditionally during `setup()` and in `fatalBlink()` so a failed self-test stays visible instead of boot-looping. |
| Supervisor task | `supervisor.cpp`, priority 5 (highest), 50 ms | Watchdog, backup timer, pyro shut-off, rate measurement, persist refresh. |
| Backup deploy timer | `kBackupDeployAfterLaunchMs` | Fires `kBackupDeployAfterLaunchMs` after launch detection, independent of the estimator or sample queue. **Default 0 = disabled. Set it from your sim (apogee time + margin).** |
| Pyro shut-off | supervisor + flight logic | Igniters turn off `kPyroOnMs` after firing even if no sample ever arrives again. `kPyroOnMs` lowered 5000 → 2000 ms; check your igniter spec. |
| Phase timeouts | `kMaxBoostMs` 15 s, `kMaxCoastMs` 60 s | BOOST→COAST and COAST→DESCENT are forced if the detection never triggers. |
| State persistence | `persist.cpp` (EEPROM) | Phase, home altitude, baro baseline, IMU calibration, elapsed flight time, deploy flag written on every transition and every 1 s in flight. |
| Restore after reset | `main.cpp` `decideRestore()` | Resumes **only** after a watchdog reset with an in-flight record. A power cycle with a stale record clears it, so pyros can never be armed on the bench by leftover state. After `kMaxInFlightRestores` (3) resets in one flight the chute is fired immediately on boot. |
| Deploy reason | `deploy_reason_t` | Logged and telemetered: ALTITUDE, BACKUP_TIMER, COAST_TIMEOUT, RESTORE_LIMIT. |

## 2. Sampling (sensors.cpp, flight_logic.cpp, app_config.h)

* Sensor task 30 ms → **10 ms (100 Hz)**.
* Barometer is a non-blocking start/read state machine (OSR 2048, 4.5 ms conversion), temperature every 10th cycle. `flight_sample_t::baro_fresh` tells the filters when the baro value is new; repeated values are no longer applied as new measurements.
* Magnetometer busy-wait removed: one status check, keep last value if not ready.
* BMI088 configured explicitly: ±24 g / 400 Hz ODR, 40 Hz BW; gyro ±2000 dps / 400 Hz, 47 Hz BW (anti-alias for 100 Hz).
* All sample-count thresholds converted to **milliseconds** (`kLaunchConfirmMs` 150, `kApogeeConfirmMs` 450, `kTouchdownConfirmMs` 750, estimator calibration 750 ms, AHRS recovery 5 s). Flight behaviour no longer silently changes with the sample rate.
* Radio priority 3 → 2 (below flight). Log queue 8 → 64 (SD latency spikes). Pre-launch ring buffer 100 → 300 samples (still 3 s).

## 3. Estimator fixes found by the simulation (vertical_filter.cpp)

These were pre-existing and would have shown up in a real flight:

* **Gate lockout after deployment.** In DESCENT the 1D filter ignores the accelerometer by design, so the parachute opening leaves it ~100 m/s wrong. The barometer innovation then exceeds the 5σ gate and *every* later baro update was rejected; the altitude estimate ran off to −6500 m and touchdown was never detected. Fix: after 30 consecutive rejections the covariance is inflated and the measurement accepted (`kVerticalFilterMaxConsecutiveBaroRejects`).
* **Restart mid-flight** (new scenario) initialises the filter with wide covariance and the persisted bias instead of assuming zero velocity.

## 4. Telemetry (telemetry.*, include/telemetry_protocol.h)

CSV text → packed binary with sequence numbers and CRC-16.

| Packet | Rate | Bytes | Contents |
|---|---|---|---|
| state 0x01 | 5 Hz | 41 | t, phase, flags, filter alt/vel, raw baro alt, quaternion, GPS lat/lon, \|accel\|, deploy reason |
| health 0x02 | 1 Hz | 90 | raw accel/gyro/mag, pressure/temp, 3D position, IMU biases, baro innovations, GPS quality, status bits, measured task rates, flight-update time, reset cause, restore count, backup-timer countdown, radio stats |

Radio settings unchanged (427.76 MHz, SF7, 125 kHz) so an existing receiver still hears it. `kRadioModemConfig1` is now a constant: 0x82 = 250 kHz allows 10 Hz.

## 5. Ground station (new: `ground/`)

`ground_station.py` reads the receiver over USB, decodes once, and serves JSON on a
WebSocket (`:8765`) and HTTP (`:8766/state`, `/health`, `/link`, `/history`). It logs a
CSV, can replay one, and has `--sim` so the dashboard team can develop with no hardware.
`receiver_esp32.ino` is the LoRa→serial bridge, `example_client.html` a working
starter dashboard (quaternion cube, altitude chart, health checks). See `ground/README.md`.

## Verify on hardware (in this order)

1. `pio run -e teensy41_debug` — fix any compile nits (most likely: BMI088 `setOdr`/`setRange` enum names, `WDOG1_*`/`SRC_SRSR` macros from `imxrt.h`, `EEPROM.h` include).
2. Bench, debug build: confirm `sensor_hz`/`flight_hz` ≈ 100 and `flight_update_max_us` well under 10 000 in the health packet or debug stream. If the update time is too long, drop the sensor rate to 20 ms first.
3. Watchdog: temporarily add an infinite loop to the flight task; the board must reset within ~2 s and the serial banner must say "WATCHDOG RESET IN FLIGHT" only if a flight was in progress (on the bench it should say nothing or "stale record… clearing").
4. Barometer: check `baro_fresh` is 1 on 9 of 10 log rows and the altitude is noise-free at rest. If the ADC reads 0 (conversion not done), raise `kBaroConversionMs` to 6.
5. Set `kBackupDeployAfterLaunchMs` from your simulation. Until you do, the banner warns that it is disabled.
6. Pyro: with igniters disconnected, confirm the pin goes high at ≤123 m in a debug phase-override test and low exactly `kPyroOnMs` later.
7. Radio: update the ground receiver to the new frames; `ground_station.py --port …` should show `bad_packets: 0` and `state_rate_hz ≈ 5`.
8. Do a full bench "flight" with the debug phase override and read the SD CSV: new columns `BaroFresh`, `DeployReason`, `Restored`.
