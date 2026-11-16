# Ground station and live telemetry API

```
 rocket                         ground
┌──────────────────┐   LoRa    ┌─────────────────┐  USB   ┌──────────────────┐  WebSocket/HTTP  ┌───────────┐
│ Teensy 4.1       │ ───────▶  │ ESP32 + RFM95   │ ─────▶ │ ground_station.py│ ───────────────▶ │ dashboard │
│ flight computer  │  427.76   │ receiver_esp32  │ lines  │ decode + serve   │  JSON            │ (any lang)│
└──────────────────┘   MHz     └─────────────────┘        └──────────────────┘                  └───────────┘
                                                                   │
                                                                   └─▶ flight_<date>.csv
```

The antennas and radios are the *link*. This directory is everything after the
link: it decodes the binary packets once, in one place, and gives the dashboard
team plain JSON so they never have to think about bytes, CRCs or radios.

## Quick start (no hardware needed)

```bash
pip install pyserial websockets
python3 ground_station.py --sim          # synthetic flight on a loop
# then open example_client.html in a browser
```

With hardware: flash `receiver_esp32/receiver_esp32.ino` (RadioHead library),
plug it in, and run `python3 ground_station.py --port /dev/ttyUSB0` (or `COM5`).
A CSV log is written automatically; replay one later with `--replay file.csv`.

## API

| Endpoint | What you get |
|---|---|
| `ws://host:8765` | One JSON message per packet as it arrives: `state` (5 Hz), `health` (1 Hz) and `link` (1 Hz). On connect you immediately get the latest state and health. |
| `GET http://host:8766/state` | Latest state packet |
| `GET http://host:8766/health` | Latest health packet |
| `GET http://host:8766/link` | Link statistics |
| `GET http://host:8766/history?seconds=60` | Array of state packets from the last N seconds (max 600) |

CORS is open, so a browser page served from anywhere can call it.

### `state` message (5 Hz)

```json
{"type":"state","seq":42,"t_ms":123456,"t_s":123.456,
 "phase":2,"phase_name":"COAST",
 "flags":{"pyro_armed":false,"deployed":false,"pyro_active":false,"gps_valid":true,
          "baro_accepted":true,"imu_cal_complete":true,"restored":false,"backup_armed":true},
 "alt_m":1234.56,"vel_mps":-12.35,"baro_alt_m":1230.0,
 "q":[0.7071,0.0,0.7071,0.0],
 "lat_deg":39.9522,"lon_deg":-75.1932,
 "accel_mag_mss":9.81,"deploy_reason":0,"deploy_reason_name":"NONE",
 "rx_time":1772000000.123,"rssi_dbm":-71.0,"snr_db":9.5}
```

| Field | Meaning |
|---|---|
| `t_ms` | flight computer uptime; use for x-axes, not `rx_time` |
| `phase` | 0 STARTUP, 1 BOOST, 2 COAST, 3 DESCENT, 4 TOUCHDOWN, 5 COMPLETE |
| `alt_m`, `vel_mps` | the deployment filter's altitude above pad and vertical velocity (+ up). These are what fires the parachute. |
| `baro_alt_m` | raw barometric altitude above pad, for comparison with `alt_m` |
| `q` | orientation quaternion `[w,x,y,z]`, body → ENU world frame (x east, y north, z up). Body +Z is the nose. After launch it is gyro-only, so slow drift is normal. |
| `lat_deg`/`lon_deg` | `null` until GPS has a fix |
| `accel_mag_mss` | magnitude of raw acceleration; ≈ 9.81 on the pad |
| `deploy_reason` | 0 none, 1 altitude (normal), 2 backup timer, 3 coast timeout, 4 restore limit |
| `flags.restored` | the flight computer rebooted mid-flight (watchdog) and resumed |
| `seq` | wraps at 256; gaps mean lost packets (`link.state_lost` counts them) |

### `health` message (1 Hz)

Raw sensors and estimator internals, for the "is something wrong?" panels:

| Field | Meaning / what to watch |
|---|---|
| `accel_mss[3]`, `gyro_rads[3]`, `mag_raw[3]` | body-frame raw sensors. On the pad: \|accel\| ≈ 9.81, gyro ≈ 0. Flat-topped traces in flight = sensor clipping. |
| `pressure_pa`, `temp_c` | barometer |
| `pos_m[3]` | 3D estimator ENU position |
| `accel_bias_mss[3]`, `gyro_bias_rads[3]` | IMU biases learned on the pad; should settle before launch and then freeze |
| `vf_accel_bias_mss` | deployment filter's vertical accel bias |
| `baro_nis`, `vf_baro_nis` | normalised barometer innovation (σ). Sustained > 3 means baro and inertial estimate disagree; > 5 means baro updates are being rejected. |
| `gps_fix_type`, `gps_sats`, `gps_pdop`, `gps_hacc_m`, `gps_alt_m` | GPS quality |
| `status.*` | sd_logging, radio_init, gps_init, mag_available, imu_bias_frozen, gps_outage, watchdog_armed, heartbeats_ok, persistence, debug_mode, pyros_disabled, ahrs_init, ahrs_gyro_only, baro3d_accepted, gps_pos_accepted, gps_vel_accepted |
| `sensor_hz`, `flight_hz` | measured task rates; expect 100 |
| `flight_update_max_us` | longest estimator+logic update in the last second; should stay well under 10 000 |
| `reset_cause`, `reset_was_watchdog`, `restore_count` | boot diagnostics |
| `backup_remaining_ms` | countdown of the backup deployment timer while armed |
| `radio_tx_count`, `radio_tx_fail` | radio statistics from the flight side |

### `link` message (1 Hz)

`rssi_dbm`, `snr_db`, `state_packets`, `health_packets`, `bad_packets` (CRC failures),
`state_lost` (sequence gaps), `last_rx_age_s`, `state_rate_hz`.

## Suggested dashboard checks

The example client implements these; thresholds are starting points.

* On the pad: \|accel\| within ±1 m/s² of 9.81, gyro < 0.05 rad/s, `imu_cal_complete` true, altitude flat.
* Always: `|q|` ≈ 1, `alt_m` − `baro_alt_m` small, `sensor_hz` ≥ 90, `heartbeats_ok`, `sd_logging`.
* In flight: `backup_remaining_ms` counting down, `vf_baro_nis` < 3.
* Alarm: `reset_was_watchdog`, `flags.restored`, `deploy_reason` ≠ 1, `last_rx_age_s` > 2.

## Wire protocol (for reference)

Defined once in `include/telemetry_protocol.h` (C++) and mirrored in
`telemetry_protocol.py`. Little-endian packed structs, CRC-16/CCITT-FALSE,
sent behind the 4-byte RadioHead header. `make -C test/host run` encodes with
the real firmware code and decodes with the Python module, so the two cannot
drift apart unnoticed.

| Packet | Size | Rate | Airtime @125 kHz/SF7 |
|---|---|---|---|
| state (0x01) | 41 B | 5 Hz | ~118 ms |
| health (0x02) | 90 B | 1 Hz | ~210 ms |

That is about 80 % radio duty cycle, which is the practical ceiling at 125 kHz.
For 10 Hz state packets set `kRadioModemConfig1 = 0x82` (250 kHz) on the flight
computer **and** `Bw250Cr45Sf128` in the receiver sketch; expect ~3 dB less range.

## Files

* `ground_station.py` — the server (serial / `--sim` / `--replay`)
* `telemetry_protocol.py` — decoder, no dependencies, importable from any Python tool
* `receiver_esp32/receiver_esp32.ino` — LoRa → serial bridge
* `example_client.html` — minimal browser dashboard: phase, altitude/velocity, CSS-3D attitude cube from the quaternion, altitude chart, health checks. Open it directly from disk.
