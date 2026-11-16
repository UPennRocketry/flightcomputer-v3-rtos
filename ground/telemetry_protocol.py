#!/usr/bin/env python3
"""Python mirror of include/telemetry_protocol.h.

Decodes the binary LoRa packets sent by the flight computer into plain dicts in
engineering units (metres, m/s, degrees, ...). No third-party dependencies.

    from telemetry_protocol import decode
    pkt = decode(raw_bytes)      # -> dict, or None if CRC/length/type is wrong

The C++ header is the source of truth. If either side changes, update both and
run `make -C test/host run`, which encodes with the real firmware code and
decodes with this module.
"""

from __future__ import annotations

import json
import struct
import sys
from typing import Optional

PROTOCOL_VERSION = 1
PACKET_TYPE_STATE = 0x01
PACKET_TYPE_HEALTH = 0x02

PHASE_NAMES = {0: "STARTUP", 1: "BOOST", 2: "COAST", 3: "DESCENT", 4: "TOUCHDOWN", 5: "COMPLETE"}
DEPLOY_REASON_NAMES = {0: "NONE", 1: "ALTITUDE", 2: "BACKUP_TIMER", 3: "COAST_TIMEOUT", 4: "RESTORE_LIMIT"}

# state_packet_t.flags
FLAG_BITS = [
    "pyro_armed", "deployed", "pyro_active", "gps_valid",
    "baro_accepted", "imu_cal_complete", "restored", "backup_armed",
]
# health_packet_t.status
STATUS_BITS = [
    "sd_logging", "radio_init", "gps_init", "mag_available",
    "imu_bias_frozen", "gps_outage", "watchdog_armed", "heartbeats_ok",
    "persistence", "debug_mode", "pyros_disabled", "ahrs_init",
    "ahrs_gyro_only", "baro3d_accepted", "gps_pos_accepted", "gps_vel_accepted",
]

# struct formats, little-endian, packed (must match the #pragma pack(1) structs)
STATE_FMT = "<BBIBBiii4hiihBH"
HEALTH_FMT = "<BBI3h3h3hih3i3h3hhhhiBBHHHBBHBBIIHH"
STATE_LEN = struct.calcsize(STATE_FMT)    # 41
HEALTH_LEN = struct.calcsize(HEALTH_FMT)  # 90
assert STATE_LEN == 41, STATE_LEN
assert HEALTH_LEN == 90, HEALTH_LEN


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _bits(value: int, names: list[str]) -> dict:
    return {name: bool(value & (1 << i)) for i, name in enumerate(names)}


def decode_state(raw: bytes) -> Optional[dict]:
    if len(raw) < STATE_LEN:
        return None
    raw = raw[:STATE_LEN]
    f = struct.unpack(STATE_FMT, raw)
    (ptype, seq, t_ms, phase, flags, alt_cm, vel_cms, baro_alt_cm,
     qw, qx, qy, qz, lat, lon, accel_mag, deploy_reason, crc) = f
    if ptype != PACKET_TYPE_STATE or crc != crc16(raw[:-2]):
        return None
    gps_valid = bool(flags & (1 << 3))
    return {
        "type": "state",
        "seq": seq,
        "t_ms": t_ms,
        "t_s": t_ms / 1000.0,
        "phase": phase,
        "phase_name": PHASE_NAMES.get(phase, f"UNKNOWN({phase})"),
        "flags": _bits(flags, FLAG_BITS),
        "alt_m": alt_cm / 100.0,
        "vel_mps": vel_cms / 100.0,
        "baro_alt_m": baro_alt_cm / 100.0,
        "q": [qw / 10000.0, qx / 10000.0, qy / 10000.0, qz / 10000.0],
        "lat_deg": lat / 1e7 if gps_valid else None,
        "lon_deg": lon / 1e7 if gps_valid else None,
        "accel_mag_mss": accel_mag / 100.0,
        "deploy_reason": deploy_reason,
        "deploy_reason_name": DEPLOY_REASON_NAMES.get(deploy_reason, str(deploy_reason)),
    }


def decode_health(raw: bytes) -> Optional[dict]:
    if len(raw) < HEALTH_LEN:
        return None
    raw = raw[:HEALTH_LEN]
    f = struct.unpack(HEALTH_FMT, raw)
    i = 0

    def take(n=1):
        nonlocal i
        v = f[i:i + n]
        i += n
        return v if n > 1 else v[0]

    ptype, seq, t_ms = take(), take(), take()
    accel = take(3); gyro = take(3); mag = take(3)
    pressure_pa = take(); temp_cc = take()
    pos = take(3); accel_bias = take(3); gyro_bias = take(3)
    vf_bias = take(); baro_nis = take(); vf_baro_nis = take()
    gps_alt_cm = take(); gps_fix = take(); gps_sats = take(); gps_pdop = take(); gps_hacc = take()
    status = take(); sensor_hz = take(); flight_hz = take(); max_us = take()
    reset_cause = take(); restore_count = take(); backup_ms = take()
    tx_count = take(); tx_fail = take(); crc = take()
    if ptype != PACKET_TYPE_HEALTH or crc != crc16(raw[:-2]):
        return None
    return {
        "type": "health",
        "seq": seq,
        "t_ms": t_ms,
        "t_s": t_ms / 1000.0,
        "accel_mss": [a / 100.0 for a in accel],
        "gyro_rads": [g / 100.0 for g in gyro],
        "mag_raw": list(mag),
        "pressure_pa": pressure_pa,
        "temp_c": temp_cc / 100.0,
        "pos_m": [p / 100.0 for p in pos],
        "accel_bias_mss": [b / 1000.0 for b in accel_bias],
        "gyro_bias_rads": [b / 10000.0 for b in gyro_bias],
        "vf_accel_bias_mss": vf_bias / 1000.0,
        "baro_nis": baro_nis / 100.0,
        "vf_baro_nis": vf_baro_nis / 100.0,
        "gps_alt_m": gps_alt_cm / 100.0,
        "gps_fix_type": gps_fix,
        "gps_sats": gps_sats,
        "gps_pdop": gps_pdop / 100.0,
        "gps_hacc_m": gps_hacc / 100.0,
        "status": _bits(status, STATUS_BITS),
        "sensor_hz": sensor_hz,
        "flight_hz": flight_hz,
        "flight_update_max_us": max_us,
        "reset_cause": reset_cause,
        "reset_was_watchdog": bool(reset_cause & (1 << 4)),
        "restore_count": restore_count,
        "backup_remaining_ms": backup_ms,
        "radio_tx_count": tx_count,
        "radio_tx_fail": tx_fail,
    }


def decode(raw: bytes) -> Optional[dict]:
    """Decode a packet of either type. Returns None if it is not valid."""
    if not raw:
        return None
    if raw[0] == PACKET_TYPE_STATE:
        return decode_state(raw)
    if raw[0] == PACKET_TYPE_HEALTH:
        return decode_health(raw)
    return None


def strip_radiohead_header(frame: bytes) -> bytes:
    """Frames received with RadioHead carry a 4-byte to/from/id/flags header."""
    return frame[4:] if len(frame) > 4 and frame[4] in (PACKET_TYPE_STATE, PACKET_TYPE_HEALTH) else frame


# ---------------------------------------------------------------------------
# Self-test used by test/host: decode bytes written by the firmware encoder.
# ---------------------------------------------------------------------------
def _selftest(state_bin: str, health_bin: str, expected_json: str) -> int:
    with open(state_bin, "rb") as f:
        sb = f.read()
    with open(health_bin, "rb") as f:
        hb = f.read()
    with open(expected_json) as f:
        exp = json.load(f)
    s_raw = struct.unpack(STATE_FMT, sb)
    h_raw = struct.unpack(HEALTH_FMT, hb)
    s = decode(sb)
    h = decode(hb)
    ok = True

    def check(name, got, want):
        nonlocal ok
        if got != want:
            ok = False
            print(f"  MISMATCH {name}: got {got!r}, firmware wrote {want!r}")

    if s is None or h is None:
        print("  decode returned None (CRC or layout mismatch)")
        return 1
    es, eh = exp["state"], exp["health"]
    check("state.seq", s["seq"], es["seq"])
    check("state.t_ms", s["t_ms"], es["t_ms"])
    check("state.phase", s["phase"], es["phase"])
    check("state.alt_cm", s_raw[5], es["alt_cm"])
    check("state.vel_cms", s_raw[6], es["vel_cms"])
    check("state.baro_alt_cm", s_raw[7], es["baro_alt_cm"])
    check("state.q", list(s_raw[8:12]), es["q"])
    check("state.lat_1e7", s_raw[12], es["lat_1e7"])
    check("state.lon_1e7", s_raw[13], es["lon_1e7"])
    check("state.accel_mag_cmss", s_raw[14], es["accel_mag_cmss"])
    check("state.flags", s_raw[4], es["flags"])
    check("health.seq", h["seq"], eh["seq"])
    check("health.pressure_pa", h["pressure_pa"], eh["pressure_pa"])
    check("health.temp_cc", h_raw[13], eh["temp_cc"])
    check("health.gps_sats", h["gps_sats"], eh["gps_sats"])
    check("health.gps_pdop_x100", h_raw[29], eh["gps_pdop_x100"])
    check("health.status", h_raw[31], eh["status"])
    check("health.sensor_hz", h["sensor_hz"], eh["sensor_hz"])
    check("health.flight_update_max_us", h["flight_update_max_us"], eh["flight_update_max_us"])
    check("health.backup_remaining_ms", h["backup_remaining_ms"], eh["backup_remaining_ms"])
    check("health.radio_tx_count", h["radio_tx_count"], eh["radio_tx_count"])
    # a flipped bit must fail the CRC
    bad = bytearray(sb); bad[10] ^= 0x01
    check("corrupt packet rejected", decode(bytes(bad)), None)
    print("  python decoder matches firmware encoder" if ok else "  PROTOCOL MISMATCH")
    print(json.dumps(s, indent=1)[:400] + "\n  ...")
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) == 5 and sys.argv[1] == "--selftest":
        sys.exit(_selftest(sys.argv[2], sys.argv[3], sys.argv[4]))
    print(__doc__)
