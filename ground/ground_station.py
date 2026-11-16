#!/usr/bin/env python3
"""Ground station: turns LoRa frames from the receiver into a live JSON API.

                 LoRa                USB serial                WebSocket / HTTP
  flight computer ---> receiver board ---------> ground_station.py ---------------> dashboards
                                                       |
                                                       +--> flight_YYYYmmdd_HHMMSS.csv

The dashboard team never touches radio code or byte layouts: they connect to
ws://<host>:8765 and receive one JSON object per packet, or poll the HTTP API.

Run against hardware:
    python3 ground_station.py --port /dev/ttyUSB0

Develop a dashboard with no hardware at all (synthetic flight, loops forever):
    python3 ground_station.py --sim

Replay a CSV this program recorded earlier, in real time:
    python3 ground_station.py --replay flight_20260301_140501.csv

Dependencies: pip install pyserial websockets

API (see README.md for the full field list):
    ws://host:8765            every packet as JSON: {"type":"state",...} / {"type":"health",...}
                              plus {"type":"link",...} once per second with RSSI/SNR/loss stats
    GET /state                latest state packet
    GET /health               latest health packet
    GET /link                 link statistics
    GET /history?seconds=60   state packets from the last N seconds (max 600)
    GET /                     this description
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import os
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import telemetry_protocol as proto  # noqa: E402

HISTORY_SECONDS = 600


# ---------------------------------------------------------------------------
# Shared store
# ---------------------------------------------------------------------------
class Store:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.latest_state: dict | None = None
        self.latest_health: dict | None = None
        self.history: deque[dict] = deque()  # state packets with wall-clock "rx_time"
        self.link = {
            "rssi_dbm": None, "snr_db": None,
            "state_packets": 0, "health_packets": 0, "bad_packets": 0,
            "state_lost": 0, "last_rx_age_s": None, "state_rate_hz": 0.0,
        }
        self._last_state_seq: int | None = None
        self._last_rx = 0.0
        self._rate_window: deque[float] = deque()
        self.subscribers: set[asyncio.Queue] = set()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.csv_writer = None
        self.csv_file = None

    def open_csv(self, path: str) -> None:
        self.csv_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "rx_time", "type", "seq", "t_ms", "phase", "alt_m", "vel_mps", "baro_alt_m",
            "qw", "qx", "qy", "qz", "lat_deg", "lon_deg", "accel_mag_mss", "flags", "deploy_reason",
            "rssi_dbm", "snr_db", "health_json",
        ])
        print(f"logging to {path}")

    def ingest(self, pkt: dict, rssi: float | None, snr: float | None) -> None:
        now = time.time()
        pkt["rx_time"] = now
        pkt["rssi_dbm"] = rssi
        pkt["snr_db"] = snr
        with self.lock:
            self._last_rx = now
            if rssi is not None:
                self.link["rssi_dbm"] = rssi
            if snr is not None:
                self.link["snr_db"] = snr
            if pkt["type"] == "state":
                self.latest_state = pkt
                self.link["state_packets"] += 1
                if self._last_state_seq is not None:
                    gap = (pkt["seq"] - self._last_state_seq - 1) % 256
                    if 0 < gap < 128:
                        self.link["state_lost"] += gap
                self._last_state_seq = pkt["seq"]
                self.history.append(pkt)
                while self.history and now - self.history[0]["rx_time"] > HISTORY_SECONDS:
                    self.history.popleft()
                self._rate_window.append(now)
                while self._rate_window and now - self._rate_window[0] > 5.0:
                    self._rate_window.popleft()
                self.link["state_rate_hz"] = round(len(self._rate_window) / 5.0, 2)
            else:
                self.latest_health = pkt
                self.link["health_packets"] += 1
            if self.csv_writer:
                self._write_csv(pkt)
        self._broadcast(pkt)

    def _write_csv(self, pkt: dict) -> None:
        if pkt["type"] == "state":
            flags = sum(1 << i for i, n in enumerate(proto.FLAG_BITS) if pkt["flags"][n])
            row = [f"{pkt['rx_time']:.3f}", "state", pkt["seq"], pkt["t_ms"], pkt["phase"],
                   pkt["alt_m"], pkt["vel_mps"], pkt["baro_alt_m"], *pkt["q"],
                   pkt["lat_deg"], pkt["lon_deg"], pkt["accel_mag_mss"], flags, pkt["deploy_reason"],
                   pkt["rssi_dbm"], pkt["snr_db"], ""]
        else:
            row = [f"{pkt['rx_time']:.3f}", "health", pkt["seq"], pkt["t_ms"]] + [""] * 13 + \
                  [pkt["rssi_dbm"], pkt["snr_db"], json.dumps(pkt)]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def bad_packet(self) -> None:
        with self.lock:
            self.link["bad_packets"] += 1

    def link_snapshot(self) -> dict:
        with self.lock:
            d = dict(self.link)
            d["last_rx_age_s"] = round(time.time() - self._last_rx, 2) if self._last_rx else None
            d["type"] = "link"
            return d

    def _broadcast(self, obj: dict) -> None:
        if not self.loop:
            return
        data = json.dumps(obj)
        for q in list(self.subscribers):
            self.loop.call_soon_threadsafe(_offer, q, data)


def _offer(q: asyncio.Queue, data: str) -> None:
    if q.full():
        try:
            q.get_nowait()
        except asyncio.QueueEmpty:
            pass
    q.put_nowait(data)


# ---------------------------------------------------------------------------
# Sources: serial receiver, simulator, replay
# ---------------------------------------------------------------------------
def serial_source(store: Store, port: str, baud: int) -> None:
    """Reads 'RX,<rssi>,<snr>,<hex>' lines from the receiver board."""
    import serial  # pyserial

    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                print(f"receiver connected on {port}")
                while True:
                    line = ser.readline().decode("ascii", "replace").strip()
                    if not line:
                        continue
                    if not line.startswith("RX,"):
                        print(f"receiver: {line}")
                        continue
                    try:
                        _, rssi, snr, hexdata = line.split(",", 3)
                        raw = proto.strip_radiohead_header(bytes.fromhex(hexdata))
                        pkt = proto.decode(raw)
                    except ValueError:
                        pkt = None
                    if pkt is None:
                        store.bad_packet()
                        continue
                    store.ingest(pkt, float(rssi), float(snr))
        except serial.SerialException as exc:
            print(f"serial: {exc}; retrying in 2 s")
            time.sleep(2)


def sim_source(store: Store) -> None:
    """A plausible synthetic flight, looping, so dashboards can be built without hardware."""
    t = 0.0
    dt = 0.2
    seq_s = seq_h = 0
    while True:
        alt = vel = 0.0
        phase = 0
        deployed = False
        launch_t = 3.0
        t = 0.0
        last_health = -10
        while phase < 5:
            if t < launch_t:
                a = 0.0
            elif t < launch_t + 2.0:
                a = 60.0
            elif deployed:
                a = (-6.0 - vel) * 2.0
            else:
                a = -9.81
            vel += a * dt
            alt = max(0.0, alt + vel * dt)
            if phase == 0 and t >= launch_t + 0.15: phase = 1
            elif phase == 1 and t >= launch_t + 2.15: phase = 2
            elif phase == 2 and vel < -3: phase = 3
            if phase == 3 and alt <= 123 and not deployed: deployed = True; fire_t = t
            if phase == 3 and alt <= 0.5 and abs(vel) < 6: phase = 4; td_t = t
            if phase == 4 and t - td_t > 10: phase = 5
            angle = 0.3 * math.sin(t * 0.7) if phase in (3, 4) else 0.05 * math.sin(t)
            q = [math.cos(angle / 2), math.sin(angle / 2) * 0.7, math.sin(angle / 2) * 0.7, 0.0]
            flags = {n: False for n in proto.FLAG_BITS}
            flags.update(gps_valid=True, baro_accepted=True, imu_cal_complete=t > 1,
                         pyro_armed=(phase == 3 and not deployed), deployed=deployed,
                         pyro_active=deployed and (t - fire_t) < 2.0, backup_armed=phase in (1, 2, 3) and not deployed)
            state = {
                "type": "state", "seq": seq_s & 0xFF, "t_ms": int(t * 1000), "t_s": t,
                "phase": phase, "phase_name": proto.PHASE_NAMES[phase], "flags": flags,
                "alt_m": round(alt + math.sin(t * 9) * 0.3, 2), "vel_mps": round(vel, 2),
                "baro_alt_m": round(alt + math.sin(t * 13) * 0.8, 2), "q": [round(x, 4) for x in q],
                "lat_deg": 39.9522 + alt * 1e-6, "lon_deg": -75.1932 + alt * 2e-6,
                "accel_mag_mss": round(abs(a + 9.81) + 0.1, 2),
                "deploy_reason": 1 if deployed else 0, "deploy_reason_name": "ALTITUDE" if deployed else "NONE",
            }
            seq_s += 1
            store.ingest(state, -70 - alt / 50, 9.5 - alt / 300)
            if t - last_health >= 1.0:
                last_health = t
                status = {n: False for n in proto.STATUS_BITS}
                status.update(sd_logging=True, radio_init=True, gps_init=True, mag_available=True,
                              imu_bias_frozen=phase > 0, watchdog_armed=True, heartbeats_ok=True,
                              persistence=True, ahrs_init=True, ahrs_gyro_only=phase > 0, baro3d_accepted=True,
                              gps_pos_accepted=True, gps_vel_accepted=True)
                health = {
                    "type": "health", "seq": seq_h & 0xFF, "t_ms": int(t * 1000), "t_s": t,
                    "accel_mss": [0.02, -0.01, round(a + 9.81, 3)], "gyro_rads": [0.001, -0.002, 0.0005],
                    "mag_raw": [120, -340, 560], "pressure_pa": int(101325 * (1 - 2.25577e-5 * alt) ** 5.25588),
                    "temp_c": 21.5, "pos_m": [round(alt * 0.01, 2), round(alt * 0.02, 2), round(alt, 2)],
                    "accel_bias_mss": [0.012, -0.034, 0.056], "gyro_bias_rads": [0.0012, -0.0034, 0.0056],
                    "vf_accel_bias_mss": 0.078, "baro_nis": round(abs(math.sin(t)) * 1.5, 2),
                    "vf_baro_nis": round(abs(math.cos(t)) * 1.2, 2), "gps_alt_m": round(alt + 12.0, 2),
                    "gps_fix_type": 3, "gps_sats": 11, "gps_pdop": 1.6, "gps_hacc_m": 2.1, "status": status,
                    "sensor_hz": 100, "flight_hz": 100, "flight_update_max_us": 820, "reset_cause": 0x01,
                    "reset_was_watchdog": False, "restore_count": 0,
                    "backup_remaining_ms": max(0, int((launch_t + 20 - t) * 1000)) if phase in (1, 2, 3) and not deployed else 0,
                    "radio_tx_count": seq_s + seq_h, "radio_tx_fail": 0,
                }
                seq_h += 1
                store.ingest(health, -70 - alt / 50, 9.5 - alt / 300)
            t += dt
            time.sleep(dt)
        time.sleep(3)


def replay_source(store: Store, path: str, speed: float) -> None:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    print(f"replaying {len(rows)} rows from {path}")
    t0 = None
    start = time.time()
    for row in rows:
        rx = float(row["rx_time"])
        if t0 is None:
            t0 = rx
        delay = (rx - t0) / speed - (time.time() - start)
        if delay > 0:
            time.sleep(delay)
        if row["type"] == "state":
            flags_int = int(row["flags"] or 0)
            pkt = {
                "type": "state", "seq": int(row["seq"]), "t_ms": int(row["t_ms"]), "t_s": int(row["t_ms"]) / 1000,
                "phase": int(row["phase"]), "phase_name": proto.PHASE_NAMES.get(int(row["phase"]), "?"),
                "flags": {n: bool(flags_int & (1 << i)) for i, n in enumerate(proto.FLAG_BITS)},
                "alt_m": float(row["alt_m"]), "vel_mps": float(row["vel_mps"]), "baro_alt_m": float(row["baro_alt_m"]),
                "q": [float(row[k]) for k in ("qw", "qx", "qy", "qz")],
                "lat_deg": float(row["lat_deg"]) if row["lat_deg"] not in ("", "None") else None,
                "lon_deg": float(row["lon_deg"]) if row["lon_deg"] not in ("", "None") else None,
                "accel_mag_mss": float(row["accel_mag_mss"]), "deploy_reason": int(row["deploy_reason"]),
                "deploy_reason_name": proto.DEPLOY_REASON_NAMES.get(int(row["deploy_reason"]), "?"),
            }
        else:
            pkt = json.loads(row["health_json"])
        rssi = float(row["rssi_dbm"]) if row["rssi_dbm"] not in ("", "None") else None
        snr = float(row["snr_db"]) if row["snr_db"] not in ("", "None") else None
        store.ingest(pkt, rssi, snr)
    print("replay finished")


# ---------------------------------------------------------------------------
# HTTP API
# ---------------------------------------------------------------------------
def make_http_handler(store: Store):
    class Handler(BaseHTTPRequestHandler):
        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/state":
                self._json(store.latest_state or {"type": "state", "empty": True})
            elif u.path == "/health":
                self._json(store.latest_health or {"type": "health", "empty": True})
            elif u.path == "/link":
                self._json(store.link_snapshot())
            elif u.path == "/history":
                secs = min(float(parse_qs(u.query).get("seconds", ["60"])[0]), HISTORY_SECONDS)
                cutoff = time.time() - secs
                with store.lock:
                    self._json([p for p in store.history if p["rx_time"] >= cutoff])
            elif u.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(__doc__.encode())
            else:
                self._json({"error": "not found"}, 404)

        def log_message(self, *args):
            pass

    return Handler


# ---------------------------------------------------------------------------
# WebSocket
# ---------------------------------------------------------------------------
async def ws_main(store: Store, host: str, port: int) -> None:
    import websockets

    store.loop = asyncio.get_running_loop()

    async def handler(ws):
        q: asyncio.Queue = asyncio.Queue(maxsize=200)
        store.subscribers.add(q)
        try:
            if store.latest_state:
                await ws.send(json.dumps(store.latest_state))
            if store.latest_health:
                await ws.send(json.dumps(store.latest_health))
            while True:
                await ws.send(await q.get())
        except Exception:
            pass
        finally:
            store.subscribers.discard(q)

    async def link_ticker():
        while True:
            await asyncio.sleep(1.0)
            store._broadcast(store.link_snapshot())

    async with websockets.serve(handler, host, port, max_queue=None):
        print(f"websocket on ws://{host}:{port}")
        await link_ticker()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the receiver board (e.g. /dev/ttyUSB0, COM5)")
    src.add_argument("--sim", action="store_true", help="synthetic flight, no hardware")
    src.add_argument("--replay", metavar="CSV", help="replay a CSV written by this program")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--http-port", type=int, default=8766)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--no-log", action="store_true", help="do not write a CSV log")
    args = ap.parse_args()

    store = Store()
    if not args.no_log and not args.replay:
        store.open_csv(time.strftime("flight_%Y%m%d_%H%M%S.csv"))

    if args.port:
        target, targs = serial_source, (store, args.port, args.baud)
    elif args.sim:
        target, targs = sim_source, (store,)
    else:
        target, targs = replay_source, (store, args.replay, args.speed)
    threading.Thread(target=target, args=targs, daemon=True).start()

    httpd = ThreadingHTTPServer((args.host, args.http_port), make_http_handler(store))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"http on http://{args.host}:{args.http_port}/state")

    try:
        asyncio.run(ws_main(store, args.host, args.ws_port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
