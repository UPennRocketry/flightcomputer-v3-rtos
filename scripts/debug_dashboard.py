#!/usr/bin/env python3
"""Live USB-serial dashboard for the Teensy flight-computer debug build.

Flash with:
    pio run -e teensy41_debug -t upload

Install dashboard deps:
    python3 -m pip install -r scripts/debug_dashboard_requirements.txt

Run:
    python3 scripts/debug_dashboard.py --port /dev/cu.usbmodemXXXX
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Iterable

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import serial
from serial.tools import list_ports


PHASE_NAMES = {
    0: "STARTUP",
    1: "BOOST",
    2: "COAST",
    3: "DESCENT",
    4: "TOUCHDOWN",
    5: "COMPLETE",
}
PHASE_VIEW_ORDER: tuple[int, ...] = (0, 1, 2, 3, 4, 5)
AUTO_PHASE_LABEL = "AUTO"


def phase_label(phase: int | None) -> str:
    if phase is None:
        return AUTO_PHASE_LABEL
    return PHASE_NAMES.get(int(phase), f"PHASE {phase}")


def phase_index_for(phase: int | None) -> int:
    if phase is None:
        return 0
    try:
        return PHASE_VIEW_ORDER.index(int(phase))
    except ValueError:
        return 0


@dataclass
class Series:
    maxlen: int
    time_s: Deque[float] = field(init=False)
    accel: list[Deque[float]] = field(init=False)
    gyro: list[Deque[float]] = field(init=False)
    accel_bias_drift: list[Deque[float]] = field(init=False)
    gyro_bias_drift: list[Deque[float]] = field(init=False)
    mag: list[Deque[float]] = field(init=False)
    quat: list[Deque[float]] = field(init=False)
    pos: list[Deque[float]] = field(init=False)
    vel: list[Deque[float]] = field(init=False)
    baro_alt_m: Deque[float] = field(init=False)
    state_alt_m: Deque[float] = field(init=False)
    state_vel_mps: Deque[float] = field(init=False)
    vertical_alt_m: Deque[float] = field(init=False)
    vertical_vel_mps: Deque[float] = field(init=False)
    vertical_accel_mss: Deque[float] = field(init=False)
    vertical_bias_mss: Deque[float] = field(init=False)
    phase: Deque[int] = field(init=False)
    radio_time_s: Deque[float] = field(init=False)
    radio_tx_done_count: Deque[int] = field(init=False)
    last_packet: dict | None = None
    packet_count: int = 0
    parse_errors: int = 0
    raw_line_count: int = 0
    dbg_line_count: int = 0
    ignored_line_count: int = 0
    event_count: int = 0
    reader_status: str = "serial not connected"
    last_reader_error: str = ""
    last_line_preview: str = ""
    command_status: str = "phase override: auto"
    history: Deque[dict] = field(init=False)

    def __post_init__(self) -> None:
        self.time_s = deque(maxlen=self.maxlen)
        self.accel = [deque(maxlen=self.maxlen) for _ in range(3)]
        self.gyro = [deque(maxlen=self.maxlen) for _ in range(3)]
        self.accel_bias_drift = [deque(maxlen=self.maxlen) for _ in range(3)]
        self.gyro_bias_drift = [deque(maxlen=self.maxlen) for _ in range(3)]
        self.mag = [deque(maxlen=self.maxlen) for _ in range(3)]
        self.quat = [deque(maxlen=self.maxlen) for _ in range(4)]
        self.pos = [deque(maxlen=self.maxlen) for _ in range(3)]
        self.vel = [deque(maxlen=self.maxlen) for _ in range(3)]
        self.baro_alt_m = deque(maxlen=self.maxlen)
        self.state_alt_m = deque(maxlen=self.maxlen)
        self.state_vel_mps = deque(maxlen=self.maxlen)
        self.vertical_alt_m = deque(maxlen=self.maxlen)
        self.vertical_vel_mps = deque(maxlen=self.maxlen)
        self.vertical_accel_mss = deque(maxlen=self.maxlen)
        self.vertical_bias_mss = deque(maxlen=self.maxlen)
        self.phase = deque(maxlen=self.maxlen)
        self.radio_time_s = deque(maxlen=self.maxlen)
        self.radio_tx_done_count = deque(maxlen=self.maxlen)
        self.history = deque(maxlen=self.maxlen)

    def add(self, packet: dict) -> None:
        t_s = float(packet["t"]) / 1000.0
        self.time_s.append(t_s)

        for target, values in (
            (self.accel, packet["accel"]),
            (self.gyro, packet["gyro"]),
            (self.mag, packet["mag"]),
            (self.quat, packet["q"]),
            (self.pos, packet["pos"]),
            (self.vel, packet["vel"]),
        ):
            for axis, value in enumerate(values):
                target[axis].append(float(value))

        self.baro_alt_m.append(float(packet["baro_alt_m"]))
        self.state_alt_m.append(float(packet["state_alt_m"]))
        self.state_vel_mps.append(float(packet["state_vel_mps"]))
        self.phase.append(int(packet["phase"]))

        vertical_filter = packet.get("vertical_filter") if isinstance(packet.get("vertical_filter"), dict) else {}
        self.vertical_alt_m.append(float(vertical_filter.get("altitude_m", packet.get("deployment_alt_m", packet["state_alt_m"]))))
        self.vertical_vel_mps.append(float(vertical_filter.get("velocity_mps", packet.get("deployment_vel_mps", packet["state_vel_mps"]))))
        self.vertical_accel_mss.append(float(vertical_filter.get("vertical_accel_mss", 0.0)))
        self.vertical_bias_mss.append(float(vertical_filter.get("accel_bias_mss", 0.0)))

        estimator = packet.get("estimator") if isinstance(packet.get("estimator"), dict) else {}
        accel_drift = list(estimator.get("accel_bias_drift_mss") or [0.0, 0.0, 0.0])
        gyro_drift = list(estimator.get("gyro_bias_drift_rads") or [0.0, 0.0, 0.0])
        accel_drift += [0.0, 0.0, 0.0]
        gyro_drift += [0.0, 0.0, 0.0]
        for axis in range(3):
            self.accel_bias_drift[axis].append(float(accel_drift[axis]))
            self.gyro_bias_drift[axis].append(math.degrees(float(gyro_drift[axis])))

        radio = packet.get("radio")
        if isinstance(radio, dict):
            self.radio_time_s.append(t_s)
            self.radio_tx_done_count.append(int(radio.get("tx_done_count", 0)))

        self.last_packet = packet
        self.history.append(packet)
        self.packet_count += 1

    def debug_packet_hz(self) -> float:
        if len(self.time_s) < 2:
            return 0.0

        dt_s = self.time_s[-1] - self.time_s[0]
        if dt_s <= 0.0:
            return 0.0
        return float(len(self.time_s) - 1) / dt_s

    def radio_tx_hz(self) -> float:
        if len(self.radio_time_s) < 2:
            return 0.0

        dt_s = self.radio_time_s[-1] - self.radio_time_s[0]
        done_delta = self.radio_tx_done_count[-1] - self.radio_tx_done_count[0]
        if dt_s <= 0.0 or done_delta <= 0:
            return 0.0
        return float(done_delta) / dt_s


def guess_serial_port() -> str | None:
    candidates = []
    for port in list_ports.comports():
        text = " ".join(
            part
            for part in (port.device, port.description, port.manufacturer)
            if part
        ).lower()
        if any(token in text for token in ("teensy", "usbmodem", "usb serial", "ttyacm")):
            candidates.append(port.device)

    if candidates:
        return candidates[0]

    ports = list(list_ports.comports())
    return ports[0].device if ports else None


def reader_thread(
    port: str,
    baud: int,
    out_queue: queue.Queue[dict],
    command_queue: queue.Queue[str],
    stop: threading.Event,
    series: Series,
) -> None:
    def send_pending_commands(ser: serial.Serial) -> None:
        while True:
            try:
                command = command_queue.get_nowait()
            except queue.Empty:
                return
            ser.write(f"{command}\n".encode("ascii"))
            ser.flush()
            series.command_status = f"sent {command}"

    while not stop.is_set():
        try:
            series.reader_status = f"opening {port}"
            with serial.Serial(port, baudrate=baud, timeout=0.25) as ser:
                series.reader_status = f"connected to {port}"
                series.last_reader_error = ""
                ser.reset_input_buffer()
                while not stop.is_set():
                    send_pending_commands(ser)
                    raw = ser.readline()
                    send_pending_commands(ser)
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    series.raw_line_count += 1
                    series.last_line_preview = line[:120]
                    dbg_index = line.find("DBG,")
                    if dbg_index < 0:
                        series.ignored_line_count += 1
                        continue
                    series.dbg_line_count += 1
                    payload = line[dbg_index + 4:]
                    try:
                        packet = json.loads(payload)
                    except json.JSONDecodeError as exc:
                        series.parse_errors += 1
                        series.last_reader_error = f"JSON parse error near char {exc.pos}: {exc.msg}"
                        continue
                    if "event" in packet:
                        series.event_count += 1
                        if packet.get("event") == "debug_phase_command":
                            status = str(packet.get("status", "unknown"))
                            detail = str(packet.get("detail", ""))
                            phase = str(packet.get("forced_phase_name", ""))
                            series.command_status = f"firmware {status}: {detail} {phase}".strip()
                        continue
                    out_queue.put(packet)
        except serial.SerialException as exc:
            series.reader_status = "serial error"
            series.last_reader_error = str(exc)
            print(f"serial: {exc}; retrying in 1s")
            time.sleep(1.0)


def normalize_quat(q: Iterable[float]) -> tuple[float, float, float, float]:
    qw, qx, qy, qz = (float(v) for v in q)
    mag = math.sqrt((qw * qw) + (qx * qx) + (qy * qy) + (qz * qz))
    if mag <= 1e-9:
        return 1.0, 0.0, 0.0, 0.0
    return qw / mag, qx / mag, qy / mag, qz / mag


def rotate_body_to_world(q: Iterable[float], v: Iterable[float]) -> tuple[float, float, float]:
    qw, qx, qy, qz = normalize_quat(q)
    vx, vy, vz = (float(component) for component in v)

    # Quaternion-vector rotation expanded as q * v * q^-1.
    tx = 2.0 * ((qy * vz) - (qz * vy))
    ty = 2.0 * ((qz * vx) - (qx * vz))
    tz = 2.0 * ((qx * vy) - (qy * vx))

    rx = vx + (qw * tx) + ((qy * tz) - (qz * ty))
    ry = vy + (qw * ty) + ((qz * tx) - (qx * tz))
    rz = vz + (qw * tz) + ((qx * ty) - (qy * tx))
    return rx, ry, rz


def relative_time(values: Deque[float]) -> list[float]:
    if not values:
        return []
    t0 = values[0]
    return [t - t0 for t in values]


def selected_indices(series: Series, selected_phase: int | None) -> list[int]:
    if selected_phase is None:
        return list(range(len(series.time_s)))
    return [index for index, phase in enumerate(series.phase) if int(phase) == selected_phase]


def values_at(values: Deque[float] | Deque[int], indices: list[int]) -> list[float]:
    items = list(values)
    return [float(items[index]) for index in indices if index < len(items)]


def history_at(series: Series, indices: list[int]) -> list[dict]:
    packets = list(series.history)
    return [packets[index] for index in indices if index < len(packets)]


def packet_phase_label(packet: dict | None) -> str:
    if not packet:
        return "WAITING"
    try:
        phase = int(packet.get("phase", -1))
    except (TypeError, ValueError):
        return "UNKNOWN"
    return str(packet.get("phase_name") or PHASE_NAMES.get(phase, "UNKNOWN"))


def set_line_data(lines, x_values: list[float], axis_values: Iterable[Iterable[float]]) -> None:
    for line, values in zip(lines, axis_values):
        line.set_data(x_values, list(values))


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def bool_label(value) -> str:
    return "yes" if as_bool(value) else "no"


def ok_label(value) -> str:
    return "OK" if as_bool(value) else "NO"


def safe_float(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def percentage(count: int, total: int) -> str:
    if total <= 0:
        return "0%"
    return f"{(100.0 * count / total):.0f}%"


def body_up_label(values: Iterable[float]) -> str:
    components = list(values)
    components += [0.0, 0.0, 1.0]
    body_up = [safe_float(components[index]) for index in range(3)]
    axis = max(range(3), key=lambda index: abs(body_up[index]))
    axis_name = ("X", "Y", "Z")[axis]
    sign = "+" if body_up[axis] >= 0.0 else "-"
    return (
        f"{sign}{axis_name} "
        f"[{body_up[0]:+.2f},{body_up[1]:+.2f},{body_up[2]:+.2f}]"
    )


def format_age_s(packet: dict, gps: dict) -> str:
    try:
        gps_time_ms = int(gps.get("timestamp_ms", 0))
        packet_time_ms = int(packet.get("t", 0))
    except (TypeError, ValueError):
        return "n/a"

    if gps_time_ms <= 0 or packet_time_ms < gps_time_ms:
        return "n/a"
    return f"{(packet_time_ms - gps_time_ms) / 1000.0:.1f}s"


def selected_estimator_summary(series: Series, selected_phase: int | None) -> list[str]:
    indices = selected_indices(series, selected_phase)
    packets = history_at(series, indices)
    label = "ALL PHASES" if selected_phase is None else phase_label(selected_phase)
    if not packets:
        return [f"Phase {label}: no samples in the current history window"]

    times = values_at(series.time_s, indices)
    duration_s = max(0.0, times[-1] - times[0]) if len(times) >= 2 else 0.0
    lines = [f"Phase {label}: samples={len(packets)} duration={duration_s:.1f}s"]

    estimators = [packet.get("estimator") for packet in packets if isinstance(packet.get("estimator"), dict)]
    if not estimators:
        lines.append("Phase KF: estimator fields not present")
        return lines

    baro_updates = sum(1 for est in estimators if as_bool(est.get("baro_update_accepted", 0)))
    gps_pos_updates = sum(1 for est in estimators if as_bool(est.get("gps_position_update_accepted", 0)))
    gps_vel_updates = sum(1 for est in estimators if as_bool(est.get("gps_velocity_update_accepted", 0)))
    gyro_only = sum(1 for est in estimators if as_bool(est.get("ahrs_gyro_only", 0)))
    accel_rejected = sum(1 for est in estimators if as_bool(est.get("ahrs_accel_rejected", 0)))
    gps_outage = sum(1 for est in estimators if as_bool(est.get("gps_outage", 0)))
    imu_samples = max(int(est.get("imu_calibration_samples", 0)) for est in estimators)
    imu_accepted = sum(1 for est in estimators if as_bool(est.get("imu_calibration_sample_accepted", 0)))
    imu_complete = any(as_bool(est.get("imu_calibration_complete", 0)) for est in estimators)
    imu_frozen = any(as_bool(est.get("imu_bias_frozen", 0)) for est in estimators)

    max_baro_nis = max(safe_float(est.get("normalized_baro_innovation", 0.0)) for est in estimators)
    max_gps_pos_nis = max(safe_float(est.get("normalized_gps_position_innovation", 0.0)) for est in estimators)
    max_gps_vel_nis = max(safe_float(est.get("normalized_gps_velocity_innovation", 0.0)) for est in estimators)

    sqrt_pos_max = [0.0, 0.0, 0.0]
    sqrt_vel_max = [0.0, 0.0, 0.0]
    for est in estimators:
        cov_pos = list(est.get("cov_pos") or []) + [0.0, 0.0, 0.0]
        cov_vel = list(est.get("cov_vel") or []) + [0.0, 0.0, 0.0]
        for axis in range(3):
            sqrt_pos_max[axis] = max(sqrt_pos_max[axis], math.sqrt(max(safe_float(cov_pos[axis]), 0.0)))
            sqrt_vel_max[axis] = max(sqrt_vel_max[axis], math.sqrt(max(safe_float(cov_vel[axis]), 0.0)))

    lines.append(
        f"Phase AHRS gyroOnly={percentage(gyro_only, len(estimators))} "
        f"accRej={percentage(accel_rejected, len(estimators))} "
        f"gpsOut={percentage(gps_outage, len(estimators))}"
    )
    lines.append(
        f"Phase IMU cal samples={imu_samples} accepted={percentage(imu_accepted, len(estimators))} "
        f"complete={bool_label(imu_complete)} frozen={bool_label(imu_frozen)}"
    )
    lines.append(
        f"Phase KF updates baro/gpsP/gpsV "
        f"{baro_updates}/{gps_pos_updates}/{gps_vel_updates}"
    )
    lines.append(
        f"Phase max NIS baro/gpsP/gpsV "
        f"{max_baro_nis:.2f}/{max_gps_pos_nis:.2f}/{max_gps_vel_nis:.2f}"
    )
    lines.append(
        f"Phase max sqrtP pos "
        f"{sqrt_pos_max[0]:.1f}/{sqrt_pos_max[1]:.1f}/{sqrt_pos_max[2]:.1f}m "
        f"velZ={sqrt_vel_max[2]:.2f}m/s"
    )

    vertical_filters = [
        packet.get("vertical_filter")
        for packet in packets
        if isinstance(packet.get("vertical_filter"), dict)
    ]
    if vertical_filters:
        vf_updates = sum(1 for vf in vertical_filters if as_bool(vf.get("baro_update_accepted", 0)))
        max_vf_nis = max(safe_float(vf.get("normalized_baro_innovation", 0.0)) for vf in vertical_filters)
        max_vf_alt_sqrt = max(
            math.sqrt(max(safe_float(vf.get("cov_altitude_m2", 0.0)), 0.0))
            for vf in vertical_filters
        )
        max_vf_vel_sqrt = max(
            math.sqrt(max(safe_float(vf.get("cov_velocity_m2ps2", 0.0)), 0.0))
            for vf in vertical_filters
        )
        lines.append(
            f"Phase 1D VF baro={vf_updates}/{len(vertical_filters)} "
            f"maxNIS={max_vf_nis:.2f} sqrtP alt/vel={max_vf_alt_sqrt:.1f}m/{max_vf_vel_sqrt:.2f}m/s"
        )
    return lines


def autoscale_2d(ax, x_values: list[float], y_sets: Iterable[Iterable[float]], default_span: float = 1.0) -> None:
    if not x_values:
        return

    ax.set_xlim(x_values[0], max(x_values[-1], x_values[0] + default_span))
    y_values = [value for values in y_sets for value in values]
    if not y_values:
        return

    y_min = min(y_values)
    y_max = max(y_values)
    if math.isclose(y_min, y_max, abs_tol=1e-6):
        y_min -= default_span
        y_max += default_span
    else:
        pad = 0.12 * (y_max - y_min)
        y_min -= pad
        y_max += pad
    ax.set_ylim(y_min, y_max)


def configure_axes(fig):
    grid = fig.add_gridspec(4, 3, height_ratios=(1.0, 1.0, 0.85, 1.1))
    ax_pose = fig.add_subplot(grid[0:3, 0], projection="3d")
    ax_metrics = fig.add_subplot(grid[3, :])
    ax_accel = fig.add_subplot(grid[0, 1])
    ax_gyro = fig.add_subplot(grid[1, 1])
    ax_imu_drift = fig.add_subplot(grid[2, 1:])
    ax_alt = fig.add_subplot(grid[0, 2])
    ax_vel = fig.add_subplot(grid[1, 2])

    ax_accel.set_title("Raw Accel")
    ax_accel.set_ylabel("m/s^2")
    ax_metrics.set_title("Init / GPS / Radio")
    ax_gyro.set_title("Raw Gyro")
    ax_gyro.set_ylabel("rad/s")
    ax_imu_drift.set_title("Startup IMU Bias Drift")
    ax_imu_drift.set_ylabel("accel m/s^2, gyro deg/s")
    ax_alt.set_title("Altitude")
    ax_alt.set_ylabel("m")
    ax_vel.set_title("Estimated Velocity")
    ax_vel.set_ylabel("m/s")

    for ax in (ax_accel, ax_gyro, ax_imu_drift, ax_alt, ax_vel):
        ax.set_xlabel("s")
        ax.grid(True, alpha=0.3)
    ax_metrics.axis("off")

    accel_lines = ax_accel.plot([], [], "r-", [], [], "g-", [], [], "b-")
    gyro_lines = ax_gyro.plot([], [], "r-", [], [], "g-", [], [], "b-")
    imu_accel_drift_lines = ax_imu_drift.plot([], [], "r-", [], [], "g-", [], [], "b-")
    imu_gyro_drift_lines = ax_imu_drift.plot([], [], "r--", [], [], "g--", [], [], "b--")
    alt_baro_line, = ax_alt.plot([], [], color="0.55", label="baro raw")
    alt_state_line, = ax_alt.plot([], [], color="tab:blue", label="deploy 1D")
    alt_3d_line, = ax_alt.plot([], [], color="tab:orange", linestyle="--", label="3D z")
    vel_lines = ax_vel.plot([], [], "r-", [], [], "g-", [], [], "b-")
    vel_vertical_line, = ax_vel.plot([], [], color="k", linestyle="--", label="deploy 1D")

    ax_accel.legend(["x", "y", "z"], loc="upper right")
    ax_gyro.legend(["x", "y", "z"], loc="upper right")
    ax_imu_drift.legend(
        ["acc x", "acc y", "acc z", "gyro x", "gyro y", "gyro z"],
        loc="upper right",
        ncol=2,
        fontsize=7,
    )
    ax_alt.legend(loc="upper right")
    ax_vel.legend(["vx", "vy", "vz", "deploy 1D"], loc="upper right")

    return {
        "pose": ax_pose,
        "metrics": ax_metrics,
        "accel": ax_accel,
        "gyro": ax_gyro,
        "imu_drift": ax_imu_drift,
        "alt": ax_alt,
        "vel": ax_vel,
        "accel_lines": accel_lines,
        "gyro_lines": gyro_lines,
        "imu_accel_drift_lines": imu_accel_drift_lines,
        "imu_gyro_drift_lines": imu_gyro_drift_lines,
        "alt_baro_line": alt_baro_line,
        "alt_state_line": alt_state_line,
        "alt_3d_line": alt_3d_line,
        "vel_lines": vel_lines,
        "vel_vertical_line": vel_vertical_line,
    }


def draw_pose(ax, series: Series, port: str, selected_phase: int | None) -> None:
    ax.clear()
    view_label = phase_label(selected_phase)
    ax.set_title("Estimated Pose")
    ax.set_xlabel("X m")
    ax.set_ylabel("Y m")
    ax.set_zlabel("Z m")

    if not series.last_packet:
        ax.text2D(0.05, 0.95, f"waiting for DBG packets on {port}", transform=ax.transAxes)
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(-1, 1)
        return

    packet = series.last_packet
    pos = tuple(float(v) for v in packet["pos"])
    quat = tuple(float(v) for v in packet["q"])
    phase = packet_phase_label(packet)

    if series.pos[0]:
        xs = list(series.pos[0])
        ys = list(series.pos[1])
        zs = list(series.pos[2])
        ax.plot(xs, ys, zs, color="0.35", linewidth=1.5, alpha=0.75)

    body_axes = (
        ((1.0, 0.0, 0.0), "r", "body X"),
        ((0.0, 1.0, 0.0), "g", "body Y"),
        ((0.0, 0.0, 1.0), "b", "body Z"),
    )
    scale = 1.0
    for axis, color, label in body_axes:
        vec = rotate_body_to_world(quat, axis)
        ax.quiver(
            pos[0],
            pos[1],
            pos[2],
            vec[0] * scale,
            vec[1] * scale,
            vec[2] * scale,
            color=color,
            linewidth=2.0,
            label=label,
        )

    ax.scatter([pos[0]], [pos[1]], [pos[2]], color="k", s=20)
    span_xy = max(2.0, abs(pos[0]) + 1.0, abs(pos[1]) + 1.0)
    z_values = list(series.pos[2]) if series.pos[2] else [pos[2]]
    z_min = min(min(z_values), pos[2]) - 1.0
    z_max = max(max(z_values), pos[2]) + 1.0

    ax.set_xlim(-span_xy, span_xy)
    ax.set_ylim(-span_xy, span_xy)
    ax.set_zlim(z_min, max(z_max, z_min + 2.0))
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass

    status = (
        f"requested={view_label} | current={phase} | packets={series.packet_count}\n"
        f"q=[{quat[0]:+.3f}, {quat[1]:+.3f}, {quat[2]:+.3f}, {quat[3]:+.3f}]\n"
        f"pos=[{pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f}] m"
    )
    ax.text2D(0.03, 0.97, status, transform=ax.transAxes, va="top")
    ax.legend(loc="lower left")


def draw_metrics(ax, series: Series, selected_phase: int | None) -> None:
    ax.clear()
    ax.set_title("Init / GPS / Radio / Estimator")
    ax.axis("off")

    if not series.last_packet:
        lines = [
            "waiting for debug telemetry",
            f"reader: {series.reader_status}",
            (
                f"raw={series.raw_line_count} dbg={series.dbg_line_count} "
                f"events={series.event_count} ignored={series.ignored_line_count} "
                f"json_errors={series.parse_errors}"
            ),
        ]
        if series.last_reader_error:
            lines.append(f"last error: {series.last_reader_error}")
        if series.last_line_preview:
            lines.append(f"last line: {series.last_line_preview}")
        ax.text(
            0.02,
            0.92,
            "\n".join(lines),
            transform=ax.transAxes,
            va="top",
            family="monospace",
            fontsize=7.2,
        )
        return

    packet = series.last_packet
    lines: list[str] = []
    lines.extend(selected_estimator_summary(series, selected_phase))
    lines.append(f"Command: {series.command_status}")
    lines.append("")

    system = packet.get("system")
    if isinstance(system, dict):
        lines.append(
            f"Init mutex={ok_label(system.get('mutexes_initialized', 0))} "
            f"queues={ok_label(system.get('queues_initialized', 0))} "
            f"debugQ={ok_label(system.get('debug_queue_initialized', 0))} "
            f"sensors={ok_label(system.get('sensors_initialized', 0))} "
            f"flight={ok_label(system.get('flight_logic_initialized', 0))}"
        )
        lines.append(
            f"Tasks sensor/flight/log/gps/radio/debug "
            f"{ok_label(system.get('sensor_task_started', 0))}/"
            f"{ok_label(system.get('flight_task_started', 0))}/"
            f"{ok_label(system.get('logger_task_started', 0))}/"
            f"{ok_label(system.get('gps_task_started', 0))}/"
            f"{ok_label(system.get('radio_task_started', 0))}/"
            f"{ok_label(system.get('debug_task_started', 0))}"
        )
        lines.append(
            f"GPS cfg={bool_label(system.get('gps_config_enabled', 0))} "
            f"init={ok_label(system.get('gps_initialized', 0))} "
            f"task={ok_label(system.get('gps_task_started', 0))}"
        )
        radio_skip = as_bool(system.get("radio_skipped_debug", 0))
        if radio_skip:
            lines.append("Radio enabled=no init=NO task=NO (debug mode skipped)")
        else:
            lines.append(
                f"Radio enabled={bool_label(system.get('radio_runtime_enabled', 0))} "
                f"init={ok_label(system.get('radio_initialized', 0))} "
                f"task={ok_label(system.get('radio_task_started', 0))}"
            )
        lines.append(
            f"SD cfg={bool_label(system.get('sd_config_enabled', 0))} "
            f"init={ok_label(system.get('sd_initialized', 0))} "
            f"logging={bool_label(system.get('sd_logging_enabled', 0))} "
            f"mag cfg={bool_label(system.get('magnetometer_config_enabled', 0))}"
        )
        if "debug_phase_override" in system:
            forced_phase = int(system.get("debug_forced_phase", 0))
            override_enabled = as_bool(system.get("debug_phase_override", 0))
            lines.append(
                f"Debug phase override={bool_label(override_enabled)} "
                f"forced={phase_label(forced_phase) if override_enabled else 'AUTO'}"
            )
    else:
        lines.append("Init status not present in debug packet")

    gps = packet.get("gps")
    if isinstance(gps, dict):
        gps_valid = bool_label(gps.get("valid", 0))
        position_valid = bool_label(gps.get("position_valid", gps.get("valid", 0)))
        altitude_valid = bool_label(gps.get("altitude_valid", 0))
        velocity_valid = bool_label(gps.get("velocity_valid", gps.get("vertical_velocity_valid", 0)))
        lines.append(
            f"GPS valid={gps_valid} pos={position_valid} vel={velocity_valid} fix={int(gps.get('fix_type', 0))} "
            f"sats={int(gps.get('satellites', 0))} age={format_age_s(packet, gps)}"
        )
        lines.append(
            f"lat/lon {float(gps.get('latitude_deg', 0.0)):+.7f}, "
            f"{float(gps.get('longitude_deg', 0.0)):+.7f}"
        )
        lines.append(
            f"alt {float(gps.get('altitude_m', 0.0)):+.1f}m ({altitude_valid})  "
            f"vZ {float(gps.get('vertical_velocity_mps', 0.0)):+.2f}m/s ({velocity_valid})"
        )
        if "horizontal_accuracy_m" in gps:
            lines.append(
                f"GPS acc h/v/s "
                f"{float(gps.get('horizontal_accuracy_m', 0.0)):.1f}m/"
                f"{float(gps.get('vertical_accuracy_m', 0.0)):.1f}m/"
                f"{float(gps.get('speed_accuracy_mps', 0.0)):.2f}m/s "
                f"pDOP={float(gps.get('p_dop', 0.0)):.2f}"
            )
    else:
        lines.append("GPS not present in debug packet")

    estimator = packet.get("estimator")
    if isinstance(estimator, dict):
        cov_pos = list(estimator.get("cov_pos") or []) + [0.0, 0.0, 0.0]
        cov_vel = list(estimator.get("cov_vel") or []) + [0.0, 0.0, 0.0]
        accel_bias = list(estimator.get("accel_bias_mss") or []) + [0.0, 0.0, 0.0]
        gyro_bias = list(estimator.get("gyro_bias_rads") or []) + [0.0, 0.0, 0.0]
        body_up = list(estimator.get("body_up") or [0.0, 0.0, 1.0])
        lines.append(
            f"AHRS init={ok_label(estimator.get('ahrs_initialized', 0))} "
            f"gyroOnly={bool_label(estimator.get('ahrs_gyro_only', 0))} "
            f"mag={bool_label(estimator.get('ahrs_using_mag', 0))} "
            f"accRej={bool_label(estimator.get('ahrs_accel_rejected', 0))}"
        )
        lines.append(
            f"IMU cal active={bool_label(estimator.get('imu_calibration_active', 0))} "
            f"accepted={bool_label(estimator.get('imu_calibration_sample_accepted', 0))} "
            f"complete={bool_label(estimator.get('imu_calibration_complete', 0))} "
            f"frozen={bool_label(estimator.get('imu_bias_frozen', 0))} "
            f"applied={bool_label(estimator.get('imu_bias_applied', 0))} "
            f"n={int(estimator.get('imu_calibration_samples', 0))}"
        )
        lines.append(f"AHRS body up {body_up_label(body_up)}")
        lines.append(
            f"IMU bias acc "
            f"{safe_float(accel_bias[0]):+.3f}/{safe_float(accel_bias[1]):+.3f}/{safe_float(accel_bias[2]):+.3f}m/s^2 "
            f"gyro "
            f"{math.degrees(safe_float(gyro_bias[0])):+.3f}/"
            f"{math.degrees(safe_float(gyro_bias[1])):+.3f}/"
            f"{math.degrees(safe_float(gyro_bias[2])):+.3f}deg/s"
        )
        lines.append(
            f"KF upd baro/gpsP/gpsV "
            f"{ok_label(estimator.get('baro_update_accepted', 0))}/"
            f"{ok_label(estimator.get('gps_position_update_accepted', 0))}/"
            f"{ok_label(estimator.get('gps_velocity_update_accepted', 0))} "
            f"gpsOut={bool_label(estimator.get('gps_outage', 0))}"
        )
        lines.append(
            f"KF sqrtP pos "
            f"{math.sqrt(max(float(cov_pos[0]), 0.0)):.1f}/"
            f"{math.sqrt(max(float(cov_pos[1]), 0.0)):.1f}/"
            f"{math.sqrt(max(float(cov_pos[2]), 0.0)):.1f}m "
            f"velZ={math.sqrt(max(float(cov_vel[2]), 0.0)):.2f}m/s"
        )
        lines.append(
            f"KF NIS baro/gpsP/gpsV "
            f"{float(estimator.get('normalized_baro_innovation', 0.0)):.2f}/"
            f"{float(estimator.get('normalized_gps_position_innovation', 0.0)):.2f}/"
            f"{float(estimator.get('normalized_gps_velocity_innovation', 0.0)):.2f}"
        )

    vertical_filter = packet.get("vertical_filter")
    if isinstance(vertical_filter, dict):
        vf_alt_sqrt = math.sqrt(max(safe_float(vertical_filter.get("cov_altitude_m2", 0.0)), 0.0))
        vf_vel_sqrt = math.sqrt(max(safe_float(vertical_filter.get("cov_velocity_m2ps2", 0.0)), 0.0))
        lines.append(
            f"1D VF init={ok_label(vertical_filter.get('initialized', 0))} "
            f"baro={ok_label(vertical_filter.get('baro_update_accepted', 0))} "
            f"alt={safe_float(vertical_filter.get('altitude_m', 0.0)):+.2f}m "
            f"vel={safe_float(vertical_filter.get('velocity_mps', 0.0)):+.2f}m/s"
        )
        lines.append(
            f"1D VF acc={safe_float(vertical_filter.get('vertical_accel_mss', 0.0)):+.2f}m/s^2 "
            f"bias={safe_float(vertical_filter.get('accel_bias_mss', 0.0)):+.3f}m/s^2 "
            f"sqrtP={vf_alt_sqrt:.1f}m/{vf_vel_sqrt:.2f}m/s "
            f"NIS={safe_float(vertical_filter.get('normalized_baro_innovation', 0.0)):.2f}"
        )

    radio = packet.get("radio")
    if isinstance(radio, dict):
        radio_enabled = as_bool(radio.get("enabled", 0))
        if radio_enabled:
            lines.append(
                f"Radio enabled=yes init={bool_label(radio.get('initialized', 0))} "
                f"busy={bool_label(radio.get('busy', 0))} tx={series.radio_tx_hz():.2f}Hz"
            )
        else:
            lines.append("Radio enabled=no (debug build skips radio TX)")

        lines.append(
            f"tx start/done/fail "
            f"{int(radio.get('tx_start_count', 0))}/"
            f"{int(radio.get('tx_done_count', 0))}/"
            f"{int(radio.get('tx_fail_count', 0))}"
        )
        lines.append(
            f"last payload {int(radio.get('last_payload_len', 0))}B  "
            f"last tx {int(radio.get('last_tx_duration_ms', 0))}ms"
        )
    else:
        lines.append("Radio not present in debug packet")

    lines.append(f"Debug RX {series.debug_packet_hz():.1f}Hz  packets={series.packet_count}  parse_errors={series.parse_errors}")
    lines.append(
        f"Reader raw/dbg/ignored {series.raw_line_count}/"
        f"{series.dbg_line_count}/{series.ignored_line_count}"
    )
    if series.last_reader_error:
        lines.append(f"Reader error: {series.last_reader_error}")

    midpoint = (len(lines) + 1) // 2
    text_style = {
        "transform": ax.transAxes,
        "va": "top",
        "family": "monospace",
        "fontsize": 7.0,
    }
    ax.text(0.01, 0.92, "\n".join(lines[:midpoint]), **text_style)
    ax.text(0.50, 0.92, "\n".join(lines[midpoint:]), **text_style)


def main() -> int:
    parser = argparse.ArgumentParser(description="Flight computer debug dashboard")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem123456")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=float, default=20.0, help="plot history window in seconds")
    parser.add_argument("--expected-hz", type=int, default=20)
    args = parser.parse_args()

    port = args.port or guess_serial_port()
    if not port:
        print("No serial port found. Pass --port explicitly.")
        return 2

    maxlen = max(10, int(args.window * args.expected_hz))
    series = Series(maxlen=maxlen)
    packets: queue.Queue[dict] = queue.Queue()
    commands: queue.Queue[str] = queue.Queue()
    stop = threading.Event()

    thread = threading.Thread(
        target=reader_thread,
        args=(port, args.baud, packets, commands, stop, series),
        daemon=True,
    )
    thread.start()

    fig = plt.figure(figsize=(14, 10))
    fig.canvas.manager.set_window_title("Flight Computer Debug Dashboard")
    axes = configure_axes(fig)
    selected_phase_index = 0
    user_selected_phase = False
    state_indicator = fig.text(
        0.5,
        0.965,
        "",
        ha="center",
        va="center",
        fontsize=10.5,
        fontweight="bold",
    )
    prev_button_ax = fig.add_axes((0.430, 0.892, 0.045, 0.030))
    next_button_ax = fig.add_axes((0.525, 0.892, 0.045, 0.030))
    prev_button_ax.set_in_layout(False)
    next_button_ax.set_in_layout(False)
    prev_button = Button(prev_button_ax, "<")
    next_button = Button(next_button_ax, ">")
    prev_button.label.set_fontsize(8)
    next_button.label.set_fontsize(8)

    def current_selected_phase() -> int:
        return PHASE_VIEW_ORDER[selected_phase_index]

    def update_state_indicator() -> None:
        selected_phase = current_selected_phase()
        selected_label = phase_label(selected_phase)
        selected_count = len(selected_indices(series, selected_phase))
        total_count = len(series.time_s)
        live_label = packet_phase_label(series.last_packet)
        state_indicator.set_text(
            f"Requested phase: {selected_label} ({selected_count}/{total_count} samples) | Current phase: {live_label}"
        )

    def send_phase_command(phase: int | None) -> None:
        command = "CMD,PHASE,AUTO" if phase is None else f"CMD,PHASE,{phase}"
        while True:
            try:
                commands.get_nowait()
            except queue.Empty:
                break
        commands.put(command)
        series.command_status = f"queued {command}"

    def cycle_phase(delta: int) -> None:
        nonlocal selected_phase_index, user_selected_phase
        user_selected_phase = True
        selected_phase_index = (selected_phase_index + delta) % len(PHASE_VIEW_ORDER)
        send_phase_command(current_selected_phase())
        update_state_indicator()
        fig.canvas.draw_idle()

    prev_button.on_clicked(lambda _event: cycle_phase(-1))
    next_button.on_clicked(lambda _event: cycle_phase(1))

    def handle_key_press(event) -> None:
        nonlocal selected_phase_index, user_selected_phase
        if event.key in ("left", "a"):
            cycle_phase(-1)
        elif event.key in ("right", "d"):
            cycle_phase(1)
        elif event.key in ("0", "1", "2", "3", "4"):
            user_selected_phase = True
            selected_phase_index = phase_index_for(int(event.key))
            send_phase_command(current_selected_phase())
            update_state_indicator()
            fig.canvas.draw_idle()
        elif event.key in ("escape", "backspace"):
            user_selected_phase = False
            send_phase_command(None)
            update_state_indicator()
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", handle_key_press)
    update_state_indicator()

    def update(_frame):
        nonlocal selected_phase_index
        while True:
            try:
                packet = packets.get_nowait()
            except queue.Empty:
                break
            try:
                series.add(packet)
            except (KeyError, TypeError, ValueError):
                series.parse_errors += 1

        if not user_selected_phase and series.last_packet is not None:
            try:
                selected_phase_index = phase_index_for(int(series.last_packet.get("phase", 0)))
            except (TypeError, ValueError):
                selected_phase_index = 0

        selected_phase = current_selected_phase()
        x_values = relative_time(series.time_s)
        accel_values = [list(values) for values in series.accel]
        gyro_values = [list(values) for values in series.gyro]
        accel_drift_values = [list(values) for values in series.accel_bias_drift]
        gyro_drift_values = [list(values) for values in series.gyro_bias_drift]
        vel_values = [list(values) for values in series.vel]
        baro_alt_values = list(series.baro_alt_m)
        vertical_alt_values = list(series.vertical_alt_m)
        vertical_vel_values = list(series.vertical_vel_mps)
        pos_z_values = list(series.pos[2])

        set_line_data(axes["accel_lines"], x_values, accel_values)
        set_line_data(axes["gyro_lines"], x_values, gyro_values)
        set_line_data(axes["imu_accel_drift_lines"], x_values, accel_drift_values)
        set_line_data(axes["imu_gyro_drift_lines"], x_values, gyro_drift_values)
        set_line_data(axes["vel_lines"], x_values, vel_values)
        axes["alt_baro_line"].set_data(x_values, baro_alt_values)
        axes["alt_state_line"].set_data(x_values, vertical_alt_values)
        axes["alt_3d_line"].set_data(x_values, pos_z_values)
        axes["vel_vertical_line"].set_data(x_values, vertical_vel_values)

        autoscale_2d(axes["accel"], x_values, accel_values)
        autoscale_2d(axes["gyro"], x_values, gyro_values)
        autoscale_2d(axes["imu_drift"], x_values, (*accel_drift_values, *gyro_drift_values), default_span=0.05)
        autoscale_2d(axes["alt"], x_values, (baro_alt_values, vertical_alt_values, pos_z_values))
        autoscale_2d(axes["vel"], x_values, (*vel_values, vertical_vel_values))
        draw_pose(axes["pose"], series, port, selected_phase)
        draw_metrics(axes["metrics"], series, selected_phase)
        update_state_indicator()

        return []

    ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    try:
        plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.86))
        plt.show()
    finally:
        stop.set()
        ani.event_source.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
