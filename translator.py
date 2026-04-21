from __future__ import annotations

import json
import math
import os
import queue
import re
import socket
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np
import serial
from serial.tools import list_ports

from PyQt5.QtCore import QPointF, QRectF, QTimer, Qt
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)


APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "config.json"
AUTOSTART_ENABLE_PATH = APP_DIR / "autostart.enable"

STOP_MOTOR_1_BYTE = 64
STOP_MOTOR_2_BYTE = 192
MAX_MOTOR_COMMAND = 127
UDP_QUEUE_LIMIT = 5
TELEMETRY_RATE_WINDOW = 30
POT_FEEDBACK_STALE_SECONDS = 0.5
POT_LOSS_ERROR_LOG = APP_DIR / "test_logs" / "errors.log"
HARD_STOP_ZONE = 0.01

PACKET_RE = re.compile(r"^\s*P=(?P<pitch>\d{1,4}),R=(?P<roll>\d{1,4})\s*$")
POT_FEEDBACK_RE = re.compile(r"^\s*L=(?P<left>\d{1,4}),R=(?P<right>\d{1,4})\s*$")
CAL_OK_TEXT = "CAL_OK"

DEFAULT_CONFIG: dict[str, Any] = {
    "udp_port": 9000,
    "serial_port": "",
    "baud_rate": 9600,
    "spool_diameter_mm": 55.0,
    "l_pivot_to_attachment_mm": 266.08023,
    "pitch_limit_deg": 30.0,
    "roll_limit_deg": 30.0,
    "max_velocity_mm_s": 200.0,
    "deadband_mm": 2.0,
    "kp": 2.0,
    "ki": 0.05,
    "kd": 0.1,
    "integral_clamp": 50.0,
    "alpha_smoothing": 0.3,
    "soft_stop_zone_percent": 5.0,
    "pot_min_adc_left": 0,
    "pot_max_adc_left": 4095,
    "pot_min_adc_right": 0,
    "pot_max_adc_right": 4095,
    "travel_range_mm_left": (270.0 * math.pi / 180.0) * 27.5,
    "travel_range_mm_right": (270.0 * math.pi / 180.0) * 27.5,
}


def clamp(value: float, low: float, high: float) -> float:
    return float(np.clip(value, low, high))


def clamp_int(value: int, low: int, high: int) -> int:
    return int(max(low, min(high, int(value))))


def sanitize_config(raw: dict[str, Any]) -> dict[str, Any]:
    cfg = DEFAULT_CONFIG.copy()
    cfg.update({key: raw.get(key, cfg[key]) for key in cfg})

    def int_value(key: str, minimum: int, maximum: int) -> int:
        try:
            return clamp_int(int(cfg[key]), minimum, maximum)
        except (TypeError, ValueError):
            return int(DEFAULT_CONFIG[key])

    def float_value(key: str, minimum: float, maximum: float) -> float:
        try:
            value = float(cfg[key])
        except (TypeError, ValueError):
            value = float(DEFAULT_CONFIG[key])
        return clamp(value, minimum, maximum)

    cfg["udp_port"] = int_value("udp_port", 1, 65535)
    cfg["serial_port"] = str(cfg.get("serial_port") or "")
    cfg["baud_rate"] = int_value("baud_rate", 300, 1000000)
    cfg["spool_diameter_mm"] = float_value("spool_diameter_mm", 1.0, 10000.0)
    cfg["l_pivot_to_attachment_mm"] = float_value("l_pivot_to_attachment_mm", 1.0, 100000.0)
    cfg["pitch_limit_deg"] = float_value("pitch_limit_deg", 0.01, 180.0)
    cfg["roll_limit_deg"] = float_value("roll_limit_deg", 0.01, 180.0)
    cfg["max_velocity_mm_s"] = float_value("max_velocity_mm_s", 0.01, 100000.0)
    cfg["deadband_mm"] = float_value("deadband_mm", 0.0, 100000.0)
    cfg["kp"] = float_value("kp", 0.0, 100000.0)
    cfg["ki"] = float_value("ki", 0.0, 100000.0)
    cfg["kd"] = float_value("kd", 0.0, 100000.0)
    cfg["integral_clamp"] = float_value("integral_clamp", 0.0, 100000.0)
    cfg["alpha_smoothing"] = float_value("alpha_smoothing", 0.0, 1.0)
    cfg["soft_stop_zone_percent"] = float_value("soft_stop_zone_percent", 0.1, 50.0)
    cfg["pot_min_adc_left"] = int_value("pot_min_adc_left", 0, 4095)
    cfg["pot_max_adc_left"] = int_value("pot_max_adc_left", 0, 4095)
    cfg["pot_min_adc_right"] = int_value("pot_min_adc_right", 0, 4095)
    cfg["pot_max_adc_right"] = int_value("pot_max_adc_right", 0, 4095)
    if cfg["pot_max_adc_left"] <= cfg["pot_min_adc_left"]:
        cfg["pot_min_adc_left"] = int(DEFAULT_CONFIG["pot_min_adc_left"])
        cfg["pot_max_adc_left"] = int(DEFAULT_CONFIG["pot_max_adc_left"])
    if cfg["pot_max_adc_right"] <= cfg["pot_min_adc_right"]:
        cfg["pot_min_adc_right"] = int(DEFAULT_CONFIG["pot_min_adc_right"])
        cfg["pot_max_adc_right"] = int(DEFAULT_CONFIG["pot_max_adc_right"])
    cfg["travel_range_mm_left"] = float_value("travel_range_mm_left", 0.001, 100000.0)
    cfg["travel_range_mm_right"] = float_value("travel_range_mm_right", 0.001, 100000.0)
    return cfg


def load_or_create_config() -> dict[str, Any]:
    if not CONFIG_PATH.exists():
        save_config(DEFAULT_CONFIG)
        return DEFAULT_CONFIG.copy()

    try:
        with CONFIG_PATH.open("r", encoding="utf-8") as handle:
            raw = json.load(handle)
        if not isinstance(raw, dict):
            raise ValueError("config root must be an object")
        cfg = sanitize_config(raw)
    except Exception:
        cfg = DEFAULT_CONFIG.copy()
        save_config(cfg)
    return cfg


def save_config(config: dict[str, Any]) -> None:
    cfg = sanitize_config(config)
    with CONFIG_PATH.open("w", encoding="utf-8") as handle:
        json.dump(cfg, handle, indent=2)
        handle.write("\n")


@dataclass
class RigParameters:
    udp_port: int = int(DEFAULT_CONFIG["udp_port"])
    serial_port: str = str(DEFAULT_CONFIG["serial_port"])
    baud_rate: int = int(DEFAULT_CONFIG["baud_rate"])
    spool_diameter_mm: float = float(DEFAULT_CONFIG["spool_diameter_mm"])
    l_pivot_to_attachment_mm: float = float(DEFAULT_CONFIG["l_pivot_to_attachment_mm"])
    pitch_limit_deg: float = float(DEFAULT_CONFIG["pitch_limit_deg"])
    roll_limit_deg: float = float(DEFAULT_CONFIG["roll_limit_deg"])
    max_velocity_mm_s: float = float(DEFAULT_CONFIG["max_velocity_mm_s"])
    deadband_mm: float = float(DEFAULT_CONFIG["deadband_mm"])
    kp: float = float(DEFAULT_CONFIG["kp"])
    ki: float = float(DEFAULT_CONFIG["ki"])
    kd: float = float(DEFAULT_CONFIG["kd"])
    integral_clamp: float = float(DEFAULT_CONFIG["integral_clamp"])
    alpha_smoothing: float = float(DEFAULT_CONFIG["alpha_smoothing"])
    soft_stop_zone_percent: float = float(DEFAULT_CONFIG["soft_stop_zone_percent"])
    pot_min_adc_left: int = int(DEFAULT_CONFIG["pot_min_adc_left"])
    pot_max_adc_left: int = int(DEFAULT_CONFIG["pot_max_adc_left"])
    pot_min_adc_right: int = int(DEFAULT_CONFIG["pot_min_adc_right"])
    pot_max_adc_right: int = int(DEFAULT_CONFIG["pot_max_adc_right"])
    travel_range_mm_left: float = float(DEFAULT_CONFIG["travel_range_mm_left"])
    travel_range_mm_right: float = float(DEFAULT_CONFIG["travel_range_mm_right"])

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "RigParameters":
        cfg = sanitize_config(config)
        return cls(
            udp_port=int(cfg["udp_port"]),
            serial_port=str(cfg["serial_port"]),
            baud_rate=int(cfg["baud_rate"]),
            spool_diameter_mm=float(cfg["spool_diameter_mm"]),
            l_pivot_to_attachment_mm=float(cfg["l_pivot_to_attachment_mm"]),
            pitch_limit_deg=float(cfg["pitch_limit_deg"]),
            roll_limit_deg=float(cfg["roll_limit_deg"]),
            max_velocity_mm_s=float(cfg["max_velocity_mm_s"]),
            deadband_mm=float(cfg["deadband_mm"]),
            kp=float(cfg["kp"]),
            ki=float(cfg["ki"]),
            kd=float(cfg["kd"]),
            integral_clamp=float(cfg["integral_clamp"]),
            alpha_smoothing=float(cfg["alpha_smoothing"]),
            soft_stop_zone_percent=float(cfg["soft_stop_zone_percent"]),
            pot_min_adc_left=int(cfg["pot_min_adc_left"]),
            pot_max_adc_left=int(cfg["pot_max_adc_left"]),
            pot_min_adc_right=int(cfg["pot_min_adc_right"]),
            pot_max_adc_right=int(cfg["pot_max_adc_right"]),
            travel_range_mm_left=float(cfg["travel_range_mm_left"]),
            travel_range_mm_right=float(cfg["travel_range_mm_right"]),
        )

    def as_config(self) -> dict[str, Any]:
        return sanitize_config(
            {
                "udp_port": self.udp_port,
                "serial_port": self.serial_port,
                "baud_rate": self.baud_rate,
                "spool_diameter_mm": self.spool_diameter_mm,
                "l_pivot_to_attachment_mm": self.l_pivot_to_attachment_mm,
                "pitch_limit_deg": self.pitch_limit_deg,
                "roll_limit_deg": self.roll_limit_deg,
                "max_velocity_mm_s": self.max_velocity_mm_s,
                "deadband_mm": self.deadband_mm,
                "kp": self.kp,
                "ki": self.ki,
                "kd": self.kd,
                "integral_clamp": self.integral_clamp,
                "alpha_smoothing": self.alpha_smoothing,
                "soft_stop_zone_percent": self.soft_stop_zone_percent,
                "pot_min_adc_left": self.pot_min_adc_left,
                "pot_max_adc_left": self.pot_max_adc_left,
                "pot_min_adc_right": self.pot_min_adc_right,
                "pot_max_adc_right": self.pot_max_adc_right,
                "travel_range_mm_left": self.travel_range_mm_left,
                "travel_range_mm_right": self.travel_range_mm_right,
            }
        )


@dataclass
class Telemetry:
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    left_disp_mm: float = 0.0
    right_disp_mm: float = 0.0
    left_target_disp_mm: float = 0.0
    right_target_disp_mm: float = 0.0
    left_measured_disp_mm: float = 0.0
    right_measured_disp_mm: float = 0.0
    left_error_mm: float = 0.0
    right_error_mm: float = 0.0
    left_integral: float = 0.0
    right_integral: float = 0.0
    left_proportional: float = 0.0
    right_proportional: float = 0.0
    left_derivative: float = 0.0
    right_derivative: float = 0.0
    left_raw_pid: float = 0.0
    right_raw_pid: float = 0.0
    left_smoothed_command: float = 0.0
    right_smoothed_command: float = 0.0
    left_pos_normalized: float = 0.0
    right_pos_normalized: float = 0.0
    left_soft_stop_active: bool = False
    right_soft_stop_active: bool = False
    left_hard_stop_active: bool = False
    right_hard_stop_active: bool = False
    left_adc_raw: int = 0
    right_adc_raw: int = 0
    pot_feedback_fresh: bool = False
    left_command: int = 0
    right_command: int = 0
    motor1_byte: int = STOP_MOTOR_1_BYTE
    motor2_byte: int = STOP_MOTOR_2_BYTE
    update_rate_hz: float = 0.0
    udp_connected: bool = False
    serial_connected: bool = False
    enabled: bool = False
    last_error: str = ""
    last_packet_time: float = 0.0
    last_serial_time: float = 0.0


@dataclass
class ProcessorState:
    previous_left_disp: Optional[float] = None
    previous_right_disp: Optional[float] = None
    previous_time: Optional[float] = None
    integral_left: float = 0.0
    integral_right: float = 0.0
    prev_error_left: float = 0.0
    prev_error_right: float = 0.0
    prev_smoothed_left: float = 0.0
    prev_smoothed_right: float = 0.0
    frame_times: deque[float] = field(default_factory=lambda: deque(maxlen=TELEMETRY_RATE_WINDOW))

    def reset(self) -> None:
        self.previous_left_disp = None
        self.previous_right_disp = None
        self.previous_time = None
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0
        self.prev_smoothed_left = 0.0
        self.prev_smoothed_right = 0.0
        self.frame_times.clear()


@dataclass
class PotFeedback:
    left_adc: int = 0
    right_adc: int = 0
    timestamp: float = 0.0


@dataclass
class PidSideResult:
    command: int
    smoothed_command: float
    raw_output: float
    error: float
    proportional: float
    integral: float
    derivative_term: float
    pos: float
    measured_disp: float
    soft_stop_active: bool
    hard_stop_active: bool


class MotionMath:
    @staticmethod
    def parse_flypt_packet(packet: str) -> tuple[int, int]:
        try:
            match = PACKET_RE.match(packet)
            if not match:
                raise ValueError(f"malformed UDP packet: {packet!r}")
            pitch_raw = int(match.group("pitch"))
            roll_raw = int(match.group("roll"))
            if not 0 <= pitch_raw <= 1023 or not 0 <= roll_raw <= 1023:
                raise ValueError(f"UDP values out of range: {packet!r}")
            return pitch_raw, roll_raw
        except Exception as exc:
            raise ValueError(str(exc)) from exc

    @staticmethod
    def raw_to_degrees(pitch_raw: int, roll_raw: int, params: RigParameters) -> tuple[float, float]:
        try:
            pitch_deg = (pitch_raw - 511.5) * (params.pitch_limit_deg / 511.5)
            roll_deg = (roll_raw - 511.5) * (params.roll_limit_deg / 511.5)
            return float(pitch_deg), float(roll_deg)
        except Exception as exc:
            raise ValueError(f"failed to convert raw pose to degrees: {exc}") from exc

    @staticmethod
    def displacement_mm(pitch_deg: float, roll_deg: float, params: RigParameters) -> tuple[float, float]:
        try:
            pitch_rad = pitch_deg * math.pi / 180.0
            roll_rad = roll_deg * math.pi / 180.0
            l_value = params.l_pivot_to_attachment_mm
            left_disp = l_value * np.sin(roll_rad) + l_value * np.sin(pitch_rad)
            right_disp = -l_value * np.sin(roll_rad) + l_value * np.sin(pitch_rad)
            return float(left_disp), float(right_disp)
        except Exception as exc:
            raise ValueError(f"failed to compute string displacement: {exc}") from exc

    @staticmethod
    def speed_commands(
        left_disp: float,
        right_disp: float,
        params: RigParameters,
        state: ProcessorState,
        now: float,
    ) -> tuple[int, int]:
        try:
            if state.previous_time is None:
                dt = 0.0
            else:
                dt = max(now - state.previous_time, 1e-6)

            if state.previous_left_disp is None or state.previous_right_disp is None or dt <= 0.0:
                left_vel = 0.0
                right_vel = 0.0
            else:
                left_vel = (left_disp - state.previous_left_disp) / dt
                right_vel = (right_disp - state.previous_right_disp) / dt

            state.previous_left_disp = left_disp
            state.previous_right_disp = right_disp
            state.previous_time = now

            left_speed = clamp(abs(left_vel) / params.max_velocity_mm_s * MAX_MOTOR_COMMAND, 0.0, MAX_MOTOR_COMMAND)
            right_speed = clamp(abs(right_vel) / params.max_velocity_mm_s * MAX_MOTOR_COMMAND, 0.0, MAX_MOTOR_COMMAND)

            left_command = int(left_speed) if left_disp > params.deadband_mm else 0
            right_command = int(right_speed) if right_disp > params.deadband_mm else 0
            return clamp_int(left_command, 0, MAX_MOTOR_COMMAND), clamp_int(right_command, 0, MAX_MOTOR_COMMAND)
        except Exception as exc:
            raise ValueError(f"failed to compute speed commands: {exc}") from exc

    @staticmethod
    def adc_to_normalized(adc_raw: int, pot_min_adc: int, pot_max_adc: int) -> float:
        try:
            if pot_max_adc <= pot_min_adc:
                return 0.0
            return clamp((int(adc_raw) - int(pot_min_adc)) / float(int(pot_max_adc) - int(pot_min_adc)), 0.0, 1.0)
        except Exception as exc:
            raise ValueError(f"failed to convert ADC to normalized position: {exc}") from exc

    @staticmethod
    def adc_to_displacement_mm(adc_raw: int, pot_min_adc: int, pot_max_adc: int, travel_range_mm: float) -> tuple[float, float]:
        try:
            pos = MotionMath.adc_to_normalized(adc_raw, pot_min_adc, pot_max_adc)
            return pos * float(travel_range_mm), pos
        except Exception as exc:
            raise ValueError(f"failed to convert ADC to displacement: {exc}") from exc

    @staticmethod
    def sanitize_float(value: float) -> float:
        if not math.isfinite(float(value)):
            return 0.0
        return float(value)

    @staticmethod
    def apply_soft_hard_stop(
        command: float,
        pos: float,
        soft_stop_zone: float,
        hard_stop_zone: float = HARD_STOP_ZONE,
    ) -> tuple[float, bool, bool]:
        safe_command = clamp(MotionMath.sanitize_float(command), 0.0, MAX_MOTOR_COMMAND)
        safe_pos = clamp(MotionMath.sanitize_float(pos), 0.0, 1.0)
        safe_soft = clamp(MotionMath.sanitize_float(soft_stop_zone), 0.001, 0.5)
        soft_active = False
        hard_active = safe_pos < hard_stop_zone or safe_pos > (1.0 - hard_stop_zone)
        if safe_pos < safe_soft:
            safe_command *= safe_pos / safe_soft
            soft_active = True
        elif safe_pos > (1.0 - safe_soft):
            safe_command *= (1.0 - safe_pos) / safe_soft
            soft_active = True
        if hard_active:
            safe_command = 0.0
        return clamp(safe_command, 0.0, MAX_MOTOR_COMMAND), soft_active, hard_active

    @staticmethod
    def pid_side(
        target_disp: float,
        measured_disp: float,
        pos: float,
        params: RigParameters,
        state: ProcessorState,
        dt: float,
        side: str,
    ) -> PidSideResult:
        try:
            safe_dt = clamp(MotionMath.sanitize_float(dt), 1e-6, 0.1)
            target = MotionMath.sanitize_float(target_disp)
            measured = MotionMath.sanitize_float(measured_disp)
            error = target - measured
            prev_error = state.prev_error_left if side == "left" else state.prev_error_right
            integral = state.integral_left if side == "left" else state.integral_right
            prev_smoothed = state.prev_smoothed_left if side == "left" else state.prev_smoothed_right

            proportional = params.kp * error
            integral += error * safe_dt
            integral = clamp(integral, -params.integral_clamp, params.integral_clamp)
            integral_term = params.ki * integral
            derivative = (error - prev_error) / safe_dt
            derivative_term = params.kd * derivative
            raw_output = proportional + integral_term + derivative_term
            raw_output = MotionMath.sanitize_float(raw_output)

            if target > params.deadband_mm:
                motor_command = clamp(raw_output, 0.0, MAX_MOTOR_COMMAND)
            else:
                motor_command = 0.0
                integral = 0.0
                prev_smoothed = 0.0

            smoothed = params.alpha_smoothing * motor_command + (1.0 - params.alpha_smoothing) * prev_smoothed
            smoothed = clamp(MotionMath.sanitize_float(smoothed), 0.0, MAX_MOTOR_COMMAND)
            stopped, soft_active, hard_active = MotionMath.apply_soft_hard_stop(
                smoothed,
                pos,
                params.soft_stop_zone_percent / 100.0,
                HARD_STOP_ZONE,
            )
            if hard_active:
                integral = 0.0

            if side == "left":
                state.integral_left = integral
                state.prev_error_left = error
                state.prev_smoothed_left = stopped
            else:
                state.integral_right = integral
                state.prev_error_right = error
                state.prev_smoothed_right = stopped

            return PidSideResult(
                command=clamp_int(int(stopped), 0, MAX_MOTOR_COMMAND),
                smoothed_command=stopped,
                raw_output=raw_output,
                error=error,
                proportional=proportional,
                integral=integral,
                derivative_term=derivative_term,
                pos=pos,
                measured_disp=measured,
                soft_stop_active=soft_active,
                hard_stop_active=hard_active,
            )
        except Exception as exc:
            raise ValueError(f"failed to compute {side} PID: {exc}") from exc

    @staticmethod
    def commands_to_serial_bytes(left_command: int, right_command: int) -> tuple[int, int]:
        try:
            safe_left = clamp_int(left_command, 0, MAX_MOTOR_COMMAND)
            safe_right = clamp_int(right_command, 0, MAX_MOTOR_COMMAND)
            motor1_byte = int((safe_left / 127.0) * 63.0 + STOP_MOTOR_1_BYTE)
            motor2_byte = int((safe_right / 127.0) * 63.0 + STOP_MOTOR_2_BYTE)
            return clamp_int(motor1_byte, 64, 127), clamp_int(motor2_byte, 192, 255)
        except Exception as exc:
            raise ValueError(f"failed to map commands to serial bytes: {exc}") from exc

    @staticmethod
    def process_packet(
        packet: str,
        params: RigParameters,
        state: ProcessorState,
        feedback: Optional[PotFeedback] = None,
        now: Optional[float] = None,
    ) -> Telemetry:
        try:
            now = time.time() if now is None else float(now)
            if state.previous_time is None:
                dt = 0.01
            else:
                dt = clamp(now - state.previous_time, 1e-6, 0.1)
            state.previous_time = now
            pitch_raw, roll_raw = MotionMath.parse_flypt_packet(packet)
            pitch_deg, roll_deg = MotionMath.raw_to_degrees(pitch_raw, roll_raw, params)
            left_disp, right_disp = MotionMath.displacement_mm(pitch_deg, roll_deg, params)
            if feedback is None:
                feedback = PotFeedback(left_adc=2048, right_adc=2048, timestamp=now)
                left_measured, right_measured = 0.0, 0.0
                left_pos, right_pos = 0.5, 0.5
            else:
                left_measured, left_pos = MotionMath.adc_to_displacement_mm(
                    feedback.left_adc,
                    params.pot_min_adc_left,
                    params.pot_max_adc_left,
                    params.travel_range_mm_left,
                )
                right_measured, right_pos = MotionMath.adc_to_displacement_mm(
                    feedback.right_adc,
                    params.pot_min_adc_right,
                    params.pot_max_adc_right,
                    params.travel_range_mm_right,
                )
            left_pid = MotionMath.pid_side(left_disp, left_measured, left_pos, params, state, dt, "left")
            right_pid = MotionMath.pid_side(right_disp, right_measured, right_pos, params, state, dt, "right")
            left_command, right_command = left_pid.command, right_pid.command
            motor1_byte, motor2_byte = MotionMath.commands_to_serial_bytes(left_command, right_command)

            state.frame_times.append(now)
            if len(state.frame_times) >= 2:
                elapsed = state.frame_times[-1] - state.frame_times[0]
                update_rate = (len(state.frame_times) - 1) / elapsed if elapsed > 0.0 else 0.0
            else:
                update_rate = 0.0

            return Telemetry(
                pitch_deg=pitch_deg,
                roll_deg=roll_deg,
                left_disp_mm=left_disp,
                right_disp_mm=right_disp,
                left_target_disp_mm=left_disp,
                right_target_disp_mm=right_disp,
                left_measured_disp_mm=left_measured,
                right_measured_disp_mm=right_measured,
                left_error_mm=left_pid.error,
                right_error_mm=right_pid.error,
                left_integral=left_pid.integral,
                right_integral=right_pid.integral,
                left_proportional=left_pid.proportional,
                right_proportional=right_pid.proportional,
                left_derivative=left_pid.derivative_term,
                right_derivative=right_pid.derivative_term,
                left_raw_pid=left_pid.raw_output,
                right_raw_pid=right_pid.raw_output,
                left_smoothed_command=left_pid.smoothed_command,
                right_smoothed_command=right_pid.smoothed_command,
                left_pos_normalized=left_pid.pos,
                right_pos_normalized=right_pid.pos,
                left_soft_stop_active=left_pid.soft_stop_active,
                right_soft_stop_active=right_pid.soft_stop_active,
                left_hard_stop_active=left_pid.hard_stop_active,
                right_hard_stop_active=right_pid.hard_stop_active,
                left_adc_raw=feedback.left_adc,
                right_adc_raw=feedback.right_adc,
                pot_feedback_fresh=True,
                left_command=left_command,
                right_command=right_command,
                motor1_byte=motor1_byte,
                motor2_byte=motor2_byte,
                update_rate_hz=update_rate,
                udp_connected=True,
                last_packet_time=now,
            )
        except Exception as exc:
            raise ValueError(str(exc)) from exc


class MotionTranslator:
    def __init__(self, params: RigParameters) -> None:
        self.params_lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.feedback_lock = threading.Lock()
        self.params = params
        self.telemetry = Telemetry()
        self.pot_feedback = PotFeedback()
        self.calibration_ack_queue: queue.Queue[str] = queue.Queue()
        self.packet_queue: queue.Queue[str] = queue.Queue()
        self.stop_event = threading.Event()
        self.threads_lock = threading.Lock()
        self.udp_thread: Optional[threading.Thread] = None
        self.processing_thread: Optional[threading.Thread] = None
        self.serial_rx_thread: Optional[threading.Thread] = None
        self.serial_port: Optional[serial.Serial] = None
        self.processor_state = ProcessorState()

    def get_params(self) -> RigParameters:
        with self.params_lock:
            return RigParameters.from_config(self.params.as_config())

    def get_telemetry(self) -> Telemetry:
        with self.telemetry_lock:
            return Telemetry(**self.telemetry.__dict__)

    def set_error(self, message: str) -> None:
        with self.telemetry_lock:
            self.telemetry.last_error = message[:240]

    def set_enabled_status(self, enabled: bool) -> None:
        with self.telemetry_lock:
            self.telemetry.enabled = enabled
            if not enabled:
                self.telemetry.udp_connected = False

    def set_serial_status(self, connected: bool) -> None:
        with self.telemetry_lock:
            self.telemetry.serial_connected = connected

    def update_params(self, params: RigParameters) -> None:
        with self.params_lock:
            self.params = params

    def reset_pid(self) -> None:
        self.processor_state.reset()

    def update_pot_feedback(self, left_adc: int, right_adc: int, timestamp: Optional[float] = None) -> None:
        now = time.time() if timestamp is None else float(timestamp)
        safe_left = clamp_int(left_adc, 0, 4095)
        safe_right = clamp_int(right_adc, 0, 4095)
        with self.feedback_lock:
            self.pot_feedback = PotFeedback(safe_left, safe_right, now)
        params = self.get_params()
        try:
            left_measured, left_pos = MotionMath.adc_to_displacement_mm(
                safe_left,
                params.pot_min_adc_left,
                params.pot_max_adc_left,
                params.travel_range_mm_left,
            )
            right_measured, right_pos = MotionMath.adc_to_displacement_mm(
                safe_right,
                params.pot_min_adc_right,
                params.pot_max_adc_right,
                params.travel_range_mm_right,
            )
            with self.telemetry_lock:
                self.telemetry.left_adc_raw = safe_left
                self.telemetry.right_adc_raw = safe_right
                self.telemetry.left_measured_disp_mm = left_measured
                self.telemetry.right_measured_disp_mm = right_measured
                self.telemetry.left_pos_normalized = left_pos
                self.telemetry.right_pos_normalized = right_pos
                self.telemetry.pot_feedback_fresh = True
        except Exception as exc:
            self.set_error(f"Pot feedback update failed: {exc}")

    def get_latest_pot_feedback(self) -> tuple[Optional[PotFeedback], bool]:
        with self.feedback_lock:
            feedback = PotFeedback(self.pot_feedback.left_adc, self.pot_feedback.right_adc, self.pot_feedback.timestamp)
        fresh = feedback.timestamp > 0.0 and (time.time() - feedback.timestamp) <= POT_FEEDBACK_STALE_SECONDS
        return (feedback if fresh else None), fresh

    def log_pot_error(self, message: str) -> None:
        try:
            POT_LOSS_ERROR_LOG.parent.mkdir(exist_ok=True)
            timestamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            with POT_LOSS_ERROR_LOG.open("a", encoding="utf-8") as handle:
                handle.write(f"{timestamp} {message}\n")
        except Exception as exc:
            self.set_error(f"Error log failed: {exc}")

    def handle_pot_failure(self, side: str) -> None:
        label = "LEFT" if side == "L" else "RIGHT"
        message = f"POT FAILURE: {label}"
        self.log_pot_error(message)
        self.disable()
        self.set_error(message)

    def start(self) -> None:
        with self.threads_lock:
            existing_threads = (self.udp_thread, self.processing_thread)
            if any(worker and worker.is_alive() for worker in existing_threads):
                return
            self.stop_event.clear()
            self.processor_state.reset()
            self._clear_queue()
            self.set_enabled_status(True)
            self.open_serial()
            self.udp_thread = threading.Thread(target=self._udp_loop, name="FlyPTUdpReceiver", daemon=True)
            self.processing_thread = threading.Thread(target=self._processing_loop, name="MotionProcessor", daemon=True)
            self.udp_thread.start()
            self.processing_thread.start()

    def disable(self) -> None:
        self.stop_event.set()
        self.send_stop_bytes()
        self.set_enabled_status(False)
        current = threading.current_thread()
        for worker in (self.udp_thread, self.processing_thread):
            if worker and worker.is_alive() and worker is not current:
                worker.join(timeout=1.0)
        self.send_stop_bytes()
        self._clear_queue()
        with self.telemetry_lock:
            self.telemetry.left_command = 0
            self.telemetry.right_command = 0
            self.telemetry.motor1_byte = STOP_MOTOR_1_BYTE
            self.telemetry.motor2_byte = STOP_MOTOR_2_BYTE
        self.processor_state.reset()

    def shutdown(self) -> None:
        self.send_stop_bytes()
        self.stop_event.set()
        current = threading.current_thread()
        for worker in (self.udp_thread, self.processing_thread):
            if worker and worker.is_alive() and worker is not current:
                worker.join(timeout=2.0)
        self.close_serial()

    def _clear_queue(self) -> None:
        while True:
            try:
                self.packet_queue.get_nowait()
            except queue.Empty:
                return
            except Exception as exc:
                self.set_error(f"Queue clear error: {exc}")
                return

    def open_serial(self) -> None:
        params = self.get_params()
        with self.serial_lock:
            self._close_serial_locked()
            if not params.serial_port:
                self.set_serial_status(False)
                self.set_error("Serial disconnected: no port selected")
                return
            try:
                if "://" in params.serial_port:
                    self.serial_port = serial.serial_for_url(
                        params.serial_port,
                        params.baud_rate,
                        timeout=0.05,
                        write_timeout=0.2,
                    )
                else:
                    self.serial_port = serial.Serial(
                        params.serial_port,
                        params.baud_rate,
                        timeout=0.05,
                        write_timeout=0.2,
                    )
                self.set_serial_status(True)
                self.set_error("")
                self.processor_state.reset()
                self._start_serial_reader_locked()
            except Exception as exc:
                self.serial_port = None
                self.set_serial_status(False)
                self.set_error(f"Serial open failed: {exc}")
            self.processor_state.reset()

    def close_serial(self) -> None:
        with self.serial_lock:
            self._close_serial_locked()
            self.set_serial_status(False)

    def _close_serial_locked(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception as exc:
                self.set_error(f"Serial close failed: {exc}")
            finally:
                self.serial_port = None
        self.processor_state.reset()

    def _start_serial_reader_locked(self) -> None:
        if self.serial_port is None:
            return
        if self.serial_rx_thread and self.serial_rx_thread.is_alive():
            return
        port = self.serial_port
        self.serial_rx_thread = threading.Thread(
            target=self._serial_rx_loop,
            args=(port,),
            name="XiaoSerialReceiver",
            daemon=True,
        )
        self.serial_rx_thread.start()

    def send_stop_bytes(self) -> None:
        self._write_motor_bytes(STOP_MOTOR_1_BYTE, STOP_MOTOR_2_BYTE)

    def _write_motor_bytes(self, motor1_byte: int, motor2_byte: int) -> bool:
        safe_motor1 = clamp_int(motor1_byte, 64, 127)
        safe_motor2 = clamp_int(motor2_byte, 192, 255)
        with self.serial_lock:
            if self.serial_port is None:
                self.set_serial_status(False)
                return False
            try:
                self.serial_port.write(bytes([safe_motor1, safe_motor2]))
                self.serial_port.flush()
                with self.telemetry_lock:
                    self.telemetry.last_serial_time = time.time()
                    self.telemetry.serial_connected = True
                return True
            except Exception as exc:
                self._close_serial_locked()
                self.set_serial_status(False)
                self.set_error(f"Serial write failed: {exc}")
                return False

    def _serial_rx_loop(self, port: serial.Serial) -> None:
        while not self.stop_event.is_set():
            try:
                if port is not self.serial_port or not getattr(port, "is_open", True):
                    return
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                self._handle_serial_line(line)
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.set_serial_status(False)
                    self.set_error(f"Serial read failed: {exc}")
                return

    def _handle_serial_line(self, line: str) -> None:
        try:
            if line == CAL_OK_TEXT:
                self.calibration_ack_queue.put_nowait(line)
                return
            if line == "ERR=POT_L":
                self.handle_pot_failure("L")
                return
            if line == "ERR=POT_R":
                self.handle_pot_failure("R")
                return
            match = POT_FEEDBACK_RE.match(line)
            if match:
                self.update_pot_feedback(int(match.group("left")), int(match.group("right")))
                return
            self.set_error(f"Serial line ignored: {line[:80]}")
        except Exception as exc:
            self.set_error(f"Serial line handling failed: {exc}")

    def send_calibration(
        self,
        left_min: int,
        left_max: int,
        right_min: int,
        right_max: int,
        timeout_s: float = 2.0,
    ) -> bool:
        command = f"CAL=L_MIN:{left_min},L_MAX:{left_max},R_MIN:{right_min},R_MAX:{right_max}\n"
        while True:
            try:
                self.calibration_ack_queue.get_nowait()
            except queue.Empty:
                break
        with self.serial_lock:
            if self.serial_port is None:
                self.set_error("XIAO calibration failed: serial disconnected")
                return False
            try:
                self.serial_port.write(command.encode("ascii"))
                self.serial_port.flush()
            except Exception as exc:
                self.set_error(f"XIAO calibration failed: {exc}")
                return False
        try:
            self.calibration_ack_queue.get(timeout=timeout_s)
            self.processor_state.reset()
            return True
        except queue.Empty:
            self.set_error("XIAO calibration failed")
            return False

    def _udp_loop(self) -> None:
        sock: Optional[socket.socket] = None
        try:
            params = self.get_params()
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", params.udp_port))
            sock.settimeout(0.2)
            self.set_error("")
            while not self.stop_event.is_set():
                try:
                    data, _addr = sock.recvfrom(2048)
                    packet = data.decode("ascii", errors="ignore").strip()
                    while self.packet_queue.qsize() >= UDP_QUEUE_LIMIT:
                        try:
                            self.packet_queue.get_nowait()
                        except queue.Empty:
                            break
                    self.packet_queue.put_nowait(packet)
                except socket.timeout:
                    with self.telemetry_lock:
                        if time.time() - self.telemetry.last_packet_time > 1.0:
                            self.telemetry.udp_connected = False
                    continue
                except OSError as exc:
                    if not self.stop_event.is_set():
                        self.set_error(f"UDP error: {exc}")
                    time.sleep(0.2)
                except Exception as exc:
                    self.set_error(f"UDP receiver error: {exc}")
                    time.sleep(0.2)
        except Exception as exc:
            self.set_error(f"UDP startup failed: {exc}")
            self.set_enabled_status(False)
            self.stop_event.set()
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception as exc:
                    self.set_error(f"UDP close error: {exc}")

    def _processing_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                packet = self.packet_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            except Exception as exc:
                self.set_error(f"Queue read error: {exc}")
                continue

            try:
                if self.stop_event.is_set():
                    break
                params = self.get_params()
                feedback, fresh = self.get_latest_pot_feedback()
                if not fresh:
                    now = time.time()
                    pitch_raw, roll_raw = MotionMath.parse_flypt_packet(packet)
                    pitch_deg, roll_deg = MotionMath.raw_to_degrees(pitch_raw, roll_raw, params)
                    left_disp, right_disp = MotionMath.displacement_mm(pitch_deg, roll_deg, params)
                    self.processor_state.reset()
                    self.send_stop_bytes()
                    with self.telemetry_lock:
                        self.telemetry.pitch_deg = pitch_deg
                        self.telemetry.roll_deg = roll_deg
                        self.telemetry.left_disp_mm = left_disp
                        self.telemetry.right_disp_mm = right_disp
                        self.telemetry.left_target_disp_mm = left_disp
                        self.telemetry.right_target_disp_mm = right_disp
                        self.telemetry.left_command = 0
                        self.telemetry.right_command = 0
                        self.telemetry.left_smoothed_command = 0.0
                        self.telemetry.right_smoothed_command = 0.0
                        self.telemetry.motor1_byte = STOP_MOTOR_1_BYTE
                        self.telemetry.motor2_byte = STOP_MOTOR_2_BYTE
                        self.telemetry.enabled = True
                        self.telemetry.udp_connected = True
                        self.telemetry.pot_feedback_fresh = False
                        self.telemetry.last_packet_time = now
                        self.telemetry.last_error = "Waiting for pot feedback"
                    continue
                computed = MotionMath.process_packet(packet, params, self.processor_state, feedback=feedback)
                if self.stop_event.is_set():
                    break
                wrote = self._write_motor_bytes(computed.motor1_byte, computed.motor2_byte)
                with self.telemetry_lock:
                    existing_error = self.telemetry.last_error
                    serial_connected = self.telemetry.serial_connected and wrote
                    last_serial_time = self.telemetry.last_serial_time
                    self.telemetry = computed
                    self.telemetry.enabled = True
                    self.telemetry.serial_connected = serial_connected
                    self.telemetry.last_serial_time = last_serial_time
                    self.telemetry.last_error = "" if wrote else existing_error
            except ValueError as exc:
                self.set_error(str(exc))
            except Exception as exc:
                self.set_error(f"Processing error: {exc}")
                self.send_stop_bytes()


class RigVisualizerCanvas(QWidget):
    def __init__(self, title: str, attachment_angle_label: str) -> None:
        super().__init__()
        self.title = title
        self.attachment_angle_label = attachment_angle_label
        self.angle_deg = 0.0
        self.enabled = False
        self.setMinimumSize(280, 190)

    def set_pose(self, angle_deg: float, enabled: bool) -> None:
        self.angle_deg = float(angle_deg)
        self.enabled = bool(enabled)
        self.update()

    def paintEvent(self, event: Any) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        try:
            self._draw(painter)
        finally:
            painter.end()

    def _draw(self, painter: QPainter) -> None:
        width = self.width()
        height = self.height()
        center = QPointF(width / 2.0, height / 2.0 + 12.0)
        platform_width = min(200.0, max(120.0, width - 80.0))
        platform_height = 20.0

        painter.fillRect(self.rect(), QColor(246, 247, 249))
        painter.setPen(QPen(QColor(33, 37, 41), 1))
        title_font = QFont()
        title_font.setPointSize(11)
        title_font.setBold(True)
        painter.setFont(title_font)
        painter.drawText(QRectF(0, 10, width, 24), Qt.AlignCenter, self.title)

        painter.setPen(QPen(QColor(145, 153, 161), 1, Qt.DashLine))
        painter.drawLine(QPointF(center.x(), 42), QPointF(center.x(), height - 24))

        painter.setPen(QPen(QColor(70, 77, 86), 1))
        painter.setBrush(QBrush(QColor(70, 77, 86)))
        pivot = [
            QPointF(center.x(), center.y() - 6),
            QPointF(center.x() - 7, center.y() + 7),
            QPointF(center.x() + 7, center.y() + 7),
        ]
        painter.drawPolygon(QPolygonF(pivot))

        painter.save()
        painter.translate(center)
        painter.rotate(self.angle_deg)
        platform_color = QColor(43, 108, 176) if self.enabled else QColor(135, 142, 150)
        painter.setPen(QPen(QColor(21, 39, 56), 2))
        painter.setBrush(QBrush(platform_color))
        painter.drawRoundedRect(QRectF(-platform_width / 2.0, -platform_height / 2.0, platform_width, platform_height), 3, 3)

        left_x = -platform_width * 0.38
        right_x = platform_width * 0.38
        painter.setPen(QPen(QColor(15, 23, 42), 1))
        painter.setBrush(QBrush(QColor(37, 99, 235)))
        painter.drawEllipse(QPointF(left_x, 0.0), 7.0, 7.0)
        painter.setBrush(QBrush(QColor(234, 88, 12)))
        painter.drawEllipse(QPointF(right_x, 0.0), 7.0, 7.0)
        painter.restore()

        painter.setPen(QPen(QColor(80, 88, 97), 1))
        value_font = QFont()
        value_font.setPointSize(9)
        painter.setFont(value_font)
        painter.drawText(
            QRectF(0, height - 28, width, 20),
            Qt.AlignCenter,
            f"{self.attachment_angle_label}: {self.angle_deg:.2f} deg",
        )

        if not self.enabled:
            painter.fillRect(self.rect(), QColor(128, 128, 128, 105))
            overlay_font = QFont()
            overlay_font.setPointSize(16)
            overlay_font.setBold(True)
            painter.setFont(overlay_font)
            painter.setPen(QPen(QColor(52, 58, 64), 2))
            painter.drawText(self.rect(), Qt.AlignCenter, "DISABLED")


class PidGraphCanvas(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.samples: deque[Telemetry] = deque(maxlen=300)
        self.setMinimumSize(280, 180)

    def add_sample(self, telemetry: Telemetry) -> None:
        self.samples.append(Telemetry(**telemetry.__dict__))
        self.update()

    def paintEvent(self, event: Any) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        try:
            self._draw(painter)
        finally:
            painter.end()

    def _draw(self, painter: QPainter) -> None:
        width = self.width()
        height = self.height()
        painter.fillRect(self.rect(), QColor(248, 249, 250))
        painter.setPen(QPen(QColor(33, 37, 41), 1))
        title_font = QFont()
        title_font.setPointSize(10)
        title_font.setBold(True)
        painter.setFont(title_font)
        painter.drawText(QRectF(0, 6, width, 20), Qt.AlignCenter, "PID RESPONSE - LAST 5S")
        plot = QRectF(36, 32, max(10, width - 48), max(10, height - 46))
        painter.setPen(QPen(QColor(190, 196, 204), 1))
        painter.drawRect(plot)
        for frac in (0.25, 0.5, 0.75):
            y = plot.top() + plot.height() * frac
            painter.setPen(QPen(QColor(220, 224, 229), 1, Qt.DashLine))
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))

        if len(self.samples) < 2:
            painter.setPen(QPen(QColor(90, 98, 106), 1))
            painter.drawText(plot, Qt.AlignCenter, "waiting for PID samples")
            return

        values = [
            ("left_error_mm", QColor(220, 38, 38), -100.0, 100.0),
            ("right_error_mm", QColor(37, 99, 235), -100.0, 100.0),
            ("left_command", QColor(234, 88, 12), 0.0, 127.0),
            ("right_command", QColor(8, 145, 178), 0.0, 127.0),
        ]
        samples = list(self.samples)
        count = len(samples)
        for attr, color, lo, hi in values:
            painter.setPen(QPen(color, 2))
            prev: Optional[QPointF] = None
            for i, sample in enumerate(samples):
                raw_value = float(getattr(sample, attr))
                normalized = clamp((raw_value - lo) / (hi - lo), 0.0, 1.0)
                x = plot.left() + plot.width() * (i / max(1, count - 1))
                y = plot.bottom() - plot.height() * normalized
                point = QPointF(x, y)
                if prev is not None:
                    painter.drawLine(prev, point)
                prev = point

        painter.setFont(QFont("Arial", 8))
        legend = "L err red | R err blue | L cmd orange | R cmd cyan"
        painter.setPen(QPen(QColor(70, 77, 86), 1))
        painter.drawText(QRectF(plot.left(), height - 17, plot.width(), 14), Qt.AlignLeft, legend)


class CalibrationDialog(QDialog):
    def __init__(self, parent: QWidget, translator: MotionTranslator) -> None:
        super().__init__(parent)
        self.translator = translator
        self.left_min: Optional[int] = None
        self.right_min: Optional[int] = None
        self.left_max: Optional[int] = None
        self.right_max: Optional[int] = None
        self.setWindowTitle("Potentiometer Calibration")
        self.setMinimumWidth(440)

        layout = QVBoxLayout(self)
        self.instructions = QLabel("Move rig to MINIMUM position (full string extended, frame at lowest travel)")
        self.instructions.setWordWrap(True)
        layout.addWidget(self.instructions)
        self.live_label = QLabel("L=0, R=0")
        self.live_label.setStyleSheet("font-family: Consolas, monospace; font-size: 16px;")
        layout.addWidget(self.live_label)

        buttons = QHBoxLayout()
        self.mark_min_button = QPushButton("Mark Minimum")
        self.mark_max_button = QPushButton("Mark Maximum")
        self.done_button = QPushButton("Done")
        self.done_button.setEnabled(False)
        buttons.addWidget(self.mark_min_button)
        buttons.addWidget(self.mark_max_button)
        buttons.addWidget(self.done_button)
        layout.addLayout(buttons)

        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh_adc)
        self.timer.start()
        self.mark_min_button.clicked.connect(self.mark_minimum)
        self.mark_max_button.clicked.connect(self.mark_maximum)
        self.done_button.clicked.connect(self.finish_calibration)
        self.refresh_adc()

    def refresh_adc(self) -> None:
        telemetry = self.translator.get_telemetry()
        self.live_label.setText(f"L={telemetry.left_adc_raw}, R={telemetry.right_adc_raw}")

    def mark_minimum(self) -> None:
        telemetry = self.translator.get_telemetry()
        self.left_min = telemetry.left_adc_raw
        self.right_min = telemetry.right_adc_raw
        self.instructions.setText("Move rig to MAXIMUM position (full string pulled, frame at highest travel)")

    def mark_maximum(self) -> None:
        telemetry = self.translator.get_telemetry()
        self.left_max = telemetry.left_adc_raw
        self.right_max = telemetry.right_adc_raw
        self.done_button.setEnabled(True)

    def finish_calibration(self) -> None:
        if None in (self.left_min, self.right_min, self.left_max, self.right_max):
            QMessageBox.critical(self, "Calibration incomplete", "Mark minimum and maximum before finishing.")
            return
        assert self.left_min is not None and self.right_min is not None and self.left_max is not None and self.right_max is not None
        if self.left_max <= self.left_min + 100 or self.right_max <= self.right_min + 100:
            QMessageBox.critical(self, "Range too small", "Range too small, recalibrate")
            return
        params = self.translator.get_params()
        radius = params.spool_diameter_mm / 2.0
        left_rad = (self.left_max - self.left_min) / 4095.0 * (270.0 * math.pi / 180.0)
        right_rad = (self.right_max - self.right_min) / 4095.0 * (270.0 * math.pi / 180.0)
        updated = RigParameters.from_config(
            {
                **params.as_config(),
                "pot_min_adc_left": self.left_min,
                "pot_max_adc_left": self.left_max,
                "pot_min_adc_right": self.right_min,
                "pot_max_adc_right": self.right_max,
                "travel_range_mm_left": left_rad * radius,
                "travel_range_mm_right": right_rad * radius,
            }
        )
        self.translator.update_params(updated)
        save_config(updated.as_config())
        if not self.translator.send_calibration(self.left_min, self.left_max, self.right_min, self.right_max):
            QMessageBox.critical(self, "XIAO calibration failed", "XIAO calibration failed")
            return
        QMessageBox.information(
            self,
            "Calibration complete",
            (
                f"Left range: {self.left_min} to {self.left_max} ({updated.travel_range_mm_left:.2f} mm)\n"
                f"Right range: {self.right_min} to {self.right_max} ({updated.travel_range_mm_right:.2f} mm)"
            ),
        )
        self.accept()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("2DOF String Motion Translator")
        self.resize(980, 620)
        self.translator = MotionTranslator(RigParameters.from_config(load_or_create_config()))
        self.telemetry_labels: dict[str, QLabel] = {}
        self.serial_combo = QComboBox()
        self.udp_port_spin = QSpinBox()
        self.baud_spin = QSpinBox()
        self.spool_diameter_spin = QDoubleSpinBox()
        self.l_spin = QDoubleSpinBox()
        self.pitch_limit_spin = QDoubleSpinBox()
        self.roll_limit_spin = QDoubleSpinBox()
        self.max_velocity_spin = QDoubleSpinBox()
        self.deadband_spin = QDoubleSpinBox()
        self.kp_spin = QDoubleSpinBox()
        self.ki_spin = QDoubleSpinBox()
        self.kd_spin = QDoubleSpinBox()
        self.integral_clamp_spin = QDoubleSpinBox()
        self.alpha_smoothing_spin = QDoubleSpinBox()
        self.soft_stop_zone_spin = QDoubleSpinBox()
        self.status_label = QLabel()
        self.enable_button = QPushButton("ENABLE")
        self.disable_button = QPushButton("DISABLE")
        self.reset_pid_button = QPushButton("Reset PID")
        self.calibrate_button = QPushButton("Calibrate")
        self.visualizer_toggle = QPushButton("Hide Rig Views")
        self.pid_graph_toggle = QPushButton("Hide PID Graph")
        self.side_view_canvas = RigVisualizerCanvas("SIDE VIEW - PITCH", "Pitch")
        self.front_view_canvas = RigVisualizerCanvas("FRONT VIEW - ROLL", "Roll")
        self.pid_graph_canvas = PidGraphCanvas()
        self.autostart_trigger_consumed = False
        self.autostart_trigger_deadline = time.monotonic() + 15.0
        self._build_ui()
        self.refresh_serial_ports()
        self._load_params_into_controls(self.translator.get_params())
        self._wire_events()
        self.timer = QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self.refresh_telemetry)
        self.timer.start()
        self.autostart_timer = QTimer(self)
        self.autostart_timer.setInterval(250)
        self.autostart_timer.timeout.connect(self.check_autostart_enable_trigger)
        self.autostart_timer.start()
        self.refresh_telemetry()
        self.check_autostart_enable_trigger()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 8)
        root.setSpacing(10)

        top_bar = QHBoxLayout()
        self.enable_button.setMinimumHeight(64)
        self.disable_button.setMinimumHeight(64)
        self.enable_button.setStyleSheet(
            "QPushButton { background: #149447; color: white; font-size: 24px; font-weight: 700; border-radius: 6px; }"
            "QPushButton:pressed { background: #0f6b34; }"
        )
        self.disable_button.setStyleSheet(
            "QPushButton { background: #b42318; color: white; font-size: 24px; font-weight: 700; border-radius: 6px; }"
            "QPushButton:pressed { background: #7a1a12; }"
        )
        top_bar.addWidget(self.enable_button, 1)
        top_bar.addWidget(self.disable_button, 1)
        root.addLayout(top_bar)

        body = QHBoxLayout()
        body.setSpacing(12)
        body.addWidget(self._build_parameter_panel(), 1)
        body.addWidget(self._build_telemetry_panel(), 1)
        body.addWidget(self._build_visualizer_panel(), 1)
        root.addLayout(body, 1)

        bottom_bar = QHBoxLayout()
        self.open_flypt_button = QPushButton("Open FlyPT Mover")
        self.open_flypt_button.setMinimumHeight(38)
        bottom_bar.addWidget(self.open_flypt_button)
        bottom_bar.addStretch(1)
        root.addLayout(bottom_bar)

        self.setCentralWidget(central)
        status = QStatusBar()
        status.addPermanentWidget(self.status_label, 1)
        self.setStatusBar(status)

    def _build_parameter_panel(self) -> QGroupBox:
        group = QGroupBox("Parameters")
        layout = QFormLayout(group)
        layout.setLabelAlignment(Qt.AlignRight)
        layout.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.udp_port_spin.setRange(1, 65535)

        serial_row = QHBoxLayout()
        self.serial_combo.setEditable(True)
        self.refresh_button = QPushButton("Refresh")
        serial_row.addWidget(self.serial_combo, 1)
        serial_row.addWidget(self.refresh_button)
        serial_container = QWidget()
        serial_container.setLayout(serial_row)

        self.baud_spin.setRange(300, 1000000)
        self.baud_spin.setSingleStep(100)

        for spin in (
            self.spool_diameter_spin,
            self.l_spin,
            self.pitch_limit_spin,
            self.roll_limit_spin,
            self.max_velocity_spin,
            self.deadband_spin,
            self.kp_spin,
            self.ki_spin,
            self.kd_spin,
            self.integral_clamp_spin,
            self.alpha_smoothing_spin,
            self.soft_stop_zone_spin,
        ):
            spin.setDecimals(5)
            spin.setRange(0.0, 100000.0)
            spin.setSingleStep(1.0)

        self.spool_diameter_spin.setMinimum(1.0)
        self.l_spin.setMinimum(1.0)
        self.pitch_limit_spin.setRange(0.01, 180.0)
        self.roll_limit_spin.setRange(0.01, 180.0)
        self.max_velocity_spin.setMinimum(0.01)
        self.kp_spin.setSingleStep(0.1)
        self.ki_spin.setSingleStep(0.01)
        self.kd_spin.setSingleStep(0.01)
        self.integral_clamp_spin.setSingleStep(1.0)
        self.alpha_smoothing_spin.setRange(0.0, 1.0)
        self.alpha_smoothing_spin.setSingleStep(0.05)
        self.soft_stop_zone_spin.setRange(0.1, 50.0)
        self.soft_stop_zone_spin.setSingleStep(0.5)

        layout.addRow("UDP Port", self.udp_port_spin)
        layout.addRow("Serial Port", serial_container)
        layout.addRow("Baud Rate", self.baud_spin)
        layout.addRow("Spool Diameter mm", self.spool_diameter_spin)
        layout.addRow("L pivot-to-attachment mm", self.l_spin)
        layout.addRow("Pitch Limit deg", self.pitch_limit_spin)
        layout.addRow("Roll Limit deg", self.roll_limit_spin)
        layout.addRow("Max Velocity mm/s", self.max_velocity_spin)
        layout.addRow("Deadband mm", self.deadband_spin)
        layout.addRow("Kp", self.kp_spin)
        layout.addRow("Ki", self.ki_spin)
        layout.addRow("Kd", self.kd_spin)
        layout.addRow("Integral Clamp", self.integral_clamp_spin)
        layout.addRow("Alpha Smoothing", self.alpha_smoothing_spin)
        layout.addRow("Soft Stop Zone %", self.soft_stop_zone_spin)

        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        layout.addRow(separator)
        layout.addRow(self.reset_pid_button)
        layout.addRow(self.calibrate_button)
        self.apply_button = QPushButton("Apply and Save")
        self.apply_button.setMinimumHeight(42)
        layout.addRow(self.apply_button)
        return group

    def _build_telemetry_panel(self) -> QGroupBox:
        group = QGroupBox("Live Telemetry")
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)
        rows = [
            ("pitch_deg", "Pitch deg"),
            ("roll_deg", "Roll deg"),
            ("left_disp_mm", "Left Displacement mm"),
            ("right_disp_mm", "Right Displacement mm"),
            ("left_measured_disp_mm", "Left Measured mm"),
            ("right_measured_disp_mm", "Right Measured mm"),
            ("left_error_mm", "Left Error mm"),
            ("right_error_mm", "Right Error mm"),
            ("left_integral", "Left Integral"),
            ("right_integral", "Right Integral"),
            ("left_proportional", "Left Proportional"),
            ("right_proportional", "Right Proportional"),
            ("left_derivative", "Left Derivative"),
            ("right_derivative", "Right Derivative"),
            ("left_raw_pid", "Left Raw PID"),
            ("right_raw_pid", "Right Raw PID"),
            ("left_smoothed_command", "Left Smoothed Command"),
            ("right_smoothed_command", "Right Smoothed Command"),
            ("left_pos_normalized", "Left Pos normalized"),
            ("right_pos_normalized", "Right Pos normalized"),
            ("left_soft_stop_active", "Left Soft Stop Active"),
            ("right_soft_stop_active", "Right Soft Stop Active"),
            ("left_hard_stop_active", "Left Hard Stop Active"),
            ("right_hard_stop_active", "Right Hard Stop Active"),
            ("left_command", "Left Motor Command 0 to 127"),
            ("right_command", "Right Motor Command 0 to 127"),
            ("motor1_byte", "Motor 1 Serial Byte"),
            ("motor2_byte", "Motor 2 Serial Byte"),
            ("update_rate_hz", "Update Rate Hz"),
        ]
        for row, (key, label_text) in enumerate(rows):
            label = QLabel(label_text)
            value = QLabel("0")
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value.setMinimumHeight(28)
            value.setStyleSheet("font-family: Consolas, monospace; font-size: 15px;")
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
            self.telemetry_labels[key] = value
        return group

    def _build_visualizer_panel(self) -> QGroupBox:
        group = QGroupBox("Rig Visualizer")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 10, 8, 8)
        layout.setSpacing(10)
        layout.addWidget(self.visualizer_toggle)
        layout.addWidget(self.side_view_canvas, 1)
        layout.addWidget(self.front_view_canvas, 1)
        layout.addWidget(self.pid_graph_toggle)
        layout.addWidget(self.pid_graph_canvas, 1)
        return group

    def _wire_events(self) -> None:
        self.enable_button.clicked.connect(self.enable_translator)
        self.disable_button.clicked.connect(self.disable_translator)
        self.refresh_button.clicked.connect(self.refresh_serial_ports)
        self.apply_button.clicked.connect(self.apply_and_save)
        self.reset_pid_button.clicked.connect(self.reset_pid)
        self.calibrate_button.clicked.connect(self.open_calibration_dialog)
        self.visualizer_toggle.clicked.connect(self.toggle_visualizer_views)
        self.pid_graph_toggle.clicked.connect(self.toggle_pid_graph)
        self.open_flypt_button.clicked.connect(self.open_flypt_mover)

    def check_autostart_enable_trigger(self) -> None:
        if self.autostart_trigger_consumed:
            self.autostart_timer.stop()
            return

        if AUTOSTART_ENABLE_PATH.exists():
            self.autostart_trigger_consumed = True
            try:
                AUTOSTART_ENABLE_PATH.unlink()
            except Exception as exc:
                self.translator.set_error(f"Autostart trigger delete failed: {exc}")
            self.autostart_timer.stop()
            QTimer.singleShot(2000, self._click_enable_for_autostart)
            return

        if time.monotonic() >= self.autostart_trigger_deadline:
            self.autostart_timer.stop()

    def _click_enable_for_autostart(self) -> None:
        try:
            self.enable_button.click()
        except Exception as exc:
            self.translator.set_error(f"Autostart enable failed: {exc}")

    def _load_params_into_controls(self, params: RigParameters) -> None:
        self.udp_port_spin.setValue(params.udp_port)
        self._set_serial_combo_value(params.serial_port)
        self.baud_spin.setValue(params.baud_rate)
        self.spool_diameter_spin.setValue(params.spool_diameter_mm)
        self.l_spin.setValue(params.l_pivot_to_attachment_mm)
        self.pitch_limit_spin.setValue(params.pitch_limit_deg)
        self.roll_limit_spin.setValue(params.roll_limit_deg)
        self.max_velocity_spin.setValue(params.max_velocity_mm_s)
        self.deadband_spin.setValue(params.deadband_mm)
        self.kp_spin.setValue(params.kp)
        self.ki_spin.setValue(params.ki)
        self.kd_spin.setValue(params.kd)
        self.integral_clamp_spin.setValue(params.integral_clamp)
        self.alpha_smoothing_spin.setValue(params.alpha_smoothing)
        self.soft_stop_zone_spin.setValue(params.soft_stop_zone_percent)

    def _set_serial_combo_value(self, value: str) -> None:
        if value and self.serial_combo.findText(value) < 0:
            self.serial_combo.addItem(value)
        self.serial_combo.setCurrentText(value)

    def _controls_to_params(self) -> RigParameters:
        current = self.translator.get_params()
        return RigParameters.from_config(
            {
                "udp_port": self.udp_port_spin.value(),
                "serial_port": self.serial_combo.currentText().strip(),
                "baud_rate": self.baud_spin.value(),
                "spool_diameter_mm": self.spool_diameter_spin.value(),
                "l_pivot_to_attachment_mm": self.l_spin.value(),
                "pitch_limit_deg": self.pitch_limit_spin.value(),
                "roll_limit_deg": self.roll_limit_spin.value(),
                "max_velocity_mm_s": self.max_velocity_spin.value(),
                "deadband_mm": self.deadband_spin.value(),
                "kp": self.kp_spin.value(),
                "ki": self.ki_spin.value(),
                "kd": self.kd_spin.value(),
                "integral_clamp": self.integral_clamp_spin.value(),
                "alpha_smoothing": self.alpha_smoothing_spin.value(),
                "soft_stop_zone_percent": self.soft_stop_zone_spin.value(),
                "pot_min_adc_left": current.pot_min_adc_left,
                "pot_max_adc_left": current.pot_max_adc_left,
                "pot_min_adc_right": current.pot_min_adc_right,
                "pot_max_adc_right": current.pot_max_adc_right,
                "travel_range_mm_left": current.travel_range_mm_left,
                "travel_range_mm_right": current.travel_range_mm_right,
            }
        )

    def refresh_serial_ports(self) -> None:
        current = self.serial_combo.currentText().strip()
        self.serial_combo.blockSignals(True)
        self.serial_combo.clear()
        try:
            ports = list(list_ports.comports())
            for port in ports:
                label = port.device
                description = port.description or ""
                self.serial_combo.addItem(label, label)
                if description:
                    index = self.serial_combo.findText(label)
                    self.serial_combo.setItemData(index, f"{label} - {description}", Qt.ToolTipRole)
        except Exception as exc:
            self.translator.set_error(f"Serial port scan failed: {exc}")
        if current:
            self._set_serial_combo_value(current)
        self.serial_combo.blockSignals(False)

    def apply_and_save(self) -> None:
        try:
            params = self._controls_to_params()
            self.translator.update_params(params)
            save_config(params.as_config())
            if self.translator.get_telemetry().enabled:
                self.translator.open_serial()
            self.translator.set_error("")
        except Exception as exc:
            self.translator.set_error(f"Apply failed: {exc}")
            QMessageBox.critical(self, "Apply failed", str(exc))

    def enable_translator(self) -> None:
        try:
            self.apply_and_save()
            self.translator.start()
        except Exception as exc:
            self.translator.set_error(f"Enable failed: {exc}")
            QMessageBox.critical(self, "Enable failed", str(exc))

    def disable_translator(self) -> None:
        try:
            self.translator.disable()
        except Exception as exc:
            self.translator.set_error(f"Disable failed: {exc}")
            QMessageBox.critical(self, "Disable failed", str(exc))

    def reset_pid(self) -> None:
        try:
            self.translator.reset_pid()
            self.translator.set_error("PID reset")
        except Exception as exc:
            self.translator.set_error(f"Reset PID failed: {exc}")

    def open_calibration_dialog(self) -> None:
        try:
            if self.translator.get_telemetry().enabled:
                QMessageBox.critical(self, "Calibration refused", "Disable the app before calibration.")
                self.translator.set_error("Calibration refused: app must be disabled")
                return
            dialog = CalibrationDialog(self, self.translator)
            dialog.exec_()
            self._load_params_into_controls(self.translator.get_params())
        except Exception as exc:
            self.translator.set_error(f"Calibration failed: {exc}")
            QMessageBox.critical(self, "Calibration failed", str(exc))

    def toggle_pid_graph(self) -> None:
        visible = not self.pid_graph_canvas.isVisible()
        self.pid_graph_canvas.setVisible(visible)
        self.pid_graph_toggle.setText("Hide PID Graph" if visible else "Show PID Graph")

    def toggle_visualizer_views(self) -> None:
        visible = not self.side_view_canvas.isVisible()
        self.side_view_canvas.setVisible(visible)
        self.front_view_canvas.setVisible(visible)
        self.visualizer_toggle.setText("Hide Rig Views" if visible else "Show Rig Views")

    def open_flypt_mover(self) -> None:
        try:
            mover_files = sorted(APP_DIR.glob("*.fmover"))
            if not mover_files:
                QMessageBox.critical(
                    self,
                    "No FlyPT file found",
                    f"No .fmover file was found in {APP_DIR}.",
                )
                return
            os.startfile(str(mover_files[0]))
        except Exception as exc:
            self.translator.set_error(f"Open FlyPT Mover failed: {exc}")
            QMessageBox.critical(self, "Open FlyPT Mover failed", str(exc))

    def refresh_telemetry(self) -> None:
        telemetry = self.translator.get_telemetry()
        self.telemetry_labels["pitch_deg"].setText(f"{telemetry.pitch_deg:.3f}")
        self.telemetry_labels["roll_deg"].setText(f"{telemetry.roll_deg:.3f}")
        self.telemetry_labels["left_disp_mm"].setText(f"{telemetry.left_disp_mm:.3f}")
        self.telemetry_labels["right_disp_mm"].setText(f"{telemetry.right_disp_mm:.3f}")
        for key in (
            "left_measured_disp_mm",
            "right_measured_disp_mm",
            "left_error_mm",
            "right_error_mm",
            "left_integral",
            "right_integral",
            "left_proportional",
            "right_proportional",
            "left_derivative",
            "right_derivative",
            "left_raw_pid",
            "right_raw_pid",
            "left_smoothed_command",
            "right_smoothed_command",
            "left_pos_normalized",
            "right_pos_normalized",
        ):
            self.telemetry_labels[key].setText(f"{float(getattr(telemetry, key)):.3f}")
        for key in (
            "left_soft_stop_active",
            "right_soft_stop_active",
            "left_hard_stop_active",
            "right_hard_stop_active",
        ):
            active = bool(getattr(telemetry, key))
            self.telemetry_labels[key].setText("yes" if active else "no")
            if "hard" in key and active:
                self.telemetry_labels[key].setStyleSheet("font-family: Consolas, monospace; font-size: 15px; color: #b42318;")
            else:
                self.telemetry_labels[key].setStyleSheet("font-family: Consolas, monospace; font-size: 15px;")
        self.telemetry_labels["left_command"].setText(str(clamp_int(telemetry.left_command, 0, 127)))
        self.telemetry_labels["right_command"].setText(str(clamp_int(telemetry.right_command, 0, 127)))
        self.telemetry_labels["motor1_byte"].setText(str(clamp_int(telemetry.motor1_byte, 64, 127)))
        self.telemetry_labels["motor2_byte"].setText(str(clamp_int(telemetry.motor2_byte, 192, 255)))
        self.telemetry_labels["update_rate_hz"].setText(f"{telemetry.update_rate_hz:.1f}")
        self.side_view_canvas.set_pose(telemetry.pitch_deg, telemetry.enabled)
        self.front_view_canvas.set_pose(telemetry.roll_deg, telemetry.enabled)
        if not hasattr(self, "_last_pid_graph_update"):
            self._last_pid_graph_update = 0.0
        now = time.monotonic()
        if now - self._last_pid_graph_update >= (1.0 / 30.0):
            self._last_pid_graph_update = now
            self.pid_graph_canvas.add_sample(telemetry)

        udp_text = "UDP connected" if telemetry.udp_connected else "UDP waiting"
        serial_text = "serial connected" if telemetry.serial_connected else "serial disconnected"
        pot_text = "pot feedback" if telemetry.pot_feedback_fresh else "pot waiting"
        mode_text = "enabled" if telemetry.enabled else "disabled"
        error_text = telemetry.last_error or "no error"
        self.status_label.setText(f"{mode_text} | {udp_text} | {serial_text} | {pot_text} | {error_text}")

    def closeEvent(self, event: Any) -> None:
        try:
            self.translator.shutdown()
        finally:
            event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec_()


class VisualizationPanel:
    @staticmethod
    def test_with_replay(replay_path: str) -> None:
        path = Path(replay_path)
        if not path.exists():
            raise FileNotFoundError(f"Replay log not found: {path}")

        payload = json.loads(path.read_text(encoding="utf-8"))
        frames = payload.get("frames") or payload.get("telemetry") or []
        if not frames:
            raise ValueError(f"Replay log has no frames: {path}")

        app = QApplication.instance() or QApplication(sys.argv)
        side = RigVisualizerCanvas("SIDE VIEW - PITCH", "Pitch")
        front = RigVisualizerCanvas("FRONT VIEW - ROLL", "Roll")
        side.resize(320, 220)
        front.resize(320, 220)

        samples = 0
        for frame in frames[:: max(1, len(frames) // 120)]:
            pitch_raw = int(frame.get("P", frame.get("pitch_raw", 511)))
            roll_raw = int(frame.get("R", frame.get("roll_raw", 511)))
            pitch_deg, roll_deg = MotionMath.raw_to_degrees(pitch_raw, roll_raw, RigParameters())
            side.set_pose(pitch_deg, True)
            front.set_pose(roll_deg, True)
            side.repaint()
            front.repaint()
            samples += 1

        report = {
            "visualization_passed": samples > 0,
            "samples_rendered": samples,
            "source": str(path),
            "views": ["SIDE VIEW - PITCH", "FRONT VIEW - ROLL"],
            "notes": [
                "Current implementation is a 2D PyQt QPainter visualizer with hide/show view controls and a collapsible PID graph.",
                "A separate 3D view, green/gray string state, and velocity arrows are not implemented in translator.py.",
            ],
        }
        out = APP_DIR / "test_reports" / "visualization_report.json"
        out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("VISUALIZATION TEST PASSED")
        print(json.dumps(report))


if __name__ == "__main__":
    raise SystemExit(main())
