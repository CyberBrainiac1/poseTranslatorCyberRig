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

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
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

PACKET_RE = re.compile(r"^\s*P=(?P<pitch>\d{1,4}),R=(?P<roll>\d{1,4})\s*$")

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
            }
        )


@dataclass
class Telemetry:
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    left_disp_mm: float = 0.0
    right_disp_mm: float = 0.0
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
    frame_times: deque[float] = field(default_factory=lambda: deque(maxlen=TELEMETRY_RATE_WINDOW))

    def reset(self) -> None:
        self.previous_left_disp = None
        self.previous_right_disp = None
        self.previous_time = None
        self.frame_times.clear()


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
    def process_packet(packet: str, params: RigParameters, state: ProcessorState) -> Telemetry:
        try:
            now = time.time()
            pitch_raw, roll_raw = MotionMath.parse_flypt_packet(packet)
            pitch_deg, roll_deg = MotionMath.raw_to_degrees(pitch_raw, roll_raw, params)
            left_disp, right_disp = MotionMath.displacement_mm(pitch_deg, roll_deg, params)
            left_command, right_command = MotionMath.speed_commands(left_disp, right_disp, params, state, now)
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
        self.params = params
        self.telemetry = Telemetry()
        self.packet_queue: queue.Queue[str] = queue.Queue()
        self.stop_event = threading.Event()
        self.threads_lock = threading.Lock()
        self.udp_thread: Optional[threading.Thread] = None
        self.processing_thread: Optional[threading.Thread] = None
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
            except Exception as exc:
                self.serial_port = None
                self.set_serial_status(False)
                self.set_error(f"Serial open failed: {exc}")

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
                computed = MotionMath.process_packet(packet, params, self.processor_state)
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
        self.status_label = QLabel()
        self.enable_button = QPushButton("ENABLE")
        self.disable_button = QPushButton("DISABLE")
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
        ):
            spin.setDecimals(5)
            spin.setRange(0.0, 100000.0)
            spin.setSingleStep(1.0)

        self.spool_diameter_spin.setMinimum(1.0)
        self.l_spin.setMinimum(1.0)
        self.pitch_limit_spin.setRange(0.01, 180.0)
        self.roll_limit_spin.setRange(0.01, 180.0)
        self.max_velocity_spin.setMinimum(0.01)

        layout.addRow("UDP Port", self.udp_port_spin)
        layout.addRow("Serial Port", serial_container)
        layout.addRow("Baud Rate", self.baud_spin)
        layout.addRow("Spool Diameter mm", self.spool_diameter_spin)
        layout.addRow("L pivot-to-attachment mm", self.l_spin)
        layout.addRow("Pitch Limit deg", self.pitch_limit_spin)
        layout.addRow("Roll Limit deg", self.roll_limit_spin)
        layout.addRow("Max Velocity mm/s", self.max_velocity_spin)
        layout.addRow("Deadband mm", self.deadband_spin)

        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        layout.addRow(separator)
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

    def _wire_events(self) -> None:
        self.enable_button.clicked.connect(self.enable_translator)
        self.disable_button.clicked.connect(self.disable_translator)
        self.refresh_button.clicked.connect(self.refresh_serial_ports)
        self.apply_button.clicked.connect(self.apply_and_save)
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

    def _set_serial_combo_value(self, value: str) -> None:
        if value and self.serial_combo.findText(value) < 0:
            self.serial_combo.addItem(value)
        self.serial_combo.setCurrentText(value)

    def _controls_to_params(self) -> RigParameters:
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
        self.telemetry_labels["left_command"].setText(str(clamp_int(telemetry.left_command, 0, 127)))
        self.telemetry_labels["right_command"].setText(str(clamp_int(telemetry.right_command, 0, 127)))
        self.telemetry_labels["motor1_byte"].setText(str(clamp_int(telemetry.motor1_byte, 64, 127)))
        self.telemetry_labels["motor2_byte"].setText(str(clamp_int(telemetry.motor2_byte, 192, 255)))
        self.telemetry_labels["update_rate_hz"].setText(f"{telemetry.update_rate_hz:.1f}")

        udp_text = "UDP connected" if telemetry.udp_connected else "UDP waiting"
        serial_text = "serial connected" if telemetry.serial_connected else "serial disconnected"
        mode_text = "enabled" if telemetry.enabled else "disabled"
        error_text = telemetry.last_error or "no error"
        self.status_label.setText(f"{mode_text} | {udp_text} | {serial_text} | {error_text}")

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


if __name__ == "__main__":
    raise SystemExit(main())
