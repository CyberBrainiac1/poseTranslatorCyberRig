from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import json
import random
import socket
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor, QImage
from PyQt5.QtWidgets import QApplication

from translator import (
    AUTOSTART_ENABLE_PATH,
    CONFIG_PATH,
    MainWindow,
    MotionMath,
    MotionTranslator,
    ProcessorState,
    RigParameters,
    RigVisualizerCanvas,
    STOP_MOTOR_1_BYTE,
    STOP_MOTOR_2_BYTE,
)


ROOT = Path(__file__).resolve().parent
OUTER_ROOT = ROOT.parent
FLYPT_DIR = OUTER_ROOT / "FlyPT Mover 3.5.8"
FLYPT_PROFILE = OUTER_ROOT / "New.Mover"
BEAMNG_EXES = [
    Path(r"C:\Program Files (x86)\Steam\steamapps\common\BeamNG.drive\BeamNG.drive.exe"),
    Path(r"C:\Program Files (x86)\Steam\steamapps\common\BeamNG.drive\Bin64\BeamNG.drive.x64.exe"),
]


@dataclass
class Result:
    name: str
    passed: bool
    detail: str


class CaptureTranslator(MotionTranslator):
    def __init__(self, params: RigParameters) -> None:
        super().__init__(params)
        self.writes: list[tuple[int, int]] = []

    def open_serial(self) -> None:
        self.set_serial_status(True)
        self.set_error("")
        self.update_pot_feedback(2048, 2048)

    def close_serial(self) -> None:
        self.set_serial_status(False)

    def _write_motor_bytes(self, motor1_byte: int, motor2_byte: int) -> bool:
        safe_1 = max(64, min(127, int(motor1_byte)))
        safe_2 = max(192, min(255, int(motor2_byte)))
        self.writes.append((safe_1, safe_2))
        with self.telemetry_lock:
            self.telemetry.last_serial_time = time.time()
            self.telemetry.serial_connected = True
        self.update_pot_feedback(2048, 2048)
        return True


def run_test(name: str, fn: Callable[[], str]) -> Result:
    try:
        detail = fn()
        print(f"PASS - {name}: {detail}")
        return Result(name, True, detail)
    except AssertionError as exc:
        detail = str(exc) or "assertion failed"
        print(f"FAIL - {name}: {detail}")
        return Result(name, False, detail)
    except Exception as exc:
        detail = f"unexpected error: {exc}"
        print(f"FAIL - {name}: {detail}")
        return Result(name, False, detail)


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def wait_for_condition(predicate: Callable[[], bool], timeout_s: float = 3.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def app_instance() -> QApplication:
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


def make_beamng_like_sequence() -> list[tuple[int, int]]:
    sequence: list[tuple[int, int]] = []
    for i in range(240):
        t = i / 239.0
        pitch = int(round(511.5 + 260.0 * __import__("math").sin(t * 6.28318530718 * 2.0)))
        roll = int(round(511.5 + 220.0 * __import__("math").sin(t * 6.28318530718 * 3.0 + 0.6)))
        sequence.append((max(0, min(1023, pitch)), max(0, min(1023, roll))))
    return sequence


def test_01_flypt_beamng_profile() -> str:
    assert FLYPT_DIR.exists(), f"FlyPT folder missing: {FLYPT_DIR}"
    assert (FLYPT_DIR / "FlyPT Mover.exe").exists(), "FlyPT Mover.exe missing"
    assert FLYPT_PROFILE.exists(), f"FlyPT profile missing: {FLYPT_PROFILE}"
    assert any(path.exists() for path in BEAMNG_EXES), "BeamNG executable not found in expected Steam paths"

    root = ET.fromstring(FLYPT_PROFILE.read_text(encoding="utf-8"))
    xml_text = ET.tostring(root, encoding="unicode")
    assert "Mover.Sources.UDP.BeamNG" in xml_text, "profile is not using BeamNG UDP source"
    assert "<NUDPort>9000</NUDPort>" in xml_text, "profile output port is not 9000"
    assert "P=&lt;PPosePitch&gt;,R=&lt;PPoseRoll&gt;&lt;13&gt;&lt;10&gt;" in FLYPT_PROFILE.read_text(encoding="utf-8")
    return "FlyPT BeamNG source and translator UDP output profile are configured"


def test_02_randomized_math_safety() -> str:
    rng = random.Random(42)
    params = RigParameters()
    state = ProcessorState(previous_left_disp=0.0, previous_right_disp=0.0, previous_time=time.time() - 0.01)
    for _ in range(1000):
        packet = f"P={rng.randrange(0, 1024)},R={rng.randrange(0, 1024)}"
        telemetry = MotionMath.process_packet(packet, params, state)
        assert 64 <= telemetry.motor1_byte <= 127, telemetry
        assert 192 <= telemetry.motor2_byte <= 255, telemetry
        if telemetry.left_disp_mm <= params.deadband_mm:
            assert telemetry.left_command == 0, telemetry
        if telemetry.right_disp_mm <= params.deadband_mm:
            assert telemetry.right_command == 0, telemetry
    return "1000 randomized packets kept Option A and byte clamps"


def test_03_beamng_like_serial_output_capture() -> str:
    params = RigParameters(udp_port=free_udp_port())
    translator = CaptureTranslator(params)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        translator.start()
        sequence = make_beamng_like_sequence()
        for pitch, roll in sequence:
            sender.sendto(f"P={pitch},R={roll}\r\n".encode("ascii"), ("127.0.0.1", params.udp_port))
            time.sleep(0.005)
        assert wait_for_condition(lambda: len(translator.writes) >= 50, 4.0), f"only {len(translator.writes)} writes captured"
        assert all(64 <= b1 <= 127 and 192 <= b2 <= 255 for b1, b2 in translator.writes)
        assert any(b1 > 64 for b1, _ in translator.writes), "left motor never pulled"
        assert any(b2 > 192 for _, b2 in translator.writes), "right motor never pulled"
        latest = translator.get_telemetry()
        assert latest.udp_connected, "UDP telemetry did not connect"
        return f"captured {len(translator.writes)} safe serial writes, latest bytes=({latest.motor1_byte},{latest.motor2_byte})"
    finally:
        sender.close()
        translator.shutdown()


def test_04_udp_burst_drop_and_recovery() -> str:
    params = RigParameters(udp_port=free_udp_port())
    translator = CaptureTranslator(params)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        translator.start()
        for i in range(350):
            packet = f"P={(i * 17) % 1024},R={(1023 - i * 11) % 1024}\r\n"
            sender.sendto(packet.encode("ascii"), ("127.0.0.1", params.udp_port))
        assert wait_for_condition(lambda: len(translator.writes) > 10, 3.0), "translator did not process burst"
        assert translator.packet_queue.qsize() <= 5, f"queue depth exceeded 5: {translator.packet_queue.qsize()}"
        assert all(64 <= b1 <= 127 and 192 <= b2 <= 255 for b1, b2 in translator.writes)
        return f"processed burst with queue depth {translator.packet_queue.qsize()} and {len(translator.writes)} writes"
    finally:
        sender.close()
        translator.shutdown()


def test_05_malformed_udp_recovery() -> str:
    params = RigParameters(udp_port=free_udp_port())
    translator = CaptureTranslator(params)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        translator.start()
        sender.sendto(b"P=511,R=511\r\n", ("127.0.0.1", params.udp_port))
        assert wait_for_condition(lambda: translator.get_telemetry().last_packet_time > 0.0, 3.0), translator.get_telemetry()
        for payload in (b"garbage", b"P=-1,R=20", b"P=2000,R=1", b"P=,R="):
            sender.sendto(payload, ("127.0.0.1", params.udp_port))
            time.sleep(0.03)
        sender.sendto(b"P=1023,R=511\r\n", ("127.0.0.1", params.udp_port))
        assert wait_for_condition(lambda: translator.get_telemetry().motor1_byte > 64, 3.0), translator.get_telemetry()
        latest = translator.get_telemetry()
        assert latest.motor1_byte > 64 and latest.motor2_byte > 192, latest
        return "malformed packets did not prevent later valid PID output"
    finally:
        sender.close()
        translator.shutdown()


def test_06_disable_writes_stop_bytes() -> str:
    params = RigParameters(udp_port=free_udp_port())
    translator = CaptureTranslator(params)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        translator.start()
        sender.sendto(b"P=511,R=511\r\n", ("127.0.0.1", params.udp_port))
        assert wait_for_condition(lambda: translator.get_telemetry().last_packet_time > 0.0, 3.0), translator.get_telemetry()
        sender.sendto(b"P=1023,R=511\r\n", ("127.0.0.1", params.udp_port))
        assert wait_for_condition(lambda: any(pair[0] > 64 and pair[1] > 192 for pair in translator.writes), 3.0), translator.writes[-5:]
        translator.disable()
        assert translator.writes[-1] == (64, 192), translator.writes[-5:]
        stopped = translator.get_telemetry()
        assert not stopped.enabled and stopped.motor1_byte == 64 and stopped.motor2_byte == 192, stopped
        return f"disable tail writes={translator.writes[-3:]}"
    finally:
        sender.close()
        translator.shutdown()


def test_07_loop_serial_url_smoke() -> str:
    params = RigParameters(udp_port=free_udp_port(), serial_port="loop://", baud_rate=9600)
    translator = MotionTranslator(params)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        translator.start()
        sender.sendto(b"P=511,R=511\r\n", ("127.0.0.1", params.udp_port))
        assert wait_for_condition(lambda: translator.get_telemetry().last_packet_time > 0.0, 3.0), translator.get_telemetry()
        sender.sendto(b"P=1023,R=511\r\n", ("127.0.0.1", params.udp_port))
        assert wait_for_condition(lambda: translator.get_telemetry().serial_connected, 3.0), translator.get_telemetry()
        assert wait_for_condition(lambda: translator.get_telemetry().last_serial_time > 0.0, 3.0), translator.get_telemetry()
        latest = translator.get_telemetry()
        assert latest.motor1_byte == 64 and latest.motor2_byte == 192, latest
        assert latest.last_serial_time > 0.0, latest
        return f"loop:// serial held stopped without pot feedback=({latest.motor1_byte},{latest.motor2_byte})"
    finally:
        sender.close()
        translator.shutdown()


def test_08_autostart_trigger_gui() -> str:
    app = app_instance()
    backup = CONFIG_PATH.read_text(encoding="utf-8") if CONFIG_PATH.exists() else None
    AUTOSTART_ENABLE_PATH.write_text("enable\n", encoding="utf-8")
    window = MainWindow()
    try:
        window.show()
        ok = {"done": False}

        def verify() -> None:
            telemetry = window.translator.get_telemetry()
            assert telemetry.enabled, telemetry
            assert not AUTOSTART_ENABLE_PATH.exists(), "autostart.enable persisted"
            ok["done"] = True
            window.close()
            app.quit()

        QTimer.singleShot(2600, verify)
        QTimer.singleShot(5000, app.quit)
        app.exec_()
        assert ok["done"], "autostart did not enable within timeout"
        return "autostart trigger deleted and GUI enabled"
    finally:
        window.translator.shutdown()
        window.close()
        if AUTOSTART_ENABLE_PATH.exists():
            AUTOSTART_ENABLE_PATH.unlink()
        if backup is not None:
            CONFIG_PATH.write_text(backup, encoding="utf-8")


def image_color_count(image: QImage, predicate: Callable[[QColor], bool]) -> int:
    count = 0
    for y in range(image.height()):
        for x in range(image.width()):
            color = QColor(image.pixel(x, y))
            if predicate(color):
                count += 1
    return count


def test_09_visualizer_render_colors_and_overlay() -> str:
    app = app_instance()
    assert app is not None
    canvas = RigVisualizerCanvas("SIDE VIEW - PITCH", "Pitch")
    canvas.resize(320, 220)
    canvas.set_pose(24.0, True)
    enabled_image = QImage(canvas.size(), QImage.Format_ARGB32)
    canvas.render(enabled_image)
    blue = image_color_count(enabled_image, lambda c: c.blue() > 150 and c.red() < 90)
    orange = image_color_count(enabled_image, lambda c: c.red() > 180 and 40 < c.green() < 140 and c.blue() < 80)
    assert blue > 10, f"blue attachment not visible enough: {blue}"
    assert orange > 10, f"orange attachment not visible enough: {orange}"

    canvas.set_pose(-18.0, False)
    disabled_image = QImage(canvas.size(), QImage.Format_ARGB32)
    canvas.render(disabled_image)
    gray_overlay = image_color_count(disabled_image, lambda c: abs(c.red() - c.green()) < 4 and abs(c.green() - c.blue()) < 4 and 90 < c.red() < 180)
    assert gray_overlay > 100, f"disabled overlay not visible enough: {gray_overlay}"
    assert canvas.angle_deg == -18.0 and not canvas.enabled
    return f"enabled attachments blue={blue}, orange={orange}; disabled gray pixels={gray_overlay}"


def test_10_config_recovery_and_sanitization() -> str:
    backup = CONFIG_PATH.read_text(encoding="utf-8") if CONFIG_PATH.exists() else None
    try:
        CONFIG_PATH.write_text('{"udp_port":999999,"baud_rate":"bad","deadband_mm":-5}', encoding="utf-8")
        from translator import load_or_create_config

        cfg = load_or_create_config()
        assert cfg["udp_port"] == 65535, cfg
        assert cfg["baud_rate"] == 9600, cfg
        assert cfg["deadband_mm"] == 0.0, cfg
        return f"sanitized udp={cfg['udp_port']} baud={cfg['baud_rate']} deadband={cfg['deadband_mm']}"
    finally:
        if backup is not None:
            CONFIG_PATH.write_text(backup, encoding="utf-8")


def main() -> int:
    tests: list[tuple[str, Callable[[], str]]] = [
        ("FlyPT/BeamNG profile availability", test_01_flypt_beamng_profile),
        ("Randomized math safety", test_02_randomized_math_safety),
        ("BeamNG-like serial output capture", test_03_beamng_like_serial_output_capture),
        ("UDP burst drop and recovery", test_04_udp_burst_drop_and_recovery),
        ("Malformed UDP recovery", test_05_malformed_udp_recovery),
        ("Disable writes stop bytes", test_06_disable_writes_stop_bytes),
        ("loop:// serial URL smoke", test_07_loop_serial_url_smoke),
        ("Autostart trigger GUI", test_08_autostart_trigger_gui),
        ("Visualizer render colors and overlay", test_09_visualizer_render_colors_and_overlay),
        ("Config recovery and sanitization", test_10_config_recovery_and_sanitization),
    ]
    print("Running deep motion workflow tests")
    results = [run_test(name, fn) for name, fn in tests]
    passed = sum(1 for result in results if result.passed)
    total = len(results)
    failed = total - passed
    print("")
    if failed:
        print(f"OVERALL FAILED: {passed}/{total} passed, {failed} failed")
        return 1
    print(f"OVERALL PASSED: {passed}/{total} passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
