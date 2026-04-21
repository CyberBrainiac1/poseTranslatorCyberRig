from __future__ import annotations

import json
import math
import os
import random
import socket
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import deep_workflow_tests as deep
from translator import MotionMath, ProcessorState, RigParameters


ROOT = Path(__file__).resolve().parent


CUSTOM_TEST_IDEAS = [
    "C01 FlyPT BeamNG profile and executable availability",
    "C02 1000 randomized pose packet no-reverse safety",
    "C03 BeamNG-like sequence captured serial output",
    "C04 UDP burst queue drop/recovery",
    "C05 Malformed UDP recovery",
    "C06 Disable stop-byte enforcement",
    "C07 loop:// serial URL smoke",
    "C08 Autostart trigger GUI",
    "C09 Visualizer render color/overlay",
    "C10 Config recovery/sanitization",
    "C11 10000-frame deterministic processing stability",
    "C12 Strict parser whitespace rejection",
    "C13 Alternating motor dominance under rapid roll",
    "C14 Saturated full-forward hold safety",
    "C15 Negative displacement always zero command",
    "C16 Deadband boundary behavior",
    "C17 Rolling update-rate sanity",
    "C18 Parameter hotswap through update_params",
    "C19 Queue starvation low-rate sender",
    "C20 Numerical finite check over full raw grid",
]


def process_packet_with_primed_state(packet: str, dt: float = 0.01):
    params = RigParameters()
    state = ProcessorState(previous_left_disp=0.0, previous_right_disp=0.0, previous_time=time.time() - dt)
    return MotionMath.process_packet(packet, params, state)


def test_c01_profile_availability() -> None:
    assert "BeamNG" in deep.test_01_flypt_beamng_profile()


def test_c02_randomized_math_safety() -> None:
    assert "1000 randomized" in deep.test_02_randomized_math_safety()


def test_c03_beamng_like_serial_capture() -> None:
    assert "safe serial writes" in deep.test_03_beamng_like_serial_output_capture()


def test_c04_udp_burst_recovery() -> None:
    assert "processed burst" in deep.test_04_udp_burst_drop_and_recovery()


def test_c05_malformed_udp_recovery() -> None:
    assert "malformed packets" in deep.test_05_malformed_udp_recovery()


def test_c06_disable_stop_bytes() -> None:
    assert "disable tail writes" in deep.test_06_disable_writes_stop_bytes()


def test_c07_loop_serial_url() -> None:
    assert "loop:// serial" in deep.test_07_loop_serial_url_smoke()


def test_c08_autostart_gui() -> None:
    assert "autostart trigger" in deep.test_08_autostart_trigger_gui()


def test_c09_visualizer_render() -> None:
    assert "enabled attachments" in deep.test_09_visualizer_render_colors_and_overlay()


def test_c10_config_recovery() -> None:
    assert "sanitized" in deep.test_10_config_recovery_and_sanitization()


def test_c11_large_pose_sequence_10000_frames() -> None:
    params = RigParameters()
    state = ProcessorState()
    start = time.perf_counter()
    for i in range(10000):
        p = int(511.5 + 330 * math.sin(i * 0.013))
        r = int(511.5 + 290 * math.sin(i * 0.017 + 0.2))
        t = MotionMath.process_packet(f"P={max(0, min(1023, p))},R={max(0, min(1023, r))}", params, state)
        assert 64 <= t.motor1_byte <= 127
        assert 192 <= t.motor2_byte <= 255
    elapsed = time.perf_counter() - start
    assert elapsed < 5.0


@pytest.mark.parametrize("packet", ["P= 511, R= 511 ", " P =511,R=511", "P=511, R=511"])
def test_c12_parser_space_edge_cases_reject_cleanly(packet: str) -> None:
    with pytest.raises(ValueError):
        MotionMath.parse_flypt_packet(packet)


def test_c13_alternating_left_right_dominance() -> None:
    params = RigParameters()
    state = ProcessorState()
    left_dominant = 0
    right_dominant = 0
    for i in range(200):
        roll = 800 if i % 2 == 0 else 200
        t = MotionMath.process_packet(f"P=700,R={roll}", params, state)
        if t.motor1_byte > t.motor2_byte - 128:
            left_dominant += 1
        if t.motor2_byte - 128 > t.motor1_byte:
            right_dominant += 1
    assert left_dominant > 20
    assert right_dominant > 20


def test_c14_saturated_full_forward_hold() -> None:
    first = process_packet_with_primed_state("P=1023,R=511")
    assert first.left_command > 0 and first.right_command > 0
    params = RigParameters()
    state = ProcessorState()
    for _ in range(500):
        t = MotionMath.process_packet("P=1023,R=511", params, state)
        assert 64 <= t.motor1_byte <= 127
        assert 192 <= t.motor2_byte <= 255


def test_c15_negative_displacement_zero_command() -> None:
    out = process_packet_with_primed_state("P=0,R=511")
    assert out.left_disp_mm < 0 and out.right_disp_mm < 0
    assert out.left_command == 0 and out.right_command == 0


def test_c16_deadband_boundary_behavior() -> None:
    params = RigParameters(deadband_mm=2.0)
    now = time.time()
    state = ProcessorState(previous_left_disp=0.0, previous_right_disp=0.0, previous_time=now - 0.01)
    left, right = MotionMath.speed_commands(1.999, 2.0, params, state, now)
    assert left == 0 and right == 0
    left, right = MotionMath.speed_commands(2.001, 2.001, params, state, now + 0.01)
    assert left >= 0 and right >= 0


def test_c17_update_rate_rolling_average() -> None:
    params = RigParameters()
    state = ProcessorState()
    for i in range(40):
        MotionMath.process_packet(f"P={511+i%5},R=511", params, state)
        time.sleep(0.002)
    t = MotionMath.process_packet("P=520,R=520", params, state)
    assert t.update_rate_hz > 50


def test_c18_parameter_hotswap_runtime_object() -> None:
    translator = deep.CaptureTranslator(RigParameters(udp_port=deep.free_udp_port(), deadband_mm=2.0))
    translator.update_params(RigParameters(udp_port=translator.get_params().udp_port, deadband_mm=100.0))
    assert translator.get_params().deadband_mm == 100.0


def test_c19_queue_starvation_low_rate_sender() -> None:
    params = RigParameters(udp_port=deep.free_udp_port())
    translator = deep.CaptureTranslator(params)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        translator.start()
        for _ in range(10):
            sender.sendto(b"P=512,R=512\r\n", ("127.0.0.1", params.udp_port))
            time.sleep(0.1)
        assert translator.packet_queue.qsize() <= 1
        assert len(translator.writes) >= 5
    finally:
        sender.close()
        translator.shutdown()


def test_c20_full_raw_grid_finite_and_clamped() -> None:
    params = RigParameters()
    state = ProcessorState()
    for p in range(0, 1024, 31):
        for r in range(0, 1024, 29):
            t = MotionMath.process_packet(f"P={p},R={r}", params, state)
            assert math.isfinite(t.pitch_deg)
            assert math.isfinite(t.roll_deg)
            assert math.isfinite(t.left_disp_mm)
            assert math.isfinite(t.right_disp_mm)
            assert 64 <= t.motor1_byte <= 127
            assert 192 <= t.motor2_byte <= 255


def test_custom_ideas_documented() -> None:
    out = ROOT / "test_reports" / "custom_stress_ideas.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"ideas": CUSTOM_TEST_IDEAS}, indent=2) + "\n", encoding="utf-8")
    assert len(CUSTOM_TEST_IDEAS) == 20
