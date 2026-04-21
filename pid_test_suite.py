from __future__ import annotations

import math
import json
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from translator import (
    MAX_MOTOR_COMMAND,
    MotionMath,
    MotionTranslator,
    PotFeedback,
    ProcessorState,
    RigParameters,
    STOP_MOTOR_1_BYTE,
    STOP_MOTOR_2_BYTE,
)

ROOT = Path(__file__).resolve().parent


@dataclass
class Result:
    name: str
    passed: bool
    detail: str


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def adc_for_disp(disp_mm: float, params: RigParameters, side: str) -> int:
    if side == "left":
        lo, hi, travel = params.pot_min_adc_left, params.pot_max_adc_left, params.travel_range_mm_left
    else:
        lo, hi, travel = params.pot_min_adc_right, params.pot_max_adc_right, params.travel_range_mm_right
    pos = 0.0 if travel <= 0 else max(0.0, min(1.0, disp_mm / travel))
    return int(round(lo + (hi - lo) * pos))


def feedback_for(left_disp: float, right_disp: float, params: RigParameters, now: float) -> PotFeedback:
    return PotFeedback(adc_for_disp(left_disp, params, "left"), adc_for_disp(right_disp, params, "right"), now)


def run(name: str, fn: Callable[[], str]) -> Result:
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


def test_zero_error_neutral() -> str:
    params = RigParameters()
    state = ProcessorState()
    now = time.time()
    telemetry = MotionMath.process_packet("P=511,R=511", params, state, feedback_for(0.0, 0.0, params, now), now=now)
    assert abs(telemetry.left_error_mm) < 0.5
    assert abs(telemetry.right_error_mm) < 0.5
    assert telemetry.left_integral == 0.0 and telemetry.right_integral == 0.0
    assert telemetry.left_command == 0 and telemetry.right_command == 0
    assert telemetry.motor1_byte == STOP_MOTOR_1_BYTE and telemetry.motor2_byte == STOP_MOTOR_2_BYTE
    return f"errors=({telemetry.left_error_mm:.3f},{telemetry.right_error_mm:.3f}) commands=(0,0)"


def test_step_response_convergence() -> str:
    params = RigParameters(kd=0.0, alpha_smoothing=1.0)
    state = ProcessorState()
    pitch, roll = MotionMath.raw_to_degrees(700, 511, params)
    target_l, target_r = MotionMath.displacement_mm(pitch, roll, params)
    measured_l = measured_r = 0.0
    last = None
    for i in range(30):
        now = 1000.0 + i * 0.01
        last = MotionMath.process_packet("P=700,R=511", params, state, feedback_for(measured_l, measured_r, params, now), now=now)
        measured_l += (target_l - measured_l) * 0.22
        measured_r += (target_r - measured_r) * 0.22
    assert last is not None
    assert abs(target_l - measured_l) < 2.0
    assert abs(target_r - measured_r) < 2.0
    assert last.left_command >= 0 and last.right_command >= 0
    return f"final_error=({target_l-measured_l:.2f},{target_r-measured_r:.2f})"


def test_integral_windup_prevention() -> str:
    params = RigParameters(kd=0.0, alpha_smoothing=1.0)
    state = ProcessorState()
    for i in range(1000):
        MotionMath.pid_side(100.0, 0.0, 0.5, params, state, 0.01, "left")
        MotionMath.pid_side(100.0, 0.0, 0.5, params, state, 0.01, "right")
    assert abs(state.integral_left) <= params.integral_clamp
    assert abs(state.integral_right) <= params.integral_clamp
    assert state.integral_left == params.integral_clamp
    assert state.integral_right == params.integral_clamp
    return f"integrals=({state.integral_left:.3f},{state.integral_right:.3f})"


def test_derivative_spike_protection() -> str:
    params = RigParameters(alpha_smoothing=1.0)
    state = ProcessorState(prev_error_left=0.0)
    result = MotionMath.pid_side(100.0, 0.0, 0.5, params, state, 0.01, "left")
    assert math.isfinite(result.raw_output)
    assert result.command == MAX_MOTOR_COMMAND
    b1, _ = MotionMath.commands_to_serial_bytes(result.command, 0)
    assert b1 == 127
    return f"raw={result.raw_output:.1f} command={result.command}"


def test_option_a_constraint() -> str:
    params = RigParameters()
    state = ProcessorState(integral_left=10.0, integral_right=10.0)
    now = time.time()
    telemetry = MotionMath.process_packet("P=0,R=511", params, state, feedback_for(0.0, 0.0, params, now), now=now)
    assert telemetry.left_disp_mm < 0 and telemetry.right_disp_mm < 0
    assert telemetry.left_command == 0 and telemetry.right_command == 0
    assert state.integral_left == 0.0 and state.integral_right == 0.0
    assert telemetry.motor1_byte >= 64 and telemetry.motor2_byte >= 192
    return "negative targets produced stop commands and reset integral"


def test_soft_stop_zone_activation() -> str:
    scaled, soft, hard = MotionMath.apply_soft_hard_stop(100.0, 0.96, 0.05)
    assert soft and not hard
    assert abs(scaled - 80.0) < 1e-6
    return f"scaled={scaled:.1f}"


def test_hard_stop_override() -> str:
    params = RigParameters(alpha_smoothing=1.0)
    state = ProcessorState(integral_left=25.0)
    result = MotionMath.pid_side(100.0, 0.0, 0.005, params, state, 0.01, "left")
    assert result.command == 0
    assert result.hard_stop_active
    assert state.integral_left == 0.0
    return "hard stop forced zero and reset integral"


def test_pid_reset_on_disable_enable() -> str:
    translator = MotionTranslator(RigParameters(udp_port=free_udp_port()))
    try:
        translator.processor_state.integral_left = 12.0
        translator.processor_state.integral_right = -8.0
        translator.processor_state.prev_smoothed_left = 50.0
        translator.disable()
        assert translator.processor_state.integral_left == 0.0
        assert translator.processor_state.integral_right == 0.0
        assert translator.processor_state.prev_smoothed_left == 0.0
        translator.start()
        assert translator.processor_state.integral_left == 0.0
        assert translator.processor_state.prev_error_left == 0.0
        return "PID state reset across disable and enable"
    finally:
        translator.shutdown()


def test_asymmetric_left_right_response() -> str:
    params = RigParameters(alpha_smoothing=1.0)
    state = ProcessorState()
    now = time.time()
    telemetry = MotionMath.process_packet("P=511,R=800", params, state, now=now)
    assert telemetry.left_disp_mm > 0 and telemetry.right_disp_mm < 0
    assert telemetry.left_command > 0 and telemetry.right_command == 0
    return f"commands=({telemetry.left_command},{telemetry.right_command})"


def test_combined_pitch_roll_tracking() -> str:
    params = RigParameters(kd=0.0, alpha_smoothing=1.0)
    state = ProcessorState()
    pitch, roll = MotionMath.raw_to_degrees(750, 650, params)
    target_l, target_r = MotionMath.displacement_mm(pitch, roll, params)
    measured_l = measured_r = 0.0
    left_higher_count = 0
    for i in range(40):
        now = 3000.0 + i * 0.01
        telemetry = MotionMath.process_packet("P=750,R=650", params, state, feedback_for(measured_l, measured_r, params, now), now=now)
        if telemetry.left_command >= telemetry.right_command:
            left_higher_count += 1
        measured_l += (target_l - measured_l) * 0.2
        measured_r += (target_r - measured_r) * 0.2
    assert target_l > target_r > 0
    assert abs(target_l - measured_l) < 2.0
    assert abs(target_r - measured_r) < 2.0
    assert left_higher_count >= 30
    return f"targets=({target_l:.2f},{target_r:.2f}) final_error=({target_l-measured_l:.2f},{target_r-measured_r:.2f})"


def main() -> int:
    tests = [
        ("PID Test 1: Zero error at neutral", test_zero_error_neutral),
        ("PID Test 2: Step response convergence", test_step_response_convergence),
        ("PID Test 3: Integral windup prevention", test_integral_windup_prevention),
        ("PID Test 4: Derivative spike protection", test_derivative_spike_protection),
        ("PID Test 5: Option A constraint validation", test_option_a_constraint),
        ("PID Test 6: Soft stop zone activation", test_soft_stop_zone_activation),
        ("PID Test 7: Hard stop override", test_hard_stop_override),
        ("PID Test 8: PID reset on disable/enable", test_pid_reset_on_disable_enable),
        ("PID Test 9: Asymmetric left/right response", test_asymmetric_left_right_response),
        ("PID Test 10: Combined pitch and roll with PID tracking", test_combined_pitch_roll_tracking),
    ]
    results = [run(name, fn) for name, fn in tests]
    passed = sum(r.passed for r in results)
    total = len(results)
    failed = total - passed
    out = ROOT / "test_reports" / "pid_suite_results.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(
        json.dumps(
            {
                "summary": {"total": total, "passed": passed, "failed": failed},
                "tests": [r.__dict__ for r in results],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    if failed:
        print(f"PID SUITE FAILED: {passed}/{total} passed, {failed} failed")
        return 1
    print(f"PID SUITE PASSED: {passed}/{total} passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
