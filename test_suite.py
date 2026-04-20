from __future__ import annotations

import socket
import sys
import time
from dataclasses import dataclass
from typing import Callable

from translator import (
    MAX_MOTOR_COMMAND,
    MotionMath,
    MotionTranslator,
    ProcessorState,
    RigParameters,
    STOP_MOTOR_1_BYTE,
    STOP_MOTOR_2_BYTE,
    Telemetry,
)


@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str


@dataclass
class PoseOutput:
    packet: str
    pitch_deg: float
    roll_deg: float
    left_disp: float
    right_disp: float
    left_command: int
    right_command: int
    motor1_byte: int
    motor2_byte: int


class MockSerialTranslator(MotionTranslator):
    def __init__(self, params: RigParameters) -> None:
        super().__init__(params)
        self.writes: list[tuple[int, int]] = []

    def open_serial(self) -> None:
        self.set_serial_status(True)
        self.set_error("")

    def close_serial(self) -> None:
        self.set_serial_status(False)

    def _write_motor_bytes(self, motor1_byte: int, motor2_byte: int) -> bool:
        safe_motor1 = max(64, min(127, int(motor1_byte)))
        safe_motor2 = max(192, min(255, int(motor2_byte)))
        self.writes.append((safe_motor1, safe_motor2))
        with self.telemetry_lock:
            self.telemetry.last_serial_time = time.time()
            self.telemetry.serial_connected = True
        return True


def pose_packet(pitch_raw: int, roll_raw: int) -> str:
    return f"P={pitch_raw},R={roll_raw}"


def compute_pose(packet: str, dt: float = 0.01) -> PoseOutput:
    params = RigParameters()
    state = ProcessorState()
    pitch_raw, roll_raw = MotionMath.parse_flypt_packet(packet)
    pitch_deg, roll_deg = MotionMath.raw_to_degrees(pitch_raw, roll_raw, params)
    left_disp, right_disp = MotionMath.displacement_mm(pitch_deg, roll_deg, params)
    now = time.time()
    state.previous_left_disp = 0.0
    state.previous_right_disp = 0.0
    state.previous_time = now - dt
    left_command, right_command = MotionMath.speed_commands(left_disp, right_disp, params, state, now)
    motor1_byte, motor2_byte = MotionMath.commands_to_serial_bytes(left_command, right_command)
    return PoseOutput(
        packet=packet,
        pitch_deg=pitch_deg,
        roll_deg=roll_deg,
        left_disp=left_disp,
        right_disp=right_disp,
        left_command=left_command,
        right_command=right_command,
        motor1_byte=motor1_byte,
        motor2_byte=motor2_byte,
    )


def run_check(name: str, check: Callable[[], str]) -> TestResult:
    try:
        detail = check()
        print(f"PASS - {name}: {detail}")
        return TestResult(name, True, detail)
    except AssertionError as exc:
        detail = str(exc) or "assertion failed"
        print(f"FAIL - {name}: {detail}")
        return TestResult(name, False, detail)
    except Exception as exc:
        detail = f"unexpected error: {exc}"
        print(f"FAIL - {name}: {detail}")
        return TestResult(name, False, detail)


def assert_between(value: float, low: float, high: float, label: str) -> None:
    assert low <= value <= high, f"{label}={value:.3f}, expected {low:.3f}..{high:.3f}"


def assert_motor_byte_ranges(output: PoseOutput) -> None:
    assert output.motor1_byte >= STOP_MOTOR_1_BYTE, f"motor1 below stop range: {output.motor1_byte}"
    assert output.motor2_byte >= STOP_MOTOR_2_BYTE, f"motor2 below stop range: {output.motor2_byte}"
    assert output.motor1_byte <= 127, f"motor1 above forward range: {output.motor1_byte}"
    assert output.motor2_byte <= 255, f"motor2 above forward range: {output.motor2_byte}"


def unit_tests() -> list[TestResult]:
    tests: list[tuple[str, Callable[[], str]]] = []

    def flat_neutral() -> str:
        out = compute_pose(pose_packet(511, 511))
        assert abs(out.left_disp) < 0.5, f"left displacement not near zero: {out.left_disp:.3f}"
        assert abs(out.right_disp) < 0.5, f"right displacement not near zero: {out.right_disp:.3f}"
        assert out.left_command == 0, f"left command expected 0, got {out.left_command}"
        assert out.right_command == 0, f"right command expected 0, got {out.right_command}"
        assert_motor_byte_ranges(out)
        return f"bytes=({out.motor1_byte},{out.motor2_byte}) disp=({out.left_disp:.3f},{out.right_disp:.3f})"

    def full_pitch_forward() -> str:
        out = compute_pose(pose_packet(1023, 511))
        assert out.left_disp > 120.0, f"left displacement not strongly positive: {out.left_disp:.3f}"
        assert out.right_disp > 120.0, f"right displacement not strongly positive: {out.right_disp:.3f}"
        assert out.left_command >= 120, f"left command not near 127: {out.left_command}"
        assert out.right_command >= 120, f"right command not near 127: {out.right_command}"
        assert_motor_byte_ranges(out)
        return f"bytes=({out.motor1_byte},{out.motor2_byte}) cmd=({out.left_command},{out.right_command})"

    def full_pitch_back() -> str:
        out = compute_pose(pose_packet(0, 511))
        assert out.left_disp < -120.0, f"left displacement not strongly negative: {out.left_disp:.3f}"
        assert out.right_disp < -120.0, f"right displacement not strongly negative: {out.right_disp:.3f}"
        assert out.left_command == 0, f"left command expected 0, got {out.left_command}"
        assert out.right_command == 0, f"right command expected 0, got {out.right_command}"
        assert_motor_byte_ranges(out)
        return f"bytes=({out.motor1_byte},{out.motor2_byte}) disp=({out.left_disp:.3f},{out.right_disp:.3f})"

    def full_roll_right() -> str:
        out = compute_pose(pose_packet(511, 1023))
        assert out.left_disp > 120.0, f"left displacement not positive: {out.left_disp:.3f}"
        assert out.right_disp < -120.0, f"right displacement not negative: {out.right_disp:.3f}"
        assert out.left_command > 0, f"left command expected nonzero, got {out.left_command}"
        assert out.right_command == 0, f"right command expected 0, got {out.right_command}"
        assert_motor_byte_ranges(out)
        return f"bytes=({out.motor1_byte},{out.motor2_byte}) cmd=({out.left_command},{out.right_command})"

    def full_roll_left() -> str:
        out = compute_pose(pose_packet(511, 0))
        assert out.left_disp < -120.0, f"left displacement not negative: {out.left_disp:.3f}"
        assert out.right_disp > 120.0, f"right displacement not positive: {out.right_disp:.3f}"
        assert out.left_command == 0, f"left command expected 0, got {out.left_command}"
        assert out.right_command > 0, f"right command expected nonzero, got {out.right_command}"
        assert_motor_byte_ranges(out)
        return f"bytes=({out.motor1_byte},{out.motor2_byte}) cmd=({out.left_command},{out.right_command})"

    def combined_forward_roll_right() -> str:
        out = compute_pose(pose_packet(900, 900))
        assert out.left_disp > 190.0, f"left displacement expected large positive, got {out.left_disp:.3f}"
        assert abs(out.right_disp) < 0.001, (
            "right displacement is mathematically expected to cancel to zero for P=900,R=900 "
            f"with equal pitch/roll limits, got {out.right_disp:.6f}"
        )
        assert out.left_command > out.right_command, (
            f"left command expected higher after cancellation, got {out.left_command}>{out.right_command}"
        )
        assert out.right_command == 0, f"right command expected 0 after cancellation, got {out.right_command}"
        assert_motor_byte_ranges(out)
        return f"bytes=({out.motor1_byte},{out.motor2_byte}) cmd=({out.left_command},{out.right_command})"

    def combined_back_roll_left() -> str:
        out = compute_pose(pose_packet(100, 100))
        assert out.left_disp < -190.0, f"left displacement expected negative, got {out.left_disp:.3f}"
        assert abs(out.right_disp) < 0.001, (
            "right displacement is mathematically expected to cancel to zero for P=100,R=100 "
            f"with equal pitch/roll limits, got {out.right_disp:.6f}"
        )
        assert out.left_command == 0, f"left command expected 0, got {out.left_command}"
        assert out.right_command == 0, f"right command expected 0, got {out.right_command}"
        assert_motor_byte_ranges(out)
        return f"bytes=({out.motor1_byte},{out.motor2_byte}) disp=({out.left_disp:.3f},{out.right_disp:.3f})"

    def max_byte_range_check() -> str:
        packets = [
            pose_packet(511, 511),
            pose_packet(1023, 511),
            pose_packet(0, 511),
            pose_packet(511, 1023),
            pose_packet(511, 0),
            pose_packet(900, 900),
            pose_packet(100, 100),
        ]
        outputs = [compute_pose(packet) for packet in packets]
        for out in outputs:
            assert_motor_byte_ranges(out)
        min_motor1 = min(out.motor1_byte for out in outputs)
        min_motor2 = min(out.motor2_byte for out in outputs)
        return f"min bytes=({min_motor1},{min_motor2})"

    def deadband_check() -> str:
        params = RigParameters()
        state = ProcessorState(previous_left_disp=0.0, previous_right_disp=0.0, previous_time=time.time() - 0.001)
        left_command, right_command = MotionMath.speed_commands(1.5, 1.5, params, state, time.time())
        assert left_command == 0, f"left command expected 0 below deadband, got {left_command}"
        assert right_command == 0, f"right command expected 0 below deadband, got {right_command}"
        return f"commands=({left_command},{right_command})"

    tests.extend(
        [
            ("Flat neutral", flat_neutral),
            ("Full pitch forward", full_pitch_forward),
            ("Full pitch back", full_pitch_back),
            ("Full roll right", full_roll_right),
            ("Full roll left", full_roll_left),
            ("Combined pitch forward and roll right", combined_forward_roll_right),
            ("Combined pitch back and roll left", combined_back_roll_left),
            ("Max possible displacement byte range", max_byte_range_check),
            ("Deadband check", deadband_check),
        ]
    )
    return [run_check(name, check) for name, check in tests]


def wait_for_packet(translator: MockSerialTranslator, packet: str, timeout_s: float = 2.0) -> Telemetry:
    deadline = time.time() + timeout_s
    expected_pitch, expected_roll = MotionMath.parse_flypt_packet(packet)
    params = translator.get_params()
    expected_pitch_deg, expected_roll_deg = MotionMath.raw_to_degrees(expected_pitch, expected_roll, params)
    latest = translator.get_telemetry()
    while time.time() < deadline:
        latest = translator.get_telemetry()
        if abs(latest.pitch_deg - expected_pitch_deg) < 0.001 and abs(latest.roll_deg - expected_roll_deg) < 0.001:
            return latest
        time.sleep(0.02)
    return latest


def phantom_udp_loop_test() -> list[TestResult]:
    packets = [
        pose_packet(511, 511),
        pose_packet(1023, 511),
        pose_packet(0, 511),
        pose_packet(511, 1023),
        pose_packet(511, 0),
        pose_packet(900, 900),
        pose_packet(100, 100),
    ]

    params = RigParameters(udp_port=9000, serial_port="", baud_rate=9600)
    translator = MockSerialTranslator(params)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    results: list[TestResult] = []

    def run_loop() -> str:
        translator.start()
        try:
            assert translator.udp_thread is not None, "UDP thread did not start"
            time.sleep(0.2)
            for packet in packets:
                expected = compute_pose(packet)
                print(
                    f"UDP SEND {packet} expected static bytes=({expected.motor1_byte},{expected.motor2_byte}) "
                    f"cmd=({expected.left_command},{expected.right_command})"
                )
                sender.sendto((packet + "\r\n").encode("ascii"), ("127.0.0.1", params.udp_port))
                latest = wait_for_packet(translator, packet)
                assert latest.udp_connected, f"UDP not connected after {packet}"
                assert latest.motor1_byte >= STOP_MOTOR_1_BYTE, f"motor1 below range after {packet}: {latest.motor1_byte}"
                assert latest.motor2_byte >= STOP_MOTOR_2_BYTE, f"motor2 below range after {packet}: {latest.motor2_byte}"
                assert 0 <= latest.left_command <= MAX_MOTOR_COMMAND, f"left command out of range after {packet}"
                assert 0 <= latest.right_command <= MAX_MOTOR_COMMAND, f"right command out of range after {packet}"
                time.sleep(1.0)
        finally:
            translator.shutdown()
            sender.close()
        return f"sent {len(packets)} packets to 127.0.0.1:{params.udp_port}; mocked writes={len(translator.writes)}"

    results.append(run_check("Phantom UDP loop with mocked serial", run_loop))
    return results


def main() -> int:
    print("Running standalone motion translator test suite")
    results = unit_tests()
    results.extend(phantom_udp_loop_test())
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
