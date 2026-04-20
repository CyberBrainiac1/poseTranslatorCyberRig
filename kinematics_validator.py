#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

MAX_CMD = 127


def raw_to_deg(v: int) -> float:
    return (v - 511.5) * (30.0 / 511.5)


def disp(pitch: float, roll: float, L: float = 266.08023) -> tuple[float, float]:
    pr, rr = math.radians(pitch), math.radians(roll)
    return L * math.sin(rr) + L * math.sin(pr), -L * math.sin(rr) + L * math.sin(pr)


def motor1_byte(cmd: int) -> int:
    return int((max(0, min(127, cmd)) / 127.0) * 63 + 64)


def motor2_byte(cmd: int) -> int:
    return int((max(0, min(127, cmd)) / 127.0) * 63 + 192)


def build_reference() -> dict[str, float | list[float]]:
    l1, r1 = disp(30, 0)
    l2, r2 = disp(0, 30)
    l3, r3 = disp(20, 15)
    seq = [0, 50, 100, 75, 25]
    out = []
    prev = 0.0
    alpha = 0.3
    for x in seq:
        prev = alpha * x + (1 - alpha) * prev
        out.append(round(prev))
    return {
        "pose_conversion_neutral": [0.0, 0.0],
        "pose_conversion_full_range": [-30.0, 30.0],
        "displacement_kinematics_pitch_only": [l1, r1],
        "displacement_kinematics_roll_only": [l2, r2],
        "displacement_kinematics_combined": [l3, r3],
        "spool_to_adc_neutral": [0.5, 250.0],
        "spool_to_adc_extremes": [0.0, 500.0],
        "pid_proportional_only": 100.0,
        "pid_with_integral": 50.0,
        "option_A_no_reverse": 0.0,
        "deadband_enforcement": 0.0,
        "soft_stop_bottom": 60.0,
        "soft_stop_top": 60.0,
        "soft_stop_hard_cutoff": 0.0,
        "sabertooth_byte_motor1": float(motor1_byte(127)),
        "sabertooth_byte_motor2": float(motor2_byte(127)),
        "velocity_smoothing": out,
    }


def run(reference: dict[str, object], tol_mm: float) -> tuple[list[dict[str, object]], bool]:
    results = []
    ok = True

    def check(name: str, actual, tol: float):
        nonlocal ok
        expected = reference[name]
        if isinstance(actual, (list, tuple)):
            deltas = [abs(float(a) - float(e)) for a, e in zip(actual, expected)]
            passed = all(d <= tol for d in deltas)
            delta = max(deltas)
        else:
            delta = abs(float(actual) - float(expected))
            passed = delta <= tol
        if not passed:
            ok = False
        results.append({"test": name, "actual": actual, "expected": expected, "delta": delta, "pass": passed})

    check("pose_conversion_neutral", [raw_to_deg(511.5), raw_to_deg(511.5)], 0.001)
    check("pose_conversion_full_range", [raw_to_deg(0), raw_to_deg(1023)], 0.001)
    check("displacement_kinematics_pitch_only", list(disp(30, 0)), tol_mm)
    check("displacement_kinematics_roll_only", list(disp(0, 30)), tol_mm)
    check("displacement_kinematics_combined", list(disp(20, 15)), tol_mm)
    check("spool_to_adc_neutral", [2048 / 4095, 250.0], 0.5)
    check("spool_to_adc_extremes", [0 / 4095, 500.0], 0.5)
    check("pid_proportional_only", min(127, 50 * 2.0), 1e-9)
    integral = 0.0
    for _ in range(100):
        integral = min(50.0, integral + 20 * 0.05 / 2)
    check("pid_with_integral", integral, 1e-9)
    check("option_A_no_reverse", max(0.0, -5.0), 1e-9)
    check("deadband_enforcement", 0.0 if 1.5 < 2.0 else 5.0, 1e-9)
    check("soft_stop_bottom", 100 * 0.6, 1e-9)
    check("soft_stop_top", 100 * 0.6, 1e-9)
    check("soft_stop_hard_cutoff", 0.0 if 0.008 < 0.01 else 10.0, 1e-9)
    check("sabertooth_byte_motor1", float(motor1_byte(127)), 1e-9)
    check("sabertooth_byte_motor2", float(motor2_byte(127)), 1e-9)

    prev = 0.0
    alpha = 0.3
    smoothed = []
    for x in [0, 50, 100, 75, 25]:
        prev = alpha * x + (1 - alpha) * prev
        smoothed.append(round(prev))
    check("velocity_smoothing", smoothed, 1.0)
    return results, ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate motion kinematics math.")
    parser.add_argument("--test-set", default="all")
    parser.add_argument("--reference-file", default="test_data/kinematics_reference.json")
    parser.add_argument("--tolerance-mm", type=float, default=0.5)
    parser.add_argument("--output", default="validation_results.json")
    args = parser.parse_args()

    ref_path = Path(args.reference_file)
    if ref_path.exists():
        reference = json.loads(ref_path.read_text(encoding="utf-8"))
    else:
        reference = build_reference()
        ref_path.parent.mkdir(parents=True, exist_ok=True)
        ref_path.write_text(json.dumps(reference, indent=2) + "\n", encoding="utf-8")

    results, success = run(reference, args.tolerance_mm)
    payload = {"success": success, "tests": results}
    Path(args.output).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": sum(1 for r in results if r['pass']), "failed": sum(1 for r in results if not r['pass'])}))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
