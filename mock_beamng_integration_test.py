from __future__ import annotations

import json
import math
import time
from pathlib import Path

from mock_beamng_telemetry import generate
from translator import MotionMath, ProcessorState, RigParameters


ROOT = Path(__file__).resolve().parent
LOG_PATH = ROOT / "test_logs" / "mock_beamng_telemetry.json"
REPORT_PATH = ROOT / "test_reports" / "mock_beamng_report.json"


def main() -> int:
    frames = generate(duration=120.0, rate=100)
    LOG_PATH.parent.mkdir(exist_ok=True)
    REPORT_PATH.parent.mkdir(exist_ok=True)
    LOG_PATH.write_text(json.dumps({"scenario": "Monaco Grand Prix lap", "rate_hz": 100, "frames": frames}, indent=2) + "\n", encoding="utf-8")

    params = RigParameters()
    state = ProcessorState()
    outputs = []
    start = time.perf_counter()
    for frame in frames:
        t = MotionMath.process_packet(f"P={frame['P']},R={frame['R']}", params, state)
        assert 64 <= t.motor1_byte <= 127
        assert 192 <= t.motor2_byte <= 255
        assert math.isfinite(t.left_disp_mm) and math.isfinite(t.right_disp_mm)
        if t.left_disp_mm <= params.deadband_mm:
            assert t.left_command == 0
        if t.right_disp_mm <= params.deadband_mm:
            assert t.right_command == 0
        outputs.append(
            {
                "time_s": frame["time_s"],
                "P": frame["P"],
                "R": frame["R"],
                "motor1_byte": t.motor1_byte,
                "motor2_byte": t.motor2_byte,
                "left_command": t.left_command,
                "right_command": t.right_command,
                "pitch_deg": t.pitch_deg,
                "roll_deg": t.roll_deg,
            }
        )
    elapsed = time.perf_counter() - start
    left_active = sum(1 for o in outputs if o["motor1_byte"] > 64)
    right_active = sum(1 for o in outputs if o["motor2_byte"] > 192)
    report = {
        "passed": True,
        "scenario": "Monaco Grand Prix lap",
        "frames": len(frames),
        "processed_seconds": 120.0,
        "compute_elapsed_s": elapsed,
        "left_active_frames": left_active,
        "right_active_frames": right_active,
        "max_motor1_byte": max(o["motor1_byte"] for o in outputs),
        "max_motor2_byte": max(o["motor2_byte"] for o in outputs),
        "final_output": outputs[-1],
        "sample_outputs": outputs[:: max(1, len(outputs) // 50)],
    }
    assert left_active > 100
    assert right_active > 100
    REPORT_PATH.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("MOCK BEAMNG TEST PASSED")
    print(json.dumps({"frames": len(frames), "elapsed_s": elapsed, "report": str(REPORT_PATH)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
