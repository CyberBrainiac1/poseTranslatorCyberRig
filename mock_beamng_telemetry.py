from __future__ import annotations

import argparse
import json
import math
import socket
import time
from pathlib import Path


def clamp_raw(value: float) -> int:
    return max(0, min(1023, int(round(value))))


def telemetry_at(t: float, duration: float) -> dict[str, float | int]:
    lap = t / max(duration, 1e-6)
    accel = max(0.0, math.sin(2 * math.pi * lap * 7.0))
    brake = max(0.0, math.sin(2 * math.pi * lap * 7.0 + math.pi))
    steering = math.sin(2 * math.pi * lap * 11.0) * (0.35 + 0.65 * math.sin(2 * math.pi * lap * 3.0) ** 2)
    speed = 45.0 + 95.0 * max(0.0, math.sin(math.pi * lap))
    bump = 0.08 * math.sin(2 * math.pi * lap * 90.0)
    pitch_norm = 0.75 * accel - 0.45 * brake + bump
    roll_norm = 0.9 * steering
    p = clamp_raw(511.5 + pitch_norm * 380.0)
    r = clamp_raw(511.5 + roll_norm * 420.0)
    return {
        "time_s": t,
        "accel_pedal": accel,
        "brake_pedal": brake,
        "steering_angle": steering,
        "vehicle_speed": speed,
        "bump": bump,
        "P": p,
        "R": r,
    }


def generate(duration: float = 120.0, rate: int = 100) -> list[dict[str, float | int]]:
    return [telemetry_at(i / rate, duration) for i in range(int(duration * rate))]


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate/send mock BeamNG telemetry converted to FlyPT P/R packets.")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--rate", type=int, default=100)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--send", action="store_true")
    parser.add_argument("--output", default="test_logs/mock_beamng_telemetry.json")
    args = parser.parse_args()

    frames = generate(args.duration, args.rate)
    out_path = Path(args.output)
    out_path.parent.mkdir(exist_ok=True)
    out_path.write_text(json.dumps({"scenario": "Monaco Grand Prix lap", "rate_hz": args.rate, "frames": frames}, indent=2) + "\n", encoding="utf-8")

    sent = 0
    if args.send:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            period = 1.0 / max(1, args.rate)
            for frame in frames:
                sock.sendto(f"P={frame['P']},R={frame['R']}\r\n".encode("ascii"), (args.host, args.port))
                sent += 1
                time.sleep(period)
        finally:
            sock.close()

    print(json.dumps({"output": str(out_path), "frames": len(frames), "sent": sent}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
