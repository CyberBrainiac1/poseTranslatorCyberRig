#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def clamp_int(v: float, lo: int = 0, hi: int = 1023) -> int:
    return max(lo, min(hi, int(round(v))))


def linear(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def scenario_values(name: str, t: float, duration: float) -> tuple[int, int]:
    if name == "acceleration":
        cycle = 10.0
        x = t % cycle
        if x < 2.5:
            p = linear(511, 800, x / 2.5)
        elif x < 5.0:
            p = 800
        elif x < 7.5:
            p = linear(800, 511, (x - 5.0) / 2.5)
        else:
            p = 511
        r = 511
    elif name == "lane_change_left":
        if t < 3.0:
            p, r = linear(600, 750, t / 3.0), linear(511, 200, t / 3.0)
        elif t < 5.0:
            p, r = 750, 200
        elif t < 8.0:
            p, r = linear(750, 600, (t - 5.0) / 3.0), linear(200, 511, (t - 5.0) / 3.0)
        else:
            p, r = 600, 511
    elif name == "lane_change_right":
        if t < 3.0:
            p, r = linear(600, 750, t / 3.0), linear(511, 800, t / 3.0)
        elif t < 5.0:
            p, r = 750, 800
        elif t < 8.0:
            p, r = linear(750, 600, (t - 5.0) / 3.0), linear(800, 511, (t - 5.0) / 3.0)
        else:
            p, r = 600, 511
    elif name == "hard_cornering":
        p = 600
        phase = int((t * 2.0) % 2)
        r = 300 if phase == 0 else 700
    elif name == "bumpy_road":
        import math

        p = 525 + 25 * math.sin(2 * math.pi * 10 * t)
        r = 511
    elif name == "braking_on_turn":
        p = linear(750, 400, min(t / 5.0, 1.0))
        r = 350
    elif name == "extreme_combined":
        frac = (t / max(duration, 1e-6)) % 1.0
        p = linear(0, 1023, frac)
        r = linear(1023, 0, frac)
    elif name == "rapid_oscillation":
        phase = int((t * 20.0) % 2)
        p = 400 if phase == 0 else 600
        r = 511
    else:
        raise ValueError(f"unknown scenario: {name}")
    return clamp_int(p), clamp_int(r)


def generate_frames(scenario: str, duration: float, frame_rate: int) -> list[dict[str, int]]:
    total = int(duration * frame_rate)
    frames: list[dict[str, int]] = []
    for i in range(total):
        t = i / frame_rate
        p, r = scenario_values(scenario, t, duration)
        frames.append({"time_ms": int(round(t * 1000.0)), "P": p, "R": r})
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate synthetic FlyPT pose sequences.")
    parser.add_argument("--scenario", default="acceleration")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--frame-rate", type=int, default=100)
    parser.add_argument("--output", default="test_data/pose_sequence.json")
    args = parser.parse_args()

    frames = generate_frames(args.scenario, args.duration, args.frame_rate)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"scenario": args.scenario, "frames": frames}
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(out_path), "frames": len(frames), "scenario": args.scenario}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
