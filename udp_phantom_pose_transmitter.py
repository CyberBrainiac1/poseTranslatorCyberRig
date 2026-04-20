#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path


def load_frames(path: Path) -> list[dict[str, int]]:
    obj = json.loads(path.read_text(encoding="utf-8"))
    frames = obj.get("frames", [])
    if not isinstance(frames, list) or not frames:
        raise ValueError("pose file must contain non-empty frames list")
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description="Simulate FlyPT Mover UDP packets.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--rate", type=int, default=100)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--pose-file", required=True)
    parser.add_argument("--output-log")
    args = parser.parse_args()

    frames = load_frames(Path(args.pose_file))
    logs: list[dict[str, object]] = []
    warnings = 0
    errors = 0
    sent = 0
    retries = 0
    frame_interval = 1.0 / max(1, min(500, args.rate))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.perf_counter()
    stop_time = start + args.duration

    try:
        for idx, frame in enumerate(frames):
            frame_due = start + frame.get("time_ms", int(idx * frame_interval * 1000)) / 1000.0
            if frame_due > stop_time:
                break
            now = time.perf_counter()
            if frame_due > now:
                time.sleep(frame_due - now)
            else:
                slip_ms = (now - frame_due) * 1000.0
                if slip_ms > 5.0:
                    warnings += 1
                    logs.append({"type": "warning", "message": "timing_slip", "slip_ms": slip_ms, "frame": idx})

            payload = f"P={int(frame['P'])},R={int(frame['R'])}\n".encode("ascii")
            success = False
            last_error = None
            for attempt in range(4):
                try:
                    sock.sendto(payload, (args.host, args.port))
                    success = True
                    break
                except OSError as exc:
                    last_error = str(exc)
                    errors += 1
                    if attempt < 3:
                        retries += 1
                        time.sleep(0.1)
            logs.append(
                {
                    "frame": idx,
                    "packet": payload.decode("ascii").strip(),
                    "actual_send_timestamp": time.time(),
                    "success": success,
                    "error": last_error,
                }
            )
            if success:
                sent += 1
            else:
                if retries >= 3:
                    summary = {
                        "packets_sent": sent,
                        "frame_count": idx + 1,
                        "duration_s": time.perf_counter() - start,
                        "dropped_frames": idx + 1 - sent,
                        "warnings": warnings,
                        "errors": errors,
                        "retries": retries,
                    }
                    print(json.dumps(summary))
                    return 1
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    summary = {
        "packets_sent": sent,
        "frame_count": len(logs),
        "duration_s": time.perf_counter() - start,
        "dropped_frames": len(logs) - sent,
        "warnings": warnings,
        "errors": errors,
        "retries": retries,
    }
    out = {"summary": summary, "events": logs}
    if args.output_log:
        Path(args.output_log).write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    else:
        print(json.dumps(out))
    print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
