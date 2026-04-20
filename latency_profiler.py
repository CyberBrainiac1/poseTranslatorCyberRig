#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import queue
import statistics
import threading
import time
from pathlib import Path


def percentile(data: list[float], pct: float) -> float:
    if not data:
        return 0.0
    idx = int(round((len(data) - 1) * pct))
    return sorted(data)[idx]


def main() -> int:
    parser = argparse.ArgumentParser(description="Profile simulated end-to-end latency")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--udp-rate", type=int, default=100)
    parser.add_argument("--serial-rate", type=int, default=100)
    parser.add_argument("--queue-max-depth", type=int, default=5)
    parser.add_argument("--output", default="latency_profile.json")
    args = parser.parse_args()

    q: queue.Queue[float] = queue.Queue(maxsize=max(1, args.queue_max_depth))
    stop = threading.Event()
    latencies: list[float] = []
    qdepth: list[int] = []
    dropped = 0
    processed = 0

    def udp_receiver() -> None:
        nonlocal dropped
        period = 1.0 / max(1, args.udp_rate)
        while not stop.is_set():
            t = time.perf_counter()
            try:
                q.put_nowait(t)
            except queue.Full:
                dropped += 1
                try:
                    q.get_nowait()
                    q.put_nowait(t)
                except Exception:
                    pass
            qdepth.append(q.qsize())
            time.sleep(period)

    def processor() -> None:
        nonlocal processed
        period = 1.0 / max(1, args.serial_rate)
        while not stop.is_set():
            try:
                created = q.get(timeout=0.05)
                latencies.append((time.perf_counter() - created) * 1000.0)
                processed += 1
            except queue.Empty:
                pass
            time.sleep(period)

    t1 = threading.Thread(target=udp_receiver, daemon=True)
    t2 = threading.Thread(target=processor, daemon=True)
    started = time.perf_counter()
    t1.start(); t2.start()
    time.sleep(args.duration)
    stop.set()
    t1.join(timeout=1)
    t2.join(timeout=1)
    elapsed = max(time.perf_counter() - started, 1e-6)

    payload = {
        "latency_mean_ms": statistics.mean(latencies) if latencies else 0.0,
        "latency_stdev_ms": statistics.pstdev(latencies) if len(latencies) > 1 else 0.0,
        "latency_min_ms": min(latencies) if latencies else 0.0,
        "latency_max_ms": max(latencies) if latencies else 0.0,
        "latency_p95_ms": percentile(latencies, 0.95),
        "latency_p99_ms": percentile(latencies, 0.99),
        "queue_depth_mean": statistics.mean(qdepth) if qdepth else 0.0,
        "queue_depth_max": max(qdepth) if qdepth else 0,
        "throughput_pps": processed / elapsed,
        "dropped_packets": dropped,
        "queue_overruns": dropped,
    }
    Path(args.output).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
