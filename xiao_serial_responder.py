#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import random
import select
import socket
import sys
import time
from pathlib import Path


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def decode(byte1: int, byte2: int) -> tuple[float, float]:
    left = (byte1 - 64) / 63.0 * 127.0
    right = (byte2 - 192) / 63.0 * 127.0
    return left, right


def to_adc(disp: float, sigma: float, rng: random.Random) -> int:
    pot_min, pot_max = 100, 3900
    travel = 500.0
    norm = clamp(disp / travel, 0.0, 1.0)
    adc = pot_min + (pot_max - pot_min) * norm + rng.gauss(0.0, sigma)
    return int(clamp(round(adc), 0, 4095))


def main() -> int:
    parser = argparse.ArgumentParser(description="Simulated XIAO serial responder")
    parser.add_argument("--mode", default="pty", choices=["pty", "tcp"])
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--motor-inertia", type=float, default=10.0)
    parser.add_argument("--pot-noise-sigma", type=float, default=5.0)
    parser.add_argument("--pot-loss-at-frame", type=int, default=0)
    parser.add_argument("--watchdog-timeout-ms", type=int, default=500)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--output-log")
    args = parser.parse_args()

    if args.mode != "tcp":
        print("pty mode unsupported in cloud; use --mode tcp", file=sys.stderr)
        return 1

    rng = random.Random(12345)
    log: list[dict[str, object]] = []
    left_disp = 250.0
    right_disp = 250.0
    target_left = 250.0
    target_right = 250.0
    commands = 0
    errors = 0
    watchdogs = 0
    tau = max(args.motor_inertia, 1.0)
    alpha = clamp(10.0 / tau, 0.05, 0.9)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    server.settimeout(0.5)
    print("XIAO_READY", flush=True)

    conn = None
    last_cmd_t = time.perf_counter()

    try:
        while conn is None:
            try:
                conn, addr = server.accept()
                conn.settimeout(0.1)
                log.append({"type": "connect", "addr": addr, "ts": time.time()})
            except socket.timeout:
                continue

        while True:
            now = time.perf_counter()
            if (now - last_cmd_t) * 1000.0 > args.watchdog_timeout_ms:
                watchdogs += 1
                target_left = 0.0
                target_right = 0.0
                last_cmd_t = now
                log.append({"type": "watchdog", "ts": time.time()})

            try:
                chunk = conn.recv(2)
                if not chunk:
                    break
                if len(chunk) < 2:
                    continue
                b1, b2 = chunk[0], chunk[1]
                orig_b1, orig_b2 = b1, b2
                if b1 < 64 or b1 > 127:
                    b1 = int(clamp(b1, 64, 127))
                    errors += 1
                if b2 < 192 or b2 > 255:
                    b2 = int(clamp(b2, 192, 255))
                    errors += 1
                left_cmd, right_cmd = decode(b1, b2)
                target_left = left_cmd / 127.0 * 500.0
                target_right = right_cmd / 127.0 * 500.0
                left_disp += (target_left - left_disp) * alpha
                right_disp += (target_right - right_disp) * alpha
                commands += 1
                last_cmd_t = time.perf_counter()

                if args.pot_loss_at_frame and commands >= args.pot_loss_at_frame:
                    adc_l, adc_r = 0, 4095
                    resp = "ERR=POT_L\n"
                else:
                    adc_l = to_adc(left_disp, args.pot_noise_sigma, rng)
                    adc_r = to_adc(right_disp, args.pot_noise_sigma, rng)
                    resp = f"L={adc_l},R={adc_r}\n"
                conn.sendall(resp.encode("ascii"))
                log.append(
                    {
                        "ts": time.time(),
                        "command": [orig_b1, orig_b2],
                        "clamped": [b1, b2],
                        "adc": [adc_l, adc_r],
                        "response": resp.strip(),
                    }
                )
            except socket.timeout:
                continue
    except KeyboardInterrupt:
        pass
    finally:
        if conn:
            conn.close()
        server.close()

    summary = {
        "commands_received": commands,
        "adc_sent": commands,
        "errors_triggered": errors,
        "watchdog_triggers": watchdogs,
        "final_position": [left_disp, right_disp],
    }
    payload = {"summary": summary, "events": log}
    if args.output_log:
        Path(args.output_log).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
