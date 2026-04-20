#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import random
import socket
import time
from pathlib import Path


def udp_payloads(fuzz_type: str, count: int) -> list[bytes]:
    presets = {
        "malformed": [b"JUNK", b"P=511", b"R=511\nGARBAGE", b"P=,R=\n"],
        "oversized": [b"P=999999,R=999999\n"],
        "undersized": [b"P=0,R=0\n"],
        "negative": [b"P=-50,R=200\n"],
        "null_bytes": [b"P=511\x00R=511\n"],
        "whitespace": [b"P = 511 , R = 511\n"],
        "rapid_fire": [b"P=511,R=511\n"],
        "duplicate": [b"P=512,R=512\n"],
        "boundary": [f"P={p},R={r}\n".encode("ascii") for p in (0, 1, 511, 1022, 1023) for r in (0, 1, 511, 1022, 1023)],
    }
    keys = list(presets) if fuzz_type == "all" else [fuzz_type]
    out: list[bytes] = []
    while len(out) < count:
        for k in keys:
            for item in presets[k]:
                out.append(item)
                if len(out) >= count:
                    break
            if len(out) >= count:
                break
    return out


def serial_payloads(fuzz_type: str, count: int) -> list[bytes]:
    rng = random.Random(7)
    out: list[bytes] = []
    while len(out) < count:
        if fuzz_type in ("all", "byte_sequence"):
            out.append(bytes([rng.randrange(0, 256), rng.randrange(0, 256)]))
        if fuzz_type in ("all", "out_of_range"):
            out.append(bytes([200, 100]))
        if fuzz_type in ("all", "repeated_bytes"):
            out.append(bytes([64, 64]))
        if fuzz_type in ("all", "single_byte"):
            out.append(bytes([64]))
        if fuzz_type in ("all", "three_bytes"):
            out.append(bytes([64, 192, 255]))
        if fuzz_type in ("all", "timing"):
            out.append(bytes([127, 255]))
    return out[:count]


def main() -> int:
    parser = argparse.ArgumentParser(description="Fuzz UDP/serial protocols")
    parser.add_argument("--protocol", default="udp", choices=["udp", "serial"])
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--device", default="localhost:5000")
    parser.add_argument("--fuzz-type", default="all")
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--output", default="fuzz_log.json")
    args = parser.parse_args()

    results = []
    crashes = 0

    if args.protocol == "udp":
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payloads = udp_payloads(args.fuzz_type, args.count)
        for idx, payload in enumerate(payloads):
            try:
                sock.sendto(payload, (args.host, args.port))
                status = "success"
            except Exception as exc:
                status = f"error:{exc}"
                crashes += 1
            results.append({"idx": idx, "payload": repr(payload), "result": status, "ts": time.time()})
        sock.close()
    else:
        host, port_str = args.device.split(":")
        payloads = serial_payloads(args.fuzz_type, args.count)
        with socket.create_connection((host, int(port_str)), timeout=2.0) as conn:
            conn.settimeout(0.2)
            for idx, payload in enumerate(payloads):
                try:
                    conn.sendall(payload)
                    if args.fuzz_type == "timing":
                        time.sleep(1.05)
                    resp = conn.recv(128)
                    status = "success" if resp else "no_response"
                except Exception as exc:
                    status = f"error:{exc}"
                    crashes += 1
                results.append({"idx": idx, "payload": list(payload), "result": status, "ts": time.time()})

    summary = {
        "attempts": len(results),
        "crashes": crashes,
        "crash_rate": 0.0 if not results else crashes / len(results),
    }
    out = {"summary": summary, "attempts": results}
    Path(args.output).write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
