from __future__ import annotations

import re
import socket
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore
except Exception:  # pragma: no cover
    serial = None


NUM_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
CSV_RE = re.compile(rf"^\s*({NUM_RE})\s*,\s*({NUM_RE})\s*$")
TAG_RE = re.compile(rf"P\s*=\s*({NUM_RE})\s*,\s*R\s*=\s*({NUM_RE})", re.IGNORECASE)


def parse_pitch_roll_line(line: str) -> Optional[Tuple[float, float]]:
    line = line.strip()
    if not line:
        return None
    m = CSV_RE.match(line)
    if m:
        return float(m.group(1)), float(m.group(2))
    m = TAG_RE.search(line)
    if m:
        return float(m.group(1)), float(m.group(2))
    return None


def list_serial_ports() -> list[str]:
    if serial is None:
        return []
    return ["loop://", *[p.device for p in serial.tools.list_ports.comports()]]


@dataclass
class SerialInput:
    port: str
    baud: int = 115200
    timeout: float = 0.01

    def __post_init__(self) -> None:
        self._ser = None

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial not installed")
        if "://" in self.port:
            self._ser = serial.serial_for_url(self.port, self.baud, timeout=self.timeout)
        else:
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
        self._ser = None

    def poll_line(self) -> Optional[str]:
        if self._ser is None:
            return None
        raw = self._ser.readline()
        if not raw:
            return None
        return raw.decode(errors="ignore").strip()


@dataclass
class UDPInput:
    host: str
    port: int

    def __post_init__(self) -> None:
        self._sock = None

    def open(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.host, self.port))
        self._sock.setblocking(False)

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
        self._sock = None

    def poll_line(self) -> Optional[str]:
        if self._sock is None:
            return None
        try:
            data, _ = self._sock.recvfrom(4096)
        except BlockingIOError:
            return None
        return data.decode(errors="ignore").strip()
