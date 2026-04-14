from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover
    serial = None


def format_output(template: str, left_counts: float, right_counts: float) -> str:
    return template.format(left_counts=int(round(left_counts)), right_counts=int(round(right_counts)))


@dataclass
class SerialOutput:
    port: str
    baud: int = 115200
    timeout: float = 0.01

    def __post_init__(self) -> None:
        self._ser = None

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial not installed")
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
        self._ser = None

    def send_line(self, payload: str) -> None:
        if self._ser is None:
            return
        self._ser.write(payload.encode())


def format_csv(left_counts: float, right_counts: float) -> str:
    return f"{int(round(left_counts))},{int(round(right_counts))}\\n"
