from __future__ import annotations

import socket
import time

from app.flypt_io import UDPInput, parse_pitch_roll_line


def test_parse_pitch_roll_line_accepts_csv_and_tagged() -> None:
    assert parse_pitch_roll_line("1.25,-2.5") == (1.25, -2.5)
    assert parse_pitch_roll_line("P=1.5,R=-2.0") == (1.5, -2.0)
    assert parse_pitch_roll_line("ignored P=3.0,R=4.0") == (3.0, 4.0)
    assert parse_pitch_roll_line("not a pose") is None


def test_udp_input_receives_tagged_pose_line() -> None:
    source = UDPInput(host="127.0.0.1", port=0)
    source.open()
    try:
        assert source._sock is not None
        port = source._sock.getsockname()[1]
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(b"P=2.5,R=-1.25\r\n", ("127.0.0.1", port))
            line = None
            for _ in range(25):
                line = source.poll_line()
                if line is not None:
                    break
                time.sleep(0.01)
            assert line == "P=2.5,R=-1.25"
            assert parse_pitch_roll_line(line) == (2.5, -1.25)
        finally:
            sender.close()
    finally:
        source.close()
