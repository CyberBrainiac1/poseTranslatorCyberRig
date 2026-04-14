from __future__ import annotations

from app.flypt_io import parse_pitch_roll_line
from app.math_core import neutral_lengths, solve_pose_to_cable
from app.models import Limits, RigConfig, SolveInput, Vec3
from app.serial_output import SerialOutput, format_output


def make_cfg() -> RigConfig:
    return RigConfig(
        moving_anchors={
            "FL": Vec3(0.36, 0.26, 0.0),
            "BL": Vec3(-0.34, 0.26, 0.0),
            "FR": Vec3(0.36, -0.26, 0.0),
            "BR": Vec3(-0.34, -0.26, 0.0),
        },
        fixed_points={
            "FL": Vec3(0.52, 0.51, 0.56),
            "BL": Vec3(-0.52, 0.51, 0.56),
            "FR": Vec3(0.52, -0.51, 0.56),
            "BR": Vec3(-0.52, -0.51, 0.56),
        },
        rotation_center=Vec3(0.0, 0.0, -0.04),
        spool_radii={"FL": 0.026, "BL": 0.024, "FR": 0.026, "BR": 0.024},
        winding_signs={"FL": 1, "BL": -1, "FR": 1, "BR": -1},
        motor_encoder_cpr=2048,
        gearbox_ratio=40,
        cable_weights={"FL": 1.0, "BL": 1.0, "FR": 1.0, "BR": 1.0},
        limits=Limits(cable_min=0.15, cable_max=1.4, motor_min_rad=-2.6, motor_max_rad=2.6, rms_error_max=0.01, slack_pay_out_threshold=0.02),
    )


def test_udp_pose_to_serial_command_pipeline_works_with_loopback() -> None:
    parsed = parse_pitch_roll_line("P=3.0,R=-1.5")
    assert parsed == (3.0, -1.5)

    cfg = make_cfg()
    result = solve_pose_to_cable(
        cfg,
        SolveInput(
            pitch_deg=parsed[0],
            roll_deg=parsed[1],
            current_cable_lengths=neutral_lengths(cfg),
            current_counts_left=0.0,
            current_counts_right=0.0,
        ),
    )
    left_counts = result.clamped_counts_left if result.clamped_counts_left is not None else result.target_counts_left
    right_counts = result.clamped_counts_right if result.clamped_counts_right is not None else result.target_counts_right
    assert left_counts is not None
    assert right_counts is not None

    output = SerialOutput(port="loop://", baud=115200, timeout=0.05)
    output.open()
    try:
        payload = format_output(
            "L={left_counts},R={right_counts}\\n",
            left_counts,
            right_counts,
            result.delta_counts_left,
            result.delta_counts_right,
        )
        output.send_line(payload)
        assert output.read_line() == payload.strip()
    finally:
        output.close()
