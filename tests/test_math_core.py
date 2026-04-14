from __future__ import annotations

import math

from app.math_core import (
    counts_from_angle_rad,
    estimate_pose_from_cable_lengths,
    ideal_cable_deltas,
    solve_motor_angles,
    solve_pose_to_cable,
)
from app.models import RigConfig, SolveInput, Vec3


def make_cfg() -> RigConfig:
    return RigConfig(
        moving_anchors={
            "FL": Vec3(0.3, 0.2, 0.0),
            "BL": Vec3(-0.3, 0.2, 0.0),
            "FR": Vec3(0.3, -0.2, 0.0),
            "BR": Vec3(-0.3, -0.2, 0.0),
        },
        fixed_points={
            "FL": Vec3(0.5, 0.5, 0.5),
            "BL": Vec3(-0.5, 0.5, 0.5),
            "FR": Vec3(0.5, -0.5, 0.5),
            "BR": Vec3(-0.5, -0.5, 0.5),
        },
        rotation_center=Vec3(0.0, 0.0, 0.0),
        spool_radii={"FL": 0.02, "BL": 0.02, "FR": 0.02, "BR": 0.02},
        winding_signs={"FL": 1, "BL": 1, "FR": 1, "BR": 1},
        counts_per_output_rev=4096,
    )


def test_zero_pose_near_zero_deltas() -> None:
    cfg = make_cfg()
    l0, _, d = ideal_cable_deltas(cfg, 0.0, 0.0)
    assert all(abs(v) < 1e-12 for v in d.values())
    assert all(v > 0 for v in l0.values())


def test_symmetric_geometry_gives_symmetric_outputs() -> None:
    cfg = make_cfg()
    out = solve_pose_to_cable(
        cfg,
        SolveInput(pitch_deg=5.0, roll_deg=0.0, current_cable_lengths={k: 0.0 for k in ["FL", "BL", "FR", "BR"]}),
    )
    assert math.isclose(out.ideal_deltas["FL"], out.ideal_deltas["FR"], rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(out.ideal_deltas["BL"], out.ideal_deltas["BR"], rel_tol=1e-6, abs_tol=1e-6)


def test_counts_conversion() -> None:
    q = 2 * math.pi
    c = counts_from_angle_rad(q, 4096)
    assert math.isclose(c, 4096.0, rel_tol=1e-9)


def test_least_squares_known_case() -> None:
    cfg = make_cfg()
    d = {"FL": 0.02, "BL": 0.04, "FR": 0.06, "BR": 0.08}
    # With equal radii/signs/weights, q is the mean delta divided by radius:
    # qL=((0.02+0.04)/2)/0.02=1.5, qR=((0.06+0.08)/2)/0.02=3.5
    ql, qr = solve_motor_angles(cfg, d)
    assert math.isclose(ql, 1.5, rel_tol=1e-9)
    assert math.isclose(qr, 3.5, rel_tol=1e-9)


def test_counts_can_be_derived_from_encoder_and_gearbox() -> None:
    cfg = make_cfg()
    cfg.counts_per_output_rev = None
    cfg.motor_encoder_cpr = 2048
    cfg.gearbox_ratio = 50
    out = solve_pose_to_cable(
        cfg,
        SolveInput(pitch_deg=3.0, roll_deg=0.0, current_cable_lengths={k: 0.0 for k in ["FL", "BL", "FR", "BR"]}),
    )
    assert out.counts_per_output_rev == 102400


def test_inverse_solver_round_trips_pose() -> None:
    cfg = make_cfg()
    forward = solve_pose_to_cable(
        cfg,
        SolveInput(pitch_deg=4.0, roll_deg=-2.0, current_cable_lengths={k: 0.0 for k in ["FL", "BL", "FR", "BR"]}),
    )
    estimate = estimate_pose_from_cable_lengths(cfg, forward.target_lengths)
    assert estimate.valid
    assert math.isclose(estimate.pitch_deg, 4.0, abs_tol=0.05)
    assert math.isclose(estimate.roll_deg, -2.0, abs_tol=0.05)
