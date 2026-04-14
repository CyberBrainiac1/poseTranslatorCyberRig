from __future__ import annotations

from app.geometry_checks import analyze_workspace, check_geometry_consistency
from app.models import RigConfig, SweepSettings, Vec3


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


def test_geometry_consistency_ok() -> None:
    result = check_geometry_consistency(make_cfg())
    assert result.asymmetric_score < 1e-6


def test_workspace_analysis_has_rows() -> None:
    cfg = make_cfg()
    out = analyze_workspace(cfg, SweepSettings(pitch_steps=5, roll_steps=4))
    assert len(out.rows) == 20
    assert len(out.heatmap) == 4
    assert len(out.heatmap[0]) == 5
