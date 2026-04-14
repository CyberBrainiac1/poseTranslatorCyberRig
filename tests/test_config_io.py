from __future__ import annotations

import json

import pytest

from app.config_io import load_config, save_config
from app.models import RigConfig, Vec3


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


def test_config_save_load(tmp_path) -> None:
    cfg = make_cfg()
    p = tmp_path / "cfg.json"
    save_config(str(p), cfg)
    loaded = load_config(str(p))
    assert loaded.spool_radii == cfg.spool_radii
    assert loaded.winding_signs == cfg.winding_signs
    assert loaded.limits.slack_pay_out_threshold == cfg.limits.slack_pay_out_threshold


def test_invalid_config_raises(tmp_path) -> None:
    p = tmp_path / "bad.json"
    p.write_text(json.dumps({"bad": 1}), encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(str(p))


def test_invalid_spool_radius_raises(tmp_path) -> None:
    cfg = make_cfg()
    payload = {
        "moving_anchors": {k: v.to_list() for k, v in cfg.moving_anchors.items()},
        "fixed_points": {k: v.to_list() for k, v in cfg.fixed_points.items()},
        "rotation_center": cfg.rotation_center.to_list(),
        "spool_radii": {"FL": 0.0, "BL": 0.02, "FR": 0.02, "BR": 0.02},
        "winding_signs": {"FL": 1, "BL": 1, "FR": 1, "BR": 1},
        "counts_per_output_rev": 4096,
        "cable_weights": {"FL": 1.0, "BL": 1.0, "FR": 1.0, "BR": 1.0},
        "limits": {"cable_min": 0.05, "cable_max": 2.0, "motor_min_rad": -3.14, "motor_max_rad": 3.14, "rms_error_max": 0.005},
    }
    p = tmp_path / "bad_radius.json"
    p.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(str(p))
