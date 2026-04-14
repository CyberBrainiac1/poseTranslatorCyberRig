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


def test_invalid_config_raises(tmp_path) -> None:
    p = tmp_path / "bad.json"
    p.write_text(json.dumps({"bad": 1}), encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(str(p))
