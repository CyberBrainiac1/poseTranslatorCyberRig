from __future__ import annotations

import json
from pathlib import Path

from app.models import RigConfig, rig_config_from_dict, rig_config_to_dict


def save_config(path: str, cfg: RigConfig) -> None:
    cfg.validate()
    payload = rig_config_to_dict(cfg)
    Path(path).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def load_config(path: str) -> RigConfig:
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError("Invalid JSON config") from exc
    cfg = rig_config_from_dict(data)
    cfg.validate()
    return cfg
