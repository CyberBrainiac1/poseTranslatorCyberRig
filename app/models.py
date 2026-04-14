from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Tuple

CABLES: Tuple[str, ...] = ("FL", "BL", "FR", "BR")
LEFT_CABLES: Tuple[str, ...] = ("FL", "BL")
RIGHT_CABLES: Tuple[str, ...] = ("FR", "BR")


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def to_list(self) -> List[float]:
        return [float(self.x), float(self.y), float(self.z)]

    @staticmethod
    def from_obj(obj: object, name: str) -> "Vec3":
        if isinstance(obj, dict):
            try:
                return Vec3(float(obj["x"]), float(obj["y"]), float(obj["z"]))
            except Exception as exc:
                raise ValueError(f"Invalid vec3 dict for {name}") from exc
        if isinstance(obj, (list, tuple)) and len(obj) == 3:
            return Vec3(float(obj[0]), float(obj[1]), float(obj[2]))
        raise ValueError(f"Invalid vec3 for {name}")


@dataclass
class Limits:
    cable_min: float = 0.05
    cable_max: float = 2.0
    motor_min_rad: float = -3.14
    motor_max_rad: float = 3.14
    rms_error_max: float = 0.005


@dataclass
class RigConfig:
    moving_anchors: Dict[str, Vec3]
    fixed_points: Dict[str, Vec3]
    rotation_center: Vec3
    spool_radii: Dict[str, float]
    winding_signs: Dict[str, int]
    counts_per_output_rev: Optional[float] = None
    motor_encoder_cpr: Optional[float] = None
    gearbox_ratio: Optional[float] = None
    cable_weights: Dict[str, float] = field(default_factory=lambda: {k: 1.0 for k in CABLES})
    limits: Limits = field(default_factory=Limits)

    def validate(self) -> None:
        for name in CABLES:
            if name not in self.moving_anchors or name not in self.fixed_points:
                raise ValueError(f"Missing anchor/fixed point for {name}")
            if name not in self.spool_radii:
                raise ValueError(f"Missing spool radius for {name}")
            if self.spool_radii[name] <= 0:
                raise ValueError(f"Spool radius must be > 0 for {name}")
            if self.winding_signs.get(name) not in (-1, 1):
                raise ValueError(f"Winding sign must be +1 or -1 for {name}")
            if self.cable_weights.get(name, 0.0) <= 0:
                raise ValueError(f"Cable weight must be > 0 for {name}")
        if self.limits.cable_min >= self.limits.cable_max:
            raise ValueError("Cable limits invalid")
        if self.limits.motor_min_rad >= self.limits.motor_max_rad:
            raise ValueError("Motor angle limits invalid")
        if self.limits.rms_error_max < 0:
            raise ValueError("RMS threshold must be >= 0")

    def effective_counts_per_output_rev(self) -> Optional[float]:
        if self.counts_per_output_rev is not None:
            return float(self.counts_per_output_rev)
        if self.motor_encoder_cpr is not None and self.gearbox_ratio is not None:
            return float(self.motor_encoder_cpr) * float(self.gearbox_ratio)
        return None


@dataclass
class SolveInput:
    pitch_deg: float
    roll_deg: float
    current_cable_lengths: Dict[str, float]
    current_counts_left: Optional[float] = None
    current_counts_right: Optional[float] = None


@dataclass
class SolveResult:
    neutral_lengths: Dict[str, float]
    target_lengths: Dict[str, float]
    ideal_deltas: Dict[str, float]
    q_left_rad: float
    q_right_rad: float
    predicted_deltas: Dict[str, float]
    residuals: Dict[str, float]
    rms_error: float
    target_counts_left: Optional[float]
    target_counts_right: Optional[float]
    delta_counts_left: Optional[float]
    delta_counts_right: Optional[float]
    clamped_counts_left: Optional[float]
    clamped_counts_right: Optional[float]
    warnings: List[str]


@dataclass
class PoseEstimate:
    pitch_deg: float
    roll_deg: float
    residual_rms: float
    valid: bool


@dataclass
class SweepSettings:
    pitch_min_deg: float = -15.0
    pitch_max_deg: float = 15.0
    roll_min_deg: float = -15.0
    roll_max_deg: float = 15.0
    pitch_steps: int = 31
    roll_steps: int = 31


@dataclass
class SweepRow:
    pitch_deg: float
    roll_deg: float
    ideal_deltas: Dict[str, float]
    q_left_rad: float
    q_right_rad: float
    residuals: Dict[str, float]
    rms_error: float
    valid: bool
    flags: List[str]


@dataclass
class WorkspaceAnalysis:
    max_rms: float
    mean_rms: float
    worst_pose: Tuple[float, float]
    valid_pct: float
    cable_length_ranges: Dict[str, Tuple[float, float]]
    motor_angle_range: Dict[str, Tuple[float, float]]
    heatmap: List[List[float]]
    rows: List[SweepRow]


def rig_config_to_dict(cfg: RigConfig) -> Dict[str, object]:
    return {
        "moving_anchors": {k: v.to_list() for k, v in cfg.moving_anchors.items()},
        "fixed_points": {k: v.to_list() for k, v in cfg.fixed_points.items()},
        "rotation_center": cfg.rotation_center.to_list(),
        "spool_radii": {k: float(v) for k, v in cfg.spool_radii.items()},
        "winding_signs": {k: int(v) for k, v in cfg.winding_signs.items()},
        "counts_per_output_rev": cfg.counts_per_output_rev,
        "motor_encoder_cpr": cfg.motor_encoder_cpr,
        "gearbox_ratio": cfg.gearbox_ratio,
        "cable_weights": {k: float(v) for k, v in cfg.cable_weights.items()},
        "limits": asdict(cfg.limits),
    }


def rig_config_from_dict(data: Dict[str, object]) -> RigConfig:
    try:
        moving_anchors = {k: Vec3.from_obj(v, f"moving_anchors.{k}") for k, v in dict(data["moving_anchors"]).items()}
        fixed_points = {k: Vec3.from_obj(v, f"fixed_points.{k}") for k, v in dict(data["fixed_points"]).items()}
        rotation_center = Vec3.from_obj(data["rotation_center"], "rotation_center")
        spool_radii = {k: float(v) for k, v in dict(data["spool_radii"]).items()}
        winding_signs = {k: int(v) for k, v in dict(data["winding_signs"]).items()}
        cable_weights_data = data.get("cable_weights", {k: 1.0 for k in CABLES})
        cable_weights = {k: float(v) for k, v in dict(cable_weights_data).items()}
        limits_data = dict(data.get("limits", {}))
        limits = Limits(
            cable_min=float(limits_data.get("cable_min", 0.05)),
            cable_max=float(limits_data.get("cable_max", 2.0)),
            motor_min_rad=float(limits_data.get("motor_min_rad", -3.14)),
            motor_max_rad=float(limits_data.get("motor_max_rad", 3.14)),
            rms_error_max=float(limits_data.get("rms_error_max", 0.005)),
        )
        cfg = RigConfig(
            moving_anchors=moving_anchors,
            fixed_points=fixed_points,
            rotation_center=rotation_center,
            spool_radii=spool_radii,
            winding_signs=winding_signs,
            counts_per_output_rev=float(data["counts_per_output_rev"]) if data.get("counts_per_output_rev") is not None else None,
            motor_encoder_cpr=float(data["motor_encoder_cpr"]) if data.get("motor_encoder_cpr") is not None else None,
            gearbox_ratio=float(data["gearbox_ratio"]) if data.get("gearbox_ratio") is not None else None,
            cable_weights=cable_weights,
            limits=limits,
        )
        cfg.validate()
        return cfg
    except KeyError as exc:
        raise ValueError(f"Missing config key: {exc}") from exc
    except TypeError as exc:
        raise ValueError("Config must be JSON object") from exc
