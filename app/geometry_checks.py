from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from app.math_core import neutral_lengths, solve_pose_to_cable
from app.models import CABLES, RigConfig, SolveInput, SweepRow, SweepSettings, WorkspaceAnalysis


@dataclass
class GeometryCheckResult:
    warnings: List[str]
    asymmetric_score: float
    neutral_lengths: Dict[str, float]


def _mirror_error(a: np.ndarray, b: np.ndarray) -> float:
    mirrored = np.array([a[0], -a[1], a[2]], dtype=float)
    return float(np.linalg.norm(mirrored - b))


def _vec3(cfg_map: Dict[str, object], key: str) -> np.ndarray:
    v = cfg_map[key]
    return np.array([v.x, v.y, v.z], dtype=float)


def check_geometry_consistency(cfg: RigConfig) -> GeometryCheckResult:
    cfg.validate()
    warnings: List[str] = []

    moving_asymmetry = _mirror_error(_vec3(cfg.moving_anchors, "FL"), _vec3(cfg.moving_anchors, "FR"))
    moving_asymmetry += _mirror_error(_vec3(cfg.moving_anchors, "BL"), _vec3(cfg.moving_anchors, "BR"))
    fixed_asymmetry = _mirror_error(_vec3(cfg.fixed_points, "FL"), _vec3(cfg.fixed_points, "FR"))
    fixed_asymmetry += _mirror_error(_vec3(cfg.fixed_points, "BL"), _vec3(cfg.fixed_points, "BR"))
    asymmetry_score = moving_asymmetry + fixed_asymmetry

    if asymmetry_score > 0.05:
        warnings.append(f"Geometry appears noticeably asymmetric (mirror score {asymmetry_score:.4f})")

    left_radius_mismatch = abs(cfg.spool_radii["FL"] - cfg.spool_radii["BL"])
    right_radius_mismatch = abs(cfg.spool_radii["FR"] - cfg.spool_radii["BR"])
    if left_radius_mismatch > 0.002:
        warnings.append(f"Left spool radii differ by {left_radius_mismatch:.4f} m; coupling error will increase")
    if right_radius_mismatch > 0.002:
        warnings.append(f"Right spool radii differ by {right_radius_mismatch:.4f} m; coupling error will increase")

    if cfg.winding_signs["FL"] != cfg.winding_signs["BL"]:
        warnings.append("Left motor winding signs differ between FL and BL")
    if cfg.winding_signs["FR"] != cfg.winding_signs["BR"]:
        warnings.append("Right motor winding signs differ between FR and BR")

    center = np.array([cfg.rotation_center.x, cfg.rotation_center.y, cfg.rotation_center.z], dtype=float)
    for key in CABLES:
        anchor = _vec3(cfg.moving_anchors, key)
        fixed = _vec3(cfg.fixed_points, key)
        if np.linalg.norm(anchor - center) < 1e-4:
            warnings.append(f"{key} moving anchor is almost on the rotation center; sensitivity will be poor")
        if fixed[2] <= anchor[2]:
            warnings.append(f"{key} fixed exit is not above its moving anchor; string routing may be problematic")

    neutral = neutral_lengths(cfg)
    for key, length in neutral.items():
        if length < cfg.limits.cable_min or length > cfg.limits.cable_max:
            warnings.append(f"{key} neutral cable length {length:.4f} m is outside configured limits")

    return GeometryCheckResult(warnings=warnings, asymmetric_score=asymmetry_score, neutral_lengths=neutral)


def analyze_workspace(cfg: RigConfig, settings: SweepSettings) -> WorkspaceAnalysis:
    cfg.validate()
    pitch_values = np.linspace(settings.pitch_min_deg, settings.pitch_max_deg, settings.pitch_steps)
    roll_values = np.linspace(settings.roll_min_deg, settings.roll_max_deg, settings.roll_steps)

    rows: List[SweepRow] = []
    heatmap: List[List[float]] = []
    cable_lengths: Dict[str, List[float]] = {k: [] for k in CABLES}
    q_left_values: List[float] = []
    q_right_values: List[float] = []
    max_rms = -1.0
    worst_pose = (0.0, 0.0)

    measured_reference = neutral_lengths(cfg)

    for roll_deg in roll_values:
        row_values: List[float] = []
        for pitch_deg in pitch_values:
            result = solve_pose_to_cable(
                cfg,
                SolveInput(
                    pitch_deg=float(pitch_deg),
                    roll_deg=float(roll_deg),
                    current_cable_lengths=measured_reference,
                ),
            )
            valid = len(result.warnings) == 0
            rows.append(
                SweepRow(
                    pitch_deg=float(pitch_deg),
                    roll_deg=float(roll_deg),
                    target_lengths=dict(result.target_lengths),
                    ideal_deltas=dict(result.ideal_deltas),
                    predicted_deltas=dict(result.predicted_deltas),
                    q_left_rad=float(result.q_left_rad),
                    q_right_rad=float(result.q_right_rad),
                    residuals=dict(result.residuals),
                    rms_error=float(result.rms_error),
                    valid=valid,
                    flags=list(result.warnings),
                )
            )
            row_values.append(float(result.rms_error))
            for key in CABLES:
                cable_lengths[key].append(float(result.target_lengths[key]))
            q_left_values.append(float(result.q_left_rad))
            q_right_values.append(float(result.q_right_rad))
            if result.rms_error > max_rms:
                max_rms = float(result.rms_error)
                worst_pose = (float(pitch_deg), float(roll_deg))
        heatmap.append(row_values)

    rms_values = np.array([row.rms_error for row in rows], dtype=float)
    valid_count = sum(1 for row in rows if row.valid)
    cable_ranges = {key: (float(np.min(values)), float(np.max(values))) for key, values in cable_lengths.items()}
    motor_ranges = {
        "left": (float(np.min(q_left_values)), float(np.max(q_left_values))),
        "right": (float(np.min(q_right_values)), float(np.max(q_right_values))),
    }

    return WorkspaceAnalysis(
        max_rms=float(max_rms if max_rms >= 0 else 0.0),
        mean_rms=float(np.mean(rms_values)) if len(rms_values) else 0.0,
        worst_pose=worst_pose,
        valid_pct=(100.0 * valid_count / len(rows)) if rows else 0.0,
        cable_length_ranges=cable_ranges,
        motor_angle_range=motor_ranges,
        heatmap=heatmap,
        pitch_values_deg=[float(v) for v in pitch_values],
        roll_values_deg=[float(v) for v in roll_values],
        rows=rows,
    )
