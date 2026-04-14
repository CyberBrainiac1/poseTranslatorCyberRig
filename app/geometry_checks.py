from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from app.math_core import solve_pose_to_cable
from app.models import CABLES, RigConfig, SolveInput, SweepRow, SweepSettings, WorkspaceAnalysis


@dataclass
class GeometryCheckResult:
    warnings: List[str]
    asymmetric_score: float


def check_geometry_consistency(cfg: RigConfig) -> GeometryCheckResult:
    warnings: List[str] = []
    cfg.validate()

    for k in CABLES:
        if cfg.spool_radii[k] <= 0:
            warnings.append(f"Invalid spool radius for {k}")

    def mirror_error(a: np.ndarray, b: np.ndarray) -> float:
        mirrored = np.array([a[0], -a[1], a[2]], dtype=float)
        return float(np.linalg.norm(mirrored - b))

    af = cfg.moving_anchors
    ff = cfg.fixed_points
    a_score = mirror_error(np.array([af["FL"].x, af["FL"].y, af["FL"].z]), np.array([af["FR"].x, af["FR"].y, af["FR"].z]))
    a_score += mirror_error(np.array([af["BL"].x, af["BL"].y, af["BL"].z]), np.array([af["BR"].x, af["BR"].y, af["BR"].z]))
    f_score = mirror_error(np.array([ff["FL"].x, ff["FL"].y, ff["FL"].z]), np.array([ff["FR"].x, ff["FR"].y, ff["FR"].z]))
    f_score += mirror_error(np.array([ff["BL"].x, ff["BL"].y, ff["BL"].z]), np.array([ff["BR"].x, ff["BR"].y, ff["BR"].z]))
    asym = a_score + f_score

    if asym > 0.05:
        warnings.append(f"Geometry appears asymmetric/mirrored mismatch (score={asym:.4f})")

    return GeometryCheckResult(warnings=warnings, asymmetric_score=asym)


def analyze_workspace(cfg: RigConfig, settings: SweepSettings) -> WorkspaceAnalysis:
    cfg.validate()
    pitch_values = np.linspace(settings.pitch_min_deg, settings.pitch_max_deg, settings.pitch_steps)
    roll_values = np.linspace(settings.roll_min_deg, settings.roll_max_deg, settings.roll_steps)

    rows: List[SweepRow] = []
    heatmap: List[List[float]] = []

    cable_lens: Dict[str, List[float]] = {k: [] for k in CABLES}
    ql: List[float] = []
    qr: List[float] = []

    max_rms = -1.0
    worst_pose = (0.0, 0.0)

    for r in roll_values:
        row_vals: List[float] = []
        for p in pitch_values:
            res = solve_pose_to_cable(
                cfg,
                SolveInput(
                    pitch_deg=float(p),
                    roll_deg=float(r),
                    current_cable_lengths={k: 0.0 for k in CABLES},
                ),
            )
            valid = len(res.warnings) == 0
            flags = list(res.warnings)
            rows.append(
                SweepRow(
                    pitch_deg=float(p),
                    roll_deg=float(r),
                    ideal_deltas=dict(res.ideal_deltas),
                    q_left_rad=float(res.q_left_rad),
                    q_right_rad=float(res.q_right_rad),
                    residuals=dict(res.residuals),
                    rms_error=float(res.rms_error),
                    valid=valid,
                    flags=flags,
                )
            )
            row_vals.append(float(res.rms_error))
            for k in CABLES:
                cable_lens[k].append(float(res.target_lengths[k]))
            ql.append(float(res.q_left_rad))
            qr.append(float(res.q_right_rad))
            if res.rms_error > max_rms:
                max_rms = float(res.rms_error)
                worst_pose = (float(p), float(r))
        heatmap.append(row_vals)

    rms_values = np.array([r.rms_error for r in rows], dtype=float)
    valid_count = sum(1 for r in rows if r.valid)

    cable_ranges = {k: (float(np.min(v)), float(np.max(v))) for k, v in cable_lens.items()}
    motor_ranges = {"left": (float(np.min(ql)), float(np.max(ql))), "right": (float(np.min(qr)), float(np.max(qr)))}

    return WorkspaceAnalysis(
        max_rms=float(max_rms),
        mean_rms=float(np.mean(rms_values)) if len(rms_values) else 0.0,
        worst_pose=worst_pose,
        valid_pct=(100.0 * valid_count / len(rows)) if rows else 0.0,
        cable_length_ranges=cable_ranges,
        motor_angle_range=motor_ranges,
        heatmap=heatmap,
        rows=rows,
    )
