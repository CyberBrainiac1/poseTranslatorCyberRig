from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, radians, sin
from typing import Dict, List, Tuple

import numpy as np

from app.models import CABLES, LEFT_CABLES, RIGHT_CABLES, PoseEstimate, RigConfig, SolveInput, SolveResult


def rx(phi_rad: float) -> np.ndarray:
    c = cos(phi_rad)
    s = sin(phi_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def ry(theta_rad: float) -> np.ndarray:
    c = cos(theta_rad)
    s = sin(theta_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rotation_matrix(roll_rad: float, pitch_rad: float) -> np.ndarray:
    return ry(pitch_rad) @ rx(roll_rad)


def _vec(v) -> np.ndarray:
    return np.array([v.x, v.y, v.z], dtype=float)


def moved_anchors(cfg: RigConfig, roll_rad: float, pitch_rad: float) -> Dict[str, np.ndarray]:
    r = rotation_matrix(roll_rad, pitch_rad)
    c = _vec(cfg.rotation_center)
    out: Dict[str, np.ndarray] = {}
    for k in CABLES:
        a = _vec(cfg.moving_anchors[k])
        out[k] = r @ (a - c) + c
    return out


def cable_lengths_for_pose(cfg: RigConfig, roll_rad: float, pitch_rad: float) -> Dict[str, float]:
    moved = moved_anchors(cfg, roll_rad, pitch_rad)
    lengths: Dict[str, float] = {}
    for k in CABLES:
        f = _vec(cfg.fixed_points[k])
        lengths[k] = float(np.linalg.norm(f - moved[k]))
    return lengths


def neutral_lengths(cfg: RigConfig) -> Dict[str, float]:
    return cable_lengths_for_pose(cfg, 0.0, 0.0)


def ideal_cable_deltas(cfg: RigConfig, roll_rad: float, pitch_rad: float) -> Tuple[Dict[str, float], Dict[str, float], Dict[str, float]]:
    l0 = neutral_lengths(cfg)
    lt = cable_lengths_for_pose(cfg, roll_rad, pitch_rad)
    deltas = {k: lt[k] - l0[k] for k in CABLES}
    return l0, lt, deltas


def _solve_side(cables: Tuple[str, str], deltas: Dict[str, float], cfg: RigConfig) -> float:
    num = 0.0
    den = 0.0
    for k in cables:
        coeff = cfg.winding_signs[k] * cfg.spool_radii[k]
        w = cfg.cable_weights[k]
        num += w * coeff * deltas[k]
        den += w * coeff * coeff
    if den <= 0:
        raise ValueError("Invalid denominator in least-squares side solve")
    return num / den


def solve_motor_angles(cfg: RigConfig, deltas: Dict[str, float]) -> Tuple[float, float]:
    q_left = _solve_side(LEFT_CABLES, deltas, cfg)
    q_right = _solve_side(RIGHT_CABLES, deltas, cfg)
    return q_left, q_right


def predicted_deltas_from_motors(cfg: RigConfig, q_left: float, q_right: float) -> Dict[str, float]:
    return {
        "FL": cfg.winding_signs["FL"] * cfg.spool_radii["FL"] * q_left,
        "BL": cfg.winding_signs["BL"] * cfg.spool_radii["BL"] * q_left,
        "FR": cfg.winding_signs["FR"] * cfg.spool_radii["FR"] * q_right,
        "BR": cfg.winding_signs["BR"] * cfg.spool_radii["BR"] * q_right,
    }


def residuals(ideal: Dict[str, float], predicted: Dict[str, float]) -> Dict[str, float]:
    return {k: ideal[k] - predicted[k] for k in CABLES}


def rms_error(values: Dict[str, float]) -> float:
    arr = np.array([values[k] for k in CABLES], dtype=float)
    return float(np.sqrt(np.mean(arr * arr)))


def counts_from_angle_rad(q_rad: float, counts_per_output_rev: float) -> float:
    return q_rad / (2.0 * pi) * counts_per_output_rev


def solve_pose_to_cable(cfg: RigConfig, solve_input: SolveInput) -> SolveResult:
    cfg.validate()
    roll = radians(solve_input.roll_deg)
    pitch = radians(solve_input.pitch_deg)
    l0, lt, ideal = ideal_cable_deltas(cfg, roll, pitch)
    q_left, q_right = solve_motor_angles(cfg, ideal)
    pred = predicted_deltas_from_motors(cfg, q_left, q_right)
    res = residuals(ideal, pred)
    rms = rms_error(res)

    cpr = cfg.effective_counts_per_output_rev()
    left_counts = counts_from_angle_rad(q_left, cpr) if cpr else None
    right_counts = counts_from_angle_rad(q_right, cpr) if cpr else None

    dleft = None
    dright = None
    if left_counts is not None and solve_input.current_counts_left is not None:
        dleft = left_counts - solve_input.current_counts_left
    if right_counts is not None and solve_input.current_counts_right is not None:
        dright = right_counts - solve_input.current_counts_right

    warnings: List[str] = []
    lim = cfg.limits
    for k in CABLES:
        if lt[k] < lim.cable_min or lt[k] > lim.cable_max:
            warnings.append(f"{k} cable length out of bounds: {lt[k]:.4f} m")
        if lt[k] <= lim.cable_min * 1.02:
            warnings.append(f"{k} may go slack / near minimum tension envelope")
    if q_left < lim.motor_min_rad or q_left > lim.motor_max_rad:
        warnings.append("Left motor angle out of limits")
    if q_right < lim.motor_min_rad or q_right > lim.motor_max_rad:
        warnings.append("Right motor angle out of limits")
    if rms > lim.rms_error_max:
        warnings.append(f"RMS coupling error {rms:.6f} exceeds threshold {lim.rms_error_max:.6f}")

    cl_left = float(np.clip(left_counts, lim.motor_min_rad / (2 * pi) * cpr, lim.motor_max_rad / (2 * pi) * cpr)) if (left_counts is not None and cpr) else None
    cl_right = float(np.clip(right_counts, lim.motor_min_rad / (2 * pi) * cpr, lim.motor_max_rad / (2 * pi) * cpr)) if (right_counts is not None and cpr) else None

    return SolveResult(
        neutral_lengths=l0,
        target_lengths=lt,
        ideal_deltas=ideal,
        q_left_rad=q_left,
        q_right_rad=q_right,
        predicted_deltas=pred,
        residuals=res,
        rms_error=rms,
        target_counts_left=left_counts,
        target_counts_right=right_counts,
        delta_counts_left=dleft,
        delta_counts_right=dright,
        clamped_counts_left=cl_left,
        clamped_counts_right=cl_right,
        warnings=warnings,
    )


def _solve_inverse_step(cfg: RigConfig, lengths: Dict[str, float], roll: float, pitch: float) -> Tuple[np.ndarray, np.ndarray]:
    eps = 1e-5
    base = cable_lengths_for_pose(cfg, roll, pitch)
    err = np.array([base[k] - lengths[k] for k in CABLES], dtype=float)

    plus_r = cable_lengths_for_pose(cfg, roll + eps, pitch)
    minus_r = cable_lengths_for_pose(cfg, roll - eps, pitch)
    plus_p = cable_lengths_for_pose(cfg, roll, pitch + eps)
    minus_p = cable_lengths_for_pose(cfg, roll, pitch - eps)

    j = np.zeros((4, 2), dtype=float)
    for i, k in enumerate(CABLES):
        j[i, 0] = (plus_r[k] - minus_r[k]) / (2 * eps)
        j[i, 1] = (plus_p[k] - minus_p[k]) / (2 * eps)
    return err, j


def estimate_pose_from_cable_lengths(cfg: RigConfig, measured_lengths: Dict[str, float], max_iter: int = 30) -> PoseEstimate:
    cfg.validate()
    for k in CABLES:
        if measured_lengths[k] <= 0:
            return PoseEstimate(0.0, 0.0, 1e9, False)

    roll = 0.0
    pitch = 0.0
    for _ in range(max_iter):
        err, j = _solve_inverse_step(cfg, measured_lengths, roll, pitch)
        try:
            step, *_ = np.linalg.lstsq(j, err, rcond=None)
        except np.linalg.LinAlgError:
            return PoseEstimate(np.degrees(pitch), np.degrees(roll), 1e9, False)
        roll -= float(step[0])
        pitch -= float(step[1])
        if np.linalg.norm(step) < 1e-8:
            break

    final = cable_lengths_for_pose(cfg, roll, pitch)
    res = {k: final[k] - measured_lengths[k] for k in CABLES}
    rms = rms_error(res)
    valid = rms <= cfg.limits.rms_error_max * 2.0
    return PoseEstimate(pitch_deg=float(np.degrees(pitch)), roll_deg=float(np.degrees(roll)), residual_rms=rms, valid=valid)
