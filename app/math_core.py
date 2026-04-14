from __future__ import annotations

from math import cos, pi, radians, sin
from typing import Dict, Tuple

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


def _vec(v: object) -> np.ndarray:
    return np.array([v.x, v.y, v.z], dtype=float)


def moved_anchors(cfg: RigConfig, roll_rad: float, pitch_rad: float) -> Dict[str, np.ndarray]:
    r = rotation_matrix(roll_rad, pitch_rad)
    c = _vec(cfg.rotation_center)
    return {k: r @ (_vec(cfg.moving_anchors[k]) - c) + c for k in CABLES}


def cable_lengths_for_pose(cfg: RigConfig, roll_rad: float, pitch_rad: float) -> Dict[str, float]:
    moved = moved_anchors(cfg, roll_rad, pitch_rad)
    return {k: float(np.linalg.norm(_vec(cfg.fixed_points[k]) - moved[k])) for k in CABLES}


def neutral_lengths(cfg: RigConfig) -> Dict[str, float]:
    return cable_lengths_for_pose(cfg, 0.0, 0.0)


def ideal_cable_deltas(cfg: RigConfig, roll_rad: float, pitch_rad: float) -> Tuple[Dict[str, float], Dict[str, float], Dict[str, float]]:
    l0 = neutral_lengths(cfg)
    lt = cable_lengths_for_pose(cfg, roll_rad, pitch_rad)
    return l0, lt, {k: lt[k] - l0[k] for k in CABLES}


def _side_least_squares(cables: Tuple[str, str], deltas: Dict[str, float], cfg: RigConfig) -> float:
    numerator = 0.0
    denominator = 0.0
    for key in cables:
        coeff = cfg.winding_signs[key] * cfg.spool_radii[key]
        weight = cfg.cable_weights[key]
        numerator += weight * coeff * deltas[key]
        denominator += weight * coeff * coeff
    if denominator <= 0:
        raise ValueError("Invalid denominator in least-squares side solve")
    return numerator / denominator


def solve_motor_angles(cfg: RigConfig, deltas: Dict[str, float]) -> Tuple[float, float]:
    return _side_least_squares(LEFT_CABLES, deltas, cfg), _side_least_squares(RIGHT_CABLES, deltas, cfg)


def per_cable_requested_angles(cfg: RigConfig, deltas: Dict[str, float]) -> Dict[str, float]:
    requested: Dict[str, float] = {}
    for key in CABLES:
        coeff = cfg.winding_signs[key] * cfg.spool_radii[key]
        if abs(coeff) < 1e-12:
            raise ValueError(f"Invalid spool coefficient for {key}")
        requested[key] = deltas[key] / coeff
    return requested


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


def counts_limits_from_motor_limits(motor_min_rad: float, motor_max_rad: float, counts_per_output_rev: float) -> Tuple[float, float]:
    return (
        counts_from_angle_rad(motor_min_rad, counts_per_output_rev),
        counts_from_angle_rad(motor_max_rad, counts_per_output_rev),
    )


def _side_mismatch(requested_angles: Dict[str, float], solved_angle: float, cables: Tuple[str, str]) -> float:
    return max(abs(requested_angles[cables[0]] - solved_angle), abs(requested_angles[cables[1]] - solved_angle))


def solve_pose_to_cable(cfg: RigConfig, solve_input: SolveInput) -> SolveResult:
    cfg.validate()
    roll_rad = radians(solve_input.roll_deg)
    pitch_rad = radians(solve_input.pitch_deg)

    neutral, target, ideal = ideal_cable_deltas(cfg, roll_rad, pitch_rad)
    requested_angles = per_cable_requested_angles(cfg, ideal)
    q_left, q_right = solve_motor_angles(cfg, ideal)
    predicted = predicted_deltas_from_motors(cfg, q_left, q_right)
    res = residuals(ideal, predicted)
    rms = rms_error(res)

    counts_per_output_rev = cfg.effective_counts_per_output_rev()
    left_counts = counts_from_angle_rad(q_left, counts_per_output_rev) if counts_per_output_rev else None
    right_counts = counts_from_angle_rad(q_right, counts_per_output_rev) if counts_per_output_rev else None

    delta_left = None
    delta_right = None
    if left_counts is not None and solve_input.current_counts_left is not None:
        delta_left = left_counts - solve_input.current_counts_left
    if right_counts is not None and solve_input.current_counts_right is not None:
        delta_right = right_counts - solve_input.current_counts_right

    side_mismatch = {
        "left": _side_mismatch(requested_angles, q_left, LEFT_CABLES),
        "right": _side_mismatch(requested_angles, q_right, RIGHT_CABLES),
    }

    warnings: list[str] = []
    limits = cfg.limits

    for key in CABLES:
        if target[key] < limits.cable_min or target[key] > limits.cable_max:
            warnings.append(f"{key} cable length out of limits: {target[key]:.4f} m")
        if target[key] <= limits.cable_min * 1.02:
            warnings.append(f"{key} is close to minimum length and may lose tension")
        current_length = solve_input.current_cable_lengths.get(key)
        if current_length is not None and current_length > 0:
            pay_out = target[key] - current_length
            if pay_out > limits.slack_pay_out_threshold:
                warnings.append(f"{key} requires {pay_out:.4f} m pay-out; check string slack risk")

    if q_left < limits.motor_min_rad or q_left > limits.motor_max_rad:
        warnings.append("Left motor angle out of limits")
    if q_right < limits.motor_min_rad or q_right > limits.motor_max_rad:
        warnings.append("Right motor angle out of limits")
    if rms > limits.rms_error_max:
        warnings.append(f"RMS coupling error {rms:.6f} exceeds threshold {limits.rms_error_max:.6f}")
    if side_mismatch["left"] > 0.05:
        warnings.append(f"Left side requests incompatible shaft angles (spread {side_mismatch['left']:.4f} rad)")
    if side_mismatch["right"] > 0.05:
        warnings.append(f"Right side requests incompatible shaft angles (spread {side_mismatch['right']:.4f} rad)")

    clamped_left = None
    clamped_right = None
    if counts_per_output_rev:
        count_min, count_max = counts_limits_from_motor_limits(limits.motor_min_rad, limits.motor_max_rad, counts_per_output_rev)
        if left_counts is not None:
            clamped_left = float(np.clip(left_counts, count_min, count_max))
            if abs(clamped_left - left_counts) > 1e-9:
                warnings.append("Left target counts were clamped by motor limits")
        if right_counts is not None:
            clamped_right = float(np.clip(right_counts, count_min, count_max))
            if abs(clamped_right - right_counts) > 1e-9:
                warnings.append("Right target counts were clamped by motor limits")

    return SolveResult(
        neutral_lengths=neutral,
        target_lengths=target,
        ideal_deltas=ideal,
        per_cable_requested_angles_rad=requested_angles,
        q_left_rad=q_left,
        q_right_rad=q_right,
        predicted_deltas=predicted,
        residuals=res,
        side_mismatch_rad=side_mismatch,
        rms_error=rms,
        counts_per_output_rev=counts_per_output_rev,
        target_counts_left=left_counts,
        target_counts_right=right_counts,
        delta_counts_left=delta_left,
        delta_counts_right=delta_right,
        clamped_counts_left=clamped_left,
        clamped_counts_right=clamped_right,
        warnings=warnings,
    )


def _solve_inverse_step(cfg: RigConfig, measured_lengths: Dict[str, float], roll_rad: float, pitch_rad: float) -> Tuple[np.ndarray, np.ndarray]:
    eps = 1e-5
    base = cable_lengths_for_pose(cfg, roll_rad, pitch_rad)
    err = np.array([base[k] - measured_lengths[k] for k in CABLES], dtype=float)

    plus_roll = cable_lengths_for_pose(cfg, roll_rad + eps, pitch_rad)
    minus_roll = cable_lengths_for_pose(cfg, roll_rad - eps, pitch_rad)
    plus_pitch = cable_lengths_for_pose(cfg, roll_rad, pitch_rad + eps)
    minus_pitch = cable_lengths_for_pose(cfg, roll_rad, pitch_rad - eps)

    jacobian = np.zeros((4, 2), dtype=float)
    for idx, key in enumerate(CABLES):
        jacobian[idx, 0] = (plus_roll[key] - minus_roll[key]) / (2.0 * eps)
        jacobian[idx, 1] = (plus_pitch[key] - minus_pitch[key]) / (2.0 * eps)
    return err, jacobian


def estimate_pose_from_cable_lengths(cfg: RigConfig, measured_lengths: Dict[str, float], max_iter: int = 40) -> PoseEstimate:
    cfg.validate()
    if any(measured_lengths.get(k, 0.0) <= 0 for k in CABLES):
        return PoseEstimate(
            pitch_deg=0.0,
            roll_deg=0.0,
            predicted_lengths={k: 0.0 for k in CABLES},
            residuals={k: 0.0 for k in CABLES},
            residual_rms=1e9,
            valid=False,
            warnings=["Measured cable lengths must all be positive"],
        )

    roll_rad = 0.0
    pitch_rad = 0.0
    damping = 1e-6
    for _ in range(max_iter):
        err, jacobian = _solve_inverse_step(cfg, measured_lengths, roll_rad, pitch_rad)
        jt_j = jacobian.T @ jacobian + damping * np.eye(2)
        jt_e = jacobian.T @ err
        try:
            step = np.linalg.solve(jt_j, jt_e)
        except np.linalg.LinAlgError:
            break
        roll_rad -= float(step[0])
        pitch_rad -= float(step[1])
        if np.linalg.norm(step) < 1e-9:
            break

    predicted = cable_lengths_for_pose(cfg, roll_rad, pitch_rad)
    res = {k: predicted[k] - measured_lengths[k] for k in CABLES}
    rms = rms_error(res)

    warnings: list[str] = []
    if rms > cfg.limits.rms_error_max:
        warnings.append(f"Measured lengths do not fit the rigid pose model cleanly (RMS {rms:.6f} m)")
    for key in CABLES:
        if measured_lengths[key] < cfg.limits.cable_min or measured_lengths[key] > cfg.limits.cable_max:
            warnings.append(f"{key} measured length is outside configured cable limits")

    return PoseEstimate(
        pitch_deg=float(np.degrees(pitch_rad)),
        roll_deg=float(np.degrees(roll_rad)),
        predicted_lengths=predicted,
        residuals=res,
        residual_rms=rms,
        valid=len(warnings) == 0,
        warnings=warnings,
    )
