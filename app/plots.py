from __future__ import annotations

from typing import Dict, List

import numpy as np
from matplotlib.axes import Axes

from app.math_core import moved_anchors
from app.models import CABLES, RigConfig


def _coord_map(cfg: RigConfig, moved: Dict[str, np.ndarray], key: str) -> tuple[np.ndarray, np.ndarray]:
    fixed = np.array([cfg.fixed_points[key].x, cfg.fixed_points[key].y, cfg.fixed_points[key].z], dtype=float)
    anchor = moved[key]
    return fixed, anchor


def _plot_platform_outline(ax: Axes, moved: Dict[str, np.ndarray], x_idx: int, y_idx: int) -> None:
    order = ["FL", "FR", "BR", "BL", "FL"]
    xs = [moved[key][x_idx] for key in order]
    ys = [moved[key][y_idx] for key in order]
    ax.plot(xs, ys, color="tab:orange", linewidth=2.2, alpha=0.85)
    ax.fill(xs, ys, color="tab:orange", alpha=0.08)


def plot_top_view(ax: Axes, cfg: RigConfig, roll_rad: float, pitch_rad: float, lengths_ok: Dict[str, bool]) -> None:
    ax.clear()
    moved = moved_anchors(cfg, roll_rad, pitch_rad)
    _plot_platform_outline(ax, moved, 0, 1)
    for key in CABLES:
        fixed, anchor = _coord_map(cfg, moved, key)
        color = "tab:green" if lengths_ok.get(key, True) else "tab:red"
        ax.plot([fixed[0], anchor[0]], [fixed[1], anchor[1]], color=color, linewidth=1.8)
        ax.scatter([fixed[0]], [fixed[1]], marker="s", color="tab:blue", s=40)
        ax.scatter([anchor[0]], [anchor[1]], marker="o", color="tab:orange", s=38)
        ax.text(anchor[0], anchor[1], f" {key}", va="bottom")
    ax.scatter([cfg.rotation_center.x], [cfg.rotation_center.y], marker="x", color="black", s=60)
    ax.set_title("Top View (x-y)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)


def plot_side_view(ax: Axes, cfg: RigConfig, roll_rad: float, pitch_rad: float, lengths_ok: Dict[str, bool]) -> None:
    ax.clear()
    moved = moved_anchors(cfg, roll_rad, pitch_rad)
    _plot_platform_outline(ax, moved, 0, 2)
    for key in CABLES:
        fixed, anchor = _coord_map(cfg, moved, key)
        color = "tab:green" if lengths_ok.get(key, True) else "tab:red"
        ax.plot([fixed[0], anchor[0]], [fixed[2], anchor[2]], color=color, linewidth=1.8)
        ax.scatter([fixed[0]], [fixed[2]], marker="s", color="tab:blue", s=40)
        ax.scatter([anchor[0]], [anchor[2]], marker="o", color="tab:orange", s=38)
        ax.text(anchor[0], anchor[2], f" {key}", va="bottom")
    ax.scatter([cfg.rotation_center.x], [cfg.rotation_center.z], marker="x", color="black", s=60)
    ax.set_title("Side View (x-z)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)


def plot_heatmap(ax: Axes, heatmap: List[List[float]], pitch_min: float, pitch_max: float, roll_min: float, roll_max: float) -> None:
    ax.clear()
    arr = np.array(heatmap, dtype=float)
    image = ax.imshow(
        arr,
        origin="lower",
        extent=[pitch_min, pitch_max, roll_min, roll_max],
        aspect="auto",
        cmap="viridis",
    )
    colorbar = getattr(ax.figure, "_pose_translator_colorbar", None)
    if colorbar is not None:
        colorbar.remove()
    ax.figure._pose_translator_colorbar = ax.figure.colorbar(image, ax=ax, label="RMS error [m]")
    ax.set_xlabel("Pitch [deg]")
    ax.set_ylabel("Roll [deg]")
    ax.set_title("Workspace RMS Heatmap")
